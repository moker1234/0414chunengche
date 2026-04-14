// Created by forlinx on 2025/12/20.
// Modified by ChatGPT on 2026/01/16

#include "serial_thread.h"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <cstdio>

#include "hex_dump.h"
#include <sys/timerfd.h>
#include <time.h>

uint64_t SerialThread::nowMs() {// timeout
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}


SerialThread::SerialThread(int serial_index,
                             SerialDriverBase& drv,
                             const std::string& name,
                             MsgQueue<SerialEvent>& rxQueue)
    : serial_index_(serial_index),
      driver_(drv),
      port_name_(name),
      rx_queue_(rxQueue) {}

SerialThread::~SerialThread() {
    stop();
}

void SerialThread::start() {
    running_ = true;

    last_tx_ts_ms_ = nowMs();// timeout
    last_rx_ts_ms_ = nowMs();
    rx_timeout_active_ = false;

    thread_ = std::thread(&SerialThread::threadFunc, this);
}

void SerialThread::stop() {
    running_ = false;
    if (event_fd_ >= 0) {
        uint64_t v = 1;
        ::write(event_fd_, &v, sizeof(v));
    }
    if (thread_.joinable())
        thread_.join();
    if (timer_fd_ >= 0) {
        ::close(timer_fd_);
        timer_fd_ = -1;
    }
}

void SerialThread::send(int serial_index, const std::vector<uint8_t>& data) {
    (void)serial_index;

    /* ===== 1. 基本状态校验 ===== */
    if (!running_) {
        LOGWARN("[SERIAL][SEND] ignored: thread not running (%s)",
                port_name_.c_str());
        return;
    }

    if (event_fd_ < 0) {
        LOGERR("[SERIAL][SEND] ignored: invalid event_fd (%s)",
               port_name_.c_str());
        return;
    }

    /* ===== 2. 串口 fd 有效性校验（关键兜底） ===== */
    int serial_fd = driver_.getFd();
    if (serial_fd < 0) {
        LOGERR("[SERIAL][SEND] ignored: serial fd invalid (%s), "
               "maybe permission denied or open failed",
               port_name_.c_str());
        return;
    }

    if (data.empty()) {
        LOGWARN("[SERIAL][SEND] ignored: empty data (%s)",
                port_name_.c_str());
        return;
    }

    /* ===== 3. 入队 ===== */
    {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        tx_queue_.push_back(data);
    }

    /* ===== 4. 唤醒 epoll 写处理 ===== */
    uint64_t v = 1;
    ssize_t n = ::write(event_fd_, &v, sizeof(v));
    if (n != sizeof(v)) {
        LOGERR("[SERIAL][SEND] eventfd write failed (%s), errno=%d",
               port_name_.c_str(), errno);
    }
}


void SerialThread::send(const std::vector<uint8_t>& data) {
    send(0, data);
}

void SerialThread::threadFunc() {
    LOG_SYS_I("SERIAL thread start name=%s idx=%d", port_name_.c_str(), serial_index_);


    pthread_setname_np(pthread_self(), port_name_.c_str());

    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    event_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);

    setupTimerFd(); // timeout

    int serial_fd = driver_.getFd();
    LOG_SYS_I("SERIAL fds name=%s serial_fd=%d event_fd=%d epoll_fd=%d",
          port_name_.c_str(), serial_fd, event_fd_, epoll_fd_);



    epoll_event ev{};

    /* ===== 串口 RX ===== */
    ev.events  = EPOLLIN | EPOLLERR | EPOLLHUP;
    ev.data.fd = serial_fd;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_fd, &ev);

    /* ===== TX 唤醒 ===== */
    ev.events  = EPOLLIN;
    ev.data.fd = event_fd_;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev);

    epoll_event events[4];

    /* ===== Timeout 检查定时器 ===== */
    if (timer_fd_ >= 0) {
        ev.events  = EPOLLIN;
        ev.data.fd = timer_fd_;
        epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, timer_fd_, &ev);
    }

    while (running_) {
        int n = epoll_wait(epoll_fd_, events, 4, -1);
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            if (events[i].data.fd == event_fd_) {
                uint64_t v;
                while (::read(event_fd_, &v, sizeof(v)) > 0) {}
                handleWrite();
            }
            else if (events[i].data.fd == serial_fd) {
                handleRead();
            }
            else if (events[i].data.fd == timer_fd_) {
                handleTimer();
            }

        }
    }

    LOGINFO("[SERIAL] thread exit name=%s", port_name_.c_str());
}
#include <csignal>
void SerialThread::handleRead() {
    uint8_t buf[256];

    ssize_t n = driver_.read(buf, sizeof(buf));
    if (n <= 0) {
        if (n < 0 && errno != EAGAIN && errno != EINTR) {
            LOGINFO("[SERIAL][RXERR] read error errno=%d", errno);
        } else {
            // LOGINFO("[SERIAL][RXDBG] port=%s idx=%d read=0 (EOF?)",
            //         port_name_.c_str(), serial_index_);
        }
        return;
    }
    // LOG_COMM_D("[SERIAL][RXDBG] port=%s idx=%d n=%zd first=%02X",
    //         port_name_.c_str(), serial_index_, n, buf[0]);
    last_rx_ts_ms_ = nowMs(); //timeout
    // LOGD(hexDump(buf, n));

    // ===== 关键修改点：直接走 RX callback =====
    if (rx_cb_) {
        std::vector<uint8_t> data(buf, buf + n);
        // LOGDEBUG("[SERIAL][RX] len=%zu",
        //        data.size());
        rx_cb_(serial_index_, data);
    } else {
        LOGINFO("[SERIAL][RX] drop %zd bytes (no rx_cb)", n);
    }
}

void SerialThread::handleWrite() {
    while (true) {
        std::vector<uint8_t> data;
        {
            std::lock_guard<std::mutex> lk(tx_mtx_);
            if (tx_queue_.empty()) break;
            data = std::move(tx_queue_.front());
            tx_queue_.pop_front();
        }

        last_tx_ts_ms_ = nowMs(); // timeout
        ssize_t n = driver_.write(data.data(), data.size());
        if (n < 0 && errno != EAGAIN && errno != EINTR) {
            LOG_COMM_D("[SERIAL][TXERR] write error errno=%d", errno);
            break;
        }
    }
}

void SerialThread::setupTimerFd() {
    timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (timer_fd_ < 0) {
        LOGERR("[SERIAL][TIMER] timerfd_create failed errno=%d (%s)",
               errno, strerror(errno));
        return;
    }

    itimerspec its{};
    // 首次触发
    its.it_value.tv_sec  = timeout_check_period_ms_ / 1000;
    its.it_value.tv_nsec = (timeout_check_period_ms_ % 1000) * 1000000;
    // 周期触发
    its.it_interval = its.it_value;

    if (timerfd_settime(timer_fd_, 0, &its, nullptr) < 0) {
        LOGERR("[SERIAL][TIMER] timerfd_settime failed errno=%d (%s)",
               errno, strerror(errno));
        ::close(timer_fd_);
        timer_fd_ = -1;
    }
}

void SerialThread::handleTimer() {
    uint64_t expirations = 0;
    // 必须读掉，否则会一直触发
    while (::read(timer_fd_, &expirations, sizeof(expirations)) > 0) {}

    checkRxTimeout();
}
// L1 串口链路诊断：用于发现“发出了但底层没收到”
// 不直接参与上层设备 online/offline 健康判定
void SerialThread::checkRxTimeout() {
    if (rx_timeout_ms_ == 0) return; // 允许关闭超时检测

    uint64_t now = nowMs();
    uint64_t last_rx = last_rx_ts_ms_.load();
    uint64_t last_tx = last_tx_ts_ms_.load();

    // 规则：只要在 timeout 窗口内有 RX，就认为没超时
     bool timeout =
    // (now >= last_rx) && (now - last_rx >= rx_timeout_ms_); // 超时条件1, 最近一次 RX 超时
     (last_tx > last_rx) && (now - last_tx >= rx_timeout_ms_); // 超时条件2, 最近一次 TX 超时

    if (timeout) {
        // 防抖：只在第一次进入超时态时上报一次
        bool expected = false;
        if (rx_timeout_active_.compare_exchange_strong(expected, true)) {
            // LOGWARN("[SERIAL][L1TIMEOUT] port=%s idx=%d no-rx for %llums (timeout=%llums) last_rx=%llums last_tx=%llums",
            //         port_name_.c_str(), serial_index_,
            //         (unsigned long long)(now - last_rx),
            //         (unsigned long long)rx_timeout_ms_,
            //         (unsigned long long)last_rx,
            //         (unsigned long long)last_tx);

            if (link_timeout_cb_) {
                LinkTimeoutInfo info{
                    serial_index_, port_name_, last_rx, last_tx, rx_timeout_ms_
                };
                link_timeout_cb_(info);
            }
        }
    } else {
        // 如果曾经超时，且现在恢复了 RX，则上报恢复事件一次
        bool expected = true;
        if (rx_timeout_active_.compare_exchange_strong(expected, false)) {
            LOGINFO("[SERIAL][L1RECOVER] port=%s idx=%d rx recovered",
                    port_name_.c_str(), serial_index_);

            if (link_recover_cb_) {
                LinkTimeoutInfo info{
                    serial_index_, port_name_, last_rx, last_tx, rx_timeout_ms_
                };
                link_recover_cb_(info);
            }
        }
    }
}

