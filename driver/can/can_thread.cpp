//
// Created by forlinx on 2025/12/17.
//

#include "can_thread.h"

#include <cerrno>
#include <cstring>

#include <pthread.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <linux/can/error.h>

// #define DEBUG_CAN

namespace
{
    static constexpr int kMaxEvents = 8;
    static constexpr std::chrono::milliseconds kReconnectMin{200};
    static constexpr std::chrono::milliseconds kReconnectMax{5000};
}

CanThread::CanThread(CanDriver& driver,
                     AppManager& app,
                     int can_index,
                     const std::string& name)
    : can_index_(can_index),
      driver_(driver),
      app_(app),
      last_can_fd_(driver.getFd())
{
    (void)name;
}

CanThread::~CanThread()
{
    stop();
}

void CanThread::start()
{
#ifdef DEBUG_CAN
    printf("[CAN][THREAD] start()\n");
#endif
    if (running_.exchange(true)) return;

    // 重连时间点初始化为“立刻可尝试”
    next_reconnect_tp_ = std::chrono::steady_clock::now();

    thread_ = std::thread(&CanThread::threadFunc, this);
}

void CanThread::stop()
{
    bool was_running = running_.exchange(false);
    (void)was_running;

    wakeup();

    if (thread_.joinable())
    {
        // 确保线程已启动
        thread_.join(); // 等待线程退出
    }

    if (epoll_fd_ >= 0)
    {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }
    if (event_fd_ >= 0)
    {
        ::close(event_fd_);
        event_fd_ = -1;
    }
}

void CanThread::send(const can_frame& frame_tx)
{
    // std::printf("[CAN#%d][%s][send] enqueue fd=%d id=0x%08X\n",
    //     can_index_, driver_.ifname().c_str(), driver_.getFd(), (uint32_t)frame_tx.can_id);
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(frame_tx);
    }

#ifdef DEBUG_CAN
    printf("[CAN][SEND] queued id=0x%X dlc=%u\n", frame_tx.can_id, frame_tx.can_dlc);
#endif

    // 若当前处于断线重连中，队列会保留，重连成功后发出
    setEpollOut(true);
    wakeup();
}

bool CanThread::hasPendingTx()
{
    std::lock_guard<std::mutex> lock(tx_mutex_);
    return !tx_queue_.empty();
}

void CanThread::wakeup()
{
    if (event_fd_ < 0) return;
    uint64_t v = 1;
    (void)::write(event_fd_, &v, sizeof(v)); // 唤醒 epoll_wait
}

/*
 * @brief 设置 CAN fd 的 EPOLLOUT 事件, EPOLLOUT: 可写事件
 *
 * @param enable 是否启用 EPOLLOUT 事件
 */
void CanThread::setEpollOut(bool enable)
{
    if (epoll_fd_ < 0) return;

    int canfd = driver_.getFd();
    if (canfd < 0) return;

    if (enable == epollout_enabled_) return;

    epoll_event ev{};
    ev.data.fd = canfd;
    ev.events = EPOLLIN;
    if (enable) ev.events |= EPOLLOUT;

    // canfd 可能已变更（重连后），优先尝试 MOD，不存在则 ADD
    if (::epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, canfd, &ev) < 0)
    {
        if (errno == ENOENT)
        {
            if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, canfd, &ev) < 0)
            {
#ifdef DEBUG_CAN
                printf("[CAN][ERROR] epoll_ctl ADD(canfd) failed: %s\n", strerror(errno));
#endif
                return;
            }
        }
        else
        {
#ifdef DEBUG_CAN
            printf("[CAN][ERROR] epoll_ctl MOD(canfd) failed: %s\n", strerror(errno));
#endif
            return;
        }
    }

    epollout_enabled_ = enable;
}

/*
 * @brief 设置 CAN fd 的 EPOLLIN 事件, EPOLLOUT(if hasPendingTx) 事件; EPOLLIN: 可读事件, EPOLLOUT: 可写事件
 *
 * @param canfd CAN fd
 * @return true 成功
 * @return false 失败
 */
bool CanThread::setupEpollForCanFd(int canfd)
{
    if (epoll_fd_ < 0 || canfd < 0) return false;

    epoll_event ev{};
    ev.data.fd = canfd;
    ev.events = EPOLLIN;
    if (hasPendingTx()) ev.events |= EPOLLOUT;

    // 先尝试 ADD；若已存在则 MOD
    if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, canfd, &ev) < 0)
    {
        if (errno != EEXIST)
        {
            printf("[CAN#%d][%s][ERROR] epoll_ctl add failed: %s\n",
                   can_index_, driver_.ifname().c_str(), strerror(errno));
            return false;
        }
        if (::epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, canfd, &ev) < 0)
        {
            printf("[CAN#%d][%s][ERROR] epoll_ctl mod failed: %s\n",
                   can_index_, driver_.ifname().c_str(), strerror(errno));
            return false;
        }
    }

    // 同步 EPOLLOUT 状态
    epollout_enabled_ = hasPendingTx();
    return true;
}

/**
 * @brief 从 epoll 中移除旧的 CAN fd
 *
 * @param oldfd 旧的 CAN fd
 */
void CanThread::removeOldCanFdFromEpoll(int oldfd)
{
    if (epoll_fd_ < 0) return;
    if (oldfd < 0) return;

    // 旧 fd 可能已关闭，DEL 失败也无所谓
    (void)::epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, oldfd, nullptr);
}

/**
 * @brief 标记需要重连
 *
 * @param reason 重连原因
 * @param err 错误码
 */
void CanThread::markNeedReconnect(const char* reason, int err)
{
    // 只要标记一次即可（避免刷屏）
    bool expected = false;
    if (need_reconnect_.compare_exchange_strong(expected, true))
    {
        //compare_exchange_strong: 比较并交换，若当前值为 expected, 则交换为 true, 否则不交换
        printf("[CAN#%d][%s][RECONN] need reconnect (%s) err=%d(%s)\n",
               can_index_, driver_.ifname().c_str(),
               reason ? reason : "unknown",
               err, strerror(err));

        // 立刻允许尝试（由退避控制）
        next_reconnect_tp_ = std::chrono::steady_clock::now();
        reconnect_delay_ = kReconnectMin;
    }
}

void CanThread::tryReconnect()
{
    if (!need_reconnect_.load()) return; // load(): 获取当前值，若当前值为 false, 则不执行重连

    auto now = std::chrono::steady_clock::now();
    if (now < next_reconnect_tp_) return;

    // 这里在 CAN 线程内执行：符合“fd 单线程”原则
    int oldfd = driver_.getFd();
    removeOldCanFdFromEpoll(oldfd);

    printf("[CAN#%d][%s][RECONN] reopen...\n", can_index_, driver_.ifname().c_str());
    bool ok = driver_.reopen();
    int newfd = driver_.getFd();

    if (!ok || newfd < 0)
    {
        printf("[CAN#%d][%s] reopen failed, next in %lld ms\n",
               can_index_, driver_.ifname().c_str(),(long long)reconnect_delay_.count());

        // 指数退避
        next_reconnect_tp_ = now + reconnect_delay_;
        reconnect_delay_ = std::min(reconnect_delay_ * 2, kReconnectMax);

        // 唤醒一下（也可以不唤醒，epoll 有 eventfd 会被 stop/send 唤醒）
        // 这里用定时唤醒机制，所以我们用 eventfd 自己唤醒一次进入下一轮逻辑
        wakeup();
        return;
    }

    // 如果驱动程序重新连接成功, 则发送 CanUp 事件
    if (driver_.reopen())
    {
        need_reconnect_ = false;

        Event ev{};
        ev.type = Event::Type::CanUp;
        ev.text = "CAN recovered";
        app_.post(ev);
    }

    // 新 fd 加入 epoll
    if (!setupEpollForCanFd(newfd))
    {
        printf("[CAN][RECONN] epoll setup for new fd failed\n");
        // 失败则继续退避重试（很少见）
        markNeedReconnect("epoll setup failed", errno);
        next_reconnect_tp_ = now + reconnect_delay_;
        reconnect_delay_ = std::min(reconnect_delay_ * 2, kReconnectMax);
        wakeup();
        return;
    }

    last_can_fd_ = newfd;
    need_reconnect_ = false;
    reconnect_delay_ = kReconnectMin;

    printf("[CAN][RECONN] reopen success, new fd=%d\n", newfd);

    // 重连成功后，若队列有待发送，打开 EPOLLOUT 并唤醒处理
    if (hasPendingTx())
    {
        setEpollOut(true);
        wakeup();
    }
}

void CanThread::threadFunc()
{
    pthread_setname_np(pthread_self(), thread_name_.c_str());

#ifdef DEBUG_CAN
    printf("[CAN][THREAD] threadFunc enter\n");
#endif

    epoll_fd_ = ::epoll_create1(EPOLL_CLOEXEC);
    // its same as epoll_create() while fill with 0, but EPOLL_CLOEXEC is more safe when the thread is fork()ed
    if (epoll_fd_ < 0)
    {
        printf("[CAN#%d][%s][ERROR] epoll create failed\n", can_index_, driver_.ifname().c_str());
       running_ = false;
        return;
    }

    event_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC); //EFD_NONBLOCK: 非阻塞模式, EFD_CLOEXEC: 关闭时自动关闭文件描述符
    if (event_fd_ < 0)
    {
        printf("[CAN#%d][%s][ERROR] eventfd create failed\n", can_index_, driver_.ifname().c_str());
       running_ = false;
        return;
    }

    // 监听 eventfd（stop/send/以及内部定时唤醒）
    {
        epoll_event ev{};
        ev.data.fd = event_fd_;
        ev.events = EPOLLIN;
        if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev) < 0)
        {
            printf("[CAN#%d][%s][ERROR] epoll_ctl add eventfd failed: %s\n",
                   can_index_, driver_.ifname().c_str(), strerror(errno));
            running_ = false;
            return;
        }
    }

    // 初次把 canfd 加入 epoll
    last_can_fd_ = driver_.getFd();
    if (last_can_fd_ >= 0) {
        (void)setupEpollForCanFd(last_can_fd_);
    }
    printf("[CAN#%d][%s] thread started, fd=%d\n", can_index_, driver_.ifname().c_str(), last_can_fd_);

    epoll_event epoll_evts[kMaxEvents]; // 用于存储 epoll 事件

    while (running_)
    {
        // 如果需要重连，这里用“最小 sleep 间隔”方式避免纯忙等：
        // - 把 epoll_wait 设置成超时（例如 200ms），配合退避 next_reconnect_tp_ 进行重连
        int timeout_ms = 200;
        if (need_reconnect_.load())
        {
            // 如果需要重连
            // 下一次重连时间点越近，timeout 越短；最短 50ms; 最长 500ms
            auto now = std::chrono::steady_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(next_reconnect_tp_ - now);
            timeout_ms = (int)std::max<long long>(50, diff.count());
            timeout_ms = std::min(timeout_ms, 500);
        }
        else
        {
            timeout_ms = -1; // 正常情况下阻塞等待事件
        }

        // 等待 epoll 事件
        int n = ::epoll_wait(epoll_fd_, epoll_evts, kMaxEvents, timeout_ms);

        // 等待 epoll 事件失败
        if (n < 0)
        {
            if (errno == EINTR) continue;
            printf("[CAN#%d][%s][ERROR] epoll_wait: %s\n",
                   can_index_, driver_.ifname().c_str(), strerror(errno));
            continue;
        }
        // 超时返回 n==0：用于重连 tick
        if (n == 0)
        {
            tryReconnect();
            continue;
        }

#ifdef DEBUG_CAN
        printf("[CAN][THREAD] epoll_wait n=%d\n", n);
#endif

        // 处理 epoll 事件
        for (int i = 0; i < n; ++i)
        {
            int fd = epoll_evts[i].data.fd;
            uint32_t ev = epoll_evts[i].events;

            // 处理 event fd 事件
            if (fd == event_fd_)
            {
                // 处理用于唤醒 epoll_wait 的 eventfd
                // 清空 eventfd
                uint64_t v;
                while (::read(event_fd_, &v, sizeof(v)) > 0)
                {
                }

                if (!running_) break; // 线程退出

                // 先尝试重连（如果需要）
                tryReconnect();

                // 重连后如果有待发送，处理一次
                if (!need_reconnect_.load() && hasPendingTx())
                {
                    handleWrite();
                }
                continue;
            }

            // 处理 CAN fd 事件
            if (fd == driver_.getFd())
            {
                // CAN fd 可能会变（重连后），所以直接用 fd==driver_.getFd() 判定
                // 错误/挂起：强制进入重连
                if (ev & (EPOLLERR | EPOLLHUP))
                {
                    markNeedReconnect("EPOLLERR/EPOLLHUP", EIO);
                    continue;
                }

                if (ev & EPOLLIN)
                {
                    handleRead();
                }
                if (ev & EPOLLOUT)
                {
                    handleWrite();
                }
            }
        }

        // 循环尾部：如果在 read/write 中标记了重连需求，尝试一次
        tryReconnect();
    }

#ifdef DEBUG_CAN
    printf("[CAN][THREAD] threadFunc exit\n");
#endif
}

void CanThread::handleRead()
{
    can_frame fr{};
    while (driver_.recv(fr))
    {
        // printf("[CAN#%d][%s][READ] id=0x%X dlc=%u data=",
        //        can_index_, driver_.ifname().c_str(),
        //        fr.can_id, fr.can_dlc);
        // for (int i = 0; i < fr.can_dlc; ++i) printf("%02X ", fr.data[i]);
        // printf("\n");
        // 读到一帧：上报给 AppManager（通过回调）
        if (rx_cb_)
        {
            rx_cb_(can_index_, fr);
        }
    }
}

void CanThread::handleWrite()
{
    if (need_reconnect_.load()) return;

#ifdef DEBUG_CAN
    printf("[CAN][WRITE] handleWrite enter\n");
#endif

    while (running_)
    {
        can_frame frame{};
        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            if (tx_queue_.empty()) break;
            frame = tx_queue_.front();
        }

        if (!driver_.send(frame))
        {
            // 如果只是 EAGAIN/EINTR，等待下一次 EPOLLOUT；如果是致命错误，触发重连
            if (CanDriver::isFatalErrno(errno))
            {
                markNeedReconnect("send fatal", errno);
            }
#ifdef DEBUG_CAN
            printf("[CAN][WRITE] send not ok: errno=%d %s\n", errno, strerror(errno));
#endif
            break;
        }

        {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            if (!tx_queue_.empty()) tx_queue_.pop();
        }

#ifdef DEBUG_CAN
        printf("[CAN][TX] sent id=0x%X dlc=%u data=", frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; ++i) printf("%02X ", frame.data[i]);
        printf("\n");
#endif
    }

    // 队列空则关闭 EPOLLOUT，避免空转
    if (hasPendingTx())
    {
        setEpollOut(true);
    }
    else
    {
        setEpollOut(false);
    }

#ifdef DEBUG_CAN
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        printf("[CAN][WRITE] exit, queue size=%zu\n", tx_queue_.size());
    }
#endif
}
