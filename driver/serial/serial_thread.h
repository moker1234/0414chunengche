//
// Created by forlinx on 2025/12/20.
//

#ifndef ENERGYSTORAGE_SERIAL_THREAD_H
#define ENERGYSTORAGE_SERIAL_THREAD_H

#pragma once

#include <thread>
#include <chrono>
#include <atomic>
#include <deque>
#include <mutex>
#include <vector>
#include <string>
#include <functional>
#include <cstdint>

#include "serial_driver_base.h"
#include "serial_event.h"
#include "../utils/queue/msg_queue.h"
#include "app_manager.h"
#include "logger.h"

struct SerialEpollCtx {
    int serial_index;
    int fd;
};

class SerialThread {
public:
    SerialThread(int serial_index,
                 SerialDriverBase& drv,
                 const std::string& name,
                 MsgQueue<SerialEvent>& rxQueue);

    ~SerialThread();

    void start();
    void stop();

    void send(const std::vector<uint8_t>& data);

    const std::string& get_name() const { return port_name_; }


    using RxCallback = std::function<void(int serial_index, const std::vector<uint8_t>&)>;
    void setRxCallback(RxCallback cb) { rx_cb_ = std::move(cb); }

    void send(int serial_index, const std::vector<uint8_t>& bytes);

    //timeout
    struct LinkTimeoutInfo {
        int serial_index;
        std::string port_name;
        uint64_t last_rx_ts_ms;
        uint64_t last_tx_ts_ms;
        uint64_t timeout_ms;
    };

    using LinkTimeoutCallback = std::function<void(const LinkTimeoutInfo&)>;
    using LinkRecoverCallback = std::function<void(const LinkTimeoutInfo&)>;

    void setLinkTimeoutCallback(LinkTimeoutCallback cb) { link_timeout_cb_ = std::move(cb); }
    void setLinkRecoverCallback(LinkRecoverCallback cb) { link_recover_cb_ = std::move(cb); }

    // 配置：超时阈值、检查周期
    void setRxTimeout(uint64_t timeout_ms) { rx_timeout_ms_ = timeout_ms; }
    void setTimeoutCheckPeriod(uint64_t period_ms) { timeout_check_period_ms_ = period_ms; }
private:
    void threadFunc();
    void handleRead();
    void handleWrite();

    //timeout
    static uint64_t nowMs();
    void setupTimerFd();
    void handleTimer();       // timerfd 触发
    // L1 串口链路诊断：用于发现“发出了但底层没收到”
    // 不直接参与上层设备 online/offline 健康判定
    void checkRxTimeout();    // 实际检测逻辑

private:
    int serial_index_{-1};
    RxCallback rx_cb_;
    std::vector<SerialEpollCtx*> epoll_ctxs_;
    std::vector<int> serial_fds_;

    SerialDriverBase& driver_;
    std::string port_name_;

    std::thread thread_;
    std::atomic<bool> running_{false};

    int epoll_fd_{-1};
    int event_fd_{-1};

    std::deque<std::vector<uint8_t>> tx_queue_;
    std::mutex tx_mtx_;

    MsgQueue<SerialEvent>& rx_queue_;

    //timeout
    // ===== L1 超时检测（链路层）=====
    std::atomic<uint64_t> last_tx_ts_ms_{0};
    std::atomic<uint64_t> last_rx_ts_ms_{0};

    uint64_t rx_timeout_ms_{1500};            // 默认 1.5s（你可按设备调整）
    uint64_t timeout_check_period_ms_{200};   // 默认 200ms 检查一次

    std::atomic<bool> rx_timeout_active_{false};  // 防止重复上报

    int timer_fd_{-1};  // timerfd，用于周期唤醒检查超时

    LinkTimeoutCallback link_timeout_cb_;
    LinkRecoverCallback link_recover_cb_;
};



#endif //ENERGYSTORAGE_SERIAL_THREAD_H