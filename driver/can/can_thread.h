//
// Created by forlinx on 2025/12/18.
//

#ifndef ENERGYSTORAGE_CAN_THREAD_H
#define ENERGYSTORAGE_CAN_THREAD_H
#pragma once

#include "can_driver.h"
#include "event.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <string>
#include <functional>

#include <thread>
#include "app_manager.h"

class AppManager;

class CanThread {
public:
    explicit CanThread(CanDriver& driver,
                       AppManager& app,
                       int can_index,
                       const std::string& name = "CAN_Thread");
    ~CanThread();

    void start();
    void stop();

    void send(const can_frame& frame_tx);

    int canIndex() const { return can_index_; }
    const std::string& get_name() const { return thread_name_; }

    using RxCallback = std::function<void(int can_index, const can_frame&)>;
    void setRxCallback(RxCallback cb) { rx_cb_ = std::move(cb); }
private:
    void threadFunc();

    void handleRead();
    void handleWrite();

    // 断线恢复相关
    void markNeedReconnect(const char* reason, int err);
    void tryReconnect(); // 带退避策略
    bool setupEpollForCanFd(int canfd); // ADD or MOD
    void removeOldCanFdFromEpoll(int oldfd);

    // epoll/队列辅助
    bool hasPendingTx();
    void setEpollOut(bool enable);
    void wakeup();


private:
    RxCallback rx_cb_;
    int can_index_{0}; // 你可以在构造里传入

    CanDriver& driver_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    AppManager& app_;

    int epoll_fd_{-1};
    int event_fd_{-1}; // 用于唤醒 epoll_wait 的 eventfd
    bool epollout_enabled_{false};
    std::string thread_name_;

    std::queue<can_frame> tx_queue_;
    std::mutex tx_mutex_;

    // 重连状态
    std::atomic<bool> need_reconnect_{false};
    int last_can_fd_{-1};

    // 重连退避策略
    std::chrono::milliseconds reconnect_delay_{200}; // 重连退避时间
    std::chrono::steady_clock::time_point next_reconnect_tp_{}; // 下一次重连时间点
};

#endif // ENERGYSTORAGE_CAN_THREAD_H
