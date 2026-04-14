//
// Created by lxy on 2026/1/11.
//
/*
 * 定时器：用于驱动设备的定时任务（如 100ms 轮询）
 */
#ifndef ENERGYSTORAGE_SCHEDULER_TIMER_H
#define ENERGYSTORAGE_SCHEDULER_TIMER_H


#pragma once
#include "logger.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

class SchedulerTimer {
public:
    using Callback = std::function<void()>;

    SchedulerTimer();
    ~SchedulerTimer();

    SchedulerTimer(const SchedulerTimer&) = delete;
    SchedulerTimer& operator=(const SchedulerTimer&) = delete;

    void start();
    void stop();

    // 添加周期任务：interval_ms 到期就调用 cb
    void addPeriodic(uint32_t interval_ms, Callback cb);

    static uint64_t nowMs(); // 获取当前时间（ms）
private:
    struct Item {
        uint32_t interval_ms{0};    // 周期时间（ms）
        uint64_t next_tick{0};      // 下一次触发时间（ms）
        Callback cb;
    };


    void threadFunc(); // 定时器线程函数

private:
    std::thread th_; // 定时器线程
    std::atomic<bool> running_{false}; // 定时器运行标志

    std::mutex mtx_; // 定时器锁
    std::vector<Item> items_; // 定时器任务队列
};


#endif //ENERGYSTORAGE_SCHEDULER_TIMER_H