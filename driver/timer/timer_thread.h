//
// Created by forlinx on 2025/12/26.
//

#ifndef ENERGYSTORAGE_TIMER_H
#define ENERGYSTORAGE_TIMER_H


#pragma once

#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <mutex>

#include "../../app/app_manager.h"
#include "event.h"
class AppManager;

class TimerThread {
public:
    struct TimerItem {
        uint32_t interval_ms;
        uint64_t next_tick;
        Event::Type event;
    };

    TimerThread(AppManager& app,
                const std::string& name = "TimerThread");
    ~TimerThread();

    void start();
    void stop();

    void addTimer(uint32_t interval_ms, Event::Type ev);

private:
    void threadFunc();
    static uint64_t nowMs();

private:
    AppManager& app_;
    std::string name_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex mutex_;
    std::vector<TimerItem> timers_;
};



#endif //ENERGYSTORAGE_TIMER_H