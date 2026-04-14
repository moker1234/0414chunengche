//
// Created by forlinx on 2025/12/26.
//

#include "timer_thread.h"
#include <iostream>

TimerThread::TimerThread(AppManager& app, const std::string& name)
    : app_(app), name_(name) {}

TimerThread::~TimerThread() {
    stop();
}

void TimerThread::start() {
    if (running_) return;

    running_ = true;
    thread_ = std::thread(&TimerThread::threadFunc, this);
}

void TimerThread::stop() {
    if (!running_) return;

    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

/*
 * @brief 添加一个定时器
 *
 * @param interval_ms 定时器间隔，单位毫秒
 * @param ev 定时器到期后要触发的事件类型
 */
void TimerThread::addTimer(uint32_t interval_ms, Event::Type ev) {
    std::lock_guard<std::mutex> lock(mutex_);
    timers_.push_back(TimerItem{
        interval_ms,
        nowMs() + interval_ms,
        ev
    });
}



void TimerThread::threadFunc() {
    while (running_) {
        uint64_t now = nowMs();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& t : timers_) {
                if (now >= t.next_tick) {
                    app_.post(Event{t.event}); // 触发事件, 通知状态机
                    t.next_tick = now + t.interval_ms;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}




uint64_t TimerThread::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch()
           ).count();
}
