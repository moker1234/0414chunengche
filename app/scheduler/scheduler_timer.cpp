#include "scheduler_timer.h"

#include <exception>

SchedulerTimer::SchedulerTimer() = default;

SchedulerTimer::~SchedulerTimer() {
    stop();
}

void SchedulerTimer::start() {
    // 如果已经运行就忽略
    bool was_running = running_.exchange(true);
    if (was_running) {
        LOGWARN("[TIMER] start() ignored: already running");
        return;
    }

    // 如果 th_ 还 joinable，说明上次没 join，先 join 掉（用于排查）
    if (th_.joinable()) {
        LOGERR("[TIMER] start(): th_ is joinable before starting new thread! "
               "Previous thread was not joined. Joining now.");
        try { th_.join(); } catch (...) {}
    }

    th_ = std::thread(&SchedulerTimer::threadFunc, this);
    LOGINFO("[TIMER] start() ok tid=%zu",
            (size_t)std::hash<std::thread::id>{}(th_.get_id()));
}

void SchedulerTimer::stop() {
    // 关键：不要早退！ running_ 只是“退出信号”，joinable 才决定要不要 join
    running_.store(false);

    if (!th_.joinable()) return;

    // 防止在自身线程里 join 自己
    if (std::this_thread::get_id() == th_.get_id()) {
        LOGERR("[TIMER] stop() called from timer thread itself, skip join to avoid deadlock");
        return;
    }

    th_.join();
    LOGINFO("[TIMER] stopped (joined)");
}

void SchedulerTimer::addPeriodic(uint32_t interval_ms, Callback cb) {
    std::lock_guard<std::mutex> lk(mtx_);
    Item it;
    it.interval_ms = interval_ms;
    it.next_tick   = nowMs() + interval_ms;
    it.cb          = std::move(cb);
    items_.push_back(std::move(it));

    LOGD("[TIMER] addPeriodic interval=%u ms, items=%zu",
         interval_ms, items_.size());
}

uint64_t SchedulerTimer::nowMs() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void SchedulerTimer::threadFunc() {
    const auto tid_hash = (size_t)std::hash<std::thread::id>{}(std::this_thread::get_id());
    LOGINFO("[TIMER] SchedulerTimer thread start tid=%zu", tid_hash);

    try {
        while (running_.load()) {
            uint64_t now = nowMs();

            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (auto& it : items_) {
                    if (!running_.load()) break;

                    if (now >= it.next_tick) {
                        if (it.cb) {
                            try {
                                it.cb();
                            } catch (const std::exception& e) {
                                LOGERR("[TIMER] callback exception: %s", e.what());
                                std::terminate();
                            } catch (...) {
                                LOGERR("[TIMER] callback unknown exception");
                                std::terminate();
                            }
                        }
                        it.next_tick = now + it.interval_ms;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception& e) {
        LOGERR("[TIMER] thread exception: %s", e.what());
        std::terminate();
    } catch (...) {
        LOGERR("[TIMER] thread unknown exception");
        std::terminate();
    }

    LOGINFO("[TIMER] SchedulerTimer thread exit tid=%zu", tid_hash);
}
