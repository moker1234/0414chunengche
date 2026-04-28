//
// Created by lxy on 2026/4/22.
//

#include "io_thread.h"

#include <chrono>

#include "getTime.h"

IoThread::IoThread(DigitalInputManager& di_mgr,
                   AiDriver& ai_drv)
    : di_mgr_(di_mgr),
      ai_drv_(ai_drv)
{
}

IoThread::~IoThread()
{
    stop();
}

void IoThread::setConfig(const Config& cfg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cfg_ = cfg;
}

void IoThread::setCallback(SampleCallback cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cb_ = std::move(cb);
}

void IoThread::start()
{
    if (running_.exchange(true)) {
        return;
    }

    th_ = std::thread(&IoThread::threadMain_, this);
}

void IoThread::stop()
{
    const bool was_running = running_.exchange(false);
    (void)was_running;

    cv_.notify_all();

    if (th_.joinable()) {
        th_.join();
    }
}

uint64_t IoThread::nowMs_()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()
    );
}

void IoThread::threadMain_()
{
    uint64_t last_di_ms = 0;
    uint64_t last_ai_ms = 0;

    while (running_.load()) {
        Config cfg_local;
        SampleCallback cb_local;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            cfg_local = cfg_;
            cb_local = cb_;
        }

        const uint64_t now = nowMs();
        bool updated = false;

        if (last_di_ms == 0 || (now - last_di_ms) >= cfg_local.di_interval_ms) {
            last_di_ms = now;

            uint64_t bits = 0;
            (void)di_mgr_.sampleBits(bits);

            {
                std::lock_guard<std::mutex> lk(mtx_);
                di_bits_ = bits;
            }

            updated = true;
        }

        if (last_ai_ms == 0 || (now - last_ai_ms) >= cfg_local.ai_interval_ms) {
            last_ai_ms = now;

            std::vector<double> volts;
            (void)ai_drv_.readAllVoltages(volts);

            {
                std::lock_guard<std::mutex> lk(mtx_);
                ai_voltages_ = std::move(volts);
            }

            updated = true;
        }

        if (updated && cb_local) {
            uint64_t bits = 0;
            std::vector<double> volts;

            {
                std::lock_guard<std::mutex> lk(mtx_);
                bits = di_bits_;
                volts = ai_voltages_;
            }

            cb_local(now, bits, volts);
        }

        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait_for(
            lk,
            std::chrono::milliseconds(cfg_local.loop_sleep_ms),
            [this] { return !running_.load(); }
        );
    }
}