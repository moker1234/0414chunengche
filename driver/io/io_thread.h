//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_IO_THREAD_H
#define ENERGYSTORAGE_IO_THREAD_H

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "../gpio/di.h"
#include "../adc/ai.h"

class IoThread {
public:
    using SampleCallback = std::function<void(uint64_t ts_ms,
                                              uint64_t di_bits,
                                              const std::vector<double>& ai_voltages)>;

    struct Config {
        uint32_t di_interval_ms{20};   // DI 采样周期
        uint32_t ai_interval_ms{50};   // ADC 电压采样周期
        uint32_t loop_sleep_ms{5};     // 主循环 sleep
    };

public:
    IoThread(DigitalInputManager& di_mgr,
             AiDriver& ai_drv);
    ~IoThread();

    void setConfig(const Config& cfg);
    void setCallback(SampleCallback cb);

    void start();
    void stop();

private:
    void threadMain_();
    static uint64_t nowMs_();

private:
    DigitalInputManager& di_mgr_;
    AiDriver& ai_drv_;

    Config cfg_{};
    SampleCallback cb_;

    std::thread th_;
    std::atomic<bool> running_{false};

    mutable std::mutex mtx_;
    std::condition_variable cv_;

    uint64_t di_bits_{0};
    std::vector<double> ai_voltages_;
};

#endif // ENERGYSTORAGE_IO_THREAD_H