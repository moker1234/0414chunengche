//
// Created by lxy on 2026/2/14.
//

#ifndef ENERGYSTORAGE_BMS_WORKER_H
#define ENERGYSTORAGE_BMS_WORKER_H


#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "bms_queue.h"
#include "protocol_base.h"

// forward
struct can_frame;
// struct DeviceData;

namespace proto::bms {

    class BmsProto;

    /**
     * BMS 工作线程：
     * - 从 BmsQueue 批量取帧（latest-wins，防止洪流堆积）
     * - 调用 BmsProto::parse 转 DeviceData
     * - 通过回调把 DeviceData 交给上层（Aggregator 或其它处理）
     * - 可节流触发“BMS 快照分发/落盘”（可选回调）
     *
     * 设计目标：
     * - CAN RX 线程只做 push（轻量）
     * - 解析/聚合/写盘在专用线程，避免影响 HMI / RS485
     */
    class BmsWorker {
    public:
        struct Config {
            // 线程 loop 的等待超时（没数据就 sleep 等待），建议 5~20ms
            uint32_t wait_timeout_ms{10};

            // 每轮最大处理帧数（防止极端情况下单轮跑太久）；0 表示不限制
            uint32_t max_frames_per_round{0};

            // 是否在 worker 内做“回调节流”（例如分发 bms snapshot）；0 表示不节流
            uint32_t dispatch_throttle_ms{0};
        };

        using DeviceDataCallback = std::function<void(const DeviceData&)>;

        // 可选：worker 内触发一次“tick”回调（用于 dispatchBms/落盘）
        using TickCallback = std::function<void()>;

    public:
        BmsWorker(BmsQueue& q, BmsProto& proto, Config cfg);

        // 禁止拷贝
        BmsWorker(const BmsWorker&) = delete;
        BmsWorker& operator=(const BmsWorker&) = delete;

        ~BmsWorker();

        void setOnDeviceData(DeviceDataCallback cb) { on_device_data_ = std::move(cb); }
        void setOnTick(TickCallback cb) { on_tick_ = std::move(cb); }

        void start();
        void stop();
        bool isRunning() const { return running_.load(); }

    private:
        void threadMain_();

        static uint64_t nowMs_();

        // 将 BmsFrame 转成 SocketCAN 的 can_frame（扩展帧）
        static void fillCanFrame_(const BmsFrame& in, can_frame& out);

    private:
        BmsQueue& q_;
        BmsProto& proto_;
        Config cfg_;

        std::atomic<bool> running_{false};
        std::thread th_;

        DeviceDataCallback on_device_data_;
        TickCallback on_tick_;

        uint64_t last_tick_ms_{0};
    };

} // namespace proto::bms


#endif //ENERGYSTORAGE_BMS_WORKER_H