//
// Created by lxy on 2026/2/14.
//

#ifndef ENERGYSTORAGE_BMS_QUEUE_H
#define ENERGYSTORAGE_BMS_QUEUE_H


#pragma once

#include <cstdint>
#include <array>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <condition_variable>

namespace proto::bms {

    /**
     * 一帧 BMS CAN 数据（轻量结构）
     * - 只存 29bit id + 8 bytes data + dlc + can_index + ts_ms
     * - 不做解析，不分配大对象，适合在 CAN RX 线程里快速 push
     */
    struct BmsFrame {
        int      can_index{0};
        uint32_t id29{0};        // 29-bit (already masked, no CAN_EFF_FLAG)
        uint8_t  dlc{8};
        std::array<uint8_t, 8> data{};
        uint64_t ts_ms{0};       // monotonic ms

        // 作为 latest-wins 合并 key（默认用 id29；如你还想区分 can_index，可自行组合）
        uint64_t key() const { return static_cast<uint64_t>(id29); }
    };

    /**
     * BMS 输入队列：多生产者（多个 CAN 线程）-> 单消费者（BmsWorker）
     *
     * 目标：
     * - 报文洪流下不堆积无限队列
     * - latest-wins：同一个 key 只保留最新一帧
     * - popAll 一次取走所有最新帧（批处理）
     *
     * 说明：
     * - key 默认为 id29（建议 canhub 方案把“设备标识码”编码进 id，从而不同设备不会冲突）
     * - 若你希望 (can_index,id29) 共同区分，改 key() 即可
     */
    class BmsQueue {
    public:
        BmsQueue() = default;

        // 禁止拷贝
        BmsQueue(const BmsQueue&) = delete;
        BmsQueue& operator=(const BmsQueue&) = delete;

        /** push：生产者调用（CAN RX 线程） */
        void push(const BmsFrame& f);

        /** popAll：消费者调用，一次取走所有最新帧（并清空缓存） */
        std::vector<BmsFrame> popAll();

        /** 等待直到队列非空或超时（ms）；返回 true=有数据 */
        bool waitForData(uint32_t timeout_ms);

        /** 当前缓存的 unique key 数量（不是总帧数） */
        size_t size() const;

        /** 清空 */
        void clear();

    private:
        mutable std::mutex mtx_;
        std::condition_variable cv_;

        // latest-wins：同 key 只保留最新帧
        std::unordered_map<uint64_t, BmsFrame> latest_;
    };

} // namespace proto::bms


#endif //ENERGYSTORAGE_BMS_QUEUE_H