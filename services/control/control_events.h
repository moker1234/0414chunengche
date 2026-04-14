// services/control/control_events.h
//
// 工业级控制：控制事件定义（外部线程 post -> ControlLoop 单线程消费）
// Created by ChatGPT on 2026/02/24.
//
#ifndef ENERGYSTORAGE_CONTROL_EVENTS_H
#define ENERGYSTORAGE_CONTROL_EVENTS_H

#pragma once

#include <cstdint>
#include <vector>
#include <string>

#include "../protocol/protocol_base.h"          // DeviceData
#include "../aggregator/system_snapshot.h"      // agg::SystemSnapshot  ✅ 新增：snapshot 事件 payload

namespace control {

    struct HmiWriteEvent {
        int rs485_index{0};
        uint8_t slave_addr{0};
        uint8_t func{0};
        uint16_t start_addr{0};
        std::vector<uint8_t> bits;
        std::vector<uint16_t> regs;
        uint64_t ts_ms{0};
    };

    struct IoSampleEvent {
        uint64_t ts_ms{0};
        uint64_t di_bits{0};
        std::vector<double> ai;
    };

    struct TickEvent {
        uint64_t ts_ms{0};
        uint32_t period_ms{0};
    };

    struct LinkHealthEvent {
        uint64_t ts_ms{0};
        std::string link_name;
        bool up{true};
    };

    struct SnapshotEvent {
        uint64_t ts_ms{0};
        agg::SystemSnapshot snap;
    };

    struct Event {
        enum class Type {
            DeviceData,       // 外设 RS485 / CAN 解析后的 DeviceData
            Snapshot,         // ✅ 新增：Aggregator 的 SystemSnapshot（用于渐进式迁移：snapshot -> logic -> HMI）
            HmiWrite,         // 屏幕写入
            IoSample,         // DI/AI 采样
            Tick,             // 控制定时
            LinkHealth,       // 链路 up/down（可选）
            Stop,             // 用于 stop() 注入，唤醒线程退出
        };

        Type type{Type::DeviceData};

        // 通用字段（可选填）
        uint64_t ts_ms{0};

        // payload（按 type 使用）
        DeviceData device_data;
        SnapshotEvent snapshot;      // ✅ 新增
        HmiWriteEvent hmi_write;
        IoSampleEvent io_sample;
        TickEvent tick;
        LinkHealthEvent link_health;

        static Event makeStop() {
            Event e;
            e.type = Type::Stop;
            return e;
        }
    };

} // namespace control

#endif // ENERGYSTORAGE_CONTROL_EVENTS_H