//
// Created by forlinx on 2025/12/28.
//

#ifndef ENERGYSTORAGE_CAN_DISPATCHER_H
#define ENERGYSTORAGE_CAN_DISPATCHER_H

#pragma once

#include <memory>
#include <unordered_map>
#include <linux/can.h>

#include "can_protocol_base.h"

// SystemConfig 在 config_loader.h 里定义（你工程已有）
#include "config_loader.h"

#include "bms/bms_proto.h"

namespace proto {
    namespace bms { class BmsQueue; }  // forward

    enum class CanParseResult {
        Parsed,      // 正常同步解析出 DeviceData（非 BMS）
        Consumed,    // 已消费（例如 BMS 入队异步处理），不应当当作 parse fail
        NoProto,     // 没有绑定协议
        ParseFail    // 有协议但解析失败
    };

    class CanDispatcher {
    public:
        CanDispatcher() = default;

        // 从 system.json 的 SystemConfig 初始化：绑定 can_index -> 协议实例
        void init(const SystemConfig& sys);

        // 解析一帧（按 can_index 选择协议）
        bool parse(int can_index, const can_frame& fr, DeviceData& out);


        // 线程安全接口
        // 接口：返回解析结果（Parsed/Consumed/NoProto/ParseFail）
        CanParseResult handle(int can_index, const can_frame& fr, DeviceData& out);
        //  配置 BMS 异步：指定哪个 can_index 走 BMS 入队
        void setBmsAsync(int bms_can_index, bms::BmsQueue* q);

    private:
        std::unordered_map<int, std::unique_ptr<CanProtocolBase>> proto_by_can_;

        // BMS async
        int bms_can_index_{-1};
        bms::BmsQueue* bms_q_{nullptr};
    };

} // namespace proto

#endif // ENERGYSTORAGE_CAN_DISPATCHER_H
