//
// Created by lxy on 2025/12/28.
//

#ifndef ENERGYSTORAGE_CAN_DISPATCHER_H
#define ENERGYSTORAGE_CAN_DISPATCHER_H

#pragma once

#include <memory>
#include <unordered_map>
#include <linux/can.h>

#include "can_protocol_base.h"
#include "config_loader.h"

namespace proto {

    namespace bms {
        class BmsQueue;
    }

    enum class CanParseResult {
        Parsed,      // 正常同步解析出 DeviceData（非 BMS）
        Consumed,    // 已消费（例如 BMS 入队异步处理），不应当当作 parse fail
        NoProto,     // 没有绑定协议
        ParseFail    // 有协议但解析失败
    };

    class CanDispatcher {
    public:
        CanDispatcher() = default;

        // 从 system.json 的 SystemConfig 初始化：绑定 can_index -> 协议实例。
        // 注意：
        // - PCU 等普通 CAN 协议仍在这里绑定同步协议对象；
        // - BMS 不再绑定同步 BmsProto，只记录为异步专线，实际队列由 setBmsAsync() 注入。
        void init(const SystemConfig& sys);

        // 旧同步接口：只给非 BMS CAN 协议保留。
        // BMS 不应再从这里解析。
        bool parse(int can_index, const can_frame& fr, DeviceData& out);

        // 推荐入口：返回 Parsed / Consumed / NoProto / ParseFail
        CanParseResult handle(int can_index, const can_frame& fr, DeviceData& out);

        // 配置 BMS 异步：指定哪个 can_index 走 BMS 入队
        void setBmsAsync(int bms_can_index, bms::BmsQueue* q);

    private:
        void stampParsedMeta_(int can_index,
                              const can_frame& fr,
                              DeviceData& out) const;

    private:
        std::unordered_map<int, std::unique_ptr<CanProtocolBase>> proto_by_can_;

        /*
         * PCU 实例映射：
         *   can_index -> pcu_instance
         *
         * pcu_instance:
         *   1 = PCU1 -> 程序内部 PCU_0
         *   2 = PCU2 -> 程序内部 PCU_1
         */
        std::unordered_map<int, uint8_t> pcu_instance_by_can_;

        // BMS async
        int bms_can_index_{-1};
        bms::BmsQueue* bms_q_{nullptr};
    };

} // namespace proto

#endif // ENERGYSTORAGE_CAN_DISPATCHER_H