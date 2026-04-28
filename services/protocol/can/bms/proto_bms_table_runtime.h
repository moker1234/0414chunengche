//
// Created by lxy on 2026/2/13.
//

#ifndef ENERGYSTORAGE_PROTO_BMS_TABLE_RUNTIME_H
#define ENERGYSTORAGE_PROTO_BMS_TABLE_RUNTIME_H

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "generated/proto_bms_table.h"

namespace proto::bms {

    /**
     * 运行时帮助函数：
     * 1. 协议消息查找，支持 canhub 改写后的 4 路实例 ID
     * 2. 信号查找
     * 3. raw -> phys
     * 4. 枚举文本查找
     */
    class ProtoBmsTableRuntime {
    public:
        // 允许传入：
        // - generated 表中的消息 ID
        // - 原始协议模板 ID
        // - canhub 运行态 ID
        static const proto_bms_gen::MessageDef* findMessage(uint32_t runtime_id29);

        static const proto_bms_gen::SignalDef* findSignal(const proto_bms_gen::MessageDef* msg,
                                                          const char* sig_name);

        static bool enumText(const proto_bms_gen::SignalDef* sig,
                             uint32_t raw,
                             const char*& out_text);

        static double rawToPhys(const proto_bms_gen::SignalDef* sig,
                                uint64_t raw_u64);

        static bool isBoolLikeSignal(const proto_bms_gen::SignalDef* sig);
        static bool isEnumLikeSignal(const proto_bms_gen::SignalDef* sig);

        // 由任意已知 ID 反推 generated 表可查找的消息 ID。
        // 实际映射集中在 BmsFrameRouter。
        static uint32_t normalizeToProtoId(uint32_t runtime_id29);

        static std::string canonicalMsgName(const proto_bms_gen::MessageDef* msg);

        static bool shouldParseAsPhysical(const proto_bms_gen::SignalDef* sig);

    private:
        static uint64_t signExtend(uint64_t raw_u64, uint32_t bit_len);
    };

} // namespace proto::bms

#endif // ENERGYSTORAGE_PROTO_BMS_TABLE_RUNTIME_H