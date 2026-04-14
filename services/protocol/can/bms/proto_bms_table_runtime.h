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
     * 1. 协议消息查找（支持 canhub 改写后的 4 路实例 ID）
     * 2. 信号查找
     * 3. raw -> phys
     * 4. 枚举文本查找
     */
    class ProtoBmsTableRuntime {
    public:
        // 允许传入“运行态 ID”（改写后），内部会尽量映射到 generated 的协议模板消息
        static const proto_bms_gen::MessageDef* findMessage(uint32_t runtime_id29);

        static const proto_bms_gen::SignalDef* findSignal(const proto_bms_gen::MessageDef* msg,
                                                          const char* sig_name);

        static bool enumText(const proto_bms_gen::SignalDef* sig,
                             uint32_t raw,
                             const char*& out_text);

        static double rawToPhys(const proto_bms_gen::SignalDef* sig, uint64_t raw_u64);

        // 一些字段其实是枚举/状态位，更适合放 value/status
        static bool isBoolLikeSignal(const proto_bms_gen::SignalDef* sig);
        static bool isEnumLikeSignal(const proto_bms_gen::SignalDef* sig);

        // 由运行态 ID 反推协议模板 ID
        static uint32_t normalizeToProtoId(uint32_t runtime_id29);

        static std::string canonicalMsgName(const proto_bms_gen::MessageDef* msg);

        static bool shouldParseAsPhysical(const proto_bms_gen::SignalDef* sig);
    private:
        static uint64_t signExtend(uint64_t raw_u64, uint32_t bit_len);
    };

} // namespace proto::bms

#endif //ENERGYSTORAGE_PROTO_BMS_TABLE_RUNTIME_H
