//
// Created by lxy on 2026/4/12.
//

#ifndef ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H
#define ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H

#pragma once

#include <cstdint>
#include <string>

#include "bms_logic_types.h"

namespace control {
    struct LogicContext;
}

namespace control::bms {

/**
 * BmsFaultEvaluator
 *
 * 新规则：
 * 1. 只根据 B2V_Fult1_32960 / B2V_Fult2 的原始故障位/原始等级做 confirmed 投影
 * 2. 不再使用“故障触发条件 / 故障恢复条件”
 * 3. 不再做延时确认
 * 4. 报文位存在 -> confirmed=true；报文位不存在/报文组离线 -> confirmed=false
 */
class BmsFaultEvaluator {
public:
    BmsFaultEvaluator() = default;

    void evaluateAll(const BmsLogicCache& cache,
                     control::LogicContext& ctx,
                     uint64_t now_ms) const;

private:
    void evaluateOne_(uint32_t inst,
                      const BmsPerInstanceCache* x,
                      control::LogicContext& ctx,
                      uint64_t now_ms) const;

    static std::string makeInstName_(uint32_t inst);
    static std::string makeSignalKey_(uint32_t inst, const char* signal);

    static void clearInstSignals_(uint32_t inst, control::LogicContext& ctx);
    static void setConfirmed_(uint32_t inst,
                              const char* signal,
                              bool on,
                              control::LogicContext& ctx);

    static void setDirect_(uint32_t inst,
                           const char* signal,
                           bool on,
                           control::LogicContext& ctx);

    static void setLevel3_(uint32_t inst,
                           const char* sig_lvl1,
                           const char* sig_lvl2,
                           const char* sig_lvl3,
                           int32_t level,
                           control::LogicContext& ctx);

    static void setLevel2_(uint32_t inst,
                           const char* sig_lvl1,
                           const char* sig_lvl2,
                           int32_t level,
                           control::LogicContext& ctx);
    static bool seenWithin_(uint64_t now_ms, uint64_t seen_ms, uint32_t hold_ms);
    static bool tmsCodeLatched_(const BmsPerInstanceCache* x,
                                uint64_t now_ms,
                                int32_t code,
                                uint32_t hold_ms);
};

} // namespace control::bms

#endif // ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H