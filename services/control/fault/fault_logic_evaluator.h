//
// Created by lxy on 2026/4/12.
//

#ifndef ENERGYSTORAGE_FAULT_LOGIC_EVALUATOR_H
#define ENERGYSTORAGE_FAULT_LOGIC_EVALUATOR_H

#pragma once

#include <cstdint>
#include <string>

namespace control {
    struct LogicContext;
}

namespace control::fault {

    /**
     * FaultLogicEvaluator
     *
     * 当前职责：
     * 1. 保留 PCU confirmed 链路
     * 2. 保留系统级 / 聚合类 confirmed 链路
     * 3. 不再处理 UPS / Smoke / Gas / Air 的设备协议原始故障
     *
     * UPS / Smoke / Gas / Air 当前链路：
     * ctx.xxx_faults
     * -> FaultRuntimeMapper::evalXxxSignal_()
     * -> FaultRuntimeMapper::debounceRule_()
     * -> FaultCenter::setActive()
     */
    class FaultLogicEvaluator {
    public:
        FaultLogicEvaluator() = default;

        /**
         * 更新全部通用 confirmed fault 信号
         */
        void evaluateAll(control::LogicContext& ctx, uint64_t now_ms) const;

    private:
        // ---------- key helpers ----------
        static std::string makeSignalKey_(const char* signal);

        // ---------- write helpers ----------
        static void setConfirmed_(const char* signal,
                                  bool on,
                                  control::LogicContext& ctx);

        // ---------- raw condition helpers ----------
        static bool rawPcu0Offline_(const control::LogicContext& ctx);
        static bool rawPcu1Offline_(const control::LogicContext& ctx);

        static bool rawPcu0EmergencyStop_(const control::LogicContext& ctx);
        static bool rawPcu1EmergencyStop_(const control::LogicContext& ctx);

        static bool rawEnvAnyAlarm_(const control::LogicContext& ctx);
        static bool rawAnyFault_(const control::LogicContext& ctx);

        static bool rawHmiCommFault_(const control::LogicContext& ctx,
                                     uint64_t now_ms);

        static bool rawRemoteCommFault_(const control::LogicContext& ctx,
                                        uint64_t now_ms);

        static bool rawSdcardFault_(const control::LogicContext& ctx);

        // ---------- evaluators ----------
        void evaluatePcu_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateLogic_(control::LogicContext& ctx, uint64_t now_ms) const;
    };

} // namespace control::fault

#endif // ENERGYSTORAGE_FAULT_LOGIC_EVALUATOR_H