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
     * 作用：
     * 1. 从 LogicContext 中读取通用原始故障真源
     * 2. 使用 fault_cond_engine 做持续时间确认
     * 3. 将已确认的通用故障信号写入 ctx.confirmed_faults.signals
     *
     * 当前这一批目标：
     * - 把 UPS / Smoke / Gas / Air / PCU / logic 聚合类故障
     *   正式接入 confirmed signal 主链
     * - mapper 只负责落码，不负责复杂持续时间判断
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

        static bool rawUpsOffline_(const control::LogicContext& ctx);
        static bool rawUpsFault_(const control::LogicContext& ctx);

        static bool rawSmokeOffline_(const control::LogicContext& ctx);
        static bool rawSmokeAlarm_(const control::LogicContext& ctx);

        static bool rawGasOffline_(const control::LogicContext& ctx);
        static bool rawGasAlarm_(const control::LogicContext& ctx);

        static bool rawAirOffline_(const control::LogicContext& ctx);

        static bool rawEnvAnyAlarm_(const control::LogicContext& ctx);
        static bool rawAnyFault_(const control::LogicContext& ctx);

        static bool rawHmiCommFault_(const control::LogicContext& ctx, uint64_t now_ms);
        static bool rawRemoteCommFault_(const control::LogicContext& ctx, uint64_t now_ms);
        static bool rawSdcardFault_(const control::LogicContext& ctx);

        // ---------- grouped evaluators ----------
        void evaluatePcu_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateUps_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateSmoke_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateGas_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateAir_(control::LogicContext& ctx, uint64_t now_ms) const;
        void evaluateLogic_(control::LogicContext& ctx, uint64_t now_ms) const;
    };

} // namespace control::fault

#endif // ENERGYSTORAGE_FAULT_LOGIC_EVALUATOR_H