//
// Created by ChatGPT on 2026/4/8.
//

#ifndef ENERGYSTORAGE_FAULT_RUNTIME_MAPPER_H
#define ENERGYSTORAGE_FAULT_RUNTIME_MAPPER_H

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../bms/bms_logic_types.h"
#include "../logic/logic_context.h"

namespace control
{
    class FaultCenter;

    /**
     * FaultRuntimeMapper
     *
     * 职责：
     * 1. 从 fault_map.jsonl 中读取运行态映射规则（source / signal / instance）
     * 2. 根据 LogicContext 中已经整理好的 runtime 真源判断某条故障是否 active
     * 3. 调用 FaultCenter::setActive(code, on)
     *
     * 不负责：
     * - 不负责 HMI 输出
     * - 不负责分页
     * - 不负责历史
     * - 不负责重新计算 online/offline aging
     */
    class FaultRuntimeMapper {
    public:
        struct Rule {
            uint16_t code{0};

            std::string code_hex;
            std::string name;

            std::string source;   // 原始 source（保留展示）
            std::string signal;   // 原始 signal（保留展示）
            uint32_t instance{0};

            bool show_hmi_current{false};
            bool show_hmi_history{false};

            // 增量适配：预归一化字段，减少运行期重复处理
            std::string source_norm; // bms / pcu / ups / smoke / gas / air / logic
            std::string signal_norm; // 统一 token
        };

        struct LoadStats {
            size_t total_items{0};
            size_t accepted_rules{0};
            size_t skipped_no_source_or_signal{0};
            size_t skipped_unsupported_source{0};
        };

    public:
        bool loadJsonl(const std::string& path, std::string* err = nullptr);

        void clear();
        bool empty() const { return rules_.empty(); }
        size_t size() const { return rules_.size(); }

        const std::vector<Rule>& rules() const { return rules_; }
        const LoadStats& loadStats() const { return stats_; }

        // 兼容保留：仅 BMS
        void applyBms(const control::bms::BmsLogicCache& cache,
                      control::FaultCenter& faults) const;

        // 主入口：统一消费 LogicContext 中的故障真源
        void applyAll(const LogicContext& ctx,
                      control::FaultCenter& faults) const;
    private:
        static bool parseBoolLoose_(const std::string& s, bool defv = false);
        static std::string trim_(const std::string& s);
        static std::string normalizeToken_(const std::string& s);
        static std::string normalizeSource_(const std::string& s);
        static std::string normalizeSignal_(const std::string& s);
        static bool tryParseInstanceFromSignal_(const std::string& signal, uint32_t& out_inst);

        static bool evalBmsSignal_(const control::bms::BmsPerInstanceCache& x,
                                   const std::string& signal);

        static bool evalPcuSignal_(const PcuOnlineState& x,
                                   const std::string& signal);

        static bool evalUpsSignal_(const UpsFaultState& x,
                                   const std::string& signal);

        static bool evalSmokeSignal_(const SmokeFaultState& x,
                                     const std::string& signal);

        static bool evalGasSignal_(const GasFaultState& x,
                                   const std::string& signal);

        static bool evalAirSignal_(const AirFaultState& x,
                                   const std::string& signal);

        static bool evalLogicSignal_(const LogicFaultState& x,
                                     const std::string& signal);

        // 增量适配：用于日志判定“规则没命中”到底是 source 问题还是 signal 问题
        static bool isKnownSignalForSource_(const std::string& source_norm,
                                            const std::string& signal_norm);

    private:
        std::vector<Rule> rules_;
        LoadStats stats_{};
    };
}

#endif // ENERGYSTORAGE_FAULT_RUNTIME_MAPPER_H