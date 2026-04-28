//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_BUSINESS_ENGINE_H
#define ENERGYSTORAGE_BUSINESS_ENGINE_H


#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../control_commands.h"

struct SnapshotItem;

namespace control {
    struct LogicContext;
    struct PcuOnlineState;
}

namespace control::bms {
    class BmsCommandManager;
    struct BmsPerInstanceCache;
}

namespace control::business {

    class BusinessEngine {
    public:
        BusinessEngine() = default;

        // 业务主入口：
        // 1) 读取 ctx 中的状态真源
        // 2) 通过 bms_mgr.mutableDesired() 修改默认命令
        // 3) 通过 out_cmds 输出 DO 等动作
        void evaluate(LogicContext& ctx,
                      uint64_t ts_ms,
                      bms::BmsCommandManager& bms_mgr,
                      std::vector<Command>& out_cmds);

    public:
        // ===== 辅助接口：你后续写业务时可直接复用 =====

        // bit0 -> DI1, bit1 -> DI2 ...
        static bool testDi(const LogicContext& ctx, int channel_id);

        // ai[0]=ADC1_V, ai[1]=ADC2_V ...
        static double readAiVoltage(const LogicContext& ctx, int adc_channel);

        static bms::BmsPerInstanceCache* getBms(LogicContext& ctx, uint32_t instance_index);
        static const bms::BmsPerInstanceCache* getBms(const LogicContext& ctx, uint32_t instance_index);

        static PcuOnlineState* getPcu(LogicContext& ctx, uint32_t instance_index);
        static const PcuOnlineState* getPcu(const LogicContext& ctx, uint32_t instance_index);

        static const SnapshotItem* findSnapshotItem(const LogicContext& ctx, const std::string& device_name);
        static bool isSnapshotOnline(const LogicContext& ctx, const std::string& device_name);

        static void emitWriteDo(std::vector<Command>& out_cmds,
                                int channel_id,
                                bool on);
    };

} // namespace control::business


#endif //ENERGYSTORAGE_BUSINESS_ENGINE_H
