// services/control/logic_hmi_output.cpp
//
// HMI 下行输出：
// - 普通变量输出（normal_map_logic.jsonl -> HMI AddressTable）
// - 故障页输出（fault_map.jsonl / FaultCenter -> HMI AddressTable）
// - 统一收口，避免输出逻辑分散在 onSnapshot_ / onTick_ 中
//
// 约束：
// - 故障页地址（尤其是 0x4125 ~ 0x4129）只允许由 FaultCenter::flushToHmi() 写入
// - 其他模块不要直接 setIntRead(0x4125~0x4129)，否则会破坏分页/排序/历史一致性
//
#include "logger.h"
#include "logic_engine.h"

namespace control {

    void LogicEngine::applyNormalHmi_(LogicContext& ctx)
    {
        if (!ctx.hmi) return;
        if (!ctx.normal_writer.loaded()) return;

        ctx.normal_writer.flushFromModel(ctx.latest_snapshot, ctx.logic_view, *ctx.hmi);
    }

    void LogicEngine::applyFaultHmi_(LogicContext& ctx)
    {
        if (!ctx.hmi) return;
        if (!ctx.fault_map_loaded) return;

        // 故障页输出统一收口到 FaultCenter。
        // 当前一阶段目标只要求现存故障页前5行代码输出到 HMI：
        //   ADDR_CUR_CODE_BASE = 0x4125
        //   FAULTS_PER_PAGE    = 5
        ctx.fault_center.flushToHmi(*ctx.hmi);
    //     LOG_THROTTLE_MS("fault_hmi_flush", 500, LOGINFO,
    // "[PROVE][FAULT_NEW_CHAIN] applyFaultHmi flush");
    }

    void LogicEngine::applyHmiOutputs_(LogicContext& ctx)
    {
        if (!ctx.hmi) {
            // LOG_COMM_D("[LOGIC][HMI_OUT] skip no_hmi");
            return;
        }

        // LOG_COMM_D("[LOGIC][HMI_OUT] begin normal_loaded=%d fault_loaded=%d",
        //            ctx.normal_writer.loaded() ? 1 : 0,
        //            ctx.fault_map_loaded ? 1 : 0);

        applyNormalHmi_(ctx);
        // applyFaultHmi_(ctx);

        // LOG_COMM_D("[LOGIC][HMI_OUT] done");
    }
} // namespace control