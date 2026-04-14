#include "logger.h"
#include "logic_engine.h"

namespace control {

    // void LogicEngine::refreshFaultPagesOnly(uint64_t ts_ms, LogicContext& ctx)
    // {
    //     ctx.last_event_ts = ts_ms;
    //
    //     // ------------------------------------------------------------
    //     // 第五批：
    //     // 先刷新“确认后的故障真源”，再做故障码映射
    //     //
    //     // 当前不依赖 onTick_()，而是由独立 fault refresh 线程
    //     // 周期调用 refreshFaultPagesOnly()。
    //     // ------------------------------------------------------------
    //
    //     // 1) 先更新 BMS confirmed faults
    //     bms_fault_evaluator_.evaluateAll(ctx.bms_cache, ctx, ts_ms);
    //
    //     // 2) 再更新通用 / VCU confirmed faults
    //     fault_logic_evaluator_.evaluateAll(ctx, ts_ms);
    //
    //     // 3) 最后统一进入现有 fault page 映射链
    //     applyFaultPages_(ctx, ts_ms);
    //
    //     // 4) 故障页输出统一收口到 FaultCenter
    //     if (ctx.hmi && ctx.fault_map_loaded) {
    //         ctx.fault_center.flushToHmi(*ctx.hmi);
    //     }
    // }
    void LogicEngine::refreshFaultPagesOnly(uint64_t ts_ms, LogicContext& ctx)
    {
        ctx.last_event_ts = ts_ms;

        // 1) 先刷新 BMS confirmed faults
        bms_fault_evaluator_.evaluateAll(ctx.bms_cache, ctx, ts_ms);

        // 2) 再刷新通用 / VCU confirmed faults
        fault_logic_evaluator_.evaluateAll(ctx, ts_ms);

        // 3) 统一进入 fault page 映射链
        applyFaultPages_(ctx, ts_ms);

        // 4) 故障页输出统一收口
        if (ctx.hmi && ctx.fault_map_loaded) {
            applyFaultHmi_(ctx);
        }

    //     LOG_THROTTLE_MS("fault_refresh_only", 1000, LOGINFO,
    // "[PROVE][FAULT_NEW_CHAIN] refreshFaultPagesOnly ts=%llu",
    // (unsigned long long)ts_ms);
    }

} // namespace control