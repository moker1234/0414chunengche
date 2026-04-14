// services/control/logic/logic_snapshot.cpp
//
// Snapshot 事件处理：保存快照、重建 logic_view、导出模型、刷新 HMI
//
#include "logger.h"
#include "logic_engine.h"

namespace control {


    static void rebuildLogicFaultSummary_(LogicContext& ctx)
    {
        // 环境类总告警：
        // 只聚合环境设备（UPS / Smoke / Gas / Air）的 alarm 语义，
        // 不把 sdcard / hmi / remote 这种 system/comm 类故障算进去。
        ctx.logic_faults.env_any_alarm =
            ctx.ups_faults.alarm_any ||
            ctx.smoke_faults.alarm_any ||
            ctx.gas_faults.alarm_any ||
            ctx.air_faults.alarm_any;

        // 总故障：
        // 这里按“系统任何故障/离线/关键系统资源故障”为 true 的口径收口。
        ctx.logic_faults.any_fault =
            ctx.logic_faults.system_estop ||
            ctx.logic_faults.sdcard_fault ||
            ctx.logic_faults.hmi_comm_fault ||
            ctx.logic_faults.remote_comm_fault ||

            ctx.logic_faults.pcu_any_offline ||
            ctx.logic_faults.bms_any_offline ||
            ctx.logic_faults.ups_offline ||
            ctx.logic_faults.smoke_offline ||
            ctx.logic_faults.gas_offline ||
            ctx.logic_faults.air_offline ||

            ctx.ups_faults.fault_any ||
            ctx.smoke_faults.fault_any ||
            ctx.gas_faults.fault_any ||
            ctx.air_faults.fault_any;
    }

    void LogicEngine::onSnapshot_(const SnapshotEvent& s,
                                  LogicContext& ctx,
                                  std::vector<Command>& out_cmds)
    {
        (void)out_cmds;

        ctx.latest_snapshot = s.snap;
        ctx.last_event_ts = s.ts_ms;

        updatePcuRuntimeHealth_(ctx, s.ts_ms);
        updateBmsRuntimeHealth_(ctx, s.ts_ms);

        // 第11批：在 snapshot 主链里统一重算 logic 聚合故障，
        // 避免新增的 system/comm 类故障没有进入 any_fault / env_any_alarm。
        rebuildLogicFaultSummary_(ctx);


        // LOG_COMM_D("[LOGIC][SNAPSHOT][AFTER_RUNTIME] pcu0{on=%d rx=%d hb=%d} pcu1{on=%d rx=%d hb=%d}",
        //            ctx.pcu0_state.online ? 1 : 0,
        //            ctx.pcu0_state.rx_alive ? 1 : 0,
        //            ctx.pcu0_state.hb_alive ? 1 : 0,
        //            ctx.pcu1_state.online ? 1 : 0,
        //            ctx.pcu1_state.rx_alive ? 1 : 0,
        //            ctx.pcu1_state.hb_alive ? 1 : 0);

        // applyFaultPages_(ctx, s.ts_ms);
        // if (ctx.hmi) {
        //     if (ctx.fault_map_loaded) {
        //         ctx.fault_pages.flushToHmi(*ctx.hmi);
        //     }
        // }

        rebuildLogicView_(ctx);
        applyHmiOutputs_(ctx);

        if (out2json) {
            model_exporter_.exportLatest(ctx.latest_snapshot, ctx.logic_view, s.ts_ms);
        }
    }
} // namespace control