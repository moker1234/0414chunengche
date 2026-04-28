// services/control/logic_hmi_output.cpp
//
// HMI 下行输出统一收口：
// 1) 普通变量输出（normal_map_logic.jsonl -> HMI）
// 2) 故障页输出（fault_hmi_layout.jsonl -> HMI）
//
// 故障显示地址不再依赖 fault_addr_layout.h。
// 当前故障页、历史故障页统一由 FaultPageManager 输出。

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

        ctx.fault_pages.flushToHmi(*ctx.hmi,
                                   ctx.fault_history_cache.get(),
                                   ctx.history_page_no);
    }

    void LogicEngine::applyHmiOutputs_(LogicContext& ctx)
    {
        if (!ctx.hmi) {
            return;
        }

        applyNormalHmi_(ctx);
        applyFaultHmi_(ctx);
    }

} // namespace control