// services/control/logic_link.cpp
//
// LinkHealth 处理：
// - 当前保持空实现
// - 预留链路上下线、降级策略、联动告警入口
//
#include "logic_engine.h"

namespace control {

void LogicEngine::onLinkHealth_(const LinkHealthEvent& lh,
                                LogicContext& ctx,
                                std::vector<Command>& out_cmds)
{
    (void)out_cmds;

    const std::string& name = lh.link_name;

    // 串口映射：按你当前项目实际口位调整
    // 这里先按常用映射：
    // serial_0 = UPS(RS232) 或 Gas(RS485_0)
    // serial_1 = Smoke
    // serial_2 = AirConditioner
    // 如果你项目里 RS232/RS485 共用 index，需要再细分命名

    if (!lh.up) {
        if (name == "serial_0") {
            ctx.ups_faults.online = false;
        } else if (name == "serial_1") {
            ctx.smoke_faults.online = false;
        } else if (name == "serial_2") {
            ctx.air_faults.online = false;
        } else if (name == "serial_3") {
            ctx.gas_faults.online = false;
        } else if (name == "can_0") {
            ctx.pcu0_state.online = false;
        } else if (name == "can_1") {
            ctx.pcu1_state.online = false;
        } else if (name == "can_2") {
            for (auto& kv : ctx.bms_cache.items) {
                kv.second.online = false;
            }
        }
    }

    // LinkUp 不直接强行置 online
    // 恢复上线仍然以收到有效数据 / runtime aging 为准

    rebuildLogicView_(ctx);
    // applyHmiOutputs_(ctx);
}

} // namespace control