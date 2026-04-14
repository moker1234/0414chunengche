// services/control/logic_engine.cpp
//
// LogicEngine 总入口 / 编排器
//
#include "logic_engine.h"

#include "../utils/logger/logger.h"
#include "../driver/driver_manager.h"

namespace control {

    void LogicEngine::init(::DriverManager& drv)
    {
        if (bms_cmd_mgr_inited_) return;

        if (bms_cmd_mgr_.init(drv, /*default_can_index=*/2)) {
            bms_cmd_mgr_inited_ = true;
        } else {
            LOGWARN("[CTRL][BMS] BmsCommandManager init failed");
        }

        model_exporter_.setOutputPath("/home/zlg/userdata/debug/model_latest_for_csv.json");
        model_exporter_.setMinIntervalMs(1000);
    }

    bool LogicEngine::loadFaultRuntimeMapFile(const std::string& path, std::string* err)
    {
        return fault_runtime_mapper_.loadJsonl(path, err);
    }

    void LogicEngine::onEvent(const Event& e, LogicContext& ctx, std::vector<Command>& out_cmds)
    {
        ctx.last_event_ts = e.ts_ms;

        switch (e.type)
        {
        case Event::Type::DeviceData:
            onDeviceData_(e.device_data, e.ts_ms, ctx, out_cmds);
            break;

        case Event::Type::Snapshot:
            onSnapshot_(e.snapshot, ctx, out_cmds);
            break;

        case Event::Type::HmiWrite:
            onHmiWrite_(e.hmi_write, ctx, out_cmds);
            break;

        case Event::Type::IoSample:
            onIoSample_(e.io_sample, ctx, out_cmds);
            break;

        case Event::Type::Tick:
            onTick_(e.tick, ctx, out_cmds);
            break;

        case Event::Type::LinkHealth:
            onLinkHealth_(e.link_health, ctx, out_cmds);
            break;

        case Event::Type::Stop:
        default:
            break;
        }
    }

} // namespace control