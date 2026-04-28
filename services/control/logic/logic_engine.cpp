// services/control/logic_engine.cpp
//
// LogicEngine 总入口 / 编排器
//
#include "logic_engine.h"

#include "config_loader.h"
#include "../utils/logger/logger.h"
#include "../driver/driver_manager.h"

namespace control {
    //找整条 BMS can_link 配置
    struct BmsCanRuntimeCfg {
        int can_index{0};
        bool tx_enable{true};
        uint32_t tx_period_ms{100};
        uint32_t v2b_cmd_id29{0x1802F3EF};
    };
    static bool loadBmsCanRuntimeCfg_(BmsCanRuntimeCfg& out, std::string* err = nullptr)
    {
        SystemConfig sys{};
        std::string local_err;
        if (!ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, local_err)) {
            if (err) *err = local_err;
            return false;
        }

        for (const auto& l : sys.can_links) {
            if (!l.enable) continue;
            if (l.protocol_type != "bms_table_v1") continue;

            out.can_index = l.can_index;
            out.tx_enable = l.tx_enable;
            out.tx_period_ms = (l.interval_ms == 0 ? 100u : l.interval_ms);
            out.v2b_cmd_id29 = (l.id_v2b_cmd == 0 ? 0x1802F3EFu : l.id_v2b_cmd);
            return true;
        }

        if (err) *err = "no enabled bms_table_v1 link found";
        return false;
    }
        void LogicEngine::loadPcuTxRuntimeCfg_()
    {
        pcu_tx_cfg_ = {};
        pcu_tx_cfg_loaded_ = false;

        SystemConfig sys{};
        std::string err;

        if (!ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, err)) {
            LOGWARN("[CTRL][PCU_TX] load system.json failed: %s", err.c_str());
            return;
        }

        for (const auto& l : sys.can_links) {
            if (!l.enable) continue;
            if (l.protocol_type != "emu_pcu_v1") continue;

            uint8_t inst = l.pcu_instance;

            /*
             * 过渡兜底：
             * 你当前原始链路通常是：
             *   can_index=1 -> PCU1
             *   can_index=2 -> PCU2
             *
             * 但最终仍推荐显式配置 pcu_instance。
             */
            if (inst == 0) {
                if (l.can_index == 1) inst = 1;
                else if (l.can_index == 2) inst = 2;
            }

            if (inst < 1 || inst > 2) {
                LOGWARN("[CTRL][PCU_TX] skip can_index=%d name=%s: invalid pcu_instance=%u",
                        l.can_index,
                        l.name.c_str(),
                        static_cast<unsigned>(l.pcu_instance));
                continue;
            }

            auto& cfg = pcu_tx_cfg_[inst - 1];

            cfg.valid = true;
            cfg.can_index = l.can_index;
            cfg.pcu_instance = inst;

            cfg.tx_enable = l.tx_enable;
            cfg.send_ctrl = l.send_ctrl;
            cfg.send_status = l.send_status;

            cfg.period_ms = (l.interval_ms == 0) ? 200u : l.interval_ms;
            cfg.next_due_ms = 0;

            cfg.id_emu_ctrl =
                (l.id_emu_ctrl == 0) ? 0x1801A0E0u : l.id_emu_ctrl;

            cfg.id_emu_status =
                (l.id_emu_status == 0) ? 0x1802A0E0u : l.id_emu_status;

            cfg.ctrl_enable_default = l.ctrl_enable_default;
            cfg.plug_default = l.plug_default;
            cfg.estop_default = l.estop_default;
            cfg.batt1_estop_default = l.batt1_estop_default;
            cfg.batt2_estop_default = l.batt2_estop_default;

            cfg.batt1_branches = 2;
            cfg.batt2_branches = 2;

            LOGINFO("[CTRL][PCU_TX] bind PCU%u can_index=%d tx_enable=%d period=%u ctrl=0x%08X status=0x%08X",
                    static_cast<unsigned>(inst),
                    cfg.can_index,
                    cfg.tx_enable ? 1 : 0,
                    static_cast<unsigned>(cfg.period_ms),
                    static_cast<unsigned>(cfg.id_emu_ctrl),
                    static_cast<unsigned>(cfg.id_emu_status));
        }
        for (std::size_t i = 0; i < pcu_tx_cfg_.size(); ++i) {
            const auto& c = pcu_tx_cfg_[i];
        }
        for (std::size_t i = 0; i < pcu_tx_cfg_.size(); ++i) {
            const auto& c = pcu_tx_cfg_[i];

            LOGINFO("[CTRL][PCU_TX][CFG_FINAL] slot=%zu valid=%d can_index=%d "
                    "pcu_instance=%u tx_enable=%d send_ctrl=%d send_status=%d "
                    "period=%u next_due=%llu ctrl=0x%08X status=0x%08X",
                    i,
                    c.valid ? 1 : 0,
                    c.can_index,
                    static_cast<unsigned>(c.pcu_instance),
                    c.tx_enable ? 1 : 0,
                    c.send_ctrl ? 1 : 0,
                    c.send_status ? 1 : 0,
                    static_cast<unsigned>(c.period_ms),
                    static_cast<unsigned long long>(c.next_due_ms),
                    static_cast<unsigned>(c.id_emu_ctrl),
                    static_cast<unsigned>(c.id_emu_status));
        }
        pcu_tx_cfg_loaded_ = true;
    }

    void LogicEngine::init(::DriverManager& drv)
    {
        if (bms_cmd_mgr_inited_ && pcu_tx_cfg_loaded_) return;

        BmsCanRuntimeCfg bms_cfg{};
        std::string err;

        if (loadBmsCanRuntimeCfg_(bms_cfg, &err)) {
            bms_cmd_tx_enabled_ = bms_cfg.tx_enable;
            bms_cmd_period_ms_  = bms_cfg.tx_period_ms;

            if (bms_cmd_mgr_.init(drv, bms_cfg.can_index, bms_cfg.v2b_cmd_id29)) {
                bms_cmd_mgr_inited_ = true;
                LOGINFO("[CTRL][BMS] BmsCommandManager init on can_index=%d tx_enable=%d period_ms=%u v2b_cmd=0x%08X",
                        bms_cfg.can_index,
                        bms_cfg.tx_enable ? 1 : 0,
                        (unsigned)bms_cfg.tx_period_ms,
                        (unsigned)bms_cfg.v2b_cmd_id29);
            } else {
                LOGWARN("[CTRL][BMS] BmsCommandManager init failed can_index=%d v2b_cmd=0x%08X",
                        bms_cfg.can_index,
                        (unsigned)bms_cfg.v2b_cmd_id29);
            }
        } else {
            LOGWARN("[CTRL][BMS] load BMS runtime cfg failed: %s", err.c_str());

            // fallback：至少保留原先能工作的最小行为
            bms_cmd_tx_enabled_ = true;
            bms_cmd_period_ms_  = 100;

            if (bms_cmd_mgr_.init(drv, 0, 0x1802F3EF)) {
                bms_cmd_mgr_inited_ = true;
                LOGWARN("[CTRL][BMS] fallback init on can_index=0 v2b_cmd=0x1802F3EF");
            } else {
                LOGWARN("[CTRL][BMS] fallback init failed");
            }
        }

        // PCU TX 也由控制面统一管理。
        // 注意：这里只加载配置，不直接发送；实际发送在 onTick_() 中输出 SendCan Command。
        loadPcuTxRuntimeCfg_();

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

        case Event::Type::HmiHeartbeat:
            ctx.hmi_seen_once = true;
            ctx.last_hmi_comm_ts = e.ts_ms;
            break;

        case Event::Type::IoSample:
            onIoSample_(e.io_sample, ctx, out_cmds);
            break;

        case Event::Type::Tick:
            onTick_(e.tick, ctx, out_cmds);
            break;



        case Event::Type::Stop:
        default:
            break;
        }
    }

} // namespace control