// services/control/logic_tick.cpp
//
// Tick 周期逻辑：
// - BMS 周期命令
// - 故障页投影
// - logic_view 刷新
// - HMI 周期下行刷新
//
#include <algorithm>
#include <cmath>

#include "logic_engine.h"
#include "../../protocol/can/pcu/proto_pcu.h"

#include "../utils/logger/logger.h"
#include "fault/fault_addr_layout.h"

namespace control {
    namespace {
        // ===================== PCU runtime timeout =====================
        constexpr uint32_t PCU_RX_TIMEOUT_MS = 1500;
        constexpr uint32_t PCU_HB_STALE_MS   = 3000;

        // ===================== BMS runtime timeout =====================
        constexpr uint32_t BMS_INSTANCE_TIMEOUT_MS      = 5000;
        constexpr uint32_t BMS_ST1_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST2_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST3_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST4_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST5_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST6_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST7_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_CURRENT_LIMIT_TIMEOUT_MS = 3000;
        constexpr uint32_t BMS_ELEC_ENERGY_TIMEOUT_MS   = 5000;
        constexpr uint32_t BMS_TM2B_TIMEOUT_MS          = 5000;
        constexpr uint32_t BMS_FIRE2B_TIMEOUT_MS        = 5000;
        constexpr uint32_t BMS_FAULT1_TIMEOUT_MS        = 5000;
        constexpr uint32_t BMS_FAULT2_TIMEOUT_MS        = 5000;

            // ===================== PCU TX helper =====================
    // 约定：di_bits bit0 -> DI1, bit1 -> DI2 ...
    static constexpr int kPcuDiChEstop = 1;

    static constexpr int kPcuDiChPlug1 = 13;
    static constexpr int kPcuDiChPlug2 = 14;
    static constexpr int kPcuDiChPlug3 = 15;
    static constexpr int kPcuDiChPlug4 = 16;
    static constexpr int kPcuDiChPlug5 = 17;
    static constexpr int kPcuDiChPlug6 = 18;

    static constexpr int kPcuAdcChPlug1 = 1;
    static constexpr int kPcuAdcChPlug2 = 2;

    static constexpr double kPcuPlugAdcThresholdV = 8.0;

    static bool testDiOnForPcu_(uint64_t di_bits, int channel_id)
    {
        if (channel_id < 1 || channel_id > 64) return false;
        return ((di_bits >> (channel_id - 1)) & 0x1ULL) != 0;
    }

    static double readAiVoltageForPcu_(const std::vector<double>& ai, int adc_channel)
    {
        if (adc_channel < 1) return 0.0;
        const std::size_t idx = static_cast<std::size_t>(adc_channel - 1);
        if (idx >= ai.size()) return 0.0;
        return ai[idx];
    }

    static bool isPlugByAdcForPcu_(double v)
    {
        // 低于 8V 视为插枪
        return v > 0.01 && v < kPcuPlugAdcThresholdV;
    }

    static bool calcPcuPlugState_(const LogicContext& ctx)
    {
        const bool plug_di_any =
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug1) ||
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug2) ||
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug3) ||
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug4) ||
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug5) ||
            testDiOnForPcu_(ctx.di_bits, kPcuDiChPlug6);

        const double adc1_v = readAiVoltageForPcu_(ctx.ai, kPcuAdcChPlug1);
        const double adc2_v = readAiVoltageForPcu_(ctx.ai, kPcuAdcChPlug2);

        const bool plug_adc_any =
            isPlugByAdcForPcu_(adc1_v) ||
            isPlugByAdcForPcu_(adc2_v);

        /*
         * 当前先按“任意插枪检测有效”置 1。
         *
         * 如果后续确认 PCU 严格要求“4 个插座均已连接才为 1”，
         * 只需要把这里改成 plug_di_all / plug_adc_all 的组合逻辑。
         */
        return plug_di_any || plug_adc_any;
    }

    static const control::bms::BmsPerInstanceCache*
    findBmsForPcu_(const LogicContext& ctx, uint32_t bms_index)
    {
        const std::string name = "BMS_" + std::to_string(bms_index);
        auto it = ctx.bms_cache.items.find(name);
        if (it == ctx.bms_cache.items.end()) return nullptr;
        return &it->second;
    }

    static bool bmsCriticalForPcu_(const control::bms::BmsPerInstanceCache* x)
    {
        if (!x) return true;

        if (!x->seen_once) return true;
        if (!x->online) return true;

        if (x->runtime_fault_stale) return true;
        if (x->rq_hv_power_off) return true;

        if (x->fault_level >= 2) return true;
        if (x->fire_fault_level >= 2) return true;
        if (x->tms_fault_level >= 2) return true;

        if (!x->hv_allow_close) return true;

        return false;
    }

    static uint16_t calcPcuDischargePowerKwX10_(
        const control::bms::BmsPerInstanceCache* x)
    {
        if (!x) return 0;

        /*
         * PCU 协议：
         *   最大放电功率范围 0~1000kW，分辨率 0.1kW/bit。
         *
         * BMS：
         *   discharge_limit_a 单位 A
         *   pack_v 单位 V
         *
         * 原始值：
         *   raw = kW / 0.1 = (A * V / 1000) * 10 = A * V / 100
         */
        if (!x->online) return 0;
        if (!x->st2_online) return 0;
        if (!x->current_limit_online) return 0;

        if (!x->pack_v_valid) return 0;
        if (!x->discharge_limit_valid) return 0;

        const double pack_v = x->pack_v;
        const double pack_i = x->pack_i;

        if (!(pack_v > 0.0) || !(pack_i > 0.0)) return 0;

        const double raw = (pack_v * pack_i) / 100.0;
        const double clamped = std::max(0.0, std::min(raw, 10000.0));

        return static_cast<uint16_t>(std::lround(clamped));
    }
        static std::string canFrameHexForPcuTx_(const can_frame& fr)
    {
        char buf[4];
        std::string s;
        s.reserve(8 * 3);

        const uint8_t n = (fr.can_dlc > 8) ? 8 : fr.can_dlc;
        for (uint8_t i = 0; i < n; ++i) {
            std::snprintf(buf, sizeof(buf), "%02X", fr.data[i]);
            if (!s.empty()) s.push_back(' ');
            s.append(buf);
        }

        return s;
    }
    }

void LogicEngine::emitPcuPeriodicCommands_(LogicContext& ctx,
                                           uint64_t now_ms,
                                           std::vector<Command>& out_cmds)
{
    if (!pcu_tx_cfg_loaded_) {
        return;
    }

    const bool plug_state = calcPcuPlugState_(ctx);

    const bool estop_by_io =
        ctx.logic_faults.system_estop ||
        testDiOnForPcu_(ctx.di_bits, kPcuDiChEstop);

    const auto* bms1 = findBmsForPcu_(ctx, 1);
    const auto* bms2 = findBmsForPcu_(ctx, 2);

    const bool batt1_estop = bmsCriticalForPcu_(bms1);
    const bool batt2_estop = bmsCriticalForPcu_(bms2);

    const bool system_enable =
        (!estop_by_io) &&
        (!batt1_estop) &&
        (!batt2_estop);

    const uint16_t batt1_kw_x10 = calcPcuDischargePowerKwX10_(bms1);
    const uint16_t batt2_kw_x10 = calcPcuDischargePowerKwX10_(bms2);

        for (auto& cfg : pcu_tx_cfg_) {
            if (!cfg.valid) {
                LOG_THROTTLE_MS("pcu_tx_skip_invalid", 3000, LOG_COMM_W,
                                "[PCU][TX][SKIP] invalid cfg pcu_instance=%u can=%d",
                                static_cast<unsigned>(cfg.pcu_instance),
                                cfg.can_index);
                continue;
            }

            if (!cfg.tx_enable) {
                LOG_THROTTLE_MS("pcu_tx_skip_disabled", 3000, LOG_COMM_W,
                                "[PCU][TX][SKIP] tx disabled pcu_instance=%u can=%d",
                                static_cast<unsigned>(cfg.pcu_instance),
                                cfg.can_index);
                continue;
            }

            if (cfg.can_index < 0) {
                LOG_THROTTLE_MS("pcu_tx_skip_bad_can", 3000, LOG_COMM_W,
                                "[PCU][TX][SKIP] bad can_index pcu_instance=%u can=%d",
                                static_cast<unsigned>(cfg.pcu_instance),
                                cfg.can_index);
                continue;
            }

        if (cfg.next_due_ms == 0) {
            cfg.next_due_ms = now_ms;
        }

        if (now_ms < cfg.next_due_ms) {
            continue;
        }

        const std::size_t cmd_begin = out_cmds.size();

        bool has_ctrl_frame = false;
        bool has_status_frame = false;

        can_frame ctrl_fr{};
        can_frame status_fr{};

        const uint8_t ctrl_heartbeat = cfg.heartbeat;

        if (cfg.send_ctrl) {
            proto::pcu::buildEmuCtrl(
                ctrl_fr,
                cfg.id_emu_ctrl,
                ctrl_heartbeat,
                plug_state ? 1u : 0u,
                estop_by_io ? 1u : 0u,
                batt1_estop ? 1u : 0u,
                batt2_estop ? 1u : 0u,
                system_enable ? 1u : 0u
            );

            Command c;
            c.type = Command::Type::SendCan;
            c.can.can_index = cfg.can_index;
            c.can.frame = ctrl_fr;
            out_cmds.push_back(c);

            has_ctrl_frame = true;
            cfg.heartbeat = static_cast<uint8_t>(cfg.heartbeat + 1);
        }

        if (cfg.send_status) {
            proto::pcu::buildEmuStatus(
                status_fr,
                cfg.id_emu_status,
                batt1_kw_x10,                     // pcu 功率计算
                batt2_kw_x10,
                cfg.batt1_branches,
                cfg.batt2_branches
            );

            Command c;
            c.type = Command::Type::SendCan;
            c.can.can_index = cfg.can_index;
            c.can.frame = status_fr;
            out_cmds.push_back(c);

            has_status_frame = true;
        }

        /*
         * 第七批：给本轮最后一条 PCU SendCan 命令附带 TX 镜像。
         *
         * 这样一轮 PCU TX 只产生一个 DeviceData：
         *   PCU_0_CTRL 或 PCU_1_CTRL
         *
         * 其中同时包含：
         *   0x1801A0E0 控制帧字段
         *   0x1802A0E0 状态帧字段
         */
if (out_cmds.size() > cmd_begin) {
    DeviceData mirror;

    const uint8_t inst = cfg.pcu_instance;
    const uint8_t runtime_index =
        (inst >= 1) ? static_cast<uint8_t>(inst - 1) : 0;

    mirror.device_name =
        "PCU_" + std::to_string(static_cast<unsigned>(runtime_index)) + "_CTRL";

    mirror.timestamp = static_cast<uint32_t>(now_ms & 0xFFFFFFFFu);

    mirror.value["__can_index"] = cfg.can_index;
    mirror.value["__pcu.instance"] = static_cast<int32_t>(inst);
    mirror.value["__pcu.runtime_index"] = static_cast<int32_t>(runtime_index);

    mirror.value["__pcu.ctrl_id"] =
        static_cast<int32_t>(cfg.id_emu_ctrl & CAN_EFF_MASK);
    mirror.value["__pcu.status_id"] =
        static_cast<int32_t>(cfg.id_emu_status & CAN_EFF_MASK);

    mirror.value["ctrl_heartbeat"] =
        static_cast<int32_t>(ctrl_heartbeat);

    mirror.status["plug_state"] = plug_state ? 1u : 0u;
    mirror.status["estop"] = estop_by_io ? 1u : 0u;
    mirror.status["batt1_estop"] = batt1_estop ? 1u : 0u;
    mirror.status["batt2_estop"] = batt2_estop ? 1u : 0u;
    mirror.status["sys_enable"] = system_enable ? 1u : 0u;

    mirror.value["batt1_kw_x10"] = static_cast<int32_t>(batt1_kw_x10);
    mirror.value["batt2_kw_x10"] = static_cast<int32_t>(batt2_kw_x10);
    mirror.value["batt1_branches"] = static_cast<int32_t>(cfg.batt1_branches);
    mirror.value["batt2_branches"] = static_cast<int32_t>(cfg.batt2_branches);

    mirror.num["batt1_kw"] = static_cast<double>(batt1_kw_x10) / 10.0;
    mirror.num["batt2_kw"] = static_cast<double>(batt2_kw_x10) / 10.0;

    mirror.str["kind"] = "pcu_tx_mirror";
    mirror.str["__pcu.msg"] = "EMU_TO_PCU_TX";
    mirror.str["__pcu.instance_name"] =
        "PCU_" + std::to_string(static_cast<unsigned>(runtime_index));
    mirror.str["__pcu.display_name"] =
        "PCU" + std::to_string(static_cast<unsigned>(inst));

    if (has_ctrl_frame) {
        mirror.str["__pcu.ctrl_raw_hex"] = canFrameHexForPcuTx_(ctrl_fr);
    }

    if (has_status_frame) {
        mirror.str["__pcu.status_raw_hex"] = canFrameHexForPcuTx_(status_fr);
    }

    /*
     * 修正版：
     * 本轮 cfg 生成的所有 PCU SendCan 命令都挂同一个镜像。
     * 这样 can_index 与 mirror.device_name 一一对应，不再只挂 out_cmds.back()。
     */
    for (std::size_t i = cmd_begin; i < out_cmds.size(); ++i) {
        if (out_cmds[i].type != Command::Type::SendCan) continue;
        if (out_cmds[i].can.can_index != cfg.can_index) continue;

        out_cmds[i].can.has_tx_mirror = true;
        out_cmds[i].can.tx_mirror = mirror;
    }
}

        cfg.next_due_ms = now_ms + cfg.period_ms;

            const std::size_t cmds_added = out_cmds.size() - cmd_begin;

            const std::string pcu_tx_log_key =
                "pcu_tx_tick_can" + std::to_string(cfg.can_index) +
                "_inst" + std::to_string(static_cast<unsigned>(cfg.pcu_instance));

            LOG_THROTTLE_KEYED_MS(
                pcu_tx_log_key.c_str(),
                1000,
                LOG_COMM_D,
                "[PCU][TX] can=%d inst=%u plug=%d estop=%d b1_estop=%d b2_estop=%d enable=%d b1_kw_x10=%u b2_kw_x10=%u cmds_added=%zu total_cmds=%zu",
                cfg.can_index,
                static_cast<unsigned>(cfg.pcu_instance),
                plug_state ? 1 : 0,
                estop_by_io ? 1 : 0,
                batt1_estop ? 1 : 0,
                batt2_estop ? 1 : 0,
                system_enable ? 1 : 0,
                static_cast<unsigned>(batt1_kw_x10),
                static_cast<unsigned>(batt2_kw_x10),
                cmds_added,
                out_cmds.size()
            );
    }
}

    void LogicEngine::onTick_(const TickEvent& t,
                              LogicContext& ctx,
                              std::vector<Command>& out_cmds)
    {
    //     LOG_THROTTLE_MS("logic_tick_alive", 1000, LOG_COMM_D,  // 20260409 检查online
    // "[LOGIC][TICK] ts=%llu pcu0{on=%d rx=%d hb=%d} pcu1{on=%d rx=%d hb=%d} bms_items=%zu",
    // (unsigned long long)t.ts_ms,
    // ctx.pcu0_state.online ? 1 : 0,
    // ctx.pcu0_state.rx_alive ? 1 : 0,
    // ctx.pcu0_state.hb_alive ? 1 : 0,
    // ctx.pcu1_state.online ? 1 : 0,
    // ctx.pcu1_state.rx_alive ? 1 : 0,
    // ctx.pcu1_state.hb_alive ? 1 : 0,
    // ctx.bms_cache.items.size());

        // -----------------------------------------------------------------
        // -----------------------------------------------------------------
        // A) runtime aging / online 判定
        // -----------------------------------------------------------------
        updatePcuRuntimeHealth_(ctx, t.ts_ms);
        updateBmsRuntimeHealth_(ctx, t.ts_ms);


        // -----------------------------------------------------------------
        // B) 先用当前 BMS runtime 真源重建默认 desired
        // -----------------------------------------------------------------
        if (bms_cmd_mgr_inited_ && bms_cmd_tx_enabled_)
        {
            bms_cmd_mgr_.rebuildDesiredFromCache(ctx.bms_cache, t.ts_ms); // 从缓存重建期望命令
        }
        // -----------------------------------------------------------------
        // C) 业务模块
        //    - 读取 ctx.bms_cache / latest_snapshot / smoke_faults / di / ai
        //    - 修改 bms_cache 里的业务控制位
        //    - 输出 DO 命令
        // -----------------------------------------------------------------
        business_engine_.evaluate(ctx, t.ts_ms, bms_cmd_mgr_, out_cmds);

        // -----------------------------------------------------------------
        // D) 发送前安全钳制
        // -----------------------------------------------------------------
        // BusinessEngine 可以在 rebuildDesiredFromCache() 之后改 desired。
        // 因此发送前必须再次根据 BMS runtime 真源钳制，避免业务层误把 stale/offline BMS 拉到 PowerOn。
        if (bms_cmd_mgr_inited_ && bms_cmd_tx_enabled_)
        {
            bms_cmd_mgr_.enforceSafetyFromCache(ctx.bms_cache, t.ts_ms);
        }

        // -----------------------------------------------------------------
        // E1) 周期性 BMS 命令：不能依赖 HMI 存在与否
        // -----------------------------------------------------------------
        if (bms_cmd_mgr_inited_ && bms_cmd_tx_enabled_)
        {
            bms_cmd_mgr_.emitPeriodicCommands(t.ts_ms, out_cmds, bms_cmd_period_ms_);
        }

        // -----------------------------------------------------------------
        // E2) 周期性 PCU 命令：长期方案
        //
        // PCU 发送不再由 Scheduler 完成。
        // 这里读取 LogicContext 中的 IO / BMS / Business 真源，
        // 生成 SendCan Command，最终由 CommandDispatcher 下发。
        // -----------------------------------------------------------------
        emitPcuPeriodicCommands_(ctx, t.ts_ms, out_cmds);


        // // -----------------------------------------------------------------
        // // D) logic 聚合故障真源
        // // -----------------------------------------------------------------
        // ctx.logic_faults.last_eval_ts_ms = t.ts_ms;
        //
        // // BMS
        // bool bms_any_offline = false;
        // for (const auto& kv : ctx.bms_cache.items) {
        //     if (!kv.second.online) {
        //         bms_any_offline = true;
        //         break;
        //     }
        // }
        // ctx.logic_faults.bms_any_offline = bms_any_offline;
        //
        // // PCU
        // ctx.logic_faults.pcu_any_offline =
        //     (!ctx.pcu0_state.online) || (!ctx.pcu1_state.online);
        //
        // // 其他设备 offline
        // ctx.logic_faults.ups_offline   = ctx.ups_faults.seen_once   && !ctx.ups_faults.online;
        // ctx.logic_faults.smoke_offline = ctx.smoke_faults.seen_once && !ctx.smoke_faults.online;
        // ctx.logic_faults.gas_offline   = ctx.gas_faults.seen_once   && !ctx.gas_faults.online;
        // ctx.logic_faults.air_offline   = ctx.air_faults.seen_once   && !ctx.air_faults.online;
        //
        // // 环境类告警
        // ctx.logic_faults.env_any_alarm =
        //     ctx.ups_faults.alarm_any ||
        //     ctx.smoke_faults.alarm_any ||
        //     ctx.gas_faults.alarm_any ||
        //     ctx.air_faults.alarm_any;
        //
        // // E-Stop
        // ctx.logic_faults.system_estop = ctx.e_stop_latched;
        //
        // // 汇总
        // ctx.logic_faults.any_fault =
        //     ctx.logic_faults.pcu_any_offline ||
        //     ctx.logic_faults.bms_any_offline ||
        //     ctx.logic_faults.ups_offline ||
        //     ctx.logic_faults.smoke_offline ||
        //     ctx.logic_faults.gas_offline ||
        //     ctx.logic_faults.air_offline ||
        //     ctx.logic_faults.env_any_alarm ||
        //     ctx.logic_faults.system_estop;
        //
        // // -----------------------------------------------------------------
        // // E) 故障页投影
        // // -----------------------------------------------------------------
        // applyFaultPages_(ctx, t.ts_ms);
        //
        // // 第八批：联调日志，直接观察“当前页前5条故障码”是否已形成
        // {
        //     const auto visible = ctx.fault_center.debugCurrentVisibleCodes();
        //     const auto rows = ctx.fault_center.debugCurrentRows();
        //     const uint16_t cur_page = ctx.fault_center.debugCurrentPageIndex();
        //     const uint16_t cur_total = ctx.fault_center.debugCurrentTotalPages();
        //
        //     auto row_or_empty = [&](size_t idx) -> FaultCenterCurrentRow {
        //         return (idx < rows.size()) ? rows[idx] : FaultCenterCurrentRow{};
        //     };
        //
        //     const auto r0 = row_or_empty(0);
        //     const auto r1 = row_or_empty(1);
        //     const auto r2 = row_or_empty(2);
        //     const auto r3 = row_or_empty(3);
        //     const auto r4 = row_or_empty(4);
        //
        //     (void)visible;(void)cur_page;(void)cur_total;
        //     (void)r0;(void)r1;(void)r2;(void)r3;(void)r4;

        //     // LOG_THROTTLE_MS(
        //     //     "fault_current_rows", 1000, LOG_SYS_I,
        //     //     "[FAULT][CUR] visible=%zu page=%u/%u "
        //     //     "r1{seq=%u code=0x%04X on=%u} "
        //     //     "r2{seq=%u code=0x%04X on=%u} "
        //     //     "r3{seq=%u code=0x%04X on=%u} "
        //     //     "r4{seq=%u code=0x%04X on=%u} "
        //     //     "r5{seq=%u code=0x%04X on=%u}",
        //     //     visible.size(),
        //     //     static_cast<unsigned>(cur_total == 0 ? 0 : cur_page + 1),
        //     //     static_cast<unsigned>(cur_total),
        //     //     r0.seq_no, r0.code, r0.on_time,
        //     //     r1.seq_no, r1.code, r1.on_time,
        //     //     r2.seq_no, r2.code, r2.on_time,
        //     //     r3.seq_no, r3.code, r3.on_time,
        //     //     r4.seq_no, r4.code, r4.on_time
        //     // );
        // }
        //
        // -----------------------------------------------------------------
        // F) 逻辑视图 / HMI 输出
        // -----------------------------------------------------------------
        //
        // PCU online 是 Tick aging 计算出来的。
        // 所以 Tick 后必须刷新 logic_view 和 HMI，才能让 pcu1_online / pcu2_online
        // 在 PCU 停包或 heartbeat stale 时主动变化。
        rebuildLogicView_(ctx);
        applyHmiOutputs_(ctx);

        LOG_THROTTLE_MS(
            "pcu_runtime_tick",
            1000,
            LOG_COMM_D,
            "[PCU][RUNTIME][TICK] "
            "pcu1{on=%d rx=%d hb=%d age_rx=%.0f age_hb=%.0f reason=%u} "
            "pcu2{on=%d rx=%d hb=%d age_rx=%.0f age_hb=%.0f reason=%u}",
            ctx.pcu0_state.online ? 1 : 0,
            ctx.pcu0_state.rx_alive ? 1 : 0,
            ctx.pcu0_state.hb_alive ? 1 : 0,
            ctx.pcu0_state.last_rx_age_ms,
            ctx.pcu0_state.last_hb_change_age_ms,
            static_cast<unsigned>(ctx.pcu0_state.offline_reason_code),
            ctx.pcu1_state.online ? 1 : 0,
            ctx.pcu1_state.rx_alive ? 1 : 0,
            ctx.pcu1_state.hb_alive ? 1 : 0,
            ctx.pcu1_state.last_rx_age_ms,
            ctx.pcu1_state.last_hb_change_age_ms,
            static_cast<unsigned>(ctx.pcu1_state.offline_reason_code)
        );
    }
    bool LogicEngine::calcFreshByTimeout_(uint64_t now_ms,
                                          uint64_t last_ms,
                                          uint32_t timeout_ms)
    {
        if (last_ms == 0) return false;
        if (now_ms < last_ms) return false;
        return (now_ms - last_ms) <= static_cast<uint64_t>(timeout_ms);
    }

    double LogicEngine::calcAgeMs_(uint64_t now_ms,
                                   uint64_t last_ms)
    {
        if (last_ms == 0) return 0.0;
        if (now_ms < last_ms) return 0.0;
        return static_cast<double>(now_ms - last_ms);
    }

    const char* LogicEngine::pcuOfflineReasonText_(int code)
    {
        switch (code) {
            case 0: return "None";
            case 1: return "NoData";
            case 2: return "RxTimeout";
            case 3: return "HeartbeatStale";
            default: return "Unknown";
        }
    }

    const char* LogicEngine::bmsOfflineReasonText_(int code)
    {
        switch (code) {
            case 0: return "None";
            case 1: return "NoData";
            case 2: return "RxTimeout";
            default: return "Unknown";
        }
    }


    void LogicEngine::updateOnePcuRuntimeHealth_(PcuOnlineState& st,
                                                 uint64_t now_ms,
                                                 uint32_t rx_timeout_ms,
                                                 uint32_t hb_stale_timeout_ms)
    {
        const bool old_online = st.online;

        st.last_rx_age_ms = calcAgeMs_(now_ms, st.last_rx_ms);
        st.last_hb_change_age_ms = calcAgeMs_(now_ms, st.last_hb_change_ms);

        st.rx_alive = st.seen_once &&
                      calcFreshByTimeout_(now_ms, st.last_rx_ms, rx_timeout_ms);

        st.hb_alive = st.has_last_heartbeat &&
                      calcFreshByTimeout_(now_ms, st.last_hb_change_ms, hb_stale_timeout_ms);

        if (!st.seen_once) {
            st.online = false;
            st.offline_reason_code = 1;
        } else if (!st.rx_alive) {
            st.online = false;
            st.offline_reason_code = 2;
        } else if (!st.hb_alive) {
            st.online = false;
            st.offline_reason_code = 3;
        } else {
            st.online = true;
            st.offline_reason_code = 0;
        }

        st.offline_reason_text = pcuOfflineReasonText_(st.offline_reason_code);

        if (st.online != old_online) {
            st.last_online_change_ms = now_ms;
            if (!st.online) {
                st.last_offline_ms = now_ms;
                st.disconnect_count += 1;
            }
        }
    }

    void LogicEngine::updatePcuRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    {
        updateOnePcuRuntimeHealth_(ctx.pcu0_state,
                                   now_ms,
                                   PCU_RX_TIMEOUT_MS,
                                   PCU_HB_STALE_MS);

        updateOnePcuRuntimeHealth_(ctx.pcu1_state,
                                   now_ms,
                                   PCU_RX_TIMEOUT_MS,
                                   PCU_HB_STALE_MS);
    }

    void LogicEngine::updateBmsRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    {
        for (auto& kv : ctx.bms_cache.items)
        {
            auto& x = kv.second;
            const bool old_online = x.online;

            x.st1_age_ms = calcAgeMs_(now_ms, x.last_st1_ms);
            x.st2_age_ms = calcAgeMs_(now_ms, x.last_st2_ms);
            x.st3_age_ms = calcAgeMs_(now_ms, x.last_st3_ms);
            x.st4_age_ms = calcAgeMs_(now_ms, x.last_st4_ms);
            x.st5_age_ms = calcAgeMs_(now_ms, x.last_st5_ms);
            x.st6_age_ms = calcAgeMs_(now_ms, x.last_st6_ms);
            x.st7_age_ms = calcAgeMs_(now_ms, x.last_st7_ms);

            x.elec_energy_age_ms   = calcAgeMs_(now_ms, x.last_elec_energy_ms);
            x.current_limit_age_ms = calcAgeMs_(now_ms, x.last_current_limit_ms);
            x.tm2b_age_ms          = calcAgeMs_(now_ms, x.last_tm2b_ms);
            x.fire2b_age_ms        = calcAgeMs_(now_ms, x.last_fire2b_ms);
            x.fault1_age_ms        = calcAgeMs_(now_ms, x.last_fault1_ms);
            x.fault2_age_ms        = calcAgeMs_(now_ms, x.last_fault2_ms);

            x.st1_online = calcFreshByTimeout_(now_ms, x.last_st1_ms, BMS_ST1_TIMEOUT_MS);
            x.st2_online = calcFreshByTimeout_(now_ms, x.last_st2_ms, BMS_ST2_TIMEOUT_MS);
            x.st3_online = calcFreshByTimeout_(now_ms, x.last_st3_ms, BMS_ST3_TIMEOUT_MS);
            x.st4_online = calcFreshByTimeout_(now_ms, x.last_st4_ms, BMS_ST4_TIMEOUT_MS);
            x.st5_online = calcFreshByTimeout_(now_ms, x.last_st5_ms, BMS_ST5_TIMEOUT_MS);
            x.st6_online = calcFreshByTimeout_(now_ms, x.last_st6_ms, BMS_ST6_TIMEOUT_MS);
            x.st7_online = calcFreshByTimeout_(now_ms, x.last_st7_ms, BMS_ST7_TIMEOUT_MS);

            x.elec_energy_online   = calcFreshByTimeout_(now_ms, x.last_elec_energy_ms,   BMS_ELEC_ENERGY_TIMEOUT_MS);
            x.current_limit_online = calcFreshByTimeout_(now_ms, x.last_current_limit_ms, BMS_CURRENT_LIMIT_TIMEOUT_MS);
            x.tm2b_online          = calcFreshByTimeout_(now_ms, x.last_tm2b_ms,          BMS_TM2B_TIMEOUT_MS);
            x.fire2b_online        = calcFreshByTimeout_(now_ms, x.last_fire2b_ms,        BMS_FIRE2B_TIMEOUT_MS);
            x.fault1_online        = calcFreshByTimeout_(now_ms, x.last_fault1_ms,        BMS_FAULT1_TIMEOUT_MS);
            x.fault2_online        = calcFreshByTimeout_(now_ms, x.last_fault2_ms,        BMS_FAULT2_TIMEOUT_MS);

            x.runtime_fault_stale = false;
            if ((x.last_st2_ms > 0 && !x.st2_online) ||
                (x.last_current_limit_ms > 0 && !x.current_limit_online) ||
                (x.last_fault1_ms > 0 && !x.fault1_online) ||
                (x.last_fault2_ms > 0 && !x.fault2_online))
            {
                x.runtime_fault_stale = true;
            }

            if (!x.seen_once) {
                x.online = false;
                x.offline_reason_code = 1;
            } else if (!calcFreshByTimeout_(now_ms, x.last_rx_ms, BMS_INSTANCE_TIMEOUT_MS)) {
                x.online = false;
                x.offline_reason_code = 2;
            } else {
                x.online = true;
                x.offline_reason_code = 0;
            }

            x.offline_reason_text = bmsOfflineReasonText_(x.offline_reason_code);

            if (x.online != old_online) {
                x.last_online_change_ms = now_ms;
                if (!x.online) {
                    x.last_offline_ms = now_ms;
                    x.disconnect_count += 1;
                }
            }
        }
    }
    static std::string joinCodesU16_(const std::vector<uint16_t>& codes)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < codes.size(); ++i)
        {
            if (i != 0) oss << ",";
            oss << "0x" << std::hex << std::uppercase << codes[i] << std::dec;
        }
        if (codes.empty())
        {
            oss << "<none>";
        }
        return oss.str();
    }
    /*
     * @brief 应用故障映射
     *
     * @param ctx 逻辑上下文引用
     * @param now_ms 当前时间戳
     * @return void
     */
    void LogicEngine::applyFaultPages_(LogicContext& ctx, uint64_t now_ms)
    {
        // 1) BMS 专用故障映射
        bms_fault_mapper_.applyToFaultPages(
            ctx.bms_cache,
            ctx,
            now_ms,
            ctx.fault_center
        );

        // 2) fault_map.jsonl 通用 runtime 规则
        // 注意：这里只能调用一次。
        // 原代码在函数末尾又调用了一次 fault_runtime_mapper_.applyAll，
        // 会造成同一周期内重复 setActive，干扰 AIR 当前故障显示。
        fault_runtime_mapper_.applyAll(ctx, ctx.fault_center, now_ms);

        LOG_THROTTLE_MS(
            "fault_apply_pages_after_mapper",
            1000,
            LOGINFO,
            "[PROVE][FAULT_APPLY] after mapper visible=%zu page=%u/%u",
            ctx.fault_center.debugCurrentVisibleCodes().size(),
            static_cast<unsigned>(ctx.fault_center.debugCurrentPageIndex()),
            static_cast<unsigned>(ctx.fault_center.debugCurrentTotalPages())
        );

        {
            const auto visible = ctx.fault_center.debugCurrentVisibleCodes();
            const uint16_t total_pages = ctx.fault_center.debugCurrentTotalPages();

            for (uint16_t p = 0; p < total_pages; ++p)
            {
                const size_t begin =
                    static_cast<size_t>(p) * control::fault::FAULTS_PER_PAGE;

                const size_t end =
                    std::min(begin + static_cast<size_t>(control::fault::FAULTS_PER_PAGE),
                             visible.size());

                std::ostringstream oss;
                for (size_t i = begin; i < end; ++i)
                {
                    if (i != begin) oss << ",";
                    oss << "0x" << std::hex << std::uppercase << visible[i] << std::dec;
                }

                if (begin >= end) {
                    oss << "<empty>";
                }

                LOG_THROTTLE_MS(
                    ("fault_page_all_" + std::to_string(p)).c_str(),
                    1000,
                    LOGINFO,
                    "[RESULT][FAULT_PAGE_ALL] page=%u/%u codes=%s",
                    static_cast<unsigned>(p + 1),
                    static_cast<unsigned>(total_pages),
                    oss.str().c_str()
                );
            }
        }
    }



} // namespace control