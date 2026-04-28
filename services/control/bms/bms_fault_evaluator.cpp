//
// Created by lxy on 2026/4/12.
//

#include "bms_fault_evaluator.h"

#include <string>

#include "logger.h"
#include "../logic/logic_context.h"

namespace control::bms
{
    namespace
    {
        static const char* const kImplementedBmsSignals[] = {
            "cell_overvoltage_lvl1",
            "cell_overvoltage_lvl2",
            "cell_overvoltage_lvl3",

            "cell_undervoltage_lvl1",
            "cell_undervoltage_lvl2",
            "cell_undervoltage_lvl3",

            "cell_overdischarge_fault",

            "total_voltage_overvoltage_lvl1",
            "total_voltage_overvoltage_lvl2",
            "total_voltage_overvoltage_lvl3",

            "total_voltage_undervoltage_lvl1",
            "total_voltage_undervoltage_lvl2",
            "total_voltage_undervoltage_lvl3",

            "cell_overtemp_lvl1",
            "cell_overtemp_lvl2",
            "cell_overtemp_lvl3",
            "cell_lowtemp_alarm",

            "temp_diff_over_lvl1",
            "temp_diff_over_lvl2",
            "temp_diff_over_lvl3",

            "fire_alarm",

            "pulse_discharge_current_overflow_lvl1",
            "pulse_discharge_current_overflow_lvl2",
            "pulse_discharge_current_overflow_lvl3",

            "pulse_charge_current_overflow_lvl1",
            "pulse_charge_current_overflow_lvl2",
            "pulse_charge_current_overflow_lvl3",

            "current_sensor_fault",
            "low_voltage_supply_alarm",
            "soc_imbalance_alarm",
            "soc_jump_alarm",

            "soc_low_lvl1",
            "soc_low_lvl2",
            "soc_low_lvl3",

            "tms_unit_fault",
            "tms_panel_aircon_fault",
            "tms_ats_water_pump_fault",
            "battery_self_protect_fault",
            "precharge_fault",

            "driving_insulation_low_lvl1",
            "driving_insulation_low_lvl2",
            "driving_insulation_low_lvl3",

            "charge_insulation_low_alarm",
            "acan_comm_fault",
            "internal_comm_fault",
            "branch_circuit_open_fault",
            "hvil_alarm",

            "heat_relay_open_fault",
            "heat_relay_weld_fault",
            "main_pos_relay_open_fault",
            "main_pos_relay_weld_fault",
            "main_neg_relay_open_fault",
            "main_neg_relay_weld_fault",

            "dc_chrg_pos1_relay_open_fault",
            "dc_chrg_pos1_relay_weld_fault",
            "dc_chrg_neg1_relay_open_fault",
            "dc_chrg_neg1_relay_weld_fault",
            "dc_chrg_pos2_relay_open_fault",
            "dc_chrg_pos2_relay_weld_fault",
            "dc_chrg_neg2_relay_open_fault",
            "dc_chrg_neg2_relay_weld_fault",

            "ac_chrg_pos_relay_open_fault",
            "ac_chrg_pos_relay_weld_fault",
            "ac_chrg_neg_relay_open_fault",
            "ac_chrg_neg_relay_weld_fault",
            "panto_chrg_pos_relay_open_fault",
            "panto_chrg_pos_relay_weld_fault",
            "panto_chrg_neg_relay_open_fault",
            "panto_chrg_neg_relay_weld_fault",

            "charge_gun_connection_abnormal",
            "storage_mismatch_alarm",
            "charge_discharge_current_overflow",
            "charge_current_overflow_alarm",
            "charge_connector_ntc_fault",
            "charge_connector_overtemp_lvl1",
            "charge_connector_overtemp_lvl2",
            "cell_consistency_poor_alarm",
            "soc_high_alarm",

            // 当前 CSV 前70行里 0x2044 没有 MsgName / Signal Name，
            // 这一批先保留信号名，但不在 evaluator 中置 true。
            "internal_hv_circuit_open_fault",
        };

        static constexpr size_t kImplementedBmsSignalCount =
            sizeof(kImplementedBmsSignals) / sizeof(kImplementedBmsSignals[0]);
    }

    std::string BmsFaultEvaluator::makeInstName_(uint32_t inst)
    {
        return "BMS_" + std::to_string(inst);
    }

    std::string BmsFaultEvaluator::makeSignalKey_(uint32_t inst, const char* signal)
    {
        return makeInstName_(inst) + "." + (signal ? signal : "unknown");
    }

    void BmsFaultEvaluator::clearInstSignals_(uint32_t inst,
                                              control::LogicContext& ctx)
    {
        for (size_t i = 0; i < kImplementedBmsSignalCount; ++i)
        {
            setConfirmed_(inst, kImplementedBmsSignals[i], false, ctx);
        }
    }

    void BmsFaultEvaluator::setConfirmed_(uint32_t inst,
                                          const char* signal,
                                          bool on,
                                          control::LogicContext& ctx)
    {
        const std::string key = makeSignalKey_(inst, signal);

        auto it = ctx.bms_confirmed_faults.signals.find(key);
        const bool existed = (it != ctx.bms_confirmed_faults.signals.end());
        const bool old = existed ? it->second : false;

        ctx.bms_confirmed_faults.signals[key] = on;

        if (!existed || old != on)
        {
            LOGINFO("[FAULT][BMS][CONFIRMED] key=%s old=%d new=%d",
                    key.c_str(),
                    old ? 1 : 0,
                    on ? 1 : 0);
        }
    }
    bool BmsFaultEvaluator::seenWithin_(uint64_t now_ms,
                                    uint64_t seen_ms,
                                    uint32_t hold_ms)
    {
        if (seen_ms == 0) return false;
        if (now_ms < seen_ms) return false;
        return (now_ms - seen_ms) <= hold_ms;
    }

    bool BmsFaultEvaluator::tmsCodeLatched_(const BmsPerInstanceCache* x,
                                            uint64_t now_ms,
                                            int32_t code,
                                            uint32_t hold_ms)
    {
        if (!x) return false;

        switch (code)
        {
        case 0x03: // 面板空调故障
            return seenWithin_(now_ms, x->tms_panel_aircon_seen_ms, hold_ms);

        case 0x1C: // ATS 水泵故障
            return seenWithin_(now_ms, x->tms_ats_water_pump_seen_ms, hold_ms);

        default:
            return false;
        }
    }

    void BmsFaultEvaluator::setDirect_(uint32_t inst,
                                       const char* signal,
                                       bool on,
                                       control::LogicContext& ctx)
    {
        setConfirmed_(inst, signal, on, ctx);
    }

    void BmsFaultEvaluator::setLevel3_(uint32_t inst,
                                       const char* sig_lvl1,
                                       const char* sig_lvl2,
                                       const char* sig_lvl3,
                                       int32_t level,
                                       control::LogicContext& ctx)
    {
        setConfirmed_(inst, sig_lvl1, level == 1, ctx);
        setConfirmed_(inst, sig_lvl2, level == 2, ctx);
        setConfirmed_(inst, sig_lvl3, level == 3, ctx);
    }

    void BmsFaultEvaluator::setLevel2_(uint32_t inst,
                                       const char* sig_lvl1,
                                       const char* sig_lvl2,
                                       int32_t level,
                                       control::LogicContext& ctx)
    {
        setConfirmed_(inst, sig_lvl1, level == 1, ctx);
        setConfirmed_(inst, sig_lvl2, level >= 2, ctx);
    }

    void BmsFaultEvaluator::evaluateAll(const BmsLogicCache& cache,
                                        control::LogicContext& ctx,
                                        uint64_t now_ms) const
    {
        (void)now_ms;

        for (uint32_t inst = 1; inst <= 4; ++inst)
        {
            const std::string name = makeInstName_(inst);

            auto it = cache.items.find(name);
            const BmsPerInstanceCache* x =
                (it == cache.items.end()) ? nullptr : &it->second;

            evaluateOne_(inst, x, ctx, now_ms);
        }

        LOG_THROTTLE_MS("bms_eval_direct_count", 2000, LOGINFO,
                        "[FAULT][BMS][EVAL_DIRECT] implemented confirmed signals=%zu",
                        kImplementedBmsSignalCount);
    }

    void BmsFaultEvaluator::evaluateOne_(uint32_t inst,
                                         const BmsPerInstanceCache* x,
                                         control::LogicContext& ctx,
                                         uint64_t now_ms) const
    {
        (void)now_ms;

        if (!x || !x->seen_once)
        {
            clearInstSignals_(inst, ctx);
            LOG_THROTTLE_MS(("bms_eval_invalid_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
                            "[FAULT][BMS][EVAL_DIRECT] inst=%u invalid instance -> cleared all confirmed signals",
                            (unsigned)inst);
            return;
        }

        // F1/F2 这批改成“报文组在线才信其原始故障位”
        // const bool fault1_ok = x->fault1_online;
        // const bool fault2_ok = x->fault2_online;

        const bool inst_online = x->online;
        const bool direct_faults_ok = inst_online;
        const bool fault1_ok = direct_faults_ok;
        const bool fault2_ok = direct_faults_ok;


        const bool fault1_stale_dbg =
            (x->last_fault1_ms > 0) && !x->fault1_online;
        const bool fault2_stale_dbg =
            (x->last_fault2_ms > 0) && !x->fault2_online;

        // ---------- F1：等级型 ----------
        setLevel3_(inst,
                   "cell_overvoltage_lvl1",
                   "cell_overvoltage_lvl2",
                   "cell_overvoltage_lvl3",
                   fault1_ok ? x->f1_over_ucell : 0,
                   ctx);

        setLevel3_(inst,
                   "cell_undervoltage_lvl1",
                   "cell_undervoltage_lvl2",
                   "cell_undervoltage_lvl3",
                   fault1_ok ? x->f1_low_ucell : 0,
                   ctx);

        setLevel3_(inst,
                   "total_voltage_overvoltage_lvl1",
                   "total_voltage_overvoltage_lvl2",
                   "total_voltage_overvoltage_lvl3",
                   fault1_ok ? x->f1_pack_over_hvolt : 0,
                   ctx);

        setLevel3_(inst,
                   "total_voltage_undervoltage_lvl1",
                   "total_voltage_undervoltage_lvl2",
                   "total_voltage_undervoltage_lvl3",
                   fault1_ok ? x->f1_pack_low_hvolt : 0,
                   ctx);

        setLevel3_(inst,
                   "cell_overtemp_lvl1",
                   "cell_overtemp_lvl2",
                   "cell_overtemp_lvl3",
                   fault1_ok ? x->f1_over_temp : 0,
                   ctx);

        setLevel3_(inst,
                   "temp_diff_over_lvl1",
                   "temp_diff_over_lvl2",
                   "temp_diff_over_lvl3",
                   fault1_ok ? x->f1_del_temp : 0,
                   ctx);

        setLevel3_(inst,
                   "soc_low_lvl1",
                   "soc_low_lvl2",
                   "soc_low_lvl3",
                   fault1_ok ? x->f1_low_soc : 0,
                   ctx);

        setLevel3_(inst,
                   "driving_insulation_low_lvl1",
                   "driving_insulation_low_lvl2",
                   "driving_insulation_low_lvl3",
                   fault1_ok ? x->f1_low_ins_res : 0,
                   ctx);

        // ---------- F1：单故障位 ----------
        setDirect_(inst, "soc_jump_alarm",
                   fault1_ok && x->f1_soc_change_fast, ctx);

        setDirect_(inst, "storage_mismatch_alarm",
                   fault1_ok && x->f1_bat_sys_not_match, ctx);

        setDirect_(inst, "hvil_alarm",
                   fault1_ok && x->f1_hvil_fault, ctx);

        setDirect_(inst, "cell_consistency_poor_alarm",
                   fault1_ok && x->f1_ucell_uniformity, ctx);

        setDirect_(inst, "soc_high_alarm",
                   fault1_ok && x->f1_over_soc, ctx);

        // ---------- F2：等级型 ----------
        setLevel3_(inst,
                   "pulse_discharge_current_overflow_lvl1",
                   "pulse_discharge_current_overflow_lvl2",
                   "pulse_discharge_current_overflow_lvl3",
                   fault2_ok ? x->f2_over_dischrg_curr_level : 0,
                   ctx);

        setLevel3_(inst,
                   "pulse_charge_current_overflow_lvl1",
                   "pulse_charge_current_overflow_lvl2",
                   "pulse_charge_current_overflow_lvl3",
                   fault2_ok ? x->f2_over_chrg_curr_in_drive_level : 0,
                   ctx);

        setLevel2_(inst,
                   "charge_connector_overtemp_lvl1",
                   "charge_connector_overtemp_lvl2",
                   fault2_ok ? x->f2_chrg_ntc_temp_over_level : 0,
                   ctx);

        // ---------- F2：单故障位 ----------
        setDirect_(inst, "cell_overdischarge_fault",
                   fault2_ok && x->f2_cell_over_dischrg, ctx);

        setDirect_(inst, "cell_lowtemp_alarm",
                   fault2_ok && x->f2_cell_low_temp, ctx);
        // Fire：
        // 直接使用 Fire2B_state 相关关键字段聚合后的真源
        const bool fire_alarm_on =
            direct_faults_ok && (
                (x->fire_value_alarm_level > 0) ||
                x->fire_by_start_sts ||
                x->fire_sensor_error_sts ||
                x->fire_hw_error_sts
            );

        setDirect_(inst, "fire_alarm", fire_alarm_on, ctx);

        // TMS：
        // 以 TM2B_FaultCode_A 为主真源，FaultLevel_A / F2_TMSErr 为兼容补充
        const bool tms_unit_fault_on =
            direct_faults_ok && (
                (x->tms_fault_code != 0) ||
                (x->tms_fault_level > 0) ||
                (fault2_ok && x->f2_tms_err)
            );

        setDirect_(inst, "tms_unit_fault", tms_unit_fault_on, ctx);

        // TMS 细分子故障：
        // 0x03 -> 面板空调故障
        // 0x1C -> ATS 水泵故障
        setDirect_(inst, "tms_panel_aircon_fault",
                   direct_faults_ok && tmsCodeLatched_(x, now_ms, 0x03, 3000), ctx);

        setDirect_(inst, "tms_ats_water_pump_fault",
                   direct_faults_ok && tmsCodeLatched_(x, now_ms, 0x1C, 3000), ctx);

        setDirect_(inst, "current_sensor_fault",
                    fault2_ok && x->f2_curr_sensor_err, ctx);

        setDirect_(inst, "low_voltage_supply_alarm",
                   fault2_ok && x->f2_power_supply_err, ctx);

        setDirect_(inst, "soc_imbalance_alarm",
                   fault2_ok && x->f2_soc_differ_err, ctx);

        setDirect_(inst, "battery_self_protect_fault",
                   fault2_ok && x->f2_pack_self_protect, ctx);

        setDirect_(inst, "precharge_fault",
                   fault2_ok && x->f2_main_loop_prechg_err, ctx);

        setDirect_(inst, "charge_insulation_low_alarm",
                   fault2_ok && x->f2_chrg_ins_low_err, ctx);

        setDirect_(inst, "acan_comm_fault",
                   fault2_ok && x->f2_acan_lost, ctx);

        setDirect_(inst, "internal_comm_fault",
                   fault2_ok && x->f2_inner_comm_err, ctx);

        setDirect_(inst, "branch_circuit_open_fault",
                   fault2_ok && x->f2_branch_break_err, ctx);

        setDirect_(inst, "heat_relay_open_fault",
                   fault2_ok && x->f2_heat_relay_open_err, ctx);

        setDirect_(inst, "heat_relay_weld_fault",
                   fault2_ok && x->f2_heat_relay_weld_err, ctx);

        setDirect_(inst, "main_pos_relay_open_fault",
                   fault2_ok && x->f2_main_pos_open_err, ctx);

        setDirect_(inst, "main_pos_relay_weld_fault",
                   fault2_ok && x->f2_main_pos_weld_err, ctx);

        setDirect_(inst, "main_neg_relay_open_fault",
                   fault2_ok && x->f2_main_neg_open_err, ctx);

        setDirect_(inst, "main_neg_relay_weld_fault",
                   fault2_ok && x->f2_main_neg_weld_err, ctx);

        setDirect_(inst, "dc_chrg_pos1_relay_open_fault",
                   fault2_ok && x->f2_dc_chrg_pos1_open_err, ctx);

        setDirect_(inst, "dc_chrg_pos1_relay_weld_fault",
                   fault2_ok && x->f2_dc_chrg_pos1_weld_err, ctx);

        setDirect_(inst, "dc_chrg_neg1_relay_open_fault",
                   fault2_ok && x->f2_dc_chrg_neg1_open_err, ctx);

        setDirect_(inst, "dc_chrg_neg1_relay_weld_fault",
                   fault2_ok && x->f2_dc_chrg_neg1_weld_err, ctx);

        setDirect_(inst, "dc_chrg_pos2_relay_open_fault",
                   fault2_ok && x->f2_dc_chrg_pos2_open_err, ctx);

        setDirect_(inst, "dc_chrg_pos2_relay_weld_fault",
                   fault2_ok && x->f2_dc_chrg_pos2_weld_err, ctx);

        setDirect_(inst, "dc_chrg_neg2_relay_open_fault",
                   fault2_ok && x->f2_dc_chrg_neg2_open_err, ctx);

        setDirect_(inst, "dc_chrg_neg2_relay_weld_fault",
                   fault2_ok && x->f2_dc_chrg_neg2_weld_err, ctx);

        setDirect_(inst, "ac_chrg_pos_relay_open_fault",
                   fault2_ok && x->f2_ac_chrg_pos_open_err, ctx);

        setDirect_(inst, "ac_chrg_pos_relay_weld_fault",
                   fault2_ok && x->f2_ac_chrg_pos_weld_err, ctx);

        setDirect_(inst, "ac_chrg_neg_relay_open_fault",
                   fault2_ok && x->f2_ac_chrg_neg_open_err, ctx);

        setDirect_(inst, "ac_chrg_neg_relay_weld_fault",
                   fault2_ok && x->f2_ac_chrg_neg_weld_err, ctx);

        setDirect_(inst, "panto_chrg_pos_relay_open_fault",
                   fault2_ok && x->f2_panto_chrg_pos_open_err, ctx);

        setDirect_(inst, "panto_chrg_pos_relay_weld_fault",
                   fault2_ok && x->f2_panto_chrg_pos_weld_err, ctx);

        setDirect_(inst, "panto_chrg_neg_relay_open_fault",
                   fault2_ok && x->f2_panto_chrg_neg_open_err, ctx);

        setDirect_(inst, "panto_chrg_neg_relay_weld_fault",
                   fault2_ok && x->f2_panto_chrg_neg_weld_err, ctx);

        setDirect_(inst, "charge_gun_connection_abnormal",
                   fault2_ok && x->f2_chrg_connect_err, ctx);

        setDirect_(inst, "charge_discharge_current_overflow",
                   fault2_ok && x->f2_over_dischrg_curr_when_in_chrg, ctx);

        setDirect_(inst, "charge_current_overflow_alarm",
                   fault2_ok && x->f2_over_chrg_curr_in_the_chrg, ctx);

        setDirect_(inst, "charge_connector_ntc_fault",
                   fault2_ok && x->f2_chrg_ntc_err, ctx);

        // 0x2044：当前 CSV 前70行未给出 MsgName / Signal Name，这一批先不直连
        setDirect_(inst, "internal_hv_circuit_open_fault", false, ctx);

        // LOG_THROTTLE_MS(("bms_eval_direct_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
        //                 "[FAULT][BMS][EVAL_DIRECT] inst=%u inst_online=%d "
        //                 "f1_online=%d f2_online=%d f1_stale=%d f2_stale=%d "
        //                 "f1{ov=%d uv=%d tov=%d tuv=%d ot=%d dt=%d soc_low=%d ins=%d soc_high=%d} "
        //                 "f2{od=%d lt=%d fire=%d pd_i=%d pc_i=%d curr=%d lv=%d soc_diff=%d tms=%d pre=%d}",
        //                 (unsigned)inst,
        //                 inst_online ? 1 : 0,
        //                 x->fault1_online ? 1 : 0,
        //                 x->fault2_online ? 1 : 0,
        //                 fault1_stale_dbg ? 1 : 0,
        //                 fault2_stale_dbg ? 1 : 0,
        //                 x->f1_over_ucell,
        //                 x->f1_low_ucell,
        //                 x->f1_pack_over_hvolt,
        //                 x->f1_pack_low_hvolt,
        //                 x->f1_over_temp,
        //                 x->f1_del_temp,
        //                 x->f1_low_soc,
        //                 x->f1_low_ins_res,
        //                 x->f1_over_soc ? 1 : 0,
        //                 x->f2_cell_over_dischrg ? 1 : 0,
        //                 x->f2_cell_low_temp ? 1 : 0,
        //                 x->f2_pack_fire_warning ? 1 : 0,
        //                 x->f2_over_dischrg_curr_level,
        //                 x->f2_over_chrg_curr_in_drive_level,
        //                 x->f2_curr_sensor_err ? 1 : 0,
        //                 x->f2_power_supply_err ? 1 : 0,
        //                 x->f2_soc_differ_err ? 1 : 0,
        //                 x->f2_tms_err ? 1 : 0,
        //                 x->f2_main_loop_prechg_err ? 1 : 0);
    }
} // namespace control::bms