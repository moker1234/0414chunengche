// services/control/bms/bms_fault_mapper.cpp
#include "bms_fault_mapper.h"

#include <string>

#include "../fault/fault_center.h"
#include "../logic/logic_context.h"
#include "../fault/fault_ids.h"
#include "bms_logic_types.h"
#include "logger.h"

namespace control::bms
{
    namespace
    {
        static const char* const kMappedBmsSignalsAudit[] = {
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
            "internal_hv_circuit_open_fault",
        };

        static constexpr size_t kMappedBmsSignalAuditCount =
            sizeof(kMappedBmsSignalsAudit) / sizeof(kMappedBmsSignalsAudit[0]);
    } // namespace

    // static bool runtimeOffline_(const BmsPerInstanceCache* x)
    // {
    //     // 没有实例缓存，按离线处理
    //     if (!x) return true;
    //
    //     // online 已经是统一健康真源，这里不再重复用 last_ok_ms/window 重算
    //     return !x->online;
    // }

    static std::string confirmedKey_(uint8_t inst, const char* signal)
    {
        return "BMS_" + std::to_string(static_cast<unsigned>(inst)) + "." +
            (signal ? signal : "unknown");
    }

    static bool confirmedOn_(const control::LogicContext& ctx,
                             uint8_t inst,
                             const char* signal)
    {
        const auto key = confirmedKey_(inst, signal);
        auto it = ctx.bms_confirmed_faults.signals.find(key);
        if (it == ctx.bms_confirmed_faults.signals.end()) return false;
        return it->second;
    }

    static uint16_t codeByInst_(uint8_t inst, uint16_t bms1_code)
    {
        if (inst < 1 || inst > 4) return bms1_code;
        return static_cast<uint16_t>(bms1_code + static_cast<uint16_t>((inst - 1) * 0x100u));
    }

    void BmsFaultMapper::applyToFaultPages(const BmsLogicCache& cache,
                                           const control::LogicContext& ctx,
                                           uint64_t now_ms,
                                           control::FaultCenter& faults) const
    {
        for (uint8_t inst = 1; inst <= 4; ++inst)
        {
            const std::string key = "BMS_" + std::to_string(inst);
            auto it = cache.items.find(key);
            const BmsPerInstanceCache* x =
                (it == cache.items.end()) ? nullptr : &it->second;
            applyOne_(inst, x, ctx, now_ms, faults);
        }
    }

    void BmsFaultMapper::applyOne_(uint8_t inst,
                                   const BmsPerInstanceCache* x,
                                   const control::LogicContext& ctx,
                                   uint64_t now_ms,
                                   control::FaultCenter& faults) const
    {
        (void)now_ms;

        // const bool offline = runtimeOffline_(x);
        //
        // const bool fault_block_hv =
        //     x && (
        //         x->hv_should_open ||
        //         x->f1_hvil_fault ||
        //         x->f1_over_chg ||
        //         (x->f1_low_ins_res >= 3) ||
        //         x->f2_pack_self_protect ||
        //         x->f2_main_loop_prechg_err ||
        //         x->f2_aux_loop_prechg_err ||
        //         x->f2_chrg_ins_low_err
        //     );
        //
        // const bool ins_low_any =
        //     x && (x->f1_low_ins_res != 0 || x->f2_chrg_ins_low_err);
        //
        // const bool runtime_fault_stale =
        //     x && x->runtime_fault_stale;
        //
        // const bool st2_stale =
        //     x && (x->last_st2_ms > 0) && !x->st2_online;
        //
        // const bool current_limit_stale =
        //     x && (x->last_current_limit_ms > 0) && !x->current_limit_online;
        //
        // const bool fault1_stale =
        //     x && (x->last_fault1_ms > 0) && !x->fault1_online;
        //
        // const bool fault2_stale =
        //     x && (x->last_fault2_ms > 0) && !x->fault2_online;

        // ---- summary/runtime ----
        // 第五批说明：
        // 1) BMS 内部故障统一由 BmsFaultEvaluator -> ctx.bms_confirmed_faults -> BmsFaultMapper 落码。
        // 2) BMS 通信故障 0x1001~0x1004 仍由 VCU/LOGIC 通信类链路处理，不在这里抢码。
        // 3) runtime stale / group stale 目前只作为命令阻断和 logic_view 诊断，不新增故障码。
        // 4) 暂不补 TMS / Fire 细颗粒覆盖，只保留当前已存在的 tms_unit_fault / fire_alarm 等聚合项。
        //
        // 如果后续 fault_ids.h / fault_map.jsonl 明确增加专用 stale 故障码，再打开：
        // faults.setActive(fault_ids::BMS_RUNTIME_STALE(inst), runtime_fault_stale);
        // faults.setActive(fault_ids::BMS_ST2_STALE(inst), st2_stale);
        // faults.setActive(fault_ids::BMS_CURRENT_LIMIT_STALE(inst), current_limit_stale);
        // faults.setActive(fault_ids::BMS_FAULT1_STALE(inst), fault1_stale);
        // faults.setActive(fault_ids::BMS_FAULT2_STALE(inst), fault2_stale);

        // (void)offline;
        // (void)fault_block_hv;
        // (void)ins_low_any;
        // (void)runtime_fault_stale;
        // (void)st2_stale;
        // (void)current_limit_stale;
        // (void)fault1_stale;
        // (void)fault2_stale;

        // ---- confirmed signals ----
        const bool c_cell_overvoltage_lvl1 = confirmedOn_(ctx, inst, "cell_overvoltage_lvl1");
        const bool c_cell_overvoltage_lvl2 = confirmedOn_(ctx, inst, "cell_overvoltage_lvl2");
        const bool c_cell_overvoltage_lvl3 = confirmedOn_(ctx, inst, "cell_overvoltage_lvl3");

        const bool c_cell_undervoltage_lvl1 = confirmedOn_(ctx, inst, "cell_undervoltage_lvl1");
        const bool c_cell_undervoltage_lvl2 = confirmedOn_(ctx, inst, "cell_undervoltage_lvl2");
        const bool c_cell_undervoltage_lvl3 = confirmedOn_(ctx, inst, "cell_undervoltage_lvl3");

        const bool c_cell_overdischarge_fault = confirmedOn_(ctx, inst, "cell_overdischarge_fault");

        const bool c_total_voltage_overvoltage_lvl1 =
            confirmedOn_(ctx, inst, "total_voltage_overvoltage_lvl1");
        const bool c_total_voltage_overvoltage_lvl2 =
            confirmedOn_(ctx, inst, "total_voltage_overvoltage_lvl2");
        const bool c_total_voltage_overvoltage_lvl3 =
            confirmedOn_(ctx, inst, "total_voltage_overvoltage_lvl3");

        const bool c_total_voltage_undervoltage_lvl1 =
            confirmedOn_(ctx, inst, "total_voltage_undervoltage_lvl1");
        const bool c_total_voltage_undervoltage_lvl2 =
            confirmedOn_(ctx, inst, "total_voltage_undervoltage_lvl2");
        const bool c_total_voltage_undervoltage_lvl3 =
            confirmedOn_(ctx, inst, "total_voltage_undervoltage_lvl3");

        const bool c_cell_overtemp_lvl1 = confirmedOn_(ctx, inst, "cell_overtemp_lvl1");
        const bool c_cell_overtemp_lvl2 = confirmedOn_(ctx, inst, "cell_overtemp_lvl2");
        const bool c_cell_overtemp_lvl3 = confirmedOn_(ctx, inst, "cell_overtemp_lvl3");
        const bool c_cell_lowtemp_alarm = confirmedOn_(ctx, inst, "cell_lowtemp_alarm");

        const bool c_temp_diff_over_lvl1 = confirmedOn_(ctx, inst, "temp_diff_over_lvl1");
        const bool c_temp_diff_over_lvl2 = confirmedOn_(ctx, inst, "temp_diff_over_lvl2");
        const bool c_temp_diff_over_lvl3 = confirmedOn_(ctx, inst, "temp_diff_over_lvl3");

        const bool c_fire_alarm = confirmedOn_(ctx, inst, "fire_alarm");

        const bool c_pulse_discharge_current_overflow_lvl1 =
            confirmedOn_(ctx, inst, "pulse_discharge_current_overflow_lvl1");
        const bool c_pulse_discharge_current_overflow_lvl2 =
            confirmedOn_(ctx, inst, "pulse_discharge_current_overflow_lvl2");
        const bool c_pulse_discharge_current_overflow_lvl3 =
            confirmedOn_(ctx, inst, "pulse_discharge_current_overflow_lvl3");

        const bool c_pulse_charge_current_overflow_lvl1 =
            confirmedOn_(ctx, inst, "pulse_charge_current_overflow_lvl1");
        const bool c_pulse_charge_current_overflow_lvl2 =
            confirmedOn_(ctx, inst, "pulse_charge_current_overflow_lvl2");
        const bool c_pulse_charge_current_overflow_lvl3 =
            confirmedOn_(ctx, inst, "pulse_charge_current_overflow_lvl3");

        const bool c_current_sensor_fault = confirmedOn_(ctx, inst, "current_sensor_fault");
        const bool c_low_voltage_supply_alarm = confirmedOn_(ctx, inst, "low_voltage_supply_alarm");
        const bool c_soc_imbalance_alarm = confirmedOn_(ctx, inst, "soc_imbalance_alarm");
        const bool c_soc_jump_alarm = confirmedOn_(ctx, inst, "soc_jump_alarm");

        const bool c_soc_low_lvl1 = confirmedOn_(ctx, inst, "soc_low_lvl1");
        const bool c_soc_low_lvl2 = confirmedOn_(ctx, inst, "soc_low_lvl2");
        const bool c_soc_low_lvl3 = confirmedOn_(ctx, inst, "soc_low_lvl3");

        const bool c_tms_unit_fault = confirmedOn_(ctx, inst, "tms_unit_fault");
        const bool c_tms_panel_aircon_fault = confirmedOn_(ctx, inst, "tms_panel_aircon_fault");
        const bool c_tms_ats_water_pump_fault = confirmedOn_(ctx, inst, "tms_ats_water_pump_fault");
        const bool c_battery_self_protect_fault = confirmedOn_(ctx, inst, "battery_self_protect_fault");
        const bool c_precharge_fault = confirmedOn_(ctx, inst, "precharge_fault");

        const bool c_driving_insulation_low_lvl1 = confirmedOn_(ctx, inst, "driving_insulation_low_lvl1");
        const bool c_driving_insulation_low_lvl2 = confirmedOn_(ctx, inst, "driving_insulation_low_lvl2");
        const bool c_driving_insulation_low_lvl3 = confirmedOn_(ctx, inst, "driving_insulation_low_lvl3");

        const bool c_charge_insulation_low_alarm = confirmedOn_(ctx, inst, "charge_insulation_low_alarm");
        const bool c_acan_comm_fault = confirmedOn_(ctx, inst, "acan_comm_fault");
        const bool c_internal_comm_fault = confirmedOn_(ctx, inst, "internal_comm_fault");
        const bool c_branch_circuit_open_fault = confirmedOn_(ctx, inst, "branch_circuit_open_fault");
        const bool c_hvil_alarm = confirmedOn_(ctx, inst, "hvil_alarm");

        const bool c_heat_relay_open_fault = confirmedOn_(ctx, inst, "heat_relay_open_fault");
        const bool c_heat_relay_weld_fault = confirmedOn_(ctx, inst, "heat_relay_weld_fault");

        const bool c_main_pos_relay_open_fault = confirmedOn_(ctx, inst, "main_pos_relay_open_fault");
        const bool c_main_pos_relay_weld_fault = confirmedOn_(ctx, inst, "main_pos_relay_weld_fault");
        const bool c_main_neg_relay_open_fault = confirmedOn_(ctx, inst, "main_neg_relay_open_fault");
        const bool c_main_neg_relay_weld_fault = confirmedOn_(ctx, inst, "main_neg_relay_weld_fault");

        const bool c_dc_chrg_pos1_relay_open_fault = confirmedOn_(ctx, inst, "dc_chrg_pos1_relay_open_fault");
        const bool c_dc_chrg_pos1_relay_weld_fault = confirmedOn_(ctx, inst, "dc_chrg_pos1_relay_weld_fault");
        const bool c_dc_chrg_neg1_relay_open_fault = confirmedOn_(ctx, inst, "dc_chrg_neg1_relay_open_fault");
        const bool c_dc_chrg_neg1_relay_weld_fault = confirmedOn_(ctx, inst, "dc_chrg_neg1_relay_weld_fault");

        const bool c_dc_chrg_pos2_relay_open_fault = confirmedOn_(ctx, inst, "dc_chrg_pos2_relay_open_fault");
        const bool c_dc_chrg_pos2_relay_weld_fault = confirmedOn_(ctx, inst, "dc_chrg_pos2_relay_weld_fault");
        const bool c_dc_chrg_neg2_relay_open_fault = confirmedOn_(ctx, inst, "dc_chrg_neg2_relay_open_fault");
        const bool c_dc_chrg_neg2_relay_weld_fault = confirmedOn_(ctx, inst, "dc_chrg_neg2_relay_weld_fault");

        const bool c_charge_gun_connection_abnormal =
            confirmedOn_(ctx, inst, "charge_gun_connection_abnormal");
        const bool c_storage_mismatch_alarm =
            confirmedOn_(ctx, inst, "storage_mismatch_alarm");
        const bool c_charge_discharge_current_overflow =
            confirmedOn_(ctx, inst, "charge_discharge_current_overflow");
        const bool c_charge_current_overflow_alarm =
            confirmedOn_(ctx, inst, "charge_current_overflow_alarm");
        const bool c_charge_connector_ntc_fault =
            confirmedOn_(ctx, inst, "charge_connector_ntc_fault");
        const bool c_charge_connector_overtemp_lvl1 =
            confirmedOn_(ctx, inst, "charge_connector_overtemp_lvl1");
        const bool c_charge_connector_overtemp_lvl2 =
            confirmedOn_(ctx, inst, "charge_connector_overtemp_lvl2");
        const bool c_cell_consistency_poor_alarm =
            confirmedOn_(ctx, inst, "cell_consistency_poor_alarm");
        const bool c_soc_high_alarm =
            confirmedOn_(ctx, inst, "soc_high_alarm");
        const bool c_internal_hv_circuit_open_fault =
            confirmedOn_(ctx, inst, "internal_hv_circuit_open_fault");

        auto set_bms_code = [&](uint16_t bms1_code, bool on)
        {
            faults.setActive(codeByInst_(inst, bms1_code), on);
        };

        // 0x2000 ~ 0x2006
        set_bms_code(0x2000, c_cell_overvoltage_lvl1);
        set_bms_code(0x2001, c_cell_overvoltage_lvl2);
        set_bms_code(0x2002, c_cell_overvoltage_lvl3);
        set_bms_code(0x2003, c_cell_undervoltage_lvl1);
        set_bms_code(0x2004, c_cell_undervoltage_lvl2);
        set_bms_code(0x2005, c_cell_undervoltage_lvl3);
        set_bms_code(0x2006, c_cell_overdischarge_fault);

        // 0x2007 ~ 0x200C
        set_bms_code(0x2007, c_total_voltage_overvoltage_lvl1);
        set_bms_code(0x2008, c_total_voltage_overvoltage_lvl2);
        set_bms_code(0x2009, c_total_voltage_overvoltage_lvl3);
        set_bms_code(0x200A, c_total_voltage_undervoltage_lvl1);
        set_bms_code(0x200B, c_total_voltage_undervoltage_lvl2);
        set_bms_code(0x200C, c_total_voltage_undervoltage_lvl3);

        // 0x200D ~ 0x2013
        set_bms_code(0x200D, c_cell_overtemp_lvl1);
        set_bms_code(0x200E, c_cell_overtemp_lvl2);
        set_bms_code(0x200F, c_cell_overtemp_lvl3);
        set_bms_code(0x2010, c_cell_lowtemp_alarm);
        set_bms_code(0x2011, c_temp_diff_over_lvl1);
        set_bms_code(0x2012, c_temp_diff_over_lvl2);
        set_bms_code(0x2013, c_temp_diff_over_lvl3);

        // 0x2014 ~ 0x2021
        set_bms_code(0x2014, c_fire_alarm);
        set_bms_code(0x2015, c_pulse_discharge_current_overflow_lvl1);
        set_bms_code(0x2016, c_pulse_discharge_current_overflow_lvl2);
        set_bms_code(0x2017, c_pulse_discharge_current_overflow_lvl3);
        set_bms_code(0x2018, c_pulse_charge_current_overflow_lvl1);
        set_bms_code(0x2019, c_pulse_charge_current_overflow_lvl2);
        set_bms_code(0x201A, c_pulse_charge_current_overflow_lvl3);
        set_bms_code(0x201B, c_current_sensor_fault);
        set_bms_code(0x201C, c_low_voltage_supply_alarm);
        set_bms_code(0x201D, c_soc_imbalance_alarm);
        set_bms_code(0x201E, c_soc_jump_alarm);
        set_bms_code(0x201F, c_soc_low_lvl1);
        set_bms_code(0x2020, c_soc_low_lvl2);
        set_bms_code(0x2021, c_soc_low_lvl3);

        // 0x2022 ~ 0x202C
        set_bms_code(0x2022, c_tms_unit_fault);
        set_bms_code(0x2023, c_battery_self_protect_fault);
        set_bms_code(0x2024, c_precharge_fault);
        set_bms_code(0x2025, c_driving_insulation_low_lvl1);
        set_bms_code(0x2026, c_driving_insulation_low_lvl2);
        set_bms_code(0x2027, c_driving_insulation_low_lvl3);
        set_bms_code(0x2028, c_charge_insulation_low_alarm);
        set_bms_code(0x2029, c_acan_comm_fault);
        set_bms_code(0x202A, c_internal_comm_fault);
        set_bms_code(0x202B, c_branch_circuit_open_fault);
        set_bms_code(0x202C, c_hvil_alarm);

        // 0x202D ~ 0x2044
        set_bms_code(0x202D, c_heat_relay_open_fault);
        set_bms_code(0x202E, c_heat_relay_weld_fault);
        set_bms_code(0x202F, c_main_pos_relay_open_fault);
        set_bms_code(0x2030, c_main_pos_relay_weld_fault);
        set_bms_code(0x2031, c_main_neg_relay_open_fault);
        set_bms_code(0x2032, c_main_neg_relay_weld_fault);
        set_bms_code(0x2033, c_dc_chrg_pos1_relay_open_fault);
        set_bms_code(0x2034, c_dc_chrg_pos1_relay_weld_fault);
        set_bms_code(0x2035, c_dc_chrg_neg1_relay_open_fault);
        set_bms_code(0x2036, c_dc_chrg_neg1_relay_weld_fault);
        set_bms_code(0x2037, c_dc_chrg_pos2_relay_open_fault);
        set_bms_code(0x2038, c_dc_chrg_pos2_relay_weld_fault);
        set_bms_code(0x2039, c_dc_chrg_neg2_relay_open_fault);
        set_bms_code(0x203A, c_dc_chrg_neg2_relay_weld_fault);
        set_bms_code(0x203B, c_charge_gun_connection_abnormal);
        set_bms_code(0x203C, c_storage_mismatch_alarm);
        set_bms_code(0x203D, c_charge_discharge_current_overflow);
        set_bms_code(0x203E, c_charge_current_overflow_alarm);
        set_bms_code(0x203F, c_charge_connector_ntc_fault);
        set_bms_code(0x2040, c_charge_connector_overtemp_lvl1);
        set_bms_code(0x2041, c_charge_connector_overtemp_lvl2);
        set_bms_code(0x2042, c_cell_consistency_poor_alarm);
        set_bms_code(0x2043, c_soc_high_alarm);
        set_bms_code(0x2044, c_internal_hv_circuit_open_fault);

        // ===== TMS 扩展落码（按 fault_map 既定 code 走，不改 json 文本）=====
        // 0x2082 : BMS1-热管理面板空调故障
        // 0x208B : BMS1-热管理ATS 水泵故障
        set_bms_code(0x2082, c_tms_panel_aircon_fault);
        set_bms_code(0x208B, c_tms_ats_water_pump_fault);

        LOG_THROTTLE_MS(("bms_mapper_direct_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
                        "[FAULT][BMS][MAP_DIRECT] inst=%u "
                        "ov={%d,%d,%d} uv={%d,%d,%d} tov={%d,%d,%d} tuv={%d,%d,%d} "
                        "ot={%d,%d,%d} lt=%d dt={%d,%d,%d} fire=%d "
                        "pd_i={%d,%d,%d} pc_i={%d,%d,%d} curr=%d lv=%d soc_diff=%d soc_jump=%d "
                        "soc_low={%d,%d,%d} soc_high=%d pre=%d ins={%d,%d,%d} chg_ins=%d "
                        "gun=%d storage=%d ntc=%d ntc_ot={%d,%d} consistency=%d inner_hv=%d",
                        (unsigned)inst,
                        c_cell_overvoltage_lvl1 ? 1 : 0,
                        c_cell_overvoltage_lvl2 ? 1 : 0,
                        c_cell_overvoltage_lvl3 ? 1 : 0,
                        c_cell_undervoltage_lvl1 ? 1 : 0,
                        c_cell_undervoltage_lvl2 ? 1 : 0,
                        c_cell_undervoltage_lvl3 ? 1 : 0,
                        c_total_voltage_overvoltage_lvl1 ? 1 : 0,
                        c_total_voltage_overvoltage_lvl2 ? 1 : 0,
                        c_total_voltage_overvoltage_lvl3 ? 1 : 0,
                        c_total_voltage_undervoltage_lvl1 ? 1 : 0,
                        c_total_voltage_undervoltage_lvl2 ? 1 : 0,
                        c_total_voltage_undervoltage_lvl3 ? 1 : 0,
                        c_cell_overtemp_lvl1 ? 1 : 0,
                        c_cell_overtemp_lvl2 ? 1 : 0,
                        c_cell_overtemp_lvl3 ? 1 : 0,
                        c_cell_lowtemp_alarm ? 1 : 0,
                        c_temp_diff_over_lvl1 ? 1 : 0,
                        c_temp_diff_over_lvl2 ? 1 : 0,
                        c_temp_diff_over_lvl3 ? 1 : 0,
                        c_fire_alarm ? 1 : 0,
                        c_pulse_discharge_current_overflow_lvl1 ? 1 : 0,
                        c_pulse_discharge_current_overflow_lvl2 ? 1 : 0,
                        c_pulse_discharge_current_overflow_lvl3 ? 1 : 0,
                        c_pulse_charge_current_overflow_lvl1 ? 1 : 0,
                        c_pulse_charge_current_overflow_lvl2 ? 1 : 0,
                        c_pulse_charge_current_overflow_lvl3 ? 1 : 0,
                        c_current_sensor_fault ? 1 : 0,
                        c_low_voltage_supply_alarm ? 1 : 0,
                        c_soc_imbalance_alarm ? 1 : 0,
                        c_soc_jump_alarm ? 1 : 0,
                        c_soc_low_lvl1 ? 1 : 0,
                        c_soc_low_lvl2 ? 1 : 0,
                        c_soc_low_lvl3 ? 1 : 0,
                        c_soc_high_alarm ? 1 : 0,
                        c_precharge_fault ? 1 : 0,
                        c_driving_insulation_low_lvl1 ? 1 : 0,
                        c_driving_insulation_low_lvl2 ? 1 : 0,
                        c_driving_insulation_low_lvl3 ? 1 : 0,
                        c_charge_insulation_low_alarm ? 1 : 0,
                        c_charge_gun_connection_abnormal ? 1 : 0,
                        c_storage_mismatch_alarm ? 1 : 0,
                        c_charge_connector_ntc_fault ? 1 : 0,
                        c_charge_connector_overtemp_lvl1 ? 1 : 0,
                        c_charge_connector_overtemp_lvl2 ? 1 : 0,
                        c_cell_consistency_poor_alarm ? 1 : 0,
                        c_internal_hv_circuit_open_fault ? 1 : 0);
    }
} // namespace control::bms
