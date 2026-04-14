//
// Created by lxy on 2026/3/10.
//

#ifndef ENERGYSTORAGE_BMS_LOGIC_TYPES_H
#define ENERGYSTORAGE_BMS_LOGIC_TYPES_H

#ifndef ENERGYSTORAGE_CONTROL_BMS_LOGIC_TYPES_H
#define ENERGYSTORAGE_CONTROL_BMS_LOGIC_TYPES_H

#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace control::bms
{
    /**
     * 单实例关键量缓存
     *
     * 设计原则：
     * 1. 先让 control 正式吃到 4 路 BMS
     * 2. 暂不把全量 parsed 层搬进 control
     * 3. 先保留“logic 必需 + HMI 概览必需”的关键字段
     * 4. 同时保留每组最后一帧 raw_hex，便于调试/过渡
     */
struct BmsPerInstanceCache {
    // ===== 实例基础信息 =====
    uint32_t bms_index{0};
    std::string instance_name;         // "BMS_1"
    std::string last_msg_name;         // "B2V_ST2"

    // 每个报文组最后一帧 raw_hex，供后续 debug / logic 过渡
    std::map<std::string, std::string> last_raw_hex_by_group;

    // ===== 实例级统一健康状态 =====
    bool seen_once{false};              // 是否至少收到过一次该实例报文
    bool online{false};                 // 运行态在线真源（第三批 Tick aging 统一重算）
    bool runtime_fault_stale{false};    // 关键运行态组是否有陈旧/缺失

    uint32_t disconnect_window_ms{3000};

    uint64_t last_rx_ms{0};             // 最近一次收到任意该实例有效报文的时间
    uint64_t last_ok_ms{0};             // 最近一次确认有效反馈时间（兼容旧逻辑）
    uint64_t last_online_change_ms{0};  // 最近一次在线状态发生变化的时间
    uint64_t last_offline_ms{0};        // 最近一次从在线 -> 离线的时间

    uint32_t disconnect_count{0};

    int offline_reason_code{0};         // 0=None 1=NoData 2=RxTimeout
    std::string offline_reason_text;    // "None" / "NoData" / "RxTimeout"

    // ===== 各报文组最近接收时间 =====
    std::map<std::string, uint64_t> last_group_rx_ms;

    uint64_t last_st1_ms{0};
    uint64_t last_st2_ms{0};
    uint64_t last_st3_ms{0};
    uint64_t last_st4_ms{0};
    uint64_t last_st5_ms{0};
    uint64_t last_st6_ms{0};
    uint64_t last_st7_ms{0};
    uint64_t last_elec_energy_ms{0};
    uint64_t last_current_limit_ms{0};
    uint64_t last_tm2b_ms{0};
    uint64_t last_fire2b_ms{0};
    uint64_t last_fault1_ms{0};
    uint64_t last_fault2_ms{0};

    // ===== 组级在线状态（第三批 Tick aging 统一重算）=====
    bool st1_online{false};
    bool st2_online{false};
    bool st3_online{false};
    bool st4_online{false};
    bool st5_online{false};
    bool st6_online{false};
    bool st7_online{false};

    bool elec_energy_online{false};
    bool current_limit_online{false};
    bool tm2b_online{false};
    bool fire2b_online{false};
    bool fault1_online{false};
    bool fault2_online{false};

    // ===== 组级年龄 =====
    double st1_age_ms{0.0};
    double st2_age_ms{0.0};
    double st3_age_ms{0.0};
    double st4_age_ms{0.0};
    double st5_age_ms{0.0};
    double st6_age_ms{0.0};
    double st7_age_ms{0.0};

    double elec_energy_age_ms{0.0};
    double current_limit_age_ms{0.0};
    double tm2b_age_ms{0.0};
    double fire2b_age_ms{0.0};
    double fault1_age_ms{0.0};
    double fault2_age_ms{0.0};

    // ===== B2V_ST1 =====
    int32_t st1_main_pos_relay_st{0};
    int32_t st1_main_neg_relay_st{0};
    int32_t st1_prechrg_relay_st{0};
    int32_t st1_dc_chrg_pos1_relay_st{0};
    int32_t st1_dc_chrg_neg1_relay_st{0};
    int32_t st1_dc_chrg_pos2_relay_st{0};
    int32_t st1_dc_chrg_neg2_relay_st{0};

    int32_t st1_heat_pos_relay_st{0};
    int32_t st1_heat_neg_relay_st{0};
    int32_t st1_panto_chrg_pos_relay_st{0};
    int32_t st1_panto_chrg_neg_relay_st{0};
    int32_t st1_ac_chrg_pos_relay_st{0};
    int32_t st1_ac_chrg_neg_relay_st{0};
    int32_t st1_aux1_relay_st{0};
    int32_t st1_aux2_relay_st{0};
    int32_t st1_aux3_relay_st{0};

    int32_t st1_bms_hv_status{0};
    int32_t st1_balance_status{0};

    int32_t st1_dc_chrg_connect_st{0};
    int32_t st1_panto_chrg_connect_st{0};
    int32_t st1_ac_chrg_connect_st{0};

    int32_t st1_chrg_mode{0};
    int32_t st1_chrg_status{0};
    int32_t st1_heating_status{0};
    int32_t st1_cooling_status{0};
    int32_t st1_rechrg_cycles{0};

    // ===== B2V_ST2 =====
    bool soc_valid{false};
    double soc{0.0};

    bool soh_valid{false};
    double soh{0.0};

    bool pack_v_valid{false};
    double pack_v{0.0};

    bool pack_i_valid{false};
    double pack_i{0};

    int32_t fault_level{0};
    int32_t fault_code{0};
    bool rq_hv_power_off{false};

    // ===== B2V_ST3 =====
    bool st3_ins_pos_valid{false};
    double st3_ins_pos_res{0.0};

    bool st3_ins_neg_valid{false};
    double st3_ins_neg_res{0.0};

    bool st3_ins_sys_valid{false};
    double st3_ins_sys_res{0.0};

    bool st3_ins_detector_valid{false};
    bool st3_ins_detector_on{false};

    // ===== B2V_ST4 =====
    bool st4_temp_max_valid{false};
    double st4_temp_max{0.0};

    bool st4_temp_min_valid{false};
    double st4_temp_min{0.0};

    bool st4_temp_avg_valid{false};
    double st4_temp_avg{0.0};

    bool st4_max_temp_pos_valid{false};
    int32_t st4_max_temp_pos{0};

    bool st4_min_temp_pos_valid{false};
    int32_t st4_min_temp_pos{0};

    // ===== B2V_ST5 =====
    bool st5_max_ucell_valid{false};
    double st5_max_ucell{0.0};

    bool st5_min_ucell_valid{false};
    double st5_min_ucell{0.0};

    bool st5_avg_ucell_valid{false};
    double st5_avg_ucell{0.0};

    // ===== B2V_ST6 =====
    int32_t st6_max_ucell_csc_no{0};
    int32_t st6_max_ucell_position{0};
    int32_t st6_min_ucell_csc_no{0};
    int32_t st6_min_ucell_position{0};

    // ===== B2V_ST7 =====
    bool st7_gun1_dc_pos_temp_valid{false};
    double st7_gun1_dc_pos_temp{0.0};

    bool st7_gun1_dc_neg_temp_valid{false};
    double st7_gun1_dc_neg_temp{0.0};

    bool st7_gun2_dc_pos_temp_valid{false};
    double st7_gun2_dc_pos_temp{0.0};

    bool st7_gun2_dc_neg_temp_valid{false};
    double st7_gun2_dc_neg_temp{0.0};

    bool st7_gun_ac_pos_temp_valid{false};
    double st7_gun_ac_pos_temp{0.0};

    bool st7_gun_ac_neg_temp_valid{false};
    double st7_gun_ac_neg_temp{0.0};

    // ===== B2V_ElecEnergy =====
    bool EE_tot_chg_energy_valid{false};
    double EE_tot_chg_energy{0.0};

    bool EE_tot_dischg_energy_valid{false};
    double EE_tot_dischg_energy{0.0};

    bool EE_single_chg_energy_valid{false};
    double EE_single_chg_energy{0.0};

    // ===== B2V_CurrentLimit =====
    bool CL_pulse_discharge_limit_valid{false};
    double CL_pulse_discharge_limit_a{0.0};

    bool CL_pulse_charge_limit_valid{false};
    double CL_pulse_charge_limit_a{0.0};

    bool CL_follow_charge_limit_valid{false};
    double CL_follow_charge_limit_a{0.0};

    bool CL_follow_discharge_limit_valid{false};
    double CL_follow_discharge_limit_a{0.0};

    // ===== 兼容旧字段（仍保留）=====
    bool charge_limit_valid{false};
    double charge_limit_a{0.0};

    bool discharge_limit_valid{false};
    double discharge_limit_a{0.0};

    // ===== TM2B_Info =====
    int32_t tms_work_state{0};
    int32_t tms_fault_level{0};

    // ===== Fire2B_Info =====
    int32_t fire_fault_level{0};
    int32_t fire_fault_code{0};

    // ===== B2V_Fault1 =====
    int32_t f1_del_temp{0};
    int32_t f1_over_temp{0};
    int32_t f1_over_ucell{0};
    int32_t f1_low_ucell{0};
    int32_t f1_low_ins_res{0};

    bool f1_ucell_uniformity{false};
    bool f1_over_chg{false};
    bool f1_over_soc{false};
    bool f1_soc_change_fast{false};
    bool f1_bat_sys_not_match{false};
    bool f1_hvil_fault{false};

    int32_t f1_fault_num{0};

    // ===== B2V_Fault2 =====
    bool f2_tms_err{false};
    bool f2_pack_self_protect{false};
    bool f2_main_loop_prechg_err{false};
    bool f2_aux_loop_prechg_err{false};
    bool f2_chrg_ins_low_err{false};

    bool f2_curr_sensor_err{false};
    bool f2_power_supply_err{false};

    bool f2_chrg_connect_err{false};
    bool f2_over_dischrg_curr_when_in_chrg{false};
    bool f2_over_chrg_curr_in_the_chrg{false};
    bool f2_chrg_ntc_err{false};
    // 第八批：原来按 bool 处理，现改为等级值
    // 0 = none
    // 1 = connector overtemp lvl1
    // 2 = connector overtemp lvl2
    // 其它值暂按 >=2 归到 lvl2
    int32_t f2_chrg_ntc_temp_over_level{0};

    bool f2_acan_lost{false};
    bool f2_inner_comm_err{false};
    bool f2_dcdc_err{false};
    bool f2_branch_break_err{false};

    bool f2_heat_relay_open_err{false};
    bool f2_heat_relay_weld_err{false};

    bool f2_main_pos_open_err{false};
    bool f2_main_pos_weld_err{false};
    bool f2_main_neg_open_err{false};
    bool f2_main_neg_weld_err{false};

    bool f2_dc_chrg_pos1_open_err{false};
    bool f2_dc_chrg_pos1_weld_err{false};
    bool f2_dc_chrg_neg1_open_err{false};
    bool f2_dc_chrg_neg1_weld_err{false};

    bool f2_dc_chrg_pos2_open_err{false};
    bool f2_dc_chrg_pos2_weld_err{false};
    bool f2_dc_chrg_neg2_open_err{false};
    bool f2_dc_chrg_neg2_weld_err{false};

    bool f2_ac_chrg_pos_open_err{false};
    bool f2_ac_chrg_pos_weld_err{false};
    bool f2_ac_chrg_neg_open_err{false};
    bool f2_ac_chrg_neg_weld_err{false};

    bool f2_panto_chrg_pos_open_err{false};
    bool f2_panto_chrg_pos_weld_err{false};
    bool f2_panto_chrg_neg_open_err{false};
    bool f2_panto_chrg_neg_weld_err{false};

    // ===== 实例级汇总控制量 =====
    bool alarm_any{false};

    // 供后续命令层/联动层继续扩展
    bool hv_allow_close{true};
    bool hv_should_open{false};

    // ===== 统一业务故障真源（第二批：从 F1/F2/TMS/Fire 归一化）=====
    // 说明：
    // 1) 这些字段不是 parser 直接输出，而是由 bms_logic_adapter.cpp 根据现有 F1/F2/TM2B/Fire2B 字段整理出来
    // 2) 后续 BmsFaultEvaluator / BmsFaultMapper 优先面向这些“业务真源”工作，
    //    而不是继续直接依赖 f1_* / f2_* 原始命名
    bool raw_fire_alarm{false};
    bool raw_current_sensor_fault{false};
    bool raw_low_voltage_supply_alarm{false};
    bool raw_soc_jump_alarm{false};
    bool raw_tms_unit_fault{false};

    bool raw_battery_self_protect_fault{false};
    bool raw_precharge_fault{false};
    bool raw_charge_insulation_low_alarm{false};

    bool raw_acan_comm_fault{false};
    bool raw_internal_comm_fault{false};
    bool raw_branch_circuit_open_fault{false};

    bool raw_hvil_alarm{false};
    bool raw_storage_mismatch_alarm{false};

    // 第五批：充电接口 / 电流 / 回路类业务真源
    bool raw_charge_gun_connection_abnormal{false};
    bool raw_charge_discharge_current_overflow{false};
    bool raw_charge_current_overflow_alarm{false};
    bool raw_charge_connector_ntc_fault{false};
    bool raw_charge_connector_overtemp_lvl1{false};
    bool raw_charge_connector_overtemp_lvl2{false};
    bool raw_internal_hv_circuit_open_fault{false};

    // 第四批：接触器/回路 open-weld 业务真源
    bool raw_heat_relay_open_fault{false};
    bool raw_heat_relay_weld_fault{false};

    bool raw_main_pos_relay_open_fault{false};
    bool raw_main_pos_relay_weld_fault{false};
    bool raw_main_neg_relay_open_fault{false};
    bool raw_main_neg_relay_weld_fault{false};

    bool raw_dc_chrg_pos1_relay_open_fault{false};
    bool raw_dc_chrg_pos1_relay_weld_fault{false};
    bool raw_dc_chrg_neg1_relay_open_fault{false};
    bool raw_dc_chrg_neg1_relay_weld_fault{false};

    bool raw_dc_chrg_pos2_relay_open_fault{false};
    bool raw_dc_chrg_pos2_relay_weld_fault{false};
    bool raw_dc_chrg_neg2_relay_open_fault{false};
    bool raw_dc_chrg_neg2_relay_weld_fault{false};

    bool raw_ac_chrg_pos_relay_open_fault{false};
    bool raw_ac_chrg_pos_relay_weld_fault{false};
    bool raw_ac_chrg_neg_relay_open_fault{false};
    bool raw_ac_chrg_neg_relay_weld_fault{false};

    bool raw_panto_chrg_pos_relay_open_fault{false};
    bool raw_panto_chrg_pos_relay_weld_fault{false};
    bool raw_panto_chrg_neg_relay_open_fault{false};
    bool raw_panto_chrg_neg_relay_weld_fault{false};
};

    /**
     * control 侧 BMS 总缓存
     */
    struct BmsLogicCache
    {
        std::map<std::string, BmsPerInstanceCache> items; // key: BMS_1~BMS_4

        void clear()
        {
            items.clear();
        }

        BmsPerInstanceCache& ensure(const std::string& instance_name, uint32_t index)
        {
            auto& x = items[instance_name];
            if (x.bms_index == 0 && index != 0) x.bms_index = index;
            if (x.instance_name.empty()) x.instance_name = instance_name;
            return x;
        }
    };
} // namespace control::bms

#endif // ENERGYSTORAGE_CONTROL_BMS_LOGIC_TYPES_H

#endif //ENERGYSTORAGE_BMS_LOGIC_TYPES_H
