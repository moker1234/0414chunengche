//
// Created by lxy on 2026/4/12.
//

#include "fault_logic_evaluator.h"

#include <string>

#include "../logic/logic_context.h"
#include "fault_condition_engine.h"
#include "logger.h"

namespace control::fault {

namespace {

inline void updateConfirmedSimple_(control::LogicContext& ctx,
                                   uint64_t now_ms,
                                   const std::string& key,
                                   bool raw_now,
                                   uint32_t assert_ms,
                                   uint32_t clear_ms)
{
    ctx.fault_cond_engine.updateCondition(
        key,
        raw_now,
        now_ms,
        assert_ms,
        clear_ms
    );
}

inline void updateConfirmedWithClear_(control::LogicContext& ctx,
                                      uint64_t now_ms,
                                      const std::string& key,
                                      bool assert_now,
                                      bool clear_now,
                                      uint32_t assert_ms,
                                      uint32_t clear_ms)
{
    ctx.fault_cond_engine.updateConditionWithClear(
        key,
        assert_now,
        clear_now,
        now_ms,
        assert_ms,
        clear_ms
    );
}

} // namespace

std::string FaultLogicEvaluator::makeSignalKey_(const char* signal)
{
    return std::string("logic.") + (signal ? signal : "unknown");
}

void FaultLogicEvaluator::setConfirmed_(const char* signal,
                                        bool on,
                                        control::LogicContext& ctx)
{
    const std::string key = makeSignalKey_(signal);

    bool old = false;
    auto it_old = ctx.confirmed_faults.signals.find(key);
    if (it_old != ctx.confirmed_faults.signals.end()) {
        old = it_old->second;
    }

    ctx.confirmed_faults.signals[key] = on;

    if (old != on) {
        LOGINFO("[FAULT][LOGIC][CONFIRMED] key=%s value=%d",
                key.c_str(),
                on ? 1 : 0);
    }
}

bool FaultLogicEvaluator::rawPcu0Offline_(const control::LogicContext& ctx)
{
    return !ctx.pcu0_state.online;
}

bool FaultLogicEvaluator::rawPcu1Offline_(const control::LogicContext& ctx)
{
    return !ctx.pcu1_state.online;
}

bool FaultLogicEvaluator::rawUpsOffline_(const control::LogicContext& ctx)
{
    return !ctx.ups_faults.online;
}

bool FaultLogicEvaluator::rawUpsFault_(const control::LogicContext& ctx)
{
    return ctx.ups_faults.fault_any;
}

bool FaultLogicEvaluator::rawSmokeOffline_(const control::LogicContext& ctx)
{
    return !ctx.smoke_faults.online;
}

bool FaultLogicEvaluator::rawSmokeAlarm_(const control::LogicContext& ctx)
{
    return ctx.smoke_faults.alarm_any;
}

bool FaultLogicEvaluator::rawGasOffline_(const control::LogicContext& ctx)
{
    return !ctx.gas_faults.online;
}

bool FaultLogicEvaluator::rawGasAlarm_(const control::LogicContext& ctx)
{
    return ctx.gas_faults.alarm_any;
}

bool FaultLogicEvaluator::rawAirOffline_(const control::LogicContext& ctx)
{
    return !ctx.air_faults.online;
}

bool FaultLogicEvaluator::rawEnvAnyAlarm_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.env_any_alarm;
}

bool FaultLogicEvaluator::rawAnyFault_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.any_fault;
}

    bool FaultLogicEvaluator::rawHmiCommFault_(const control::LogicContext& ctx, uint64_t now_ms)
{
    // 第八批：弱在线真源
    //
    // 规则：
    // 1) 未见过 HMI 写入之前，不主动报 hmi_comm_fault，避免开机即误报码。
    // 2) 见过 HMI 写入后，如果超过 hmi_comm_timeout_ms 仍无新的 HMI 写入，
    //    则视为 raw hmi_comm_fault。
    // 3) 如果其他链路后续已经显式把 logic_faults.hmi_comm_fault 置位，也兼容保留。
    if (ctx.logic_faults.hmi_comm_fault) {
        return true;
    }

    if (!ctx.hmi_seen_once) {
        return false;
    }

    if (ctx.last_hmi_write_ts == 0) {
        return false;
    }

    if (now_ms < ctx.last_hmi_write_ts) {
        return false;
    }

    return (now_ms - ctx.last_hmi_write_ts) > ctx.hmi_comm_timeout_ms;
}

    bool FaultLogicEvaluator::rawRemoteCommFault_(const control::LogicContext& ctx, uint64_t now_ms)
{
    // 第九批：remote 弱在线真源（预留接口版）
    // 规则：
    // 1) 如果其他模块已经显式把 logic_faults.remote_comm_fault 置位，则直接认为 raw fault；
    // 2) 如果还从未见过 remote 数据，不主动报码，避免“功能未接入时开机即报码”；
    // 3) 一旦见过 remote 数据，超过 remote_comm_timeout_ms 没再收到，则视为 raw remote_comm_fault。
    if (ctx.logic_faults.remote_comm_fault) {
        return true;
    }

    if (!ctx.remote_seen_once) {
        return false;
    }

    if (ctx.last_remote_rx_ts == 0) {
        return false;
    }

    if (now_ms < ctx.last_remote_rx_ts) {
        return false;
    }

    return (now_ms - ctx.last_remote_rx_ts) > ctx.remote_comm_timeout_ms;
}

    bool FaultLogicEvaluator::rawSdcardFault_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.sdcard_fault;
}

void FaultLogicEvaluator::evaluatePcu_(control::LogicContext& ctx, uint64_t now_ms) const
{
    // PCU0 offline: 持续 3s 触发，恢复即时
    {
        const char* signal = "pcu0_offline";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx,
            now_ms,
            key,
            rawPcu0Offline_(ctx),
            !rawPcu0Offline_(ctx),
            3000,
            0
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // PCU1 offline: 持续 3s 触发，恢复即时
    {
        const char* signal = "pcu1_offline";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx,
            now_ms,
            key,
            rawPcu1Offline_(ctx),
            !rawPcu1Offline_(ctx),
            3000,
            0
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
}

void FaultLogicEvaluator::evaluateUps_(control::LogicContext& ctx, uint64_t now_ms) const
{
    const auto& u = ctx.ups_faults;

    // UPS 通信故障 (离线)
    {
        const char* signal = "ups_offline";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawUpsOffline_(ctx),
            !rawUpsOffline_(ctx),
            3000, 0
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // UPS 总故障
    {
        const char* signal = "ups_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawUpsFault_(ctx),
            !rawUpsFault_(ctx),
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // 使用 lambda 简化大量防抖逻辑的编写
    auto eval_ups = [&](const char* signal, bool raw_now) {
        const std::string key = makeSignalKey_(signal);
        // 统一使用: 触发需持续 3000ms，恢复需持续 1000ms 的防抖策略
        updateConfirmedWithClear_(ctx, now_ms, key, raw_now, !raw_now, 3000, 1000);
        // 20260415
        bool is_confirmed = ctx.fault_cond_engine.getConfirmed(key);
        if (
            // std::string(signal) == "ups_bus_overvoltage_fault" ||
            std::string(signal) == "ups_epo_critical_fault") {
            // 每秒打印一次，防止刷屏
            LOG_THROTTLE_MS("test_ups_debounce", 1000, LOGINFO,
                "[DEBUG_UPS_DEBOUNCE] signal: %s | raw_in: %d | confirmed_out: %d",
                signal, raw_now, is_confirmed);
        }

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    };

    // ---- Q1/WA 基础状态位 ----
    eval_ups("ups_mains_abnormal", u.mains_abnormal);
    eval_ups("ups_battery_low_state", u.battery_low_state || (u.battery_low != 0));
    eval_ups("ups_bypass_mode", u.bypass_mode || (u.bypass_active != 0));
    eval_ups("ups_ups_fault_state", u.ups_fault_state);
    eval_ups("ups_backup_mode", u.backup_mode);
    eval_ups("ups_self_test_active", u.self_test_active);

    // ---- 32 个 Warning Bits (警告位) ----
    eval_ups("ups_internal_warning", u.internal_warning);
    eval_ups("ups_epo_active", u.epo_active);
    eval_ups("ups_module_unlock", u.module_unlock);
    // u.mains_abnormal (Bit3) 已经在上面的基础状态中合并处理，这里无需重复 eval
    eval_ups("ups_neutral_lost", u.neutral_lost);
    eval_ups("ups_mains_phase_error", u.mains_phase_error);
    eval_ups("ups_ln_reverse", u.ln_reverse);
    eval_ups("ups_bypass_abnormal", u.bypass_abnormal);
    eval_ups("ups_bypass_phase_error", u.bypass_phase_error);
    eval_ups("ups_battery_not_connected", u.battery_not_connected);
    eval_ups("ups_battery_low_warning", u.battery_low_warning);
    eval_ups("ups_battery_overcharge", u.battery_overcharge);
    eval_ups("ups_battery_reverse", u.battery_reverse);
    eval_ups("ups_overload_warning", u.overload_warning);
    eval_ups("ups_overload_alarm", u.overload_alarm);
    eval_ups("ups_fan_fault", u.fan_fault);
    eval_ups("ups_bypass_cover_open", u.bypass_cover_open);
    eval_ups("ups_charger_fault", u.charger_fault);
    eval_ups("ups_position_error", u.position_error);
    eval_ups("ups_boot_condition_not_met", u.boot_condition_not_met);
    eval_ups("ups_redundancy_lost", u.redundancy_lost);
    eval_ups("ups_module_loose", u.module_loose);
    eval_ups("ups_battery_maint_due", u.battery_maint_due);
    eval_ups("ups_inspection_maint_due", u.inspection_maint_due);
    eval_ups("ups_warranty_maint_due", u.warranty_maint_due);
    eval_ups("ups_temp_low_warning", u.temp_low_warning);
    eval_ups("ups_temp_high_warning", u.temp_high_warning);
    eval_ups("ups_battery_overtemp", u.battery_overtemp);
    eval_ups("ups_fan_maint_due", u.fan_maint_due);
    eval_ups("ups_bus_cap_maint_due", u.bus_cap_maint_due);
    eval_ups("ups_system_overload", u.system_overload);
    eval_ups("ups_reserved_warning", u.reserved_warning);

    // ---- 60 个 Fault Codes (故障) ----
    eval_ups("ups_bus_overvoltage_fault", u.bus_overvoltage_fault);
    eval_ups("ups_bus_undervoltage_fault", u.bus_undervoltage_fault);
    eval_ups("ups_bus_imbalance_fault", u.bus_imbalance_fault);
    eval_ups("ups_bus_short_circuit", u.bus_short_circuit);
    eval_ups("ups_inv_softstart_timeout", u.inv_softstart_timeout);
    eval_ups("ups_inv_overvoltage_fault", u.inv_overvoltage_fault);
    eval_ups("ups_inv_undervoltage_fault", u.inv_undervoltage_fault);
    eval_ups("ups_output_short_circuit", u.output_short_circuit);
    eval_ups("ups_r_inv_short_circuit", u.r_inv_short_circuit);
    eval_ups("ups_s_inv_short_circuit", u.s_inv_short_circuit);
    eval_ups("ups_t_inv_short_circuit", u.t_inv_short_circuit);
    eval_ups("ups_rs_short_circuit", u.rs_short_circuit);
    eval_ups("ups_st_short_circuit", u.st_short_circuit);
    eval_ups("ups_tr_short_circuit", u.tr_short_circuit);
    eval_ups("ups_reverse_power_fault", u.reverse_power_fault);
    eval_ups("ups_r_reverse_power_fault", u.r_reverse_power_fault);
    eval_ups("ups_s_reverse_power_fault", u.s_reverse_power_fault);
    eval_ups("ups_t_reverse_power_fault", u.t_reverse_power_fault);
    eval_ups("ups_total_reverse_power_fault", u.total_reverse_power_fault);
    eval_ups("ups_current_imbalance_fault", u.current_imbalance_fault);
    eval_ups("ups_overload_fault", u.overload_fault);
    eval_ups("ups_overtemp_fault", u.overtemp_fault);
    eval_ups("ups_inv_relay_fail_close", u.inv_relay_fail_close);
    eval_ups("ups_inv_relay_stuck", u.inv_relay_stuck);
    eval_ups("ups_mains_scr_fault", u.mains_scr_fault);
    eval_ups("ups_battery_scr_fault", u.battery_scr_fault);
    eval_ups("ups_bypass_scr_fault", u.bypass_scr_fault);
    eval_ups("ups_rectifier_fault", u.rectifier_fault);
    eval_ups("ups_input_overcurrent_fault", u.input_overcurrent_fault);
    eval_ups("ups_wiring_error", u.wiring_error);
    eval_ups("ups_comm_cable_disconnected", u.comm_cable_disconnected);
    eval_ups("ups_host_cable_fault", u.host_cable_fault);
    eval_ups("ups_can_comm_fault", u.can_comm_fault);
    eval_ups("ups_sync_signal_fault", u.sync_signal_fault);
    eval_ups("ups_power_supply_fault", u.power_supply_fault);
    eval_ups("ups_all_fan_fault", u.all_fan_fault);
    eval_ups("ups_dsp_error", u.dsp_error);
    eval_ups("ups_charger_softstart_timeout", u.charger_softstart_timeout);
    eval_ups("ups_all_module_fault", u.all_module_fault);
    eval_ups("ups_mains_ntc_open_fault", u.mains_ntc_open_fault);
    eval_ups("ups_mains_fuse_open_fault", u.mains_fuse_open_fault);
    eval_ups("ups_output_imbalance_fault", u.output_imbalance_fault);
    eval_ups("ups_input_mismatch_fault", u.input_mismatch_fault);
    eval_ups("ups_eeprom_data_lost", u.eeprom_data_lost);
    eval_ups("ups_mains_support_failed", u.mains_support_failed);
    eval_ups("ups_power_failed", u.power_failed);
    eval_ups("ups_system_overload_fault", u.system_overload_fault);
    eval_ups("ups_ads7869_error", u.ads7869_error);
    eval_ups("ups_bypass_mode_no_op", u.bypass_mode_no_op);
    eval_ups("ups_op_breaker_off_parallel", u.op_breaker_off_parallel);
    eval_ups("ups_r_bus_fuse_fault", u.r_bus_fuse_fault);
    eval_ups("ups_s_bus_fuse_fault", u.s_bus_fuse_fault);
    eval_ups("ups_t_bus_fuse_fault", u.t_bus_fuse_fault);
    eval_ups("ups_ntc_fault", u.ntc_fault);
    eval_ups("ups_parallel_cable_fault", u.parallel_cable_fault);
    eval_ups("ups_battery_fault", u.battery_fault);
    eval_ups("ups_frequent_overcurrent_fault", u.frequent_overcurrent_fault);
    eval_ups("ups_battery_overcharge_fault", u.battery_overcharge_fault);
    eval_ups("ups_battery_overcharge_persist", u.battery_overcharge_persist);
    eval_ups("ups_epo_critical_fault", u.epo_critical_fault);
}

void FaultLogicEvaluator::evaluateSmoke_(control::LogicContext& ctx, uint64_t now_ms) const
{
    const auto& s = ctx.smoke_faults;

    {
        const char* signal = "smoke_offline";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, rawSmokeOffline_(ctx), !rawSmokeOffline_(ctx), 3000, 0);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "smoke_alarm";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, rawSmokeAlarm_(ctx), !rawSmokeAlarm_(ctx), 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "smoke_smoke_sensor_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, s.smoke_sensor_fault, !s.smoke_sensor_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "smoke_pollution_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, s.smoke_pollution_fault, !s.smoke_pollution_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "smoke_temp_sensor_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, s.temp_sensor_fault, !s.temp_sensor_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
}

void FaultLogicEvaluator::evaluateGas_(control::LogicContext& ctx, uint64_t now_ms) const
{
    const auto& g = ctx.gas_faults;

    {
        const char* signal = "gas_offline";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, rawGasOffline_(ctx), !rawGasOffline_(ctx), 3000, 0);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "gas_alarm";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, rawGasAlarm_(ctx), !rawGasAlarm_(ctx), 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "gas_sensor_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, g.sensor_fault, !g.sensor_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "gas_low_alarm";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, g.low_alarm, !g.low_alarm, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "gas_high_alarm";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, g.high_alarm, !g.high_alarm, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
}

void FaultLogicEvaluator::evaluateAir_(control::LogicContext& ctx, uint64_t now_ms) const
{
    const auto& a = ctx.air_faults;

    {
        const char* signal = "air_offline";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, rawAirOffline_(ctx), !rawAirOffline_(ctx), 3000, 0);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    auto eval_air = [&](const char* signal, bool raw_now) {
        LOG_THROTTLE_MS("air_eval_raw", 1000, LOGINFO, // 20260414
    "[FAULT][AIR][EVAL] online=%d low_humi=%d coil_freeze=%d exhaust_high=%d",
    ctx.air_faults.online ? 1 : 0,
    ctx.air_faults.low_humidity_alarm ? 1 : 0,
    ctx.air_faults.coil_freeze_protect ? 1 : 0,
    ctx.air_faults.exhaust_high_temp_alarm ? 1 : 0);
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, raw_now, !raw_now, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    };

    eval_air("air_high_temp_alarm", a.high_temp_alarm);
    eval_air("air_low_temp_alarm", a.low_temp_alarm);
    eval_air("air_high_humidity_alarm", a.high_humidity_alarm);
    eval_air("air_low_humidity_alarm", a.low_humidity_alarm);
    eval_air("air_coil_freeze_protect", a.coil_freeze_protect);
    eval_air("air_exhaust_high_temp_alarm", a.exhaust_high_temp_alarm);

    eval_air("air_coil_temp_sensor_fault", a.coil_temp_sensor_fault);
    eval_air("air_outdoor_temp_sensor_fault", a.outdoor_temp_sensor_fault);
    eval_air("air_condenser_temp_sensor_fault", a.condenser_temp_sensor_fault);
    eval_air("air_indoor_temp_sensor_fault", a.indoor_temp_sensor_fault);
    eval_air("air_exhaust_temp_sensor_fault", a.exhaust_temp_sensor_fault);
    eval_air("air_humidity_sensor_fault", a.humidity_sensor_fault);

    eval_air("air_internal_fan_fault", a.internal_fan_fault);
    eval_air("air_external_fan_fault", a.external_fan_fault);
    eval_air("air_compressor_fault", a.compressor_fault);
    eval_air("air_heater_fault", a.heater_fault);
    eval_air("air_emergency_fan_fault", a.emergency_fan_fault);

    eval_air("air_high_pressure_alarm", a.high_pressure_alarm);
    eval_air("air_low_pressure_alarm", a.low_pressure_alarm);
    eval_air("air_water_alarm", a.water_alarm);
    eval_air("air_smoke_alarm", a.smoke_alarm);
    eval_air("air_gating_alarm", a.gating_alarm);

    eval_air("air_high_pressure_lock", a.high_pressure_lock);
    eval_air("air_low_pressure_lock", a.low_pressure_lock);
    eval_air("air_exhaust_lock", a.exhaust_lock);

    eval_air("air_ac_over_voltage_alarm", a.ac_over_voltage_alarm);
    eval_air("air_ac_under_voltage_alarm", a.ac_under_voltage_alarm);
    eval_air("air_ac_power_loss", a.ac_power_loss);
    eval_air("air_lose_phase_alarm", a.lose_phase_alarm);
    eval_air("air_freq_fault", a.freq_fault);
    eval_air("air_anti_phase_alarm", a.anti_phase_alarm);
    eval_air("air_dc_over_voltage_alarm", a.dc_over_voltage_alarm);
    eval_air("air_dc_under_voltage_alarm", a.dc_under_voltage_alarm);
}

void FaultLogicEvaluator::evaluateLogic_(control::LogicContext& ctx, uint64_t now_ms) const
{
    // ------------------------------------------------------------
    // 第14批：
    // 先在 fault refresh 主链里把 system/logic 原始聚合重新收一遍，
    // 避免某些路径（HMI 写入 / remote 数据到达 / IO 更新）没有先经过 snapshot，
    // 导致 any_fault / env_any_alarm 落后于明细真源。
    // ------------------------------------------------------------

    const bool raw_hmi_fault    = rawHmiCommFault_(ctx, now_ms);
    const bool raw_remote_fault = rawRemoteCommFault_(ctx, now_ms);
    const bool raw_sd_fault     = rawSdcardFault_(ctx);

    // 把弱在线推导结果回写到 raw logic fault 真源
    ctx.logic_faults.hmi_comm_fault    = raw_hmi_fault;
    ctx.logic_faults.remote_comm_fault = raw_remote_fault;
    ctx.logic_faults.sdcard_fault      = raw_sd_fault;

    // 环境类总告警：只保留 UPS / Smoke / Gas / Air 的 alarm 语义
    ctx.logic_faults.env_any_alarm =
        ctx.ups_faults.alarm_any ||
        ctx.smoke_faults.alarm_any ||
        ctx.gas_faults.alarm_any ||
        ctx.air_faults.alarm_any;

    // 总故障：system/comm + offline 聚合 + 设备 fault 聚合
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

    // ------------------------------------------------------------
    // 聚合项 confirmed
    // ------------------------------------------------------------
    {
        const char* signal = "env_any_alarm";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawEnvAnyAlarm_(ctx),
            !rawEnvAnyAlarm_(ctx),
            2000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "any_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawAnyFault_(ctx),
            !rawAnyFault_(ctx),
            2000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // ------------------------------------------------------------
    // system / comm 类 confirmed
    // ------------------------------------------------------------

    // HMI 通信故障：持续 3000ms 触发，恢复 1000ms
    {
        const char* signal = "hmi_comm_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            raw_hmi_fault,
            !raw_hmi_fault,
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // Remote 通信故障：持续 3000ms 触发，恢复 1000ms
    {
        const char* signal = "remote_comm_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            raw_remote_fault,
            !raw_remote_fault,
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // SDCard 故障：持续 3000ms 触发，恢复 1000ms
    {
        const char* signal = "sdcard_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            raw_sd_fault,
            !raw_sd_fault,
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
}

void FaultLogicEvaluator::evaluateAll(control::LogicContext& ctx, uint64_t now_ms) const
{
    evaluatePcu_(ctx, now_ms);
    evaluateUps_(ctx, now_ms);
    evaluateSmoke_(ctx, now_ms);
    evaluateGas_(ctx, now_ms);
    evaluateAir_(ctx, now_ms);
    evaluateLogic_(ctx, now_ms);
}

} // namespace control::fault