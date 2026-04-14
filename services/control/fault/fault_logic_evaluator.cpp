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

    // UPS 通信故障
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

    // ---- Q1 / WA 状态位 ----
    {
        const char* signal = "ups_mains_abnormal";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.mains_abnormal, !u.mains_abnormal, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_battery_low";
        const std::string key = makeSignalKey_(signal);
        const bool raw_now = u.battery_low_state || (u.battery_low != 0) || u.bat_low_warning;
        updateConfirmedWithClear_(ctx, now_ms, key, raw_now, !raw_now, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bypass_mode";
        const std::string key = makeSignalKey_(signal);
        const bool raw_now = u.bypass_mode || (u.bypass_active != 0);
        updateConfirmedWithClear_(ctx, now_ms, key, raw_now, !raw_now, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // ---- warning bits ----
    {
        const char* signal = "ups_internal_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.internal_warning, !u.internal_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_epo_active";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.epo_active, !u.epo_active, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_module_unlock";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.module_unlock, !u.module_unlock, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_line_loss";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.line_loss, !u.line_loss, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bypass_loss";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bypass_loss, !u.bypass_loss, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bat_open";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bat_open, !u.bat_open, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_over_chg_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.over_chg_warning, !u.over_chg_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_overload_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.overload_warning, !u.overload_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_overload_fail";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.overload_fail, !u.overload_fail, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_fan_lock_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.fan_lock_warning, !u.fan_lock_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_chg_fail";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.chg_fail, !u.chg_fail, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_temp_low_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.temp_low_warning, !u.temp_low_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_temp_high_warning";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.temp_high_warning, !u.temp_high_warning, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bat_overtemp";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bat_overtemp, !u.bat_overtemp, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // ---- fault code ----
    {
        const char* signal = "ups_bus_soft_timeout";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bus_soft_timeout, !u.bus_soft_timeout, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bus_over";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bus_over, !u.bus_over, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bus_under";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bus_under, !u.bus_under, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bus_unbalance";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bus_unbalance, !u.bus_unbalance, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_bus_short";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.bus_short, !u.bus_short, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_inv_soft_timeout";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.inv_soft_timeout, !u.inv_soft_timeout, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_inv_volt_high";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.inv_volt_high, !u.inv_volt_high, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_inv_volt_low";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.inv_volt_low, !u.inv_volt_low, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_op_volt_short";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.op_volt_short, !u.op_volt_short, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_over_load_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.over_load_fault, !u.over_load_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_over_temperature";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.over_temperature, !u.over_temperature, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_comm_line_loss";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.comm_line_loss, !u.comm_line_loss, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_power_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.power_fault, !u.power_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_ups_all_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.ups_all_fault, !u.ups_all_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_battery_abnormal";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.battery_abnormal, !u.battery_abnormal, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_battery_over_charge_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.battery_over_charge_fault, !u.battery_over_charge_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
    {
        const char* signal = "ups_epo_fault";
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, u.epo_fault, !u.epo_fault, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
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