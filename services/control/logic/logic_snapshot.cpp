// services/control/logic/logic_snapshot.cpp
//
// Snapshot 事件处理：保存快照、重建 logic_view、导出模型、刷新 HMI
//
#include "logger.h"
#include "logic_engine.h"

namespace control {
    static void clearAirRuntimePayload_(AirFaultState& a)
    {
        a.alarm_any = false;
        a.fault_any = false;

        a.run_state = 0;
        a.power_state = 0;
        a.indoor_temp_c = 0.0;
        a.humidity_percent = 0.0;

        a.high_temp_alarm = false;
        a.low_temp_alarm = false;
        a.high_humidity_alarm = false;
        a.low_humidity_alarm = false;
        a.coil_freeze_protect = false;
        a.exhaust_high_temp_alarm = false;

        a.coil_temp_sensor_fault = false;
        a.outdoor_temp_sensor_fault = false;
        a.condenser_temp_sensor_fault = false;
        a.indoor_temp_sensor_fault = false;
        a.exhaust_temp_sensor_fault = false;
        a.humidity_sensor_fault = false;

        a.internal_fan_fault = false;
        a.external_fan_fault = false;
        a.compressor_fault = false;
        a.heater_fault = false;
        a.emergency_fan_fault = false;

        a.high_pressure_alarm = false;
        a.low_pressure_alarm = false;
        a.water_alarm = false;
        a.smoke_alarm = false;
        a.gating_alarm = false;

        a.high_pressure_lock = false;
        a.low_pressure_lock = false;
        a.exhaust_lock = false;

        a.ac_over_voltage_alarm = false;
        a.ac_under_voltage_alarm = false;
        a.ac_power_loss = false;
        a.lose_phase_alarm = false;
        a.freq_fault = false;
        a.anti_phase_alarm = false;
        a.dc_over_voltage_alarm = false;
        a.dc_under_voltage_alarm = false;
    }
    static void clearAirFaultPayload_(AirFaultState& a)
    {
        // 只清 AIR 设备自身的运行/告警/故障真源。
        // seen_once / last_rx_ms 是否保留，由调用处决定。
        a.alarm_any = false;
        a.fault_any = false;

        a.run_state = 0;
        a.power_state = 0;
        a.indoor_temp_c = 0.0;
        a.humidity_percent = 0.0;

        // 0x1028~0x102D
        a.high_temp_alarm = false;
        a.low_temp_alarm = false;
        a.high_humidity_alarm = false;
        a.low_humidity_alarm = false;
        a.coil_freeze_protect = false;
        a.exhaust_high_temp_alarm = false;

        // 0x102E~0x1033
        a.coil_temp_sensor_fault = false;
        a.outdoor_temp_sensor_fault = false;
        a.condenser_temp_sensor_fault = false;
        a.indoor_temp_sensor_fault = false;
        a.exhaust_temp_sensor_fault = false;
        a.humidity_sensor_fault = false;

        // 0x1034~0x1038
        a.internal_fan_fault = false;
        a.external_fan_fault = false;
        a.compressor_fault = false;
        a.heater_fault = false;
        a.emergency_fan_fault = false;

        // 0x1039~0x103D
        a.high_pressure_alarm = false;
        a.low_pressure_alarm = false;
        a.water_alarm = false;
        a.smoke_alarm = false;
        a.gating_alarm = false;

        // 0x103E~0x1040
        a.high_pressure_lock = false;
        a.low_pressure_lock = false;
        a.exhaust_lock = false;

        // 0x1041~0x1048
        a.ac_over_voltage_alarm = false;
        a.ac_under_voltage_alarm = false;
        a.ac_power_loss = false;
        a.lose_phase_alarm = false;
        a.freq_fault = false;
        a.anti_phase_alarm = false;
        a.dc_over_voltage_alarm = false;
        a.dc_under_voltage_alarm = false;
    }
    static void clearUpsFaultPayload_(UpsFaultState& u)
{
    // 保留：
    // - seen_once：表示 UPS 曾经出现过，不清
    // - last_rx_ms：表示最后一次收到 UPS 数据的时间，不清
    // - online：由外层根据 snapshot health 设置，不在这里改

    u.q1_seen = false;
    u.q6_seen = false;
    u.wa_seen = false;

    u.alarm_any = false;
    u.fault_any = false;

    u.work_mode = 0;
    u.battery_low = 0;
    u.bypass_active = 0;
    u.ups_fault_code = 0;

    // ---- Q1 / WA 基础状态位 ----
    u.q1_status_bits = 0;
    u.wa_status_bits = 0;

    u.mains_abnormal = false;
    u.battery_low_state = false;
    u.bypass_mode = false;
    u.ups_fault_state = false;
    u.backup_mode = false;
    u.self_test_active = false;

    // ---- Q6 warning bits ----
    u.warning_bits = 0;

    u.internal_warning = false;
    u.epo_active = false;
    u.module_unlock = false;
    u.neutral_lost = false;
    u.mains_phase_error = false;
    u.ln_reverse = false;
    u.bypass_abnormal = false;
    u.bypass_phase_error = false;
    u.battery_not_connected = false;
    u.battery_low_warning = false;
    u.battery_overcharge = false;
    u.battery_reverse = false;
    u.overload_warning = false;
    u.overload_alarm = false;
    u.fan_fault = false;
    u.bypass_cover_open = false;
    u.charger_fault = false;
    u.position_error = false;
    u.boot_condition_not_met = false;
    u.redundancy_lost = false;
    u.module_loose = false;
    u.battery_maint_due = false;
    u.inspection_maint_due = false;
    u.warranty_maint_due = false;
    u.temp_low_warning = false;
    u.temp_high_warning = false;
    u.battery_overtemp = false;
    u.fan_maint_due = false;
    u.bus_cap_maint_due = false;
    u.system_overload = false;
    u.reserved_warning = false;

    // ---- Q6 fault containers ----
    u.fault_bits = 0;
    u.fault_code_1 = 0;
    u.fault_code_2 = 0;
    u.fault_code_3 = 0;
    u.fault_code_4 = 0;

    // ---- UPS Fault Table ----
    u.bus_softstart_timeout = false;
    u.bus_overvoltage_fault = false;
    u.bus_undervoltage_fault = false;
    u.bus_imbalance_fault = false;
    u.bus_short_circuit = false;

    u.inv_softstart_timeout = false;
    u.inv_overvoltage_fault = false;
    u.inv_undervoltage_fault = false;
    u.output_short_circuit = false;
    u.r_inv_short_circuit = false;
    u.s_inv_short_circuit = false;
    u.t_inv_short_circuit = false;
    u.rs_short_circuit = false;
    u.st_short_circuit = false;
    u.tr_short_circuit = false;

    u.reverse_power_fault = false;
    u.r_reverse_power_fault = false;
    u.s_reverse_power_fault = false;
    u.t_reverse_power_fault = false;
    u.total_reverse_power_fault = false;
    u.current_imbalance_fault = false;
    u.overload_fault = false;
    u.overtemp_fault = false;

    u.inv_relay_fail_close = false;
    u.inv_relay_stuck = false;
    u.mains_scr_fault = false;
    u.battery_scr_fault = false;
    u.bypass_scr_fault = false;
    u.rectifier_fault = false;
    u.input_overcurrent_fault = false;
    u.wiring_error = false;

    u.comm_cable_disconnected = false;
    u.host_cable_fault = false;
    u.can_comm_fault = false;
    u.sync_signal_fault = false;
    u.power_supply_fault = false;
    u.all_fan_fault = false;
    u.dsp_error = false;
    u.charger_softstart_timeout = false;
    u.all_module_fault = false;

    u.mains_ntc_open_fault = false;
    u.mains_fuse_open_fault = false;
    u.output_imbalance_fault = false;
    u.input_mismatch_fault = false;
    u.eeprom_data_lost = false;
    u.mains_support_failed = false;
    u.power_failed = false;
    u.system_overload_fault = false;
    u.ads7869_error = false;
    u.bypass_mode_no_op = false;
    u.op_breaker_off_parallel = false;

    u.r_bus_fuse_fault = false;
    u.s_bus_fuse_fault = false;
    u.t_bus_fuse_fault = false;
    u.ntc_fault = false;
    u.parallel_cable_fault = false;
    u.battery_fault = false;
    u.frequent_overcurrent_fault = false;
    u.battery_overcharge_fault = false;
    u.epo_critical_fault = false;

    // 当前协议表没有独立 code 的遗留字段，也清掉
    u.battery_overcharge_persist = false;
}
    static void clearSmokeFaultPayload_(SmokeFaultState& s)
    {
        // 保留：
        // - seen_once
        // - last_rx_ms
        // - online 由外层根据 snapshot health 设置
        //
        // 只清设备内部告警/故障，避免离线后旧故障挂死。
        s.alarm_any = false;
        s.fault_any = false;

        s.smoke_percent = 0.0;
        s.temp_c = 0.0;

        s.smoke_alarm = false;
        s.temp_alarm = false;

        s.smoke_sensor_fault = false;
        s.smoke_pollution_fault = false;
        s.temp_sensor_fault = false;
    }

    static void clearGasFaultPayload_(GasFaultState& g)
    {
        // 保留：
        // - seen_once
        // - last_rx_ms
        // - online 由外层根据 snapshot health 设置
        //
        // 只清设备内部告警/故障，避免离线后旧故障挂死。
        g.alarm_any = false;
        g.fault_any = false;

        g.status_code = 0;
        g.gas_value = 0.0;

        g.sensor_fault = false;
        g.low_alarm = false;
        g.high_alarm = false;
    }
    static void rebuildLogicFaultSummary_(LogicContext& ctx)
    {
        // 环境类总告警：
        // 只聚合环境设备（UPS / Smoke / Gas / Air）的 alarm 语义，
        // 不把 sdcard / hmi / remote 这种 system/comm 类故障算进去。
        ctx.logic_faults.env_any_alarm =
            ctx.ups_faults.alarm_any ||
            ctx.smoke_faults.alarm_any ||
            ctx.gas_faults.alarm_any ||
            ctx.air_faults.alarm_any;

        // 总故障：
        // 这里按“系统任何故障/离线/关键系统资源故障”为 true 的口径收口。
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
    }
void updateDiscreteFaultRuntimeFromSnapshot_(LogicContext& ctx,
                                             const agg::SystemSnapshot& snap,
                                             uint64_t ts_ms)
{
    // ------------------------------------------------------------
    // UPS
    // ------------------------------------------------------------
    {
        auto it_ups = snap.items.find("UPS");

        if (it_ups == snap.items.end()) {
            ctx.ups_faults.online = false;
            ctx.logic_faults.ups_offline = true;

            // UPS 离线时，只保留 0x1008 通信故障；
            // 清除 UPS 内部 warning/fault，避免旧故障挂死。
            clearUpsFaultPayload_(ctx.ups_faults);
        } else {
            const auto& item = it_ups->second;

            ctx.ups_faults.seen_once = true;
            ctx.ups_faults.online = item.online;
            ctx.ups_faults.last_rx_ms = item.ts_ms;

            ctx.logic_faults.ups_offline = !item.online;

            if (!item.online) {
                clearUpsFaultPayload_(ctx.ups_faults);
            }
        }
    }

    // ------------------------------------------------------------
    // Smoke / TSS
    // ------------------------------------------------------------
    {
        auto it_smoke = snap.items.find("SmokeSensor");

        if (it_smoke == snap.items.end()) {
            ctx.smoke_faults.online = false;
            ctx.logic_faults.smoke_offline = true;

            // Smoke 离线时，只保留 0x100A 通信故障；
            // 清除 TSS 内部报警/故障，避免旧故障挂死。
            clearSmokeFaultPayload_(ctx.smoke_faults);
        } else {
            const auto& item = it_smoke->second;

            ctx.smoke_faults.seen_once = true;
            ctx.smoke_faults.online = item.online;
            ctx.smoke_faults.last_rx_ms = item.ts_ms;

            ctx.logic_faults.smoke_offline = !item.online;

            if (!item.online) {
                clearSmokeFaultPayload_(ctx.smoke_faults);
            }
        }
    }

    // ------------------------------------------------------------
    // AirConditioner
    // ------------------------------------------------------------
    {
        auto it_air = snap.items.find("AirConditioner");

        if (it_air == snap.items.end()) {
            ctx.air_faults.online = false;
            ctx.logic_faults.air_offline = true;

            // AIR 离线时，只保留 0x1007 通信故障；
            // 清除 0x1028~0x1048 这些设备内部故障，避免旧告警挂死。
            clearAirFaultPayload_(ctx.air_faults);
        } else {
            const auto& item = it_air->second;

            ctx.air_faults.seen_once = true;
            ctx.air_faults.online = item.online;
            ctx.air_faults.last_rx_ms = item.ts_ms;

            ctx.logic_faults.air_offline = !item.online;

            if (!item.online) {
                clearAirFaultPayload_(ctx.air_faults);
            }
        }
    }

    // ------------------------------------------------------------
    // GasDetector
    // ------------------------------------------------------------
    {
        auto it_gas = snap.items.find("GasDetector");

        if (it_gas == snap.items.end()) {
            ctx.gas_faults.online = false;
            ctx.logic_faults.gas_offline = true;

            // Gas 离线时，只保留 0x100B 通信故障；
            // 清除设备内部报警/故障，避免旧故障挂死。
            clearGasFaultPayload_(ctx.gas_faults);
        } else {
            const auto& item = it_gas->second;

            ctx.gas_faults.seen_once = true;
            ctx.gas_faults.online = item.online;
            ctx.gas_faults.last_rx_ms = item.ts_ms;

            ctx.logic_faults.gas_offline = !item.online;

            // 每次 snapshot 都从 Aggregator 的 gas_channels 重新构建一次完整汇总。
            // 这一步是 Gas 多气体循环更新的关键：不能只看最后一帧。
            clearGasFaultPayload_(ctx.gas_faults);

            if (item.online) {
                for (const auto& kv : item.gas_channels) {
                    const auto& ch = kv.second;
                    if (!ch.valid) {
                        continue;
                    }

                    ctx.gas_faults.status_code |= static_cast<uint32_t>(ch.status);

                    // gas_value 只是显示/调试值；对于多气体，保留最后遍历到的值即可。
                    // 具体分气体数值仍以 Aggregator/logic_view 中 gas_channels 为准。
                    ctx.gas_faults.gas_value = ch.value;

                    ctx.gas_faults.sensor_fault =
                        ctx.gas_faults.sensor_fault || ch.fault_any;

                    ctx.gas_faults.low_alarm =
                        ctx.gas_faults.low_alarm || ch.low_alarm;

                    ctx.gas_faults.high_alarm =
                        ctx.gas_faults.high_alarm || ch.high_alarm;

                    ctx.gas_faults.alarm_any =
                        ctx.gas_faults.alarm_any || ch.alarm_any;

                    ctx.gas_faults.fault_any =
                        ctx.gas_faults.fault_any || ch.fault_any;
                }
            }
        }
    }

    (void)ts_ms;
}

    void LogicEngine::onSnapshot_(const SnapshotEvent& s,
                                  LogicContext& ctx,
                                  std::vector<Command>& out_cmds)
    {
        (void)out_cmds;

        ctx.latest_snapshot = s.snap;
        ctx.last_event_ts = s.ts_ms;

        updatePcuRuntimeHealth_(ctx, s.ts_ms);
        updateBmsRuntimeHealth_(ctx, s.ts_ms);
        updateDiscreteFaultRuntimeFromSnapshot_(ctx, s.snap, s.ts_ms);

        // 第11批：在 snapshot 主链里统一重算 logic 聚合故障，
        // 避免新增的 system/comm 类故障没有进入 any_fault / env_any_alarm。
        rebuildLogicFaultSummary_(ctx);


        // LOG_COMM_D("[LOGIC][SNAPSHOT][AFTER_RUNTIME] pcu0{on=%d rx=%d hb=%d} pcu1{on=%d rx=%d hb=%d}",
        //            ctx.pcu0_state.online ? 1 : 0,
        //            ctx.pcu0_state.rx_alive ? 1 : 0,
        //            ctx.pcu0_state.hb_alive ? 1 : 0,
        //            ctx.pcu1_state.online ? 1 : 0,
        //            ctx.pcu1_state.rx_alive ? 1 : 0,
        //            ctx.pcu1_state.hb_alive ? 1 : 0);

        // applyFaultPages_(ctx, s.ts_ms);
        // if (ctx.hmi) {
        //     if (ctx.fault_map_loaded) {
        //         ctx.fault_pages.flushToHmi(*ctx.hmi);
        //     }
        // }

        rebuildLogicView_(ctx);
        applyHmiOutputs_(ctx);

        if (out2json) {
            model_exporter_.exportLatest(ctx.latest_snapshot, ctx.logic_view, s.ts_ms);
        }
    }
} // namespace control