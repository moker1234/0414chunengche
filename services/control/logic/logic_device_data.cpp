// services/control/logic_device_data.cpp
//
// DeviceData 事件处理 + PCU 在线状态维护
//
#include "logger.h"
#include "logic_engine.h"

namespace control
{
    namespace
    {
        // constexpr uint32_t PCU_RX_TIMEOUT_MS = 1500;
        // constexpr uint32_t PCU_HB_STALE_MS   = 3000;
    }

    static void recoverPcuRuntimeOnDeviceData_(const DeviceData& d,
                                               LogicContext& ctx)
    {
        const std::string& dev = d.device_name;
        const uint64_t ts = d.timestamp;

        auto fill_one = [&](auto& st)
        {
            st.online = true;
            st.rx_alive = true;
            st.hb_alive = true;
            st.last_rx_ms = ts;

            if (st.last_hb_change_ms == 0)
            {
                st.last_hb_change_ms = ts;
            }
        };

        if (dev == "PCU_0")
        {
            fill_one(ctx.pcu0_state);

            return;
        }

        if (dev == "PCU_1")
        {
            fill_one(ctx.pcu1_state);
        }

        // 其他任何设备一律不处理
    }

    static void recoverBmsRuntimeOnDeviceData_(const DeviceData& d,
                                               LogicContext& ctx)
    {
        const std::string& dev = d.device_name;
        const uint64_t ts = d.timestamp;

        if (dev != "BMS_1" &&
            dev != "BMS_2" &&
            dev != "BMS_3" &&
            dev != "BMS_4")
        {
            return;
        }

        auto& x = ctx.bms_cache.items[dev];
        x.seen_once = true;
        x.online = true;
        x.last_rx_ms = ts;

        if (x.last_st1_ms == 0) x.last_st1_ms = ts;
        if (x.last_st2_ms == 0) x.last_st2_ms = ts;
        if (x.last_fault1_ms == 0) x.last_fault1_ms = ts;
        if (x.last_fault2_ms == 0) x.last_fault2_ms = ts;
        if (x.last_current_limit_ms == 0) x.last_current_limit_ms = ts;

        LOG_THROTTLE_MS("bms_recover_diag", 1000, LOG_COMM_D,
                        "[BMS][RECOVER] dev=%s ts=%llu bms_items=%zu",
                        dev.c_str(),
                        (unsigned long long)ts,
                        ctx.bms_cache.items.size());
    }

    static bool recoverRemoteRuntimeOnDeviceData_(const DeviceData& d,
                                                  LogicContext& ctx)
    {
        const std::string& dev = d.device_name;
        const uint64_t ts = d.timestamp;

        // remote 还没有具体程序，先约定几种可能的 device_name
        const bool is_remote =
            (dev == "Remote") ||
            (dev == "REMOTE") ||
            (dev == "RemoteIO") ||
            (dev == "REMOTE_IO") ||
            (dev == "RemoteIo");

        if (!is_remote)
        {
            return false;
        }

        ctx.remote_seen_once = true;
        ctx.last_remote_rx_ts = ts;

        // 收到 remote 数据，立即清 raw remote_comm_fault
        ctx.logic_faults.remote_comm_fault = false;

        // 同步一次 any_fault，避免 fault refresh 前短暂滞后
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
            ctx.logic_faults.air_offline;

        LOG_THROTTLE_MS("remote_recover_diag", 1000, LOG_COMM_D,
                        "[REMOTE][RECOVER] dev=%s ts=%llu",
                        dev.c_str(),
                        (unsigned long long)ts);

        return true;
    }

    void LogicEngine::onDeviceData_(const DeviceData& d,
                                    uint64_t ts,
                                    LogicContext& ctx,
                                    std::vector<Command>& out_cmds)
    {
        (void)out_cmds;

        // recoverPcuRuntimeOnDeviceData_(d, ctx);
        // recoverBmsRuntimeOnDeviceData_(d, ctx);

        // LOG_THROTTLE_MS("logic_devdata_diag", 1000, LOG_COMM_D,
        //     "[LOGIC][DEV_DATA] dev=%s pcu0_on=%d pcu1_on=%d bms_items=%zu",
        //     d.device_name.c_str(),
        //     ctx.pcu0_state.online ? 1 : 0,
        //     ctx.pcu1_state.online ? 1 : 0,
        //     ctx.bms_cache.items.size()
        // );

        DeviceData copy = d;
        copy.timestamp = static_cast<uint32_t>(ts);
        ctx.latest_device[copy.device_name] = std::move(copy);

        // remote 预留入口
        // 当前 remote 还没有具体程序，这里先把 device_name 入口预留好。
        // 后续 remote 只要投递约定名字的 DeviceData，就能自动刷新 remote 原始真源。
        if (recoverRemoteRuntimeOnDeviceData_(d, ctx))
        {
            return;
        }

        // BMS：交给专用适配器维护控制面缓存
        if (d.device_name == "BMS")
        {
            bms_adapter_.onDeviceData(d, ts, ctx.bms_cache);
            return;
        }

        // UPS：记录原始到达与基础告警真源
        if (d.device_name == "UPS")
        {
            auto& u = ctx.ups_faults;
            u.seen_once = true;
            u.last_rx_ms = ts;

            if (auto it = d.value.find("__ups_cmd"); it != d.value.end())
            {
                if (it->second == 1) u.q1_seen = true;
                if (it->second == 2) u.q6_seen = true;
                if (it->second == 3) u.wa_seen = true;
            }

            auto read_i32 = [&](std::initializer_list<const char*> keys, int defv = 0) -> int
            {
                for (const char* k : keys)
                {
                    auto itv = d.value.find(k);
                    if (itv != d.value.end()) return static_cast<int>(itv->second);
                    auto its = d.status.find(k);
                    if (its != d.status.end()) return static_cast<int>(its->second);
                    auto itn = d.num.find(k);
                    if (itn != d.num.end()) return static_cast<int>(itn->second);
                }
                return defv;
            };

            auto read_u32 = [&](std::initializer_list<const char*> keys, uint32_t defv = 0u) -> uint32_t
            {
                for (const char* k : keys)
                {
                    auto itv = d.value.find(k);
                    if (itv != d.value.end()) return static_cast<uint32_t>(itv->second);
                    auto its = d.status.find(k);
                    if (its != d.status.end()) return static_cast<uint32_t>(its->second);
                    auto itn = d.num.find(k);
                    if (itn != d.num.end()) return static_cast<uint32_t>(itn->second);
                }
                return defv;
            };

            // ==== 基础读取 ====
            u.work_mode = read_i32({"work_mode", "mode", "system.mode"}, u.work_mode);
            u.battery_low = read_i32({"battery_low", "battery.low", "status.battery_low"}, u.battery_low);
            u.bypass_active = read_i32({"bypass_active", "bypass", "status.bypass"}, u.bypass_active);
            u.ups_fault_code = read_i32({"fault_code", "ups_fault_code"}, u.ups_fault_code);

            // ==== 1. 提取 Q1/WA 基础状态字 ====
            const uint32_t status_bits = read_u32(
                {"status_bits", "ups_status_bits", "q1_status_bits", "wa_status_bits"}, 0u);

            // 使用局部变量暂存 Q1 的市电异常状态 (Bit 7)，避免直接赋值给 u.mains_abnormal 被后面的覆盖
            bool q1_mains_abnormal_flag = ((status_bits >> 7) & 0x1u) != 0;

            // 提取其他 Q1 状态
            u.battery_low_state = ((status_bits >> 6) & 0x1u) != 0;
            u.bypass_mode = ((status_bits >> 5) & 0x1u) != 0;
            u.ups_fault_state = ((status_bits >> 4) & 0x1u) != 0;
            u.backup_mode = ((status_bits >> 3) & 0x1u) != 0;
            u.self_test_active = ((status_bits >> 2) & 0x1u) != 0;

            // ==== 2. 提取 Q6 警告字 ====
            u.warning_bits = read_u32({"warning_bits", "ups_warning_bits"}, u.warning_bits);

            // ==== 3. 展开 32 位 Warning 状态 ====
            u.internal_warning = ((u.warning_bits >> 0) & 0x1u) != 0;
            u.epo_active = ((u.warning_bits >> 1) & 0x1u) != 0;
            u.module_unlock = ((u.warning_bits >> 2) & 0x1u) != 0;

            // 修复：市电异常合并 (Q1的Bit7 || Q6的Bit3)
            u.mains_abnormal = q1_mains_abnormal_flag || (((u.warning_bits >> 3) & 0x1u) != 0);
            u.neutral_lost = ((u.warning_bits >> 4) & 0x1u) != 0;
            u.mains_phase_error = ((u.warning_bits >> 5) & 0x1u) != 0;
            u.ln_reverse = ((u.warning_bits >> 6) & 0x1u) != 0;
            u.bypass_abnormal = ((u.warning_bits >> 7) & 0x1u) != 0;
            u.bypass_phase_error = ((u.warning_bits >> 8) & 0x1u) != 0;
            u.battery_not_connected = ((u.warning_bits >> 9) & 0x1u) != 0;
            u.battery_low_warning = ((u.warning_bits >> 10) & 0x1u) != 0;
            u.battery_overcharge = ((u.warning_bits >> 11) & 0x1u) != 0;
            u.battery_reverse = ((u.warning_bits >> 12) & 0x1u) != 0;
            u.overload_warning = ((u.warning_bits >> 13) & 0x1u) != 0;
            u.overload_alarm = ((u.warning_bits >> 14) & 0x1u) != 0;
            u.fan_fault = ((u.warning_bits >> 15) & 0x1u) != 0;
            u.bypass_cover_open = ((u.warning_bits >> 16) & 0x1u) != 0;
            u.charger_fault = ((u.warning_bits >> 17) & 0x1u) != 0;
            u.position_error = ((u.warning_bits >> 18) & 0x1u) != 0;
            u.boot_condition_not_met = ((u.warning_bits >> 19) & 0x1u) != 0;
            u.redundancy_lost = ((u.warning_bits >> 20) & 0x1u) != 0;
            u.module_loose = ((u.warning_bits >> 21) & 0x1u) != 0;
            u.battery_maint_due = ((u.warning_bits >> 22) & 0x1u) != 0;
            u.inspection_maint_due = ((u.warning_bits >> 23) & 0x1u) != 0;
            u.warranty_maint_due = ((u.warning_bits >> 24) & 0x1u) != 0;
            u.temp_low_warning = ((u.warning_bits >> 25) & 0x1u) != 0;
            u.temp_high_warning = ((u.warning_bits >> 26) & 0x1u) != 0;
            u.battery_overtemp = ((u.warning_bits >> 27) & 0x1u) != 0;
            u.fan_maint_due = ((u.warning_bits >> 28) & 0x1u) != 0;
            u.bus_cap_maint_due = ((u.warning_bits >> 29) & 0x1u) != 0;
            u.system_overload = ((u.warning_bits >> 30) & 0x1u) != 0;
            u.reserved_warning = ((u.warning_bits >> 31) & 0x1u) != 0;

            // ==== 4. 提取 60 个 Fault Code 状态 ====

            // 使用 -999 作为特殊的默认值，以此判断字典中是否真的包含了故障码数据
            // 如果是 Q1 或 WA 报文，fc1 将等于 -999
            int fc1 = read_i32( {"ups_fault_code_1"}, -999);

            // 核心修复：只有当报文中确实存在故障数据时（如 Q6），才去刷新状态
            // 否则（Q1/WA），直接跳过，保持上一周期的故障状态不变！
            if (fc1 != -999) {
                int fc2 = read_i32( {"ups_fault_code_2"}, 0);
                int fc3 = read_i32( {"ups_fault_code_3"}, 0);
                int fc4 = read_i32( {"ups_fault_code_4"}, 0);

                auto has_fault = [&](int code) {
                    return (fc1 == code) || (fc2 == code) || (fc3 == code) || (fc4 == code);
                };

                u.bus_overvoltage_fault = has_fault(1);
                u.bus_undervoltage_fault = has_fault(2);
                u.bus_imbalance_fault = has_fault(3);
                u.bus_short_circuit = has_fault(4);
                u.inv_softstart_timeout = has_fault(5);
                u.inv_overvoltage_fault = has_fault(6);
                u.inv_undervoltage_fault = has_fault(7);
                u.output_short_circuit = has_fault(8);
                u.r_inv_short_circuit = has_fault(9);
                u.s_inv_short_circuit = has_fault(10);
                u.t_inv_short_circuit = has_fault(11);
                u.rs_short_circuit = has_fault(12);
                u.st_short_circuit = has_fault(13);
                u.tr_short_circuit = has_fault(14);
                u.reverse_power_fault = has_fault(15);
                u.r_reverse_power_fault = has_fault(16);
                u.s_reverse_power_fault = has_fault(17);
                u.t_reverse_power_fault = has_fault(18);
                u.total_reverse_power_fault = has_fault(19);
                u.current_imbalance_fault = has_fault(20);
                u.overload_fault = has_fault(21);
                u.overtemp_fault = has_fault(22);
                u.inv_relay_fail_close = has_fault(23);
                u.inv_relay_stuck = has_fault(24);
                u.mains_scr_fault = has_fault(25);
                u.battery_scr_fault = has_fault(26);
                u.bypass_scr_fault = has_fault(27);
                u.rectifier_fault = has_fault(28);
                u.input_overcurrent_fault = has_fault(29);
                u.wiring_error = has_fault(30);
                u.comm_cable_disconnected = has_fault(31);
                u.host_cable_fault = has_fault(32);
                u.can_comm_fault = has_fault(33);
                u.sync_signal_fault = has_fault(34);
                u.power_supply_fault = has_fault(35);
                u.all_fan_fault = has_fault(36);
                u.dsp_error = has_fault(37);
                u.charger_softstart_timeout = has_fault(38);
                u.all_module_fault = has_fault(39);
                u.mains_ntc_open_fault = has_fault(40);
                u.mains_fuse_open_fault = has_fault(41);
                u.output_imbalance_fault = has_fault(42);
                u.input_mismatch_fault = has_fault(43);
                u.eeprom_data_lost = has_fault(44);
                u.mains_support_failed = has_fault(45);
                u.power_failed = has_fault(46);
                u.system_overload_fault = has_fault(47);
                u.ads7869_error = has_fault(48);
                u.bypass_mode_no_op = has_fault(49);
                u.op_breaker_off_parallel = has_fault(50);
                u.r_bus_fuse_fault = has_fault(51);
                u.s_bus_fuse_fault = has_fault(52);
                u.t_bus_fuse_fault = has_fault(53);
                u.ntc_fault = has_fault(54);
                u.parallel_cable_fault = has_fault(55);
                u.battery_fault = has_fault(56);
                u.frequent_overcurrent_fault = has_fault(57);
                u.battery_overcharge_fault = has_fault(58);
                u.battery_overcharge_persist = has_fault(59);
                u.epo_critical_fault = has_fault(60);
            }
            // --------------------------------------------------

            u.alarm_any =
                u.battery_low_state ||
                u.bypass_mode ||
                (u.warning_bits != 0) ||
                (u.ups_fault_code != 0);

            u.fault_any =
                u.ups_fault_state ||
                (u.ups_fault_code != 0);

            return;
        }

        // Smoke：记录原始到达与基础告警真源
        if (d.device_name == "SmokeSensor")
        {
            auto& s = ctx.smoke_faults;
            s.seen_once = true;
            s.last_rx_ms = ts;

            if (auto it = d.num.find("smoke_percent"); it != d.num.end())
            {
                s.smoke_percent = it->second;
            }
            if (auto it = d.num.find("temp"); it != d.num.end())
            {
                s.temp_c = it->second;
            }

            int alarm = 0;
            if (auto it = d.num.find("alarm"); it != d.num.end())
            {
                alarm = static_cast<int>(it->second);
            }

            int fault = 0;
            if (auto it = d.num.find("fault"); it != d.num.end())
            {
                fault = static_cast<int>(it->second);
            }

            s.smoke_alarm = (alarm != 0);
            s.temp_alarm = false; // 当前协议解析未单独提供温度报警位，先留 false

            s.smoke_sensor_fault = (fault & 0x01) != 0;
            s.smoke_pollution_fault = (fault & 0x02) != 0;
            s.temp_sensor_fault = (fault & 0x04) != 0;

            s.alarm_any =
                s.smoke_alarm ||
                s.temp_alarm;

            s.fault_any =
                s.smoke_sensor_fault ||
                s.smoke_pollution_fault ||
                s.temp_sensor_fault;

            return;
        }

        // Gas：记录原始到达与基础告警真源
        if (d.device_name == "GasDetector")
        {
            auto& g = ctx.gas_faults;
            g.seen_once = true;
            g.last_rx_ms = ts;

            if (auto it = d.num.find("gas_value"); it != d.num.end())
            {
                g.gas_value = it->second;
            }
            if (auto it = d.num.find("status"); it != d.num.end())
            {
                g.status_code = static_cast<uint32_t>(it->second);
            }

            g.sensor_fault = (g.status_code & 0x0001u) != 0;
            g.low_alarm = (g.status_code & 0x0002u) != 0;
            g.high_alarm = (g.status_code & 0x0004u) != 0;

            g.alarm_any = g.low_alarm || g.high_alarm;
            g.fault_any = g.sensor_fault;
            return;
        }

        // AirConditioner：记录原始到达与基础告警真源
        if (d.device_name == "AirConditioner")
        {
            auto& a = ctx.air_faults;
            a.seen_once = true;
            a.last_rx_ms = ts;

            auto read_num = [&](std::initializer_list<const char*> keys, double defv = 0.0) -> double
            {
                for (const char* k : keys)
                {
                    auto it = d.num.find(k);
                    if (it != d.num.end()) return it->second;
                }
                return defv;
            };
            auto read_i32 = [&](std::initializer_list<const char*> keys, int defv = 0) -> int
            {
                for (const char* k : keys)
                {
                    auto itv = d.value.find(k);
                    if (itv != d.value.end()) return static_cast<int>(itv->second);
                    auto its = d.status.find(k);
                    if (its != d.status.end()) return static_cast<int>(its->second);
                    auto itn = d.num.find(k);
                    if (itn != d.num.end()) return static_cast<int>(itn->second);
                }
                return defv;
            };

            a.run_state = read_i32({"run.overall", "run_state"}, a.run_state);
            a.power_state = read_i32({"remote.power", "power_state"}, a.power_state);
            a.indoor_temp_c = read_num({"temp.indoor_c", "indoor_temp_c"}, a.indoor_temp_c);
            a.humidity_percent = read_num({"humidity_percent", "humidity"}, a.humidity_percent);

            a.high_temp_alarm = read_i32({"alarm.high_temp"}, 0) != 0;
            a.low_temp_alarm = read_i32({"alarm.low_temp"}, 0) != 0;
            a.high_humidity_alarm = read_i32({"alarm.high_hum"}, 0) != 0;
            a.low_humidity_alarm = read_i32({"alarm.low_hum"}, 0) != 0;
            a.coil_freeze_protect = read_i32({"alarm.coil_freeze"}, 0) != 0;
            a.exhaust_high_temp_alarm = read_i32({"alarm.exhaust_high_temp"}, 0) != 0;

            a.coil_temp_sensor_fault = read_i32({"alarm.coil_sensor_fault"}, 0) != 0;
            a.outdoor_temp_sensor_fault = read_i32({   "alarm.outdoor_sensor_fault" }, 0) != 0;
            a.condenser_temp_sensor_fault = read_i32({ "alarm.condense_sensor_fault"  }, 0) != 0;
            a.indoor_temp_sensor_fault = read_i32({ "alarm.indoor_sensor_fault"  }, 0) != 0;
            a.exhaust_temp_sensor_fault = read_i32({ "alarm.exhaust_sensor_fault" }, 0) != 0;
            a.humidity_sensor_fault = read_i32({"alarm.hum_sensor_fault"}, 0) != 0;

            a.internal_fan_fault = read_i32({"alarm.inner_fan_fault"}, 0) != 0;
            a.external_fan_fault = read_i32({"alarm.outer_fan_fault"}, 0) != 0;
            a.compressor_fault = read_i32({"alarm.compressor_fault"}, 0) != 0;
            a.heater_fault = read_i32({"alarm.heater_fault"}, 0) != 0;
            a.emergency_fan_fault = read_i32({"alarm.emergency_fan_fault"}, 0) != 0;

            a.high_pressure_alarm = read_i32({"alarm.high_pressure"}, 0) != 0;
            a.low_pressure_alarm = read_i32({"alarm.low_pressure"}, 0) != 0;
            a.water_alarm = read_i32({"alarm.water",}, 0) != 0;
            a.smoke_alarm = read_i32({"alarm.smoke"}, 0) != 0;
            a.gating_alarm = read_i32({"alarm.door"}, 0) != 0;

            a.high_pressure_lock = read_i32({"alarm.high_pressure_lock"}, 0) != 0;
            a.low_pressure_lock = read_i32({"alarm.low_pressure_lock"}, 0) != 0;
            a.exhaust_lock = read_i32({"alarm.exhaust_lock"}, 0) != 0;

            a.ac_over_voltage_alarm = read_i32({"alarm.ac_over_v"}, 0) != 0;
            a.ac_under_voltage_alarm = read_i32({"alarm.ac_under_v"}, 0) != 0;
            a.ac_power_loss = read_i32({"alarm.ac_power_loss"}, 0) != 0;
            a.lose_phase_alarm = read_i32({"alarm.phase_loss"}, 0) != 0;
            a.freq_fault = read_i32({"alarm.freq_abnormal"}, 0) != 0;
            a.anti_phase_alarm = read_i32({"alarm.reverse_phase"}, 0) != 0;
            a.dc_over_voltage_alarm = read_i32({"alarm.dc_over_v"}, 0) != 0;
            a.dc_under_voltage_alarm = read_i32({"alarm.dc_under_v"}, 0) != 0;

            a.alarm_any =
                a.high_temp_alarm ||
                a.low_temp_alarm ||
                a.high_humidity_alarm ||
                a.low_humidity_alarm ||
                a.coil_freeze_protect ||
                a.exhaust_high_temp_alarm ||
                a.high_pressure_alarm ||
                a.low_pressure_alarm ||
                a.water_alarm ||
                a.smoke_alarm ||
                a.gating_alarm ||
                a.high_pressure_lock ||
                a.low_pressure_lock ||
                a.exhaust_lock ||
                a.ac_over_voltage_alarm ||
                a.ac_under_voltage_alarm ||
                a.ac_power_loss ||
                a.lose_phase_alarm ||
                a.freq_fault ||
                a.anti_phase_alarm ||
                a.dc_over_voltage_alarm ||
                a.dc_under_voltage_alarm;

            a.fault_any =
                a.coil_temp_sensor_fault ||
                a.outdoor_temp_sensor_fault ||
                a.condenser_temp_sensor_fault ||
                a.indoor_temp_sensor_fault ||
                a.exhaust_temp_sensor_fault ||
                a.humidity_sensor_fault ||
                a.internal_fan_fault ||
                a.external_fan_fault ||
                a.compressor_fault ||
                a.heater_fault ||
                a.emergency_fan_fault;

            return;
        }
        // PCU：维护在线状态
        // 兼容原始名 PCU 与归一化名 PCU_0 / PCU_1
        if (d.device_name == "PCU" ||
            d.device_name == "PCU_0" ||
            d.device_name == "PCU_1")
        {
            updatePcuOnlineState_(d, ts, ctx);
        }
    }

    void LogicEngine::updatePcuOnlineState_(const DeviceData& d, uint64_t ts, LogicContext& ctx)
    {
        uint32_t instance = 0;
        uint32_t heartbeat = 0;
        uint32_t cabinet_id = 0;

        // 当前策略：
        // 1. 优先按 cabinet_id 分路
        // 2. 回退按归一化设备名
        // 3. 最后回退按 __can_index
        if (!tryResolvePcuInstance_(d, instance)) return;
        if (!tryGetPcuHeartbeat_(d, heartbeat)) return;

        PcuOnlineState* st = nullptr;
        if (instance == 0) st = &ctx.pcu0_state;
        else if (instance == 1) st = &ctx.pcu1_state;
        else return;

        st->seen_once = true;
        st->last_rx_ms = ts;

        if (tryGetPcuCabinetId_(d, cabinet_id))
        {
            st->cabinet_id = cabinet_id;
        }

        // 维护 PCU 急停位：parsePcuStatus() 已把 Byte1 解析进 DeviceData
        {
            bool estop = false;

            auto itv = d.value.find("estop");
            if (itv != d.value.end())
            {
                estop = (itv->second != 0);
            }
            else
            {
                auto its = d.status.find("estop");
                if (its != d.status.end())
                {
                    estop = (its->second != 0);
                }
                else
                {
                    auto itn = d.num.find("estop");
                    if (itn != d.num.end())
                    {
                        estop = (itn->second != 0.0);
                    }
                }
            }

            st->estop = estop;
        }

        if (!st->has_last_heartbeat)
        {
            st->last_heartbeat = heartbeat;
            st->has_last_heartbeat = true;
            st->last_hb_change_ms = ts;
            st->hb_repeat_count = 0;
            st->last_hb_delta = 0;
        }
        else
        {
            const uint32_t prev = st->last_heartbeat;
            const uint32_t curr = heartbeat;

            if (prev == curr)
            {
                st->hb_repeat_count += 1;
                st->last_hb_delta = 0;
            }
            else
            {
                const int32_t delta = static_cast<int32_t>(curr) - static_cast<int32_t>(prev);
                st->last_hb_delta = delta;

                // 第一版：只记录明显异常跳变，不直接拉掉 online
                if (std::abs(delta) > 1 && std::abs(delta) < 250)
                {
                    st->hb_jump_err_count += 1;
                }

                st->last_heartbeat = curr;
                st->last_hb_change_ms = ts;
                st->hb_repeat_count = 0;
            }
        }

        // 这里只更新“原始观测量”
        // 最终 rx_alive / hb_alive / online 由 onTick() 周期统一重算
    }

    bool LogicEngine::tryResolvePcuInstance_(const DeviceData& d, uint32_t& out_instance)
    {
        // 1) 优先按 cabinet_id 分路：3 -> pcu1, 4 -> pcu2
        uint32_t cabinet_id = 0;
        if (tryGetPcuCabinetId_(d, cabinet_id))
        {
            if (cabinet_id == 3)
            {
                out_instance = 0;
                return true;
            }
            if (cabinet_id == 4)
            {
                out_instance = 1;
                return true;
            }
        }

        // 2) 回退：兼容归一化设备名
        if (d.device_name == "PCU_0")
        {
            out_instance = 0;
            return true;
        }
        if (d.device_name == "PCU_1")
        {
            out_instance = 1;
            return true;
        }

        // 3) 最后回退：附加元信息 __can_index
        auto itv = d.value.find("__can_index");
        if (itv != d.value.end())
        {
            const uint32_t idx = static_cast<uint32_t>(itv->second);
            if (idx == 0 || idx == 1)
            {
                out_instance = idx;
                return true;
            }
        }

        auto its = d.status.find("__can_index");
        if (its != d.status.end())
        {
            const uint32_t idx = static_cast<uint32_t>(its->second);
            if (idx == 0 || idx == 1)
            {
                out_instance = idx;
                return true;
            }
        }

        auto itn = d.num.find("__can_index");
        if (itn != d.num.end())
        {
            const uint32_t idx = static_cast<uint32_t>(itn->second);
            if (idx == 0 || idx == 1)
            {
                out_instance = idx;
                return true;
            }
        }

        return false;
    }

    bool LogicEngine::tryGetPcuCabinetId_(const DeviceData& d, uint32_t& out_cabinet_id)
    {
        auto itv = d.value.find("cabinet_id");
        if (itv != d.value.end())
        {
            out_cabinet_id = static_cast<uint32_t>(itv->second);
            return true;
        }

        auto its = d.status.find("cabinet_id");
        if (its != d.status.end())
        {
            out_cabinet_id = static_cast<uint32_t>(its->second);
            return true;
        }

        auto itn = d.num.find("cabinet_id");
        if (itn != d.num.end())
        {
            out_cabinet_id = static_cast<uint32_t>(itn->second);
            return true;
        }

        return false;
    }

    bool LogicEngine::tryGetPcuHeartbeat_(const DeviceData& d, uint32_t& out_heartbeat)
    {
        auto itv = d.value.find("heartbeat");
        if (itv != d.value.end())
        {
            out_heartbeat = static_cast<uint32_t>(itv->second);
            return true;
        }

        auto its = d.status.find("heartbeat");
        if (its != d.status.end())
        {
            out_heartbeat = static_cast<uint32_t>(its->second);
            return true;
        }

        auto itn = d.num.find("heartbeat");
        if (itn != d.num.end())
        {
            out_heartbeat = static_cast<uint32_t>(itn->second);
            return true;
        }

        return false;
    }
} // namespace control
