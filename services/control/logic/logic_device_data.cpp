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

    // static void recoverBmsRuntimeOnDeviceData_(const DeviceData& d,
    //                                            LogicContext& ctx)
    // {
    //     const std::string& dev = d.device_name;
    //     const uint64_t ts = d.timestamp;
    //
    //     if (dev != "BMS_1" &&
    //         dev != "BMS_2" &&
    //         dev != "BMS_3" &&
    //         dev != "BMS_4")
    //     {
    //         return;
    //     }
    //
    //     auto& x = ctx.bms_cache.items[dev];
    //     x.seen_once = true;
    //     x.online = true;
    //     x.last_rx_ms = ts;
    //
    //     if (x.last_st1_ms == 0) x.last_st1_ms = ts;
    //     if (x.last_st2_ms == 0) x.last_st2_ms = ts;
    //     if (x.last_fault1_ms == 0) x.last_fault1_ms = ts;
    //     if (x.last_fault2_ms == 0) x.last_fault2_ms = ts;
    //     if (x.last_current_limit_ms == 0) x.last_current_limit_ms = ts;
    //
    //     LOG_THROTTLE_MS("bms_recover_diag", 1000, LOG_COMM_D,
    //                     "[BMS][RECOVER] dev=%s ts=%llu bms_items=%zu",
    //                     dev.c_str(),
    //                     (unsigned long long)ts,
    //                     ctx.bms_cache.items.size());
    // }

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

    namespace {

        static bool readU32FromDeviceData_(const DeviceData& d,
                                           const char* key,
                                           uint32_t& out)
        {
            if (!key || !*key) return false;

            if (auto it = d.value.find(key); it != d.value.end()) {
                out = static_cast<uint32_t>(it->second);
                return true;
            }

            if (auto it = d.status.find(key); it != d.status.end()) {
                out = static_cast<uint32_t>(it->second);
                return true;
            }

            if (auto it = d.num.find(key); it != d.num.end()) {
                out = static_cast<uint32_t>(it->second);
                return true;
            }

            return false;
        }

        static bool readBoolFromDeviceData_(const DeviceData& d,
                                            const char* key,
                                            bool& out)
        {
            uint32_t u = 0;
            if (!readU32FromDeviceData_(d, key, u)) return false;
            out = (u != 0);
            return true;
        }

    } // namespace

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
        if (d.device_name == "HMI_SYS_HEARTBEAT") {
            ctx.hmi_seen_once = true;
            // 注意：如果你将 logic_context.h 里的变量改名了，这里同步改。没改就用原名：
            ctx.last_hmi_comm_ts = ts;
            return; // 纯心跳包，拦截后直接返回，不走后续的实体设备逻辑
        }

        // BMS：交给专用适配器维护控制面缓存
        if (d.device_name == "BMS")
        {
            bms_adapter_.onDeviceData(d, ts, ctx.bms_cache);
            return;
        }

        // UPS：记录原始到达与基础告警真源
// UPS：记录原始到达与 Q1/Q6/WA 真源
if (d.device_name == "UPS")
{
    auto& u = ctx.ups_faults;

    u.seen_once = true;
    u.online = true;
    u.last_rx_ms = ts;

    int ups_cmd = 0;
    if (auto it = d.value.find("__ups_cmd"); it != d.value.end()) {
        ups_cmd = static_cast<int>(it->second);
    }

    if (ups_cmd == 1) u.q1_seen = true;
    if (ups_cmd == 2) u.q6_seen = true;
    if (ups_cmd == 3) u.wa_seen = true;

    auto status_u32 = [&](const char* key, uint32_t defv = 0u) -> uint32_t {
        auto it = d.status.find(key);
        if (it != d.status.end()) return static_cast<uint32_t>(it->second);
        return defv;
    };

    auto value_i32 = [&](const char* key, int defv = 0) -> int {
        auto it = d.value.find(key);
        if (it != d.value.end()) return static_cast<int>(it->second);
        return defv;
    };

    auto apply_status_bits = [&](uint32_t bits) {
        const bool mains_abnormal_from_status = ((bits >> 7) & 0x1u) != 0;

        u.battery_low_state = ((bits >> 6) & 0x1u) != 0;
        u.bypass_mode       = ((bits >> 5) & 0x1u) != 0;
        u.ups_fault_state   = ((bits >> 4) & 0x1u) != 0;
        u.backup_mode       = ((bits >> 3) & 0x1u) != 0;
        u.self_test_active  = ((bits >> 2) & 0x1u) != 0;

        // Q6 warning bit3 也代表市电异常，所以这里只 OR，不直接清掉。
        u.mains_abnormal = mains_abnormal_from_status ||
                           (((u.warning_bits >> 3) & 0x1u) != 0);
    };

    if (ups_cmd == 1) {
        u.q1_status_bits = status_u32("q1.status.bits", 0u);
        apply_status_bits(u.q1_status_bits);
    }

    if (ups_cmd == 3) {
        u.wa_status_bits = status_u32("wa.status.bits", 0u);
        apply_status_bits(u.wa_status_bits);
    }

    if (ups_cmd == 2) {
        u.work_mode = value_i32("system.mode", u.work_mode);

        u.warning_bits = status_u32("warning.bits", 0u);
        u.internal_warning       = ((u.warning_bits >> 0)  & 0x1u) != 0;
        u.epo_active             = ((u.warning_bits >> 1)  & 0x1u) != 0;
        u.module_unlock          = ((u.warning_bits >> 2)  & 0x1u) != 0;
        u.mains_abnormal         = (((u.warning_bits >> 3) & 0x1u) != 0) ||
                                   (((u.q1_status_bits >> 7) & 0x1u) != 0) ||
                                   (((u.wa_status_bits >> 7) & 0x1u) != 0);
        u.neutral_lost           = ((u.warning_bits >> 4)  & 0x1u) != 0;
        u.mains_phase_error      = ((u.warning_bits >> 5)  & 0x1u) != 0;
        u.ln_reverse             = ((u.warning_bits >> 6)  & 0x1u) != 0;
        u.bypass_abnormal        = ((u.warning_bits >> 7)  & 0x1u) != 0;
        u.bypass_phase_error     = ((u.warning_bits >> 8)  & 0x1u) != 0;
        u.battery_not_connected  = ((u.warning_bits >> 9)  & 0x1u) != 0;
        u.battery_low_warning    = ((u.warning_bits >> 10) & 0x1u) != 0;
        u.battery_overcharge     = ((u.warning_bits >> 11) & 0x1u) != 0;
        u.battery_reverse        = ((u.warning_bits >> 12) & 0x1u) != 0;
        u.overload_warning       = ((u.warning_bits >> 13) & 0x1u) != 0;
        u.overload_alarm         = ((u.warning_bits >> 14) & 0x1u) != 0;
        u.fan_fault              = ((u.warning_bits >> 15) & 0x1u) != 0;
        u.bypass_cover_open      = ((u.warning_bits >> 16) & 0x1u) != 0;
        u.charger_fault          = ((u.warning_bits >> 17) & 0x1u) != 0;
        u.position_error         = ((u.warning_bits >> 18) & 0x1u) != 0;
        u.boot_condition_not_met = ((u.warning_bits >> 19) & 0x1u) != 0;
        u.redundancy_lost        = ((u.warning_bits >> 20) & 0x1u) != 0;
        u.module_loose           = ((u.warning_bits >> 21) & 0x1u) != 0;
        u.battery_maint_due      = ((u.warning_bits >> 22) & 0x1u) != 0;
        u.inspection_maint_due   = ((u.warning_bits >> 23) & 0x1u) != 0;
        u.warranty_maint_due     = ((u.warning_bits >> 24) & 0x1u) != 0;
        u.temp_low_warning       = ((u.warning_bits >> 25) & 0x1u) != 0;
        u.temp_high_warning      = ((u.warning_bits >> 26) & 0x1u) != 0;
        u.battery_overtemp       = ((u.warning_bits >> 27) & 0x1u) != 0;
        u.fan_maint_due          = ((u.warning_bits >> 28) & 0x1u) != 0;
        u.bus_cap_maint_due      = ((u.warning_bits >> 29) & 0x1u) != 0;
        u.system_overload        = ((u.warning_bits >> 30) & 0x1u) != 0;
        u.reserved_warning       = ((u.warning_bits >> 31) & 0x1u) != 0;

        u.fault_bits = status_u32("fault.bits", 0u);
        u.ups_fault_code = static_cast<int>(u.fault_bits);

        u.fault_code_1 = value_i32("fault.code.1", 0);
        u.fault_code_2 = value_i32("fault.code.2", 0);
        u.fault_code_3 = value_i32("fault.code.3", 0);
        u.fault_code_4 = value_i32("fault.code.4", 0);

        auto has_fault = [&](int code) -> bool {
            return u.fault_code_1 == code ||
                   u.fault_code_2 == code ||
                   u.fault_code_3 == code ||
                   u.fault_code_4 == code;
        };

        u.bus_softstart_timeout      = has_fault(1);
        u.bus_overvoltage_fault      = has_fault(2);
        u.bus_undervoltage_fault     = has_fault(3);
        u.bus_imbalance_fault        = has_fault(4);
        u.bus_short_circuit          = has_fault(5);

        u.inv_softstart_timeout      = has_fault(6);
        u.inv_overvoltage_fault      = has_fault(7);
        u.inv_undervoltage_fault     = has_fault(8);
        u.output_short_circuit       = has_fault(9);
        u.r_inv_short_circuit        = has_fault(10);
        u.s_inv_short_circuit        = has_fault(11);
        u.t_inv_short_circuit        = has_fault(12);
        u.rs_short_circuit           = has_fault(13);
        u.st_short_circuit           = has_fault(14);
        u.tr_short_circuit           = has_fault(15);

        u.reverse_power_fault        = has_fault(16);
        u.r_reverse_power_fault      = has_fault(17);
        u.s_reverse_power_fault      = has_fault(18);
        u.t_reverse_power_fault      = has_fault(19);
        u.total_reverse_power_fault  = has_fault(20);
        u.current_imbalance_fault    = has_fault(21);
        u.overload_fault             = has_fault(22);
        u.overtemp_fault             = has_fault(23);

        u.inv_relay_fail_close       = has_fault(24);
        u.inv_relay_stuck            = has_fault(25);
        u.mains_scr_fault            = has_fault(26);
        u.battery_scr_fault          = has_fault(27);
        u.bypass_scr_fault           = has_fault(28);
        u.rectifier_fault            = has_fault(29);
        u.input_overcurrent_fault    = has_fault(30);
        u.wiring_error               = has_fault(31);

        u.comm_cable_disconnected    = has_fault(32);
        u.host_cable_fault           = has_fault(33);
        u.can_comm_fault             = has_fault(34);
        u.sync_signal_fault          = has_fault(35);
        u.power_supply_fault         = has_fault(36);
        u.all_fan_fault              = has_fault(37);
        u.dsp_error                  = has_fault(38);
        u.charger_softstart_timeout  = has_fault(39);
        u.all_module_fault           = has_fault(40);

        u.mains_ntc_open_fault       = has_fault(41);
        u.mains_fuse_open_fault      = has_fault(42);
        u.output_imbalance_fault     = has_fault(43);
        u.input_mismatch_fault       = has_fault(44);
        u.eeprom_data_lost           = has_fault(45);
        u.mains_support_failed       = has_fault(46);
        u.power_failed               = has_fault(47);
        u.system_overload_fault      = has_fault(48);
        u.ads7869_error              = has_fault(49);
        u.bypass_mode_no_op          = has_fault(50);
        u.op_breaker_off_parallel    = has_fault(51);

        u.r_bus_fuse_fault           = has_fault(52);
        u.s_bus_fuse_fault           = has_fault(53);
        u.t_bus_fuse_fault           = has_fault(54);
        u.ntc_fault                  = has_fault(55);
        u.parallel_cable_fault       = has_fault(56);
        u.battery_fault              = has_fault(57);
        u.frequent_overcurrent_fault = has_fault(58);
        u.battery_overcharge_fault   = has_fault(59);
        u.epo_critical_fault         = has_fault(60);

        u.battery_overcharge_persist = false;
    }

    u.battery_low = u.battery_low_state ? 1 : 0;
    u.bypass_active = u.bypass_mode ? 1 : 0;

    u.alarm_any =
        u.mains_abnormal ||
        u.battery_low_state ||
        u.bypass_mode ||
        (u.warning_bits != 0);

    u.fault_any =
        u.ups_fault_state ||
        (u.fault_bits != 0);

    return;
}

         // Smoke：记录原始到达与 TSS 故障表真源
        //
        // 说明：
        // 1. onDeviceData_ 是“收到设备数据”的即时入口；
        // 2. 因此这里必须把 online 置 true；
        // 3. 后续真正的离线 aging 仍由 snapshot health / Scheduler 同步；
        // 4. FaultRuntimeMapper direct 链路会直接读取 ctx.smoke_faults。
        if (d.device_name == "SmokeSensor")
        {
            static constexpr double kTssTempAlarmHighC = 60.0;

            auto& s = ctx.smoke_faults;

            s.seen_once = true;
            s.online = true;
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

            auto status_on = [&](const char* key) -> bool
            {
                auto it = d.status.find(key);
                return it != d.status.end() && it->second != 0u;
            };

            // 0x101E: TSS_smoke_alarm
            s.smoke_alarm =
                status_on("TSS_smoke_alarm") ||
                ((alarm & 0x00FF) == 0x01);

            // 0x101F: TSS_temp_alarm，自定判断
            s.temp_alarm =
                (s.temp_c >= kTssTempAlarmHighC);

            // 0x1020: TSS_SS_alarm
            s.smoke_sensor_fault =
                status_on("TSS_SS_alarm") ||
                ((fault & 0x01) != 0);

            // 0x1021: TSS_SPollution_alarm
            s.smoke_pollution_fault =
                status_on("TSS_SPollution_alarm") ||
                ((fault & 0x02) != 0);

            // 0x1022: TSS_temp_fault
            s.temp_sensor_fault =
                status_on("TSS_temp_fault") ||
                ((fault & 0x04) != 0);

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
        //
        // 注意：
        // 1. Gas 协议是多气体循环更新；
        // 2. 单帧只代表当前 type_code 对应的一个气体；
        // 3. 这里不能因为当前帧正常就立刻清掉其它气体曾经上报的报警；
        // 4. 完整汇总真源由 logic_snapshot.cpp 根据 Aggregator 的 gas_channels 重算；
        // 5. 这里做即时 OR 聚合，避免 fault refresh 在线程时序上早于 snapshot 时漏判。
        if (d.device_name == "GasDetector")
        {
            auto& g = ctx.gas_faults;

            g.seen_once = true;
            g.online = true;
            g.last_rx_ms = ts;

            double gas_value = g.gas_value;
            if (auto it = d.num.find("gas_value"); it != d.num.end())
            {
                gas_value = it->second;
            }
            else if (d.gas.valid)
            {
                gas_value = d.gas.value;
            }

            uint32_t status = 0;
            bool has_status = false;

            if (auto it = d.num.find("status"); it != d.num.end())
            {
                status = static_cast<uint32_t>(it->second);
                has_status = true;
            }
            else if (d.gas.valid)
            {
                status = static_cast<uint32_t>(d.gas.status);
                has_status = true;
            }

            g.gas_value = gas_value;

            if (has_status)
            {
                // 多气体循环更新：先做 OR 聚合，不被最后一帧覆盖。
                // Snapshot 到来后会基于 Aggregator 的 gas_channels 做完整重算。
                g.status_code |= status;

                const bool sensor_fault = (status & 0x0001u) != 0;
                const bool low_alarm    = (status & 0x0002u) != 0;
                const bool high_alarm   = (status & 0x0004u) != 0;

                g.sensor_fault = g.sensor_fault || sensor_fault;
                g.low_alarm    = g.low_alarm    || low_alarm;
                g.high_alarm   = g.high_alarm   || high_alarm;

                g.alarm_any = g.low_alarm || g.high_alarm;
                g.fault_any = g.sensor_fault;
            }

            return;
        }

        // AirConditioner：记录原始到达与 AIR 故障真源
        // AirConditioner：分段 DeviceData 模式
        //
        // 关键原则：
        // 1. AirConditionerProto 现在每次只输出一个段的数据。
        // 2. S3_Alarms 才包含 alarm.* 字段。
        // 3. 非 S3 帧绝不能把 ctx.air_faults 里的告警/故障位清零。
        // 4. AIR 故障位只有在收到新的 alarm.* 段时才更新。
        if (d.device_name == "AirConditioner")
        {
            auto& a = ctx.air_faults;

            a.seen_once = true;
            a.last_rx_ms = ts;
            a.online = true;

            int air_stage = -1;
            if (auto it = d.value.find("__air_stage"); it != d.value.end()) {
                air_stage = static_cast<int>(it->second);
            }

            auto has_status_prefix = [&](const char* prefix) -> bool
            {
                const std::string p(prefix ? prefix : "");
                for (const auto& kv : d.status) {
                    if (kv.first.rfind(p, 0) == 0) {
                        return true;
                    }
                }
                return false;
            };

            const bool is_alarm_segment =
                (air_stage == 3) || has_status_prefix("alarm.");

            auto read_num = [&](std::initializer_list<const char*> keys,
                                double defv) -> double
            {
                for (const char* k : keys)
                {
                    auto it = d.num.find(k);
                    if (it != d.num.end()) {
                        return it->second;
                    }
                }
                return defv;
            };

            auto read_i32 = [&](std::initializer_list<const char*> keys,
                                int defv) -> int
            {
                for (const char* k : keys)
                {
                    auto itv = d.value.find(k);
                    if (itv != d.value.end()) {
                        return static_cast<int>(itv->second);
                    }

                    auto its = d.status.find(k);
                    if (its != d.status.end()) {
                        return static_cast<int>(its->second);
                    }

                    auto itn = d.num.find(k);
                    if (itn != d.num.end()) {
                        return static_cast<int>(itn->second);
                    }
                }
                return defv;
            };

            // ----------------------------
            // S1_RunState：只更新运行状态
            // ----------------------------
            if (air_stage == 1 || d.num.find("run.overall") != d.num.end()) {
                a.run_state =
                    read_i32({"run.overall"}, a.run_state);
            }

            // ----------------------------
            // S2_Sensors：只更新显示/业务量
            // ----------------------------
            if (air_stage == 2 ||
                d.num.find("temp.indoor_c") != d.num.end() ||
                d.num.find("humidity_percent") != d.num.end())
            {
                a.indoor_temp_c =
                    read_num({"temp.indoor_c"}, a.indoor_temp_c);

                a.humidity_percent =
                    read_num({"humidity_percent"}, a.humidity_percent);
            }

            // 0x0801 remote.power 已经不再主动轮询。
            // 若以后恢复该字段，只在字段存在时更新，不参与清故障。
            if (d.num.find("remote.power") != d.num.end()) {
                a.power_state =
                    read_i32({"remote.power"}, a.power_state);
            }

            // ----------------------------
            // S3_Alarms：只在告警段更新 0x1028~0x1048 真源
            // ----------------------------
            if (is_alarm_segment)
            {
                // 0x1028~0x102D：环境/保护类告警
                a.high_temp_alarm =
                    read_i32({"alarm.high_temp_alarm"}, 0) != 0;

                a.low_temp_alarm =
                    read_i32({"alarm.low_temp_alarm"}, 0) != 0;

                a.high_humidity_alarm =
                    read_i32({"alarm.high_humidity_alarm"}, 0) != 0;

                a.low_humidity_alarm =
                    read_i32({"alarm.low_humidity_alarm"}, 0) != 0;

                a.coil_freeze_protect =
                    read_i32({"alarm.coil_freeze_protect"}, 0) != 0;

                a.exhaust_high_temp_alarm =
                    read_i32({"alarm.exhaust_high_temp_alarm"}, 0) != 0;

                // 0x102E~0x1033：传感器失效
                a.coil_temp_sensor_fault =
                    read_i32({"alarm.coil_temp_sensor_fault"}, 0) != 0;

                a.outdoor_temp_sensor_fault =
                    read_i32({"alarm.outdoor_temp_sensor_fault"}, 0) != 0;

                a.condenser_temp_sensor_fault =
                    read_i32({"alarm.condenser_temp_sensor_fault"}, 0) != 0;

                a.indoor_temp_sensor_fault =
                    read_i32({"alarm.indoor_temp_sensor_fault"}, 0) != 0;

                a.exhaust_temp_sensor_fault =
                    read_i32({"alarm.exhaust_temp_sensor_fault"}, 0) != 0;

                a.humidity_sensor_fault =
                    read_i32({"alarm.humidity_sensor_fault"}, 0) != 0;

                // 0x1034~0x1038：执行部件故障
                a.internal_fan_fault =
                    read_i32({"alarm.internal_fan_fault"}, 0) != 0;

                a.external_fan_fault =
                    read_i32({"alarm.external_fan_fault"}, 0) != 0;

                a.compressor_fault =
                    read_i32({"alarm.compressor_fault"}, 0) != 0;

                a.heater_fault =
                    read_i32({"alarm.heater_fault"}, 0) != 0;

                a.emergency_fan_fault =
                    read_i32({"alarm.emergency_fan_fault"}, 0) != 0;

                // 0x1039~0x103D：压力/水浸/烟感/门禁
                a.high_pressure_alarm =
                    read_i32({"alarm.high_pressure_alarm"}, 0) != 0;

                a.low_pressure_alarm =
                    read_i32({"alarm.low_pressure_alarm"}, 0) != 0;

                a.water_alarm =
                    read_i32({"alarm.water_alarm"}, 0) != 0;

                a.smoke_alarm =
                    read_i32({"alarm.smoke_alarm"}, 0) != 0;

                a.gating_alarm =
                    read_i32({"alarm.gating_alarm"}, 0) != 0;

                // 0x103E~0x1040：锁定类
                a.high_pressure_lock =
                    read_i32({"alarm.high_pressure_lock"}, 0) != 0;

                a.low_pressure_lock =
                    read_i32({"alarm.low_pressure_lock"}, 0) != 0;

                a.exhaust_lock =
                    read_i32({"alarm.exhaust_lock"}, 0) != 0;

                // 0x1041~0x1048：电源/相序/频率/直流类
                a.ac_over_voltage_alarm =
                    read_i32({"alarm.ac_over_voltage_alarm"}, 0) != 0;

                a.ac_under_voltage_alarm =
                    read_i32({"alarm.ac_under_voltage_alarm"}, 0) != 0;

                a.ac_power_loss =
                    read_i32({"alarm.ac_power_loss"}, 0) != 0;

                a.lose_phase_alarm =
                    read_i32({"alarm.lose_phase_alarm"}, 0) != 0;

                a.freq_fault =
                    read_i32({"alarm.freq_fault"}, 0) != 0;

                a.anti_phase_alarm =
                    read_i32({"alarm.anti_phase_alarm"}, 0) != 0;

                a.dc_over_voltage_alarm =
                    read_i32({"alarm.dc_over_voltage_alarm"}, 0) != 0;

                a.dc_under_voltage_alarm =
                    read_i32({"alarm.dc_under_voltage_alarm"}, 0) != 0;

                // 综合告警：只在告警段重算
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

                // 综合故障：只在告警段重算
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
            }

            LOG_THROTTLE_MS(
                "air_runtime_segment_state",
                500,
                LOGINFO,
                "[AIR][RUNTIME] stage=%d is_alarm=%d online=%d "
                "low=%d freeze=%d exhaust=%d alarm_any=%d fault_any=%d",
                air_stage,
                is_alarm_segment ? 1 : 0,
                a.online ? 1 : 0,
                a.low_humidity_alarm ? 1 : 0,
                a.coil_freeze_protect ? 1 : 0,
                a.exhaust_high_temp_alarm ? 1 : 0,
                a.alarm_any ? 1 : 0,
                a.fault_any ? 1 : 0
            );

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
    void LogicEngine::feedHmiAlive(uint64_t now_ms, LogicContext& ctx) {
        ctx.hmi_seen_once = true;
        ctx.last_hmi_comm_ts = now_ms; // 沿用原名以减少全局改名，但其逻辑含义已变
    }
void LogicEngine::updatePcuOnlineState_(const DeviceData& d,
                                        uint64_t ts,
                                        LogicContext& ctx)
{
    uint32_t runtime_index = 0;
    uint32_t heartbeat = 0;

    if (!tryResolvePcuInstance_(d, runtime_index)) {
        LOG_THROTTLE_MS("pcu_runtime_unresolved_instance", 1000, LOG_COMM_W,
                        "[PCU][RUNTIME] unresolved instance device=%s",
                        d.device_name.c_str());
        return;
    }

    if (!tryGetPcuHeartbeat_(d, heartbeat)) {
        LOG_THROTTLE_MS("pcu_runtime_no_heartbeat", 1000, LOG_COMM_W,
                        "[PCU][RUNTIME] no heartbeat device=%s runtime_index=%u",
                        d.device_name.c_str(),
                        static_cast<unsigned>(runtime_index));
        return;
    }

    PcuOnlineState* st = nullptr;
    if (runtime_index == 0) {
        st = &ctx.pcu0_state;   // 内部 PCU_0，对应 HMI PCU1
    } else if (runtime_index == 1) {
        st = &ctx.pcu1_state;   // 内部 PCU_1，对应 HMI PCU2
    } else {
        return;
    }

    /*
     * 这里仅更新原始观测量。
     * 不在 DeviceData 到达时直接判 online/offline。
     * 最终 online 统一由 Tick 中 updatePcuRuntimeHealth_() 计算。
     */
    st->seen_once = true;
    st->last_rx_ms = ts;

    uint32_t cabinet_id = 0;
    if (tryGetPcuCabinetId_(d, cabinet_id)) {
        st->cabinet_id = cabinet_id;
    }

    uint32_t pcu_state = 0;
    if (readU32FromDeviceData_(d, "pcu_state", pcu_state) ||
        readU32FromDeviceData_(d, "__pcu.pcu_state_raw", pcu_state)) {
        st->pcu_state = pcu_state;
        st->pcu_state_valid = true;
    }

    bool estop = false;
    if (readBoolFromDeviceData_(d, "estop", estop) ||
        readBoolFromDeviceData_(d, "__pcu.estop_raw", estop)) {
        st->estop = estop;
    }

    if (!st->has_last_heartbeat) {
        st->last_heartbeat = heartbeat;
        st->has_last_heartbeat = true;
        st->last_hb_change_ms = ts;
        st->hb_repeat_count = 0;
        st->last_hb_delta = 0;
    } else {
        const uint32_t prev = st->last_heartbeat;
        const uint32_t curr = heartbeat;

        if (prev == curr) {
            st->hb_repeat_count += 1;
            st->last_hb_delta = 0;
        } else {
            /*
             * 心跳是 0~255 循环值。
             * 这里不因为跳变异常直接拉 offline，只记录诊断。
             * offline 仍由 rx_alive / hb_alive 统一决定。
             */
            int32_t delta = static_cast<int32_t>(curr) - static_cast<int32_t>(prev);
            if (prev > 240 && curr < 16) {
                delta = static_cast<int32_t>(curr + 256u - prev);
            }

            st->last_hb_delta = delta;

            if (delta != 1) {
                st->hb_jump_err_count += 1;
            }

            st->last_heartbeat = curr;
            st->last_hb_change_ms = ts;
            st->hb_repeat_count = 0;
        }
    }

    LOG_THROTTLE_MS("pcu_runtime_observe", 1000, LOG_COMM_D,
                    "[PCU][RUNTIME][OBS] idx=%u seen=1 hb=%u state=%u estop=%d cab=%u rx_ms=%llu",
                    static_cast<unsigned>(runtime_index),
                    static_cast<unsigned>(st->last_heartbeat),
                    static_cast<unsigned>(st->pcu_state),
                    st->estop ? 1 : 0,
                    static_cast<unsigned>(st->cabinet_id),
                    static_cast<unsigned long long>(st->last_rx_ms));
}

    bool LogicEngine::tryResolvePcuInstance_(const DeviceData& d,
                                             uint32_t& out_instance)
    {
        /*
         * 输出 out_instance 是 runtime index：
         *   0 = PCU_0 = HMI/故障表 PCU1
         *   1 = PCU_1 = HMI/故障表 PCU2
         *
         * 第八批收敛后，实例分路只认：
         *   1. __pcu.instance：1/2 转成 0/1
         *   2. 归一化后的 device_name：PCU_0 / PCU_1
         *
         * __can_index 只用于日志诊断，不再作为分路依据。
         * cabinet_id 只作为协议字段保存，不再作为分路依据。
         */

        uint32_t pcu_instance = 0;
        if (readU32FromDeviceData_(d, "__pcu.instance", pcu_instance) ||
            readU32FromDeviceData_(d, "pcu_instance", pcu_instance))
        {
            if (pcu_instance == 1) {
                out_instance = 0;
                return true;
            }

            if (pcu_instance == 2) {
                out_instance = 1;
                return true;
            }
        }

        if (d.device_name == "PCU_0") {
            out_instance = 0;
            return true;
        }

        if (d.device_name == "PCU_1") {
            out_instance = 1;
            return true;
        }

        uint32_t can_index = 0;
        const bool has_can_index =
            readU32FromDeviceData_(d, "__can_index", can_index);

        LOG_THROTTLE_MS("pcu_runtime_unresolved_instance_detail",
                        1000,
                        LOG_COMM_W,
                        "[PCU][RUNTIME] unresolved instance device=%s has_can=%d can_index=%u has_pcu_instance=%d",
                        d.device_name.c_str(),
                        has_can_index ? 1 : 0,
                        static_cast<unsigned>(can_index),
                        (d.value.find("__pcu.instance") != d.value.end()) ? 1 : 0);

        return false;
    }

    bool LogicEngine::tryGetPcuCabinetId_(const DeviceData& d,
                                          uint32_t& out_cabinet_id)
    {
        if (readU32FromDeviceData_(d, "cabinet_id", out_cabinet_id)) {
            return true;
        }

        if (readU32FromDeviceData_(d, "__pcu.cabinet_id", out_cabinet_id)) {
            return true;
        }

        return false;
    }

    bool LogicEngine::tryGetPcuHeartbeat_(const DeviceData& d,
                                          uint32_t& out_heartbeat)
    {
        if (readU32FromDeviceData_(d, "heartbeat", out_heartbeat)) {
            return true;
        }

        if (readU32FromDeviceData_(d, "__pcu.heartbeat", out_heartbeat)) {
            return true;
        }

        return false;
    }
} // namespace control
