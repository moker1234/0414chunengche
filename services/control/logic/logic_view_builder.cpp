// services/control/logic_view_builder.cpp
//
// logic_view 构建：从 latest_snapshot + context 派生 HMI/逻辑显示模型
//
#include "logic_engine.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>

#include "../utils/logger/logger.h"

namespace control {

    namespace {
        // // 设备在线超时时间（毫秒）
        // constexpr uint32_t UPS_ONLINE_TIMEOUT_MS    = 5000;
        // constexpr uint32_t SMOKE_ONLINE_TIMEOUT_MS  = 5000;
        // constexpr uint32_t GAS_ONLINE_TIMEOUT_MS    = 5000;
        // constexpr uint32_t AIRCON_ONLINE_TIMEOUT_MS = 5000;
        // constexpr uint32_t BMS_ONLINE_TIMEOUT_MS    = 5000;
        //
        // // PCU 在线状态
        // constexpr uint32_t PCU_RX_TIMEOUT_MS = 1500;
        // constexpr uint32_t PCU_HB_STALE_MS   = 3000;

        static double readGasChannelValueOr(const nlohmann::json& snap, int ch, double defv)
        {
            if (!snap.is_object()) return defv;

            auto it_items = snap.find("items");
            if (it_items == snap.end() || !it_items->is_object()) return defv;

            auto it_gas = it_items->find("GasDetector");
            if (it_gas == it_items->end() || !it_gas->is_object()) return defv;

            auto it_channels = it_gas->find("gas_channels");
            if (it_channels == it_gas->end() || !it_channels->is_object()) return defv;

            const std::string key = std::to_string(ch);
            auto it_ch = it_channels->find(key);
            if (it_ch == it_channels->end() || !it_ch->is_object()) return defv;

            auto it_valid = it_ch->find("valid");
            if (it_valid != it_ch->end())
            {
                bool valid = false;
                if (it_valid->is_boolean()) valid = it_valid->get<bool>();
                else if (it_valid->is_number()) valid = (it_valid->get<double>() != 0.0);
                if (!valid) return defv;
            }

            auto it_val = it_ch->find("value");
            if (it_val == it_ch->end()) return defv;

            if (it_val->is_number()) return it_val->get<double>();
            if (it_val->is_boolean()) return it_val->get<bool>() ? 1.0 : 0.0;
            return defv;
        }

        static int readGasChannelStatusOr(const nlohmann::json& snap, int ch, int defv)
        {
            if (!snap.is_object()) return defv;

            auto it_items = snap.find("items");
            if (it_items == snap.end() || !it_items->is_object()) return defv;

            auto it_gas = it_items->find("GasDetector");
            if (it_gas == it_items->end() || !it_gas->is_object()) return defv;

            auto it_channels = it_gas->find("gas_channels");
            if (it_channels == it_gas->end() || !it_channels->is_object()) return defv;

            const std::string key = std::to_string(ch);
            auto it_ch = it_channels->find(key);
            if (it_ch == it_channels->end() || !it_ch->is_object()) return defv;

            auto it_status = it_ch->find("status");
            if (it_status == it_ch->end()) return defv;

            if (it_status->is_number_integer()) return it_status->get<int>();
            if (it_status->is_number()) return static_cast<int>(it_status->get<double>());
            if (it_status->is_boolean()) return it_status->get<bool>() ? 1 : 0;
            return defv;
        }

        static const nlohmann::json* resolvePathCompatLocal(const nlohmann::json& root, const char* path)
        {
            if (!root.is_object() || !path || !*path) return nullptr;

            std::vector<std::string> tokens;
            {
                std::string cur;
                for (const char* p = path; ; ++p)
                {
                    const char c = *p;
                    if (c == '.' || c == '\0')
                    {
                        if (!cur.empty()) tokens.push_back(cur);
                        cur.clear();
                        if (c == '\0') break;
                    }
                    else
                    {
                        cur.push_back(c);
                    }
                }
            }

            const nlohmann::json* node = &root;

            for (size_t i = 0; i < tokens.size(); ++i)
            {
                if (!node->is_object()) return nullptr;

                const auto& tk = tokens[i];

                auto it = node->find(tk);
                if (it != node->end())
                {
                    node = &(*it);
                    continue;
                }

                std::string joined = tk;
                for (size_t k = i + 1; k < tokens.size(); ++k)
                {
                    joined.push_back('.');
                    joined += tokens[k];
                }

                auto it2 = node->find(joined);
                if (it2 != node->end())
                {
                    node = &(*it2);
                    return node;
                }

                return nullptr;
            }

            return node;
        }

        static double readJsonNumberOr(const nlohmann::json& root, const char* path, double defv)
        {
            const nlohmann::json* node = resolvePathCompatLocal(root, path);
            if (!node) return defv;

            if (node->is_number()) return node->get<double>();
            if (node->is_boolean()) return node->get<bool>() ? 1.0 : 0.0;
            return defv;
        }

        static bool readJsonBoolOr(const nlohmann::json& root, const char* path, bool defv)
        {
            const nlohmann::json* node = resolvePathCompatLocal(root, path);
            if (!node) return defv;

            if (node->is_boolean()) return node->get<bool>();
            if (node->is_number()) return node->get<double>() != 0.0;
            return defv;
        }

        static uint32_t currentUnixSeconds32_()
        {
            const std::time_t t = std::time(nullptr);
            // LOG_COMM_D("[LOGIC_ENGINE]timestamp: %ld", t);
            if (t <= 0) return 0;
            return static_cast<uint32_t>(t);
        }

        static void splitU32ToU16_(uint32_t v, uint16_t& hi, uint16_t& lo)
        {
            hi = static_cast<uint16_t>((v >> 16) & 0xFFFFu);
            lo = static_cast<uint16_t>(v & 0xFFFFu);
        }

        static const SnapshotItem* findSnapshotItemOrNull(const agg::SystemSnapshot& snap,
                                                  const std::string& device_name)
        {
            auto it = snap.items.find(device_name);
            if (it == snap.items.end()) return nullptr;
            return &it->second;
        }

        static void writeDiagFields(nlohmann::json& v,
                                    const std::string& prefix,
                                    const SnapshotItem* item)
        {
            if (!item)
            {
                v[prefix + "_online"] = 0;
                v[prefix + "_last_ok_ms"] = 0;
                v[prefix + "_last_offline_ms"] = 0;
                v[prefix + "_disconnect_count"] = 0;
                return;
            }

            v[prefix + "_online"] = item->online ? 1 : 0;
            v[prefix + "_last_ok_ms"] = static_cast<double>(item->last_ok_ms);
            v[prefix + "_last_offline_ms"] = static_cast<double>(item->last_offline_ms);
            v[prefix + "_disconnect_count"] = static_cast<int>(item->disconnect_count);
        }
    } // namespace

    void LogicEngine::rebuildLogicView_(LogicContext& ctx)
    {
        const nlohmann::json snap = ctx.latest_snapshot.toJson();
        nlohmann::json v = nlohmann::json::object();

        // 1) mode / status
        v["mode"] = (ctx.mode == LogicContext::Mode::Auto) ? 0 : 1;
        v["e_stop_latched"] = ctx.e_stop_latched ? 1 : 0;

        // 2) 系统级原始值
        const double soc_raw = readJsonNumberOr(snap, "system.bms_soc", 0.0);
        const double pack_v_raw = readJsonNumberOr(snap, "system.bms_pack_v", 0.0);
        const double pack_i_raw = readJsonNumberOr(snap, "system.bms_pack_i", 0.0);

        const bool gas_alarm_raw = readJsonBoolOr(snap, "system.gas_alarm", false);
        const bool smoke_alarm_raw = readJsonBoolOr(snap, "system.smoke_alarm", false);
        const bool ac_alarm_raw = readJsonBoolOr(snap, "system.ac_alarm", false);
        const bool bms_alarm_raw = readJsonBoolOr(snap, "system.bms_alarm_any", false);

        // 3) GasDetector
        const double gas_combustible_raw = readGasChannelValueOr(snap, 0, 0.0);
        const double gas_co_raw = readGasChannelValueOr(snap, 1, 0.0);
        const double gas_o2_raw = readGasChannelValueOr(snap, 2, 0.0);
        const double gas_temperature_raw = readGasChannelValueOr(snap, 3, 0.0);
        const double gas_humidity_raw = readGasChannelValueOr(snap, 4, 0.0);
        const double gas_co2_raw = readGasChannelValueOr(snap, 5, 0.0);

        const int gas_ch0_status = readGasChannelStatusOr(snap, 0, 0);
        const int gas_ch1_status = readGasChannelStatusOr(snap, 1, 0);
        const int gas_ch2_status = readGasChannelStatusOr(snap, 2, 0);
        const int gas_ch3_status = readGasChannelStatusOr(snap, 3, 0);
        const int gas_ch4_status = readGasChannelStatusOr(snap, 4, 0);
        const int gas_ch5_status = readGasChannelStatusOr(snap, 5, 0);

        const int gas_detector_state = static_cast<int>(
            readJsonNumberOr(snap, "items.GasDetector.health.state", 0.0));

        // 4) AirConditioner
        const double ac_overall_state_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.overall", 0.0);
        const double ac_inner_fan_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.inner_fan", 0.0);
        const double ac_outer_fan_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.outer_fan", 0.0);
        const double ac_compressor_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.compressor", 0.0);
        const double ac_heater_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.heater", 0.0);
        const double ac_em_fan_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.run_state.fields.run.em_fan", 0.0);

        const double ac_coil_temp_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.temp.coil_c", 0.0);
        const double ac_outdoor_temp_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.temp.outdoor_c", 0.0);
        const double ac_condense_temp_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.temp.condense_c", 0.0);
        const double ac_indoor_temp_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.temp.indoor_c", 0.0);
        const double ac_humidity_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.humidity_percent", 0.0);
        const double ac_exhaust_temp_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.temp.exhaust_c", 0.0);
        const double ac_current_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.current_a", 0.0);
        const double ac_ac_voltage_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.ac_voltage_v", 0.0);
        const double ac_dc_voltage_raw = readJsonNumberOr(
            snap, "items.AirConditioner.data.sensor_state.fields.dc_voltage_v", 0.0);

        // 5) SmokeSensor
        const double smoke_alarm_num_raw = readJsonNumberOr(
            snap, "items.SmokeSensor.data.num.alarm", 0.0);
        const double smoke_fault_raw = readJsonNumberOr(
            snap, "items.SmokeSensor.data.num.fault", 0.0);
        const double smoke_warn_level_raw = readJsonNumberOr(
            snap, "items.SmokeSensor.data.num.warn_level", 0.0);
        const double smoke_percent_raw = readJsonNumberOr(
            snap, "items.SmokeSensor.data.num.smoke_percent", 0.0);
        const double smoke_temperature_raw = readJsonNumberOr(
            snap, "items.SmokeSensor.data.num.temp", 0.0);

        // 6) UPS
        const double ups_system_mode_raw = readJsonNumberOr(
            snap, "items.UPS.data.Q6.value.system.mode", 0.0);
        const double ups_battery_remain_sec_raw = readJsonNumberOr(
            snap, "items.UPS.data.Q6.value.battery.remain.sec", 0.0);
        const double ups_battery_capacity_raw = readJsonNumberOr(
            snap, "items.UPS.data.Q6.value.battery.capacity", 0.0);
        const double ups_fault_bits_raw = readJsonNumberOr(
            snap, "items.UPS.data.Q6.status.fault.bits", 0.0);
        const double ups_warning_bits_raw = readJsonNumberOr(
            snap, "items.UPS.data.Q6.status.warning.bits", 0.0);

        // 7) 显示级处理
        const double soc_disp = std::clamp(soc_raw, 0.0, 100.0);
        const double pack_v_disp = std::max(0.0, pack_v_raw);
        const double pack_i_disp = pack_i_raw;

        const double gas_combustible_disp = std::max(0.0, gas_combustible_raw);
        const double gas_co_disp = std::max(0.0, gas_co_raw);
        const double gas_o2_disp = std::max(0.0, gas_o2_raw);
        const double gas_temperature_disp = gas_temperature_raw;
        const double gas_humidity_disp = std::clamp(gas_humidity_raw, 0.0, 100.0);
        const double gas_co2_disp = std::max(0.0, gas_co2_raw);

        const double ac_coil_temp_disp = ac_coil_temp_raw;
        const double ac_outdoor_temp_disp = ac_outdoor_temp_raw;
        const double ac_condense_temp_disp = ac_condense_temp_raw;
        const double ac_indoor_temp_disp = ac_indoor_temp_raw;
        const double ac_humidity_disp = std::clamp(ac_humidity_raw, 0.0, 100.0);
        const double ac_exhaust_temp_disp = ac_exhaust_temp_raw;
        const double ac_current_disp = std::max(0.0, ac_current_raw);
        const double ac_ac_voltage_disp = std::max(0.0, ac_ac_voltage_raw);
        const double ac_dc_voltage_disp = std::max(0.0, ac_dc_voltage_raw);

        const double smoke_alarm_num_disp = std::max(0.0, smoke_alarm_num_raw);
        const double smoke_fault_disp = std::max(0.0, smoke_fault_raw);
        const double smoke_warn_level_disp = std::max(0.0, smoke_warn_level_raw);
        const double smoke_percent_disp = std::max(0.0, smoke_percent_raw);
        const double smoke_temperature_disp = smoke_temperature_raw;

        const double ups_system_mode_disp = std::max(0.0, ups_system_mode_raw);
        const double ups_battery_capacity_disp = std::clamp(ups_battery_capacity_raw, 0.0, 100.0);
        const double ups_fault_bits_disp = std::max(0.0, ups_fault_bits_raw);
        const double ups_warning_bits_disp = std::max(0.0, ups_warning_bits_raw);

        const uint32_t ups_battery_remain_sec_u32 =
            static_cast<uint32_t>(std::max(0.0, ups_battery_remain_sec_raw));

        const uint16_t ups_battery_remain_sec_hi =
            static_cast<uint16_t>((ups_battery_remain_sec_u32 >> 16) & 0xFFFFu);
        const uint16_t ups_battery_remain_sec_lo =
            static_cast<uint16_t>(ups_battery_remain_sec_u32 & 0xFFFFu);

        // 8) 系统类
        v["soc"] = soc_disp;
        v["pack_voltage"] = pack_v_disp;
        v["pack_current"] = pack_i_disp;

        // 9) AC
        v["ac_overall_state"] = static_cast<int>(ac_overall_state_raw);
        v["ac_inner_fan"] = static_cast<int>(ac_inner_fan_raw);
        v["ac_outer_fan"] = static_cast<int>(ac_outer_fan_raw);
        v["ac_compressor"] = static_cast<int>(ac_compressor_raw);
        v["ac_heater"] = static_cast<int>(ac_heater_raw);
        v["ac_em_fan"] = static_cast<int>(ac_em_fan_raw);

        v["ac_coil_temp"] = ac_coil_temp_disp;
        v["ac_outdoor_temp"] = ac_outdoor_temp_disp;
        v["ac_condense_temp"] = ac_condense_temp_disp;
        v["ac_indoor_temp"] = ac_indoor_temp_disp;
        v["ac_humidity"] = ac_humidity_disp;
        v["ac_exhaust_temp"] = ac_exhaust_temp_disp;
        v["ac_current"] = ac_current_disp;
        v["ac_ac_voltage"] = ac_ac_voltage_disp;
        v["ac_dc_voltage"] = ac_dc_voltage_disp;

        // 10) Smoke
        v["smoke_alarm"] = static_cast<int>(smoke_alarm_num_disp);
        v["smoke_fault"] = static_cast<int>(smoke_fault_disp);
        v["smoke_warn_level"] = static_cast<int>(smoke_warn_level_disp);
        v["smoke_percent"] = smoke_percent_disp;
        v["smoke_temperature"] = smoke_temperature_disp;

        // 11) Gas
        v["gas_detector_state"] = gas_detector_state;
        v["gas_combustible"] = gas_combustible_disp;
        v["gas_co"] = gas_co_disp;
        v["gas_o2"] = gas_o2_disp;
        v["gas_temperature"] = gas_temperature_disp;
        v["gas_humidity"] = gas_humidity_disp;
        v["gas_co2"] = gas_co2_disp;

        v["gas_combustible_status"] = gas_ch0_status;
        v["gas_co_status"] = gas_ch1_status;
        v["gas_o2_status"] = gas_ch2_status;
        v["gas_temperature_status"] = gas_ch3_status;
        v["gas_humidity_status"] = gas_ch4_status;
        v["gas_co2_status"] = gas_ch5_status;

        // 12) UPS
        v["ups_system_mode"] = static_cast<int>(ups_system_mode_disp);
        v["ups_battery_remain_sec_hi"] = ups_battery_remain_sec_hi;
        v["ups_battery_remain_sec_lo"] = ups_battery_remain_sec_lo;
        v["ups_battery_capacity"] = static_cast<int>(ups_battery_capacity_disp);
        v["ups_fault_bits"] = static_cast<int>(ups_fault_bits_disp);
        v["ups_warning_bits"] = static_cast<int>(ups_warning_bits_disp);

        // 12.5) BMS 4 路概览
        {
            const nlohmann::json jbms = bms_adapter_.buildLogicView(ctx.bms_cache);
            v["bms"] = jbms;

            for (auto it = jbms.begin(); it != jbms.end(); ++it)
            {
                if (it.value().is_primitive()) {
                    v[it.key()] = it.value();
                }
            }

            v["system_bms_fault_any"] = jbms.value("bms_fault_any", 0);
            v["system_bms_fault_block_any"] = jbms.value("bms_fault_block_any", 0);
            v["system_bms_fault_count_block_hv"] = jbms.value("bms_fault_count_block_hv", 0);

            v["system_bms_ins_any"] = jbms.value("bms_ins_any", 0);
            v["system_bms_ins_block_any"] = jbms.value("bms_ins_block_any", 0);
            v["system_bms_ins_count_low"] = jbms.value("bms_ins_count_low", 0);
            v["system_bms_ins_valid_count"] = jbms.value("bms_ins_valid_count", 0);

            v["system_bms_f2_any"] = jbms.value("bms_f2_any", 0);
            v["system_bms_f2_hv_block_any"] = jbms.value("bms_f2_hv_block_any", 0);
            v["system_bms_f2_contact_err_any"] = jbms.value("bms_f2_contact_err_any", 0);
            v["system_bms_f2_comm_err_any"] = jbms.value("bms_f2_comm_err_any", 0);

            v["system_bms_f1_any"] = jbms.value("bms_f1_any", 0);
            v["system_bms_f1_block_hv_any"] = jbms.value("bms_f1_block_hv_any", 0);
            v["system_bms_f1_contact_err_any"] = jbms.value("bms_f1_contact_err_any", 0);
        }

                // 12.55) BMS runtime / age / online（第四批：直接读第三批 Tick aging 真源）
        {
            int runtime_online_count = 0;
            int runtime_any_stale = 0;

            for (int idx = 1; idx <= 4; ++idx)
            {
                const std::string key  = "BMS_" + std::to_string(idx);
                const std::string base = "bms_" + std::to_string(idx);

                double st1_age_ms = 0.0, st2_age_ms = 0.0, st3_age_ms = 0.0, st4_age_ms = 0.0;
                double st5_age_ms = 0.0, st6_age_ms = 0.0, st7_age_ms = 0.0;
                double elec_energy_age_ms = 0.0;
                double current_limit_age_ms = 0.0;
                double tm2b_age_ms = 0.0, fire2b_age_ms = 0.0;
                double fault1_age_ms = 0.0, fault2_age_ms = 0.0;

                int st1_online = 0, st2_online = 0, st3_online = 0, st4_online = 0;
                int st5_online = 0, st6_online = 0, st7_online = 0;
                int elec_energy_online = 0;
                int current_limit_online = 0;
                int tm2b_online = 0, fire2b_online = 0;
                int fault1_online = 0, fault2_online = 0;

                int runtime_online = 0;
                int runtime_fault_stale = 0;
                int offline_reason_code = 0;
                std::string offline_reason_text = "None";

                auto it = ctx.bms_cache.items.find(key);
                if (it != ctx.bms_cache.items.end())
                {
                    const auto& x = it->second;

                    st1_age_ms = x.st1_age_ms;
                    st2_age_ms = x.st2_age_ms;
                    st3_age_ms = x.st3_age_ms;
                    st4_age_ms = x.st4_age_ms;
                    st5_age_ms = x.st5_age_ms;
                    st6_age_ms = x.st6_age_ms;
                    st7_age_ms = x.st7_age_ms;

                    elec_energy_age_ms = x.elec_energy_age_ms;
                    current_limit_age_ms = x.current_limit_age_ms;
                    tm2b_age_ms = x.tm2b_age_ms;
                    fire2b_age_ms = x.fire2b_age_ms;
                    fault1_age_ms = x.fault1_age_ms;
                    fault2_age_ms = x.fault2_age_ms;

                    st1_online = x.st1_online ? 1 : 0;
                    st2_online = x.st2_online ? 1 : 0;
                    st3_online = x.st3_online ? 1 : 0;
                    st4_online = x.st4_online ? 1 : 0;
                    st5_online = x.st5_online ? 1 : 0;
                    st6_online = x.st6_online ? 1 : 0;
                    st7_online = x.st7_online ? 1 : 0;

                    elec_energy_online = x.elec_energy_online ? 1 : 0;
                    current_limit_online = x.current_limit_online ? 1 : 0;
                    tm2b_online = x.tm2b_online ? 1 : 0;
                    fire2b_online = x.fire2b_online ? 1 : 0;
                    fault1_online = x.fault1_online ? 1 : 0;
                    fault2_online = x.fault2_online ? 1 : 0;

                    runtime_online = x.online ? 1 : 0;
                    runtime_fault_stale = x.runtime_fault_stale ? 1 : 0;
                    offline_reason_code = static_cast<int>(x.offline_reason_code);
                    offline_reason_text = x.offline_reason_text.empty() ? "None" : x.offline_reason_text;

                    if (runtime_online) runtime_online_count++;
                    if (runtime_fault_stale) runtime_any_stale = 1;
                }

                v[base + "_st1_age_ms"] = st1_age_ms; v[base + "_st1_online"] = st1_online;
                v[base + "_st2_age_ms"] = st2_age_ms; v[base + "_st2_online"] = st2_online;
                v[base + "_st3_age_ms"] = st3_age_ms; v[base + "_st3_online"] = st3_online;
                v[base + "_st4_age_ms"] = st4_age_ms; v[base + "_st4_online"] = st4_online;
                v[base + "_st5_age_ms"] = st5_age_ms; v[base + "_st5_online"] = st5_online;
                v[base + "_st6_age_ms"] = st6_age_ms; v[base + "_st6_online"] = st6_online;
                v[base + "_st7_age_ms"] = st7_age_ms; v[base + "_st7_online"] = st7_online;

                v[base + "_elec_energy_age_ms"] = elec_energy_age_ms;
                v[base + "_elec_energy_online"] = elec_energy_online;

                v[base + "_current_limit_online"] = current_limit_online;
                v[base + "_current_limit_age_ms"] = current_limit_age_ms;

                v[base + "_tm2b_online"] = tm2b_online;
                v[base + "_tm2b_age_ms"] = tm2b_age_ms;

                v[base + "_fire2b_online"] = fire2b_online;
                v[base + "_fire2b_age_ms"] = fire2b_age_ms;

                v[base + "_fault1_online"] = fault1_online;
                v[base + "_fault1_age_ms"] = fault1_age_ms;
                v[base + "_fault2_online"] = fault2_online;
                v[base + "_fault2_age_ms"] = fault2_age_ms;

                v[base + "_runtime_online"] = runtime_online;
                v[base + "_runtime_fault_stale"] = runtime_fault_stale;
                v[base + "_offline_reason_code"] = offline_reason_code;
                v[base + "_offline_reason_text"] = offline_reason_text;

                if (v.contains("bms") && v["bms"].is_object() &&
                    v["bms"].contains(base) && v["bms"][base].is_object())
                {
                    auto& jb = v["bms"][base];
                    jb["st1_age_ms"] = st1_age_ms; jb["st1_online"] = st1_online;
                    jb["st2_age_ms"] = st2_age_ms; jb["st2_online"] = st2_online;
                    jb["st3_age_ms"] = st3_age_ms; jb["st3_online"] = st3_online;
                    jb["st4_age_ms"] = st4_age_ms; jb["st4_online"] = st4_online;
                    jb["st5_age_ms"] = st5_age_ms; jb["st5_online"] = st5_online;
                    jb["st6_age_ms"] = st6_age_ms; jb["st6_online"] = st6_online;
                    jb["st7_age_ms"] = st7_age_ms; jb["st7_online"] = st7_online;

                    jb["elec_energy_age_ms"] = elec_energy_age_ms;
                    jb["elec_energy_online"] = elec_energy_online;

                    jb["current_limit_online"] = current_limit_online;
                    jb["current_limit_age_ms"] = current_limit_age_ms;

                    jb["tm2b_online"] = tm2b_online;
                    jb["tm2b_age_ms"] = tm2b_age_ms;

                    jb["fire2b_online"] = fire2b_online;
                    jb["fire2b_age_ms"] = fire2b_age_ms;

                    jb["fault1_online"] = fault1_online;
                    jb["fault1_age_ms"] = fault1_age_ms;
                    jb["fault2_online"] = fault2_online;
                    jb["fault2_age_ms"] = fault2_age_ms;

                    jb["runtime_online"] = runtime_online;
                    jb["runtime_fault_stale"] = runtime_fault_stale;
                    jb["offline_reason_code"] = offline_reason_code;
                    jb["offline_reason_text"] = offline_reason_text;
                }
            }

            v["bms_runtime_online_count"] = runtime_online_count;
            v["bms_runtime_any_stale"] = runtime_any_stale;
        }

        // 12.6) BMS 命令发送状态
        if (bms_cmd_mgr_inited_)
        {
            const auto cmd_view = bms_cmd_mgr_.buildCommandView(ctx.last_event_ts, 500);

            int tx_count_active = 0;
            bool tx_any = false;

            for (int idx = 1; idx <= 4; ++idx)
            {
                const std::string base = "bms_" + std::to_string(idx);

                int cmd_hv_onoff = 0;
                int cmd_enable = 0;
                int cmd_life = 0;
                int cmd_valid = 0;
                int cmd_tx_alive = 0;
                int cmd_has_last_sent = 0;
                int cmd_reason_code = 0;

                double cmd_last_build_ms = 0.0;
                double cmd_last_send_ms = 0.0;

                std::string cmd_source;
                std::string cmd_reason_text;
                std::string cmd_hv_text;
                std::string cmd_tx_state_text;

                int cmd_dlc = 0;
                double cmd_last_send_age_ms = 0.0;

                std::string cmd_can_id_hex;
                std::string cmd_frame_hex;

                auto it = cmd_view.find(static_cast<uint32_t>(idx));
                if (it != cmd_view.end())
                {
                    const auto& cv = it->second;

                    cmd_hv_onoff = cv.hv_onoff;
                    cmd_enable = cv.system_enable;
                    cmd_life = cv.life_signal;
                    cmd_valid = cv.valid ? 1 : 0;
                    cmd_tx_alive = cv.tx_alive ? 1 : 0;
                    cmd_has_last_sent = cv.has_last_sent ? 1 : 0;
                    cmd_last_build_ms = static_cast<double>(cv.last_build_ts_ms);
                    cmd_last_send_ms = static_cast<double>(cv.last_send_ts_ms);
                    cmd_source = cv.source;
                    cmd_reason_code = cv.reason_code;
                    cmd_reason_text = cv.reason_text;
                    cmd_hv_text = cv.hv_text;
                    cmd_tx_state_text = cv.tx_state_text;

                    cmd_dlc = cv.dlc;
                    cmd_last_send_age_ms = cv.last_send_age_ms;
                    cmd_can_id_hex = cv.can_id_hex;
                    cmd_frame_hex = cv.frame_hex;

                    if (cv.tx_alive)
                    {
                        tx_any = true;
                        tx_count_active++;
                    }
                }

                v[base + "_cmd_hv_onoff"] = cmd_hv_onoff;
                v[base + "_cmd_enable"] = cmd_enable;
                v[base + "_cmd_life"] = cmd_life;
                v[base + "_cmd_valid"] = cmd_valid;
                v[base + "_cmd_tx_alive"] = cmd_tx_alive;
                v[base + "_cmd_has_last_sent"] = cmd_has_last_sent;
                v[base + "_cmd_last_build_ms"] = cmd_last_build_ms;
                v[base + "_cmd_last_send_ms"] = cmd_last_send_ms;
                v[base + "_cmd_source"] = cmd_source;

                v[base + "_cmd_reason_code"] = cmd_reason_code;
                v[base + "_cmd_reason_text"] = cmd_reason_text;
                v[base + "_cmd_hv_text"] = cmd_hv_text;
                v[base + "_cmd_tx_state_text"] = cmd_tx_state_text;

                v[base + "_cmd_dlc"] = cmd_dlc;
                v[base + "_cmd_last_send_age_ms"] = cmd_last_send_age_ms;
                v[base + "_cmd_can_id_hex"] = cmd_can_id_hex;
                v[base + "_cmd_frame_hex"] = cmd_frame_hex;

                if (v.contains("bms") && v["bms"].is_object() &&
                    v["bms"].contains(base) && v["bms"][base].is_object())
                {
                    auto& jb = v["bms"][base];
                    jb["cmd_hv_onoff"] = cmd_hv_onoff;
                    jb["cmd_enable"] = cmd_enable;
                    jb["cmd_life"] = cmd_life;
                    jb["cmd_valid"] = cmd_valid;
                    jb["cmd_tx_alive"] = cmd_tx_alive;
                    jb["cmd_has_last_sent"] = cmd_has_last_sent;
                    jb["cmd_last_build_ms"] = cmd_last_build_ms;
                    jb["cmd_last_send_ms"] = cmd_last_send_ms;
                    jb["cmd_source"] = cmd_source;

                    jb["cmd_reason_code"] = cmd_reason_code;
                    jb["cmd_reason_text"] = cmd_reason_text;
                    jb["cmd_hv_text"] = cmd_hv_text;
                    jb["cmd_tx_state_text"] = cmd_tx_state_text;

                    jb["cmd_dlc"] = cmd_dlc;
                    jb["cmd_last_send_age_ms"] = cmd_last_send_age_ms;
                    jb["cmd_can_id_hex"] = cmd_can_id_hex;
                    jb["cmd_frame_hex"] = cmd_frame_hex;
                }
            }

            v["bms_cmd_tx_any"] = tx_any ? 1 : 0;
            v["bms_cmd_tx_count_active"] = tx_count_active;

            int any_open_reason = 0;
            int any_fault_block = 0;

            for (const auto& kv : cmd_view)
            {
                const auto& cv = kv.second;
                if (cv.reason_code == 3) any_open_reason = 1;
                if (cv.reason_code == 4 || cv.reason_code == 5 || cv.reason_code == 6) {
                    any_fault_block = 1;
                }
            }

            v["bms_cmd_reason_any_open"] = any_open_reason;
            v["bms_cmd_reason_any_fault_block"] = any_fault_block;
            v["bms_cmd_last_any_send_ms"] = static_cast<double>(ctx.last_event_ts);
        }

        // 13) 告警位汇总
        v["gas_alarm"] = gas_alarm_raw ? 1 : 0;
        v["smoke_alarm_any"] = smoke_alarm_raw ? 1 : 0;
        v["ac_alarm"] = ac_alarm_raw ? 1 : 0;
        v["bms_alarm_any"] = bms_alarm_raw ? 1 : 0;

        uint16_t alarm_bits = 0;
        if (gas_alarm_raw) alarm_bits |= (1u << 0);
        if (smoke_alarm_raw) alarm_bits |= (1u << 1);
        if (ac_alarm_raw) alarm_bits |= (1u << 2);
        if (bms_alarm_raw) alarm_bits |= (1u << 3);
        if (v.value("system_bms_fault_block_any", 0) != 0) alarm_bits |= (1u << 4);
        if (v.value("system_bms_ins_block_any", 0) != 0) alarm_bits |= (1u << 5);
        if (v.value("system_bms_f2_hv_block_any", 0) != 0) alarm_bits |= (1u << 6);
        if (v.value("system_bms_f2_comm_err_any", 0) != 0) alarm_bits |= (1u << 7);
        if (v.value("system_bms_f1_block_hv_any", 0) != 0) alarm_bits |= (1u << 8);
        v["alarm_bits"] = alarm_bits;

        // 14) 通信状态
        const uint64_t now_ms = ctx.last_event_ts;

        v["ups_online"] =
            isSnapshotItemOnline_(ctx.latest_snapshot, "UPS", now_ms, 0) ? 1 : 0;
        v["smoke_online"] =
            isSnapshotItemOnline_(ctx.latest_snapshot, "SmokeSensor", now_ms, 0) ? 1 : 0;
        v["gas_online"] =
            isSnapshotItemOnline_(ctx.latest_snapshot, "GasDetector", now_ms, 0) ? 1 : 0;
        v["aircon_online"] =
            isSnapshotItemOnline_(ctx.latest_snapshot, "AirConditioner", now_ms, 0) ? 1 : 0;

        auto& bms = ctx.bms_cache.items;
        auto bms_online = [&](int idx) -> int {
            const std::string key = "BMS_" + std::to_string(idx);
            auto it = ctx.bms_cache.items.find(key);
            if (it == ctx.bms_cache.items.end()) return 0;

            const auto& x = it->second;
            return x.online ? 1 : 0;
        };

        v["bms1_online"] = bms_online(1);
        v["bms2_online"] = bms_online(2);
        v["bms3_online"] = bms_online(3);
        v["bms4_online"] = bms_online(4);

        int bms_count_online = 0;
        bms_count_online += v["bms1_online"].get<int>();
        bms_count_online += v["bms2_online"].get<int>();
        bms_count_online += v["bms3_online"].get<int>();
        bms_count_online += v["bms4_online"].get<int>();

        v["bms_count_online"] = bms_count_online;

        for (int idx = 1; idx <= 4; ++idx)
        {
            const std::string key = "BMS_" + std::to_string(idx);
            const std::string base = "bms" + std::to_string(idx);

            auto it = ctx.bms_cache.items.find(key);
            if (it == ctx.bms_cache.items.end())
            {
                v[base + "_last_ok_ms"] = 0.0;
                v[base + "_last_offline_ms"] = 0.0;
                v[base + "_disconnect_count"] = 0;
                v[base + "_offline_reason_code"] = 0;
                v[base + "_offline_reason_text"] = "None";
                continue;
            }

            const auto& x = it->second;
            v[base + "_last_ok_ms"] = static_cast<double>(x.last_rx_ms);
            v[base + "_last_offline_ms"] = static_cast<double>(x.last_offline_ms);
            v[base + "_disconnect_count"] = static_cast<int>(x.disconnect_count);
            v[base + "_offline_reason_code"] = static_cast<int>(x.offline_reason_code);
            v[base + "_offline_reason_text"] =
                x.offline_reason_text.empty() ? "None" : x.offline_reason_text;
        }

        // 注意映射关系：can0 -> PCU_0 -> pcu1_online，can1 -> PCU_1 -> pcu2_online
        v["pcu1_online"] = ctx.pcu0_state.online ? 1 : 0;
        v["pcu2_online"] = ctx.pcu1_state.online ? 1 : 0;

        v["pcu1_rx_alive"] = ctx.pcu0_state.rx_alive ? 1 : 0;
        v["pcu1_hb_alive"] = ctx.pcu0_state.hb_alive ? 1 : 0;
        v["pcu1_last_rx_ms"] = static_cast<double>(ctx.pcu0_state.last_rx_ms);
        v["pcu1_last_hb_change_ms"] = static_cast<double>(ctx.pcu0_state.last_hb_change_ms);
        v["pcu1_heartbeat"] = static_cast<int>(ctx.pcu0_state.last_heartbeat);
        v["pcu1_hb_repeat_count"] = static_cast<int>(ctx.pcu0_state.hb_repeat_count);
        v["pcu1_hb_jump_err_count"] = static_cast<int>(ctx.pcu0_state.hb_jump_err_count);
        v["pcu1_online_reason_code"] = static_cast<int>(ctx.pcu0_state.offline_reason_code);
        v["pcu1_online_reason_text"] = pcuOfflineReasonText_(ctx.pcu0_state.offline_reason_code);
        v["pcu1_last_rx_age_ms"] = ctx.pcu0_state.last_rx_age_ms;
        v["pcu1_last_hb_change_age_ms"] = ctx.pcu0_state.last_hb_change_age_ms;
        v["pcu1_last_hb_delta"] = static_cast<int>(ctx.pcu0_state.last_hb_delta);

        v["pcu2_rx_alive"] = ctx.pcu1_state.rx_alive ? 1 : 0;
        v["pcu2_hb_alive"] = ctx.pcu1_state.hb_alive ? 1 : 0;
        v["pcu2_last_rx_ms"] = static_cast<double>(ctx.pcu1_state.last_rx_ms);
        v["pcu2_last_hb_change_ms"] = static_cast<double>(ctx.pcu1_state.last_hb_change_ms);
        v["pcu2_heartbeat"] = static_cast<int>(ctx.pcu1_state.last_heartbeat);
        v["pcu2_hb_repeat_count"] = static_cast<int>(ctx.pcu1_state.hb_repeat_count);
        v["pcu2_hb_jump_err_count"] = static_cast<int>(ctx.pcu1_state.hb_jump_err_count);
        v["pcu2_online_reason_code"] = static_cast<int>(ctx.pcu1_state.offline_reason_code);
        v["pcu2_online_reason_text"] = pcuOfflineReasonText_(ctx.pcu1_state.offline_reason_code);
        v["pcu2_last_rx_age_ms"] = ctx.pcu1_state.last_rx_age_ms;
        v["pcu2_last_hb_change_age_ms"] = ctx.pcu1_state.last_hb_change_age_ms;
        v["pcu2_last_hb_delta"] = static_cast<int>(ctx.pcu1_state.last_hb_delta);

        // ============================================================
        // 统一设备断连诊断字段（供 HMI / 日志 / 后续诊断页使用）
        // ============================================================
        {
            const auto* gas_item   = findSnapshotItemOrNull(ctx.latest_snapshot, "GasDetector");
            const auto* smoke_item = findSnapshotItemOrNull(ctx.latest_snapshot, "SmokeSensor");
            const auto* air_item   = findSnapshotItemOrNull(ctx.latest_snapshot, "AirConditioner");
            const auto* ups_item   = findSnapshotItemOrNull(ctx.latest_snapshot, "UPS");

            writeDiagFields(v, "gas",    gas_item);
            writeDiagFields(v, "smoke",  smoke_item);
            writeDiagFields(v, "aircon", air_item);
            writeDiagFields(v, "ups",    ups_item);

            // PCU 不再复用 snapshot item.online 作为真源，这里覆写为 runtime 结果
            v["pcu1_online"] = ctx.pcu0_state.online ? 1 : 0;
            v["pcu1_last_ok_ms"] = static_cast<double>(ctx.pcu0_state.last_rx_ms);
            v["pcu1_last_offline_ms"] = static_cast<double>(ctx.pcu0_state.last_offline_ms);
            v["pcu1_disconnect_count"] = static_cast<int>(ctx.pcu0_state.disconnect_count);

            v["pcu2_online"] = ctx.pcu1_state.online ? 1 : 0;
            v["pcu2_last_ok_ms"] = static_cast<double>(ctx.pcu1_state.last_rx_ms);
            v["pcu2_last_offline_ms"] = static_cast<double>(ctx.pcu1_state.last_offline_ms);
            v["pcu2_disconnect_count"] = static_cast<int>(ctx.pcu1_state.disconnect_count);
        }
        // 15) 预留执行状态
        v["run_state"] = 0;
        v["cmd_result"] = 0;

        // 16) 时间戳（秒级）
        const uint32_t ts32 = currentUnixSeconds32_();
        uint16_t ts_hi = 0;
        uint16_t ts_lo = 0;
        splitU32ToU16_(ts32, ts_hi, ts_lo);
        v["timestamp_hi"] = ts_hi;
        v["timestamp_lo"] = ts_lo;
        // LOG_THROTTLE_MS("logic_diag_health", 2000, LOG_COMM_D,
        //     "[LOGIC][DIAG] gas{on=%d ok=%0.f off=%0.f dc=%d} "
        //     "smoke{on=%d ok=%0.f off=%0.f dc=%d} "
        //     "air{on=%d ok=%0.f off=%0.f dc=%d} "
        //     "ups{on=%d ok=%0.f off=%0.f dc=%d}",
        //     v.value("gas_online", 0),
        //     v.value("gas_last_ok_ms", 0.0),
        //     v.value("gas_last_offline_ms", 0.0),
        //     v.value("gas_disconnect_count", 0),
        //
        //     v.value("smoke_online", 0),
        //     v.value("smoke_last_ok_ms", 0.0),
        //     v.value("smoke_last_offline_ms", 0.0),
        //     v.value("smoke_disconnect_count", 0),
        //
        //     v.value("aircon_online", 0),
        //     v.value("aircon_last_ok_ms", 0.0),
        //     v.value("aircon_last_offline_ms", 0.0),
        //     v.value("aircon_disconnect_count", 0),
        //
        //     v.value("ups_online", 0),
        //     v.value("ups_last_ok_ms", 0.0),
        //     v.value("ups_last_offline_ms", 0.0),
        //     v.value("ups_disconnect_count", 0)
        // );
        // LOG_THROTTLE_MS("logic_diag_online_full", 1000, LOG_COMM_D, // 20260409 检查online
        //     "[LOGIC][ONLINE] "
        //     "gas=%d smoke=%d air=%d ups=%d "
        //     "pcu1=%d rx1=%d hb1=%d pcu2=%d rx2=%d hb2=%d "
        //     "bms1=%d bms2=%d bms3=%d bms4=%d cnt=%d",
        //     v.value("gas_online", 0),
        //     v.value("smoke_online", 0),
        //     v.value("aircon_online", 0),
        //     v.value("ups_online", 0),
        //
        //     v.value("pcu1_online", 0),
        //     v.value("pcu1_rx_alive", 0),
        //     v.value("pcu1_hb_alive", 0),
        //     v.value("pcu2_online", 0),
        //     v.value("pcu2_rx_alive", 0),
        //     v.value("pcu2_hb_alive", 0),
        //
        //     v.value("bms1_online", 0),
        //     v.value("bms2_online", 0),
        //     v.value("bms3_online", 0),
        //     v.value("bms4_online", 0),
        //     v.value("bms_count_online", 0)
        // );
        ctx.logic_view = std::move(v);
    }

    bool LogicEngine::isSnapshotItemOnline_(const agg::SystemSnapshot& snap,
                                            const std::string& device_name,
                                            uint64_t /*now_ms*/,
                                            uint32_t /*unused*/)
    {
        auto it = snap.items.find(device_name);
        if (it == snap.items.end()) return false;

        // online 的唯一真源应当是 scheduler -> aggregator 写进 snapshot 的 item.online
        return it->second.online;
    }

    bool LogicEngine::isBmsInstanceOnline_(const bms::BmsLogicCache& cache,
                                           uint32_t instance_index,
                                           uint64_t /*now_ms*/,
                                           uint32_t /*timeout_ms*/)
    {
        const std::string key = "BMS_" + std::to_string(instance_index);
        auto it = cache.items.find(key);
        if (it == cache.items.end()) return false;
        return it->second.online;
    }

} // namespace control