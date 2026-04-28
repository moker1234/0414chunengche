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
#include "logic_factor.h"

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

static const nlohmann::json* findGasChannel_(const nlohmann::json& snap, int ch)
{
    if (!snap.is_object()) return nullptr;

    auto it_items = snap.find("items");
    if (it_items == snap.end() || !it_items->is_object()) return nullptr;

    auto it_gas = it_items->find("GasDetector");
    if (it_gas == it_items->end() || !it_gas->is_object()) return nullptr;

    auto it_channels = it_gas->find("gas_channels");
    if (it_channels == it_gas->end() || !it_channels->is_object()) return nullptr;

    const std::string key = std::to_string(ch);
    auto it_ch = it_channels->find(key);
    if (it_ch == it_channels->end() || !it_ch->is_object()) return nullptr;

    auto it_valid = it_ch->find("valid");
    if (it_valid != it_ch->end()) {
        bool valid = false;
        if (it_valid->is_boolean()) {
            valid = it_valid->get<bool>();
        } else if (it_valid->is_number()) {
            valid = (it_valid->get<double>() != 0.0);
        }
        if (!valid) return nullptr;
    }

    return &(*it_ch);
}

static double readGasChannelValueOr(const nlohmann::json& snap, int ch, double defv)
{
    const nlohmann::json* jc = findGasChannel_(snap, ch);
    if (!jc) return defv;

    auto it_val = jc->find("value");
    if (it_val == jc->end()) return defv;

    if (it_val->is_number()) return it_val->get<double>();
    if (it_val->is_boolean()) return it_val->get<bool>() ? 1.0 : 0.0;

    return defv;
}

static int readGasChannelStatusOr(const nlohmann::json& snap, int ch, int defv)
{
    const nlohmann::json* jc = findGasChannel_(snap, ch);
    if (!jc) return defv;

    auto it_status = jc->find("status");
    if (it_status == jc->end()) return defv;

    if (it_status->is_number_integer()) return it_status->get<int>();
    if (it_status->is_number()) return static_cast<int>(it_status->get<double>());
    if (it_status->is_boolean()) return it_status->get<bool>() ? 1 : 0;

    return defv;
}

static bool readGasChannelBoolOr(const nlohmann::json& snap,
                                 int ch,
                                 const char* key,
                                 bool defv)
{
    const nlohmann::json* jc = findGasChannel_(snap, ch);
    if (!jc || !key || !*key) return defv;

    auto it = jc->find(key);
    if (it == jc->end()) return defv;

    if (it->is_boolean()) return it->get<bool>();
    if (it->is_number_integer()) return it->get<int>() != 0;
    if (it->is_number_unsigned()) return it->get<unsigned>() != 0;
    if (it->is_number_float()) return it->get<double>() != 0.0;

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
                static int16_t roundScaledToI16_(double v, double factor)
        {
            if (!std::isfinite(v)) {
                return 0;
            }

            const double y = v * factor;

            if (y > static_cast<double>(std::numeric_limits<int16_t>::max())) {
                return std::numeric_limits<int16_t>::max();
            }

            if (y < static_cast<double>(std::numeric_limits<int16_t>::min())) {
                return std::numeric_limits<int16_t>::min();
            }

            return static_cast<int16_t>(std::llround(y));
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
                static void copyLogicViewKeyIfMissing_(nlohmann::json& v,
                                               const char* dst,
                                               const char* src)
        {
            if (!dst || !*dst || !src || !*src) return;
            if (!v.is_object()) return;

            if (v.find(dst) != v.end()) return;

            auto it = v.find(src);
            if (it == v.end()) return;

            v[dst] = *it;
        }

        static int jsonIntOr_(const nlohmann::json& v,
                              const char* key,
                              int defv)
        {
            if (!v.is_object() || !key || !*key) return defv;

            auto it = v.find(key);
            if (it == v.end()) return defv;

            if (it->is_number_integer()) return it->get<int>();
            if (it->is_number_unsigned()) return static_cast<int>(it->get<unsigned>());
            if (it->is_number_float()) return static_cast<int>(it->get<double>());
            if (it->is_boolean()) return it->get<bool>() ? 1 : 0;

            return defv;
        }

        static void addNormalMapCompatibilityAliases_(nlohmann::json& v)
        {
            if (!v.is_object()) return;

            /*
             * AirConditioner：
             * 旧字段 ac_* 保留，新 jsonl 推荐 aircon_*。
             */
            copyLogicViewKeyIfMissing_(v, "aircon_run_overall", "ac_overall_state");
            copyLogicViewKeyIfMissing_(v, "aircon_inner_fan", "ac_inner_fan");
            copyLogicViewKeyIfMissing_(v, "aircon_outer_fan", "ac_outer_fan");
            copyLogicViewKeyIfMissing_(v, "aircon_compressor", "ac_compressor");
            copyLogicViewKeyIfMissing_(v, "aircon_heater", "ac_heater");
            copyLogicViewKeyIfMissing_(v, "aircon_em_fan", "ac_em_fan");

            copyLogicViewKeyIfMissing_(v, "aircon_coil_temp", "ac_coil_temp");
            copyLogicViewKeyIfMissing_(v, "aircon_outdoor_temp", "ac_outdoor_temp");
            copyLogicViewKeyIfMissing_(v, "aircon_condense_temp", "ac_condense_temp");
            copyLogicViewKeyIfMissing_(v, "aircon_indoor_temp", "ac_indoor_temp");
            copyLogicViewKeyIfMissing_(v, "aircon_humidity", "ac_humidity");
            copyLogicViewKeyIfMissing_(v, "aircon_exhaust_temp", "ac_exhaust_temp");
            copyLogicViewKeyIfMissing_(v, "aircon_current", "ac_current");
            copyLogicViewKeyIfMissing_(v, "aircon_ac_voltage", "ac_ac_voltage");
            copyLogicViewKeyIfMissing_(v, "aircon_dc_voltage", "ac_dc_voltage");

            copyLogicViewKeyIfMissing_(v, "aircon_coil_temp_eng", "ac_coil_temp_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_outdoor_temp_eng", "ac_outdoor_temp_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_condense_temp_eng", "ac_condense_temp_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_indoor_temp_eng", "ac_indoor_temp_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_humidity_eng", "ac_humidity_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_exhaust_temp_eng", "ac_exhaust_temp_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_current_eng", "ac_current_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_ac_voltage_eng", "ac_ac_voltage_eng");
            copyLogicViewKeyIfMissing_(v, "aircon_dc_voltage_eng", "ac_dc_voltage_eng");

            /*
             * UPS：
             * 保留 hi/lo，同时补一个完整秒数，方便后续 HMI 或调试页读取。
             */
            if (v.find("ups_battery_remain_sec") == v.end())
            {
                const int hi = jsonIntOr_(v, "ups_battery_remain_sec_hi", 0);
                const int lo = jsonIntOr_(v, "ups_battery_remain_sec_lo", 0);

                const uint32_t sec =
                    (static_cast<uint32_t>(hi & 0xFFFF) << 16) |
                    static_cast<uint32_t>(lo & 0xFFFF);

                v["ups_battery_remain_sec"] = static_cast<double>(sec);
            }

            /*
             * BMS online / health 兼容：
             * 旧逻辑里同时存在 bms1_online 和 bms_1_online。
             * 新 jsonl 统一推荐 bms_1_online。
             */
            for (int idx = 1; idx <= 4; ++idx)
            {
                const std::string n = std::to_string(idx);

                const std::string old_prefix = "bms" + n;
                const std::string new_prefix = "bms_" + n;

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_online").c_str(),
                    (old_prefix + "_online").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_last_ok_ms").c_str(),
                    (old_prefix + "_last_ok_ms").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_last_offline_ms").c_str(),
                    (old_prefix + "_last_offline_ms").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_disconnect_count").c_str(),
                    (old_prefix + "_disconnect_count").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_offline_reason_code").c_str(),
                    (old_prefix + "_offline_reason_code").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_offline_reason_text").c_str(),
                    (old_prefix + "_offline_reason_text").c_str());

                /*
                 * BMS 首页别名：
                 * 有些 HMI 点位用“电压/电流/SOC”这种概览名，
                 * 这里补 alias，不改变 bms_adapter_ 的原始字段。
                 */
                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_pack_voltage").c_str(),
                    (new_prefix + "_st2_pack_v").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_pack_current").c_str(),
                    (new_prefix + "_st2_pack_i").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_pack_soc").c_str(),
                    (new_prefix + "_st2_soc").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_pack_soh").c_str(),
                    (new_prefix + "_st2_soh").c_str());

                /*
                 * TMS / Fire 语义别名：
                 * bms_adapter_ 第四批已经输出这些字段；
                 * 这里不额外计算，只保证 normal_map 可以稳定引用。
                 */
                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_tms_status").c_str(),
                    (new_prefix + "_tms_work_state").c_str());

                copyLogicViewKeyIfMissing_(v,
                    (new_prefix + "_fire_alarm_level").c_str(),
                    (new_prefix + "_fire_value_alarm_level").c_str());
            }
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
        const double gas_co_raw          = readGasChannelValueOr(snap, 1, 0.0);
        const double gas_o2_raw          = readGasChannelValueOr(snap, 2, 0.0);
        const double gas_temperature_raw = readGasChannelValueOr(snap, 3, 0.0);
        const double gas_humidity_raw    = readGasChannelValueOr(snap, 4, 0.0);
        const double gas_co2_raw         = readGasChannelValueOr(snap, 5, 0.0);

        const int gas_ch0_status = readGasChannelStatusOr(snap, 0, 0);
        const int gas_ch1_status = readGasChannelStatusOr(snap, 1, 0);
        const int gas_ch2_status = readGasChannelStatusOr(snap, 2, 0);
        const int gas_ch3_status = readGasChannelStatusOr(snap, 3, 0);
        const int gas_ch4_status = readGasChannelStatusOr(snap, 4, 0);
        const int gas_ch5_status = readGasChannelStatusOr(snap, 5, 0);

        const uint16_t gas_status_all =
            static_cast<uint16_t>(
                gas_ch0_status |
                gas_ch1_status |
                gas_ch2_status |
                gas_ch3_status |
                gas_ch4_status |
                gas_ch5_status
            );

        const bool gas_fault_any_raw =
            readGasChannelBoolOr(snap, 0, "fault_any",  (gas_ch0_status & 0x0001) != 0) ||
            readGasChannelBoolOr(snap, 1, "fault_any",  (gas_ch1_status & 0x0001) != 0) ||
            readGasChannelBoolOr(snap, 2, "fault_any",  (gas_ch2_status & 0x0001) != 0) ||
            readGasChannelBoolOr(snap, 3, "fault_any",  (gas_ch3_status & 0x0001) != 0) ||
            readGasChannelBoolOr(snap, 4, "fault_any",  (gas_ch4_status & 0x0001) != 0) ||
            readGasChannelBoolOr(snap, 5, "fault_any",  (gas_ch5_status & 0x0001) != 0);

        const bool gas_low_alarm_raw =
            readGasChannelBoolOr(snap, 0, "low_alarm",  (gas_ch0_status & 0x0002) != 0) ||
            readGasChannelBoolOr(snap, 1, "low_alarm",  (gas_ch1_status & 0x0002) != 0) ||
            readGasChannelBoolOr(snap, 2, "low_alarm",  (gas_ch2_status & 0x0002) != 0) ||
            readGasChannelBoolOr(snap, 3, "low_alarm",  (gas_ch3_status & 0x0002) != 0) ||
            readGasChannelBoolOr(snap, 4, "low_alarm",  (gas_ch4_status & 0x0002) != 0) ||
            readGasChannelBoolOr(snap, 5, "low_alarm",  (gas_ch5_status & 0x0002) != 0);

        const bool gas_high_alarm_raw =
            readGasChannelBoolOr(snap, 0, "high_alarm", (gas_ch0_status & 0x0004) != 0) ||
            readGasChannelBoolOr(snap, 1, "high_alarm", (gas_ch1_status & 0x0004) != 0) ||
            readGasChannelBoolOr(snap, 2, "high_alarm", (gas_ch2_status & 0x0004) != 0) ||
            readGasChannelBoolOr(snap, 3, "high_alarm", (gas_ch3_status & 0x0004) != 0) ||
            readGasChannelBoolOr(snap, 4, "high_alarm", (gas_ch4_status & 0x0004) != 0) ||
            readGasChannelBoolOr(snap, 5, "high_alarm", (gas_ch5_status & 0x0004) != 0);

        const bool gas_alarm_any_raw =
            gas_low_alarm_raw || gas_high_alarm_raw;

        const int gas_detector_state = static_cast<int>(gas_status_all);
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

        const int gas_combustible_disp = roundScaledToI16_(std::max(0.0, gas_combustible_raw), gas_factor.combustible);
        const int gas_co_disp = roundScaledToI16_(std::max(0.0, gas_co_raw), gas_factor.co);
        const int gas_o2_disp = roundScaledToI16_(std::max(0.0, gas_o2_raw), gas_factor.o2);
        const int gas_temperature_disp = roundScaledToI16_(gas_temperature_raw, gas_factor.temperature);
        const int gas_humidity_disp =  roundScaledToI16_(std::clamp(gas_humidity_raw, 0.0, 100.0), gas_factor.humidity);
        const int gas_co2_disp = roundScaledToI16_(std::max(0.0, gas_co2_raw), gas_factor.co2);

        // AirConditioner：
        // - *_eng 保留工程值，供调试 / 模型导出 / 后续业务查看；
        // - 原 HMI 使用字段 ac_* 输出整数缩放值，避免依赖 normal_map_logic.jsonl 的 scale。

        //  AirConditioner 传感器值处理
        const bool ac_coil_temp_invalid =    ctx.air_faults.coil_temp_sensor_fault || (ac_coil_temp_raw >= 199.9);
        const bool ac_outdoor_temp_invalid = ctx.air_faults.outdoor_temp_sensor_fault || (ac_outdoor_temp_raw >= 199.9);
        const bool ac_condense_temp_invalid =ctx.air_faults.condenser_temp_sensor_fault || (ac_condense_temp_raw >= 199.9);
        const bool ac_indoor_temp_invalid = ctx.air_faults.indoor_temp_sensor_fault || (ac_indoor_temp_raw >= 199.9);
        const bool ac_exhaust_temp_invalid =ctx.air_faults.exhaust_temp_sensor_fault || (ac_exhaust_temp_raw >= 199.9);
        const bool ac_humidity_invalid =    ctx.air_faults.humidity_sensor_fault || (ac_humidity_raw == 120.0) ||  (ac_humidity_raw >= 32767.0);
        const double ac_coil_temp_eng =  ac_coil_temp_invalid ? 0.0 : ac_coil_temp_raw;
        const double ac_outdoor_temp_eng = ac_outdoor_temp_invalid ? 0.0 : ac_outdoor_temp_raw;
        const double ac_condense_temp_eng = ac_condense_temp_invalid ? 0.0 : ac_condense_temp_raw;
        const double ac_indoor_temp_eng = ac_indoor_temp_invalid ? 0.0 : ac_indoor_temp_raw;
        const double ac_humidity_eng =  ac_humidity_invalid ? 0.0 : std::clamp(ac_humidity_raw, 0.0, 100.0);
        const double ac_exhaust_temp_eng = ac_exhaust_temp_invalid ? 0.0 : ac_exhaust_temp_raw;

        const double ac_current_eng = std::max(0.0, ac_current_raw);
        const double ac_ac_voltage_eng = std::max(0.0, ac_ac_voltage_raw);
        const double ac_dc_voltage_eng = std::max(0.0, ac_dc_voltage_raw);
        /*采用 normal_map_logic.jsonl 的“缩放”后，logic_view 输出工程值，不再提前转 HMI 整数。*/
        const double ac_coil_temp_hmi = ac_coil_temp_eng;
        const double ac_outdoor_temp_hmi = ac_outdoor_temp_eng;
        const double ac_condense_temp_hmi = ac_condense_temp_eng;
        const double ac_indoor_temp_hmi = ac_indoor_temp_eng;
        const double ac_humidity_hmi = ac_humidity_eng;
        const double ac_exhaust_temp_hmi = ac_exhaust_temp_eng;
        const double ac_current_hmi = ac_current_eng;
        const double ac_ac_voltage_hmi = ac_ac_voltage_eng;
        const double ac_dc_voltage_hmi = ac_dc_voltage_eng;

        const double smoke_alarm_num_disp = std::max(0.0, smoke_alarm_num_raw);
        const double smoke_fault_disp = std::max(0.0, smoke_fault_raw);
        const double smoke_warn_level_disp = std::max(0.0, smoke_warn_level_raw);
        const double smoke_percent_disp = std::max(0.0, smoke_percent_raw);
        const double smoke_temperature_disp = smoke_temperature_raw;

        const double ups_system_mode_disp =
            std::max(0.0, ups_system_mode_raw);

        const double ups_battery_capacity_disp =
            std::clamp(ups_battery_capacity_raw, 0.0, 100.0);

        const double ups_fault_bits_disp =
            std::max(0.0, ups_fault_bits_raw);

        const double ups_warning_bits_disp =
            std::max(0.0, ups_warning_bits_raw);

        const uint32_t ups_battery_remain_sec_u32 =
            static_cast<uint32_t>(std::max(0.0, ups_battery_remain_sec_raw));

        const uint16_t ups_battery_remain_sec_hi =
            static_cast<uint16_t>((ups_battery_remain_sec_u32 >> 16) & 0xFFFFu);

        const uint16_t ups_battery_remain_sec_lo =
            static_cast<uint16_t>(ups_battery_remain_sec_u32 & 0xFFFFu);

        const uint32_t ups_fault_bits_u32 =
            static_cast<uint32_t>(std::max(0.0, ups_fault_bits_disp));

        const uint16_t ups_fault_bits_hi =
            static_cast<uint16_t>((ups_fault_bits_u32 >> 16) & 0xFFFFu);

        const uint16_t ups_fault_bits_lo =
            static_cast<uint16_t>(ups_fault_bits_u32 & 0xFFFFu);

        const uint32_t ups_warning_bits_u32 =
            static_cast<uint32_t>(std::max(0.0, ups_warning_bits_disp));

        const uint16_t ups_warning_bits_hi =
            static_cast<uint16_t>((ups_warning_bits_u32 >> 16) & 0xFFFFu);

        const uint16_t ups_warning_bits_lo =
            static_cast<uint16_t>(ups_warning_bits_u32 & 0xFFFFu);
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

        // HMI 使用字段：工程值。
        // NormalHmiWriter 根据 normal_map_logic.jsonl 的“缩放”反算 HMI 整数。
        v["ac_coil_temp"] = ac_coil_temp_hmi;
        v["ac_outdoor_temp"] = ac_outdoor_temp_hmi;
        v["ac_condense_temp"] = ac_condense_temp_hmi;
        v["ac_indoor_temp"] = ac_indoor_temp_hmi;
        v["ac_humidity"] = ac_humidity_hmi;
        v["ac_exhaust_temp"] = ac_exhaust_temp_hmi;
        v["ac_current"] = ac_current_hmi;
        v["ac_ac_voltage"] = ac_ac_voltage_hmi;
        v["ac_dc_voltage"] = ac_dc_voltage_hmi;

        // 工程值保留字段：不接 HMI 主显示，只供调试 / 导出 / 后续业务使用。
        v["ac_coil_temp_eng"] = ac_coil_temp_eng;
        v["ac_outdoor_temp_eng"] = ac_outdoor_temp_eng;
        v["ac_condense_temp_eng"] = ac_condense_temp_eng;
        v["ac_indoor_temp_eng"] = ac_indoor_temp_eng;
        v["ac_humidity_eng"] = ac_humidity_eng;
        v["ac_exhaust_temp_eng"] = ac_exhaust_temp_eng;
        v["ac_current_eng"] = ac_current_eng;
        v["ac_ac_voltage_eng"] = ac_ac_voltage_eng;
        v["ac_dc_voltage_eng"] = ac_dc_voltage_eng;

        // 10) Smoke
        {
            const int smoke_alarm_raw_i =
                static_cast<int>(smoke_alarm_num_disp);

            const int smoke_fault_raw_i =
                static_cast<int>(smoke_fault_disp);

            const bool smoke_alarm_on =
                ctx.smoke_faults.smoke_alarm ||
                (smoke_alarm_raw_i != 0);

            const bool smoke_sensor_fault_on =
                ctx.smoke_faults.smoke_sensor_fault ||
                ((smoke_fault_raw_i & 0x01) != 0);

            const bool smoke_pollution_fault_on =
                ctx.smoke_faults.smoke_pollution_fault ||
                ((smoke_fault_raw_i & 0x02) != 0);

            const bool smoke_temp_sensor_fault_on =
                ctx.smoke_faults.temp_sensor_fault ||
                ((smoke_fault_raw_i & 0x04) != 0);

            const bool smoke_fault_any_on =
                smoke_sensor_fault_on ||
                smoke_pollution_fault_on ||
                smoke_temp_sensor_fault_on;

            v["smoke_online"] = ctx.smoke_faults.online ? 1 : 0;
            v["smoke_offline"] = ctx.smoke_faults.online ? 0 : 1;

            // 原始寄存器显示
            v["smoke_alarm"] = smoke_alarm_on ? 1 : 0;
            v["smoke_fault"] = smoke_fault_raw_i;
            v["smoke_warn_level"] = static_cast<int>(smoke_warn_level_disp);
            v["smoke_percent"] = smoke_percent_disp;
            v["smoke_temperature"] = smoke_temperature_disp;

            // 语义化故障真源，供 HMI / 调试 / 后续 normal_map_logic 使用
            v["smoke_fault_any"] = smoke_fault_any_on ? 1 : 0;
            v["smoke_sensor_fault"] = smoke_sensor_fault_on ? 1 : 0;
            v["smoke_pollution_fault"] = smoke_pollution_fault_on ? 1 : 0;
            v["smoke_temp_sensor_fault"] = smoke_temp_sensor_fault_on ? 1 : 0;

            // 探测器状态：0=正常；非0=协议 fault bitfield。
            // 通信状态单独看 smoke_online/smoke_offline。
            v["smoke_detector_state"] = smoke_fault_raw_i;
        }

        // 11) Gas
        v["gas_detector_state"] = gas_detector_state;

        v["gas_combustible"] = gas_combustible_disp;
        v["gas_co"]          = gas_co_disp;
        v["gas_o2"]          = gas_o2_disp;
        v["gas_temperature"] = gas_temperature_disp;
        v["gas_humidity"]    = gas_humidity_disp;
        v["gas_co2"]         = gas_co2_disp;

        v["gas_combustible_status"] = gas_ch0_status;
        v["gas_co_status"]          = gas_ch1_status;
        v["gas_o2_status"]          = gas_ch2_status;
        v["gas_temperature_status"] = gas_ch3_status;
        v["gas_humidity_status"]    = gas_ch4_status;
        v["gas_co2_status"]         = gas_ch5_status;

        // Gas 故障真源，供 HMI / 调试 / 后续 confirmed_faults 使用
        v["gas_fault_any"]  = gas_fault_any_raw ? 1 : 0;
        v["gas_low_alarm"]  = gas_low_alarm_raw ? 1 : 0;
        v["gas_high_alarm"] = gas_high_alarm_raw ? 1 : 0;
        v["gas_alarm_any"]  = gas_alarm_any_raw ? 1 : 0;

        // 12) UPS
        v["ups_system_mode"] = static_cast<int>(ups_system_mode_disp);

        // 完整 32 位秒数，供 words=2 的 HMI map 直接绑定。
        v["ups_battery_remain_sec"] =
            static_cast<double>(ups_battery_remain_sec_u32);

        // 兼容旧 normal_map_logic.jsonl 里 xxx_hi + xxx_lo 的写法。
        v["ups_battery_remain_sec_hi"] = ups_battery_remain_sec_hi;
        v["ups_battery_remain_sec_lo"] = ups_battery_remain_sec_lo;

        v["ups_battery_capacity"] = static_cast<int>(ups_battery_capacity_disp);

        // 完整 32 位容器。
        v["ups_fault_bits"] =
            static_cast<double>(ups_fault_bits_u32);

        v["ups_warning_bits"] =
            static_cast<double>(ups_warning_bits_u32);

        // 16 位拆分，供 HMI 分两个寄存器显示完整故障/告警容器。
        v["ups_fault_bits_hi"] = ups_fault_bits_hi;
        v["ups_fault_bits_lo"] = ups_fault_bits_lo;
        v["ups_warning_bits_hi"] = ups_warning_bits_hi;
        v["ups_warning_bits_lo"] = ups_warning_bits_lo;
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

        /// ============================================================
// PCU runtime 真源投影
//
// 内部命名：
//   PCU_0 -> HMI/故障表 PCU1
//   PCU_1 -> HMI/故障表 PCU2
//
// 注意：
//   不从 snapshot item.online 读取 PCU 在线状态。
//   PCU online 只来自 ctx.pcu0_state / ctx.pcu1_state。
// ============================================================
auto write_pcu_runtime = [&](const char* prefix, const PcuOnlineState& s)
{
    v[std::string(prefix) + "_online"] = s.online ? 1 : 0;

    v[std::string(prefix) + "_rx_alive"] = s.rx_alive ? 1 : 0;
    v[std::string(prefix) + "_hb_alive"] = s.hb_alive ? 1 : 0;

    v[std::string(prefix) + "_seen_once"] = s.seen_once ? 1 : 0;

    v[std::string(prefix) + "_last_rx_ms"] =
        static_cast<double>(s.last_rx_ms);

    v[std::string(prefix) + "_last_hb_change_ms"] =
        static_cast<double>(s.last_hb_change_ms);

    v[std::string(prefix) + "_last_rx_age_ms"] =
        s.last_rx_age_ms;

    v[std::string(prefix) + "_last_hb_change_age_ms"] =
        s.last_hb_change_age_ms;

    v[std::string(prefix) + "_heartbeat"] =
        static_cast<int>(s.last_heartbeat);

    v[std::string(prefix) + "_hb_repeat_count"] =
        static_cast<int>(s.hb_repeat_count);

    v[std::string(prefix) + "_hb_jump_err_count"] =
        static_cast<int>(s.hb_jump_err_count);

    v[std::string(prefix) + "_last_hb_delta"] =
        static_cast<int>(s.last_hb_delta);

    v[std::string(prefix) + "_online_reason_code"] =
        static_cast<int>(s.offline_reason_code);

    v[std::string(prefix) + "_online_reason_text"] =
        pcuOfflineReasonText_(s.offline_reason_code);

    v[std::string(prefix) + "_last_offline_ms"] =
        static_cast<double>(s.last_offline_ms);

    v[std::string(prefix) + "_disconnect_count"] =
        static_cast<int>(s.disconnect_count);

    v[std::string(prefix) + "_cabinet_id"] =
        static_cast<int>(s.cabinet_id);

    v[std::string(prefix) + "_pcu_state"] =
        s.pcu_state_valid ? static_cast<int>(s.pcu_state) : 0;

    /*
     * estop_raw 保留最后观测值。
     * estop 只在 online 时作为有效急停状态输出。
     * 后续第五批故障映射也应该按 online && estop 触发 PCU急停，
     * 避免 PCU 离线后保留旧急停位。
     */
    v[std::string(prefix) + "_estop_raw"] =
        s.estop ? 1 : 0;

    v[std::string(prefix) + "_estop"] =
        (s.online && s.estop) ? 1 : 0;
};

write_pcu_runtime("pcu1", ctx.pcu0_state);
write_pcu_runtime("pcu2", ctx.pcu1_state);

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
        // ============================================================
        // 15) IO / 急停 / 插枪投影（第4批）
        //
        // 约定：
        // - di_bits: bit0 -> DI1, bit12 -> DI13, ... bit17 -> DI18
        // - ctx.ai:  ai[0] = ADC1_V, ai[1] = ADC2_V
        // - DI 低电平有效已在 driver / logic_io 阶段折算为“逻辑 ON”
        // ============================================================
        {
            auto test_di = [&](int channel_id) -> int {
                if (channel_id < 1 || channel_id > 64) return 0;
                const int bit = channel_id - 1;
                return (((ctx.di_bits >> bit) & 0x1ULL) != 0ULL) ? 1 : 0;
            };

            auto read_ai_v = [&](int adc_index) -> double {
                if (adc_index < 1) return 0.0;
                const std::size_t idx = static_cast<std::size_t>(adc_index - 1);
                if (idx >= ctx.ai.size()) return 0.0;
                const double v_ai = ctx.ai[idx];
                return std::isfinite(v_ai) ? v_ai : 0.0;
            };
            auto adc_plug = [&](int adc_index) -> int {
                const double vv = read_ai_v(adc_index);
                return (std::isfinite(vv) && vv < 8.0) ? 1 : 0;
            };

            const int di1_estop = test_di(1);

            const int di13_plug = test_di(13);
            const int di14_plug = test_di(14);
            const int di15_plug = test_di(15);
            const int di16_plug = test_di(16);
            const int di17_plug = test_di(17);
            const int di18_plug = test_di(18);

            const int plug_di_any =
                di13_plug || di14_plug || di15_plug ||
                di16_plug || di17_plug || di18_plug;
            const double adc1_v = read_ai_v(1);
            const double adc2_v = read_ai_v(2);

            const int adc1_plug_detected = adc_plug(1);
            const int adc2_plug_detected = adc_plug(2);

            const int plug_any =
                plug_di_any || adc1_plug_detected || adc2_plug_detected;

            // ---- 原始 IO ----
            v["io_last_ts_ms"] = static_cast<double>(ctx.last_io_ts);
            v["io_di_bits"] = static_cast<double>(ctx.di_bits);
            // ---- DI ----
            v["di1_estop"] = di1_estop;
            v["di13_plug"] = di13_plug;
            v["di14_plug"] = di14_plug;
            v["di15_plug"] = di15_plug;
            v["di16_plug"] = di16_plug;
            v["di17_plug"] = di17_plug;
            v["di18_plug"] = di18_plug;

            // ---- ADC ----
            v["adc1_voltage_v"] = adc1_v;
            v["adc2_voltage_v"] = adc2_v;
            v["adc1_plug_detected"] = adc1_plug_detected;
            v["adc2_plug_detected"] = adc2_plug_detected;
            // ---- 业务汇总 ----
            v["system_estop"] = ctx.logic_faults.system_estop ? 1 : 0;
            v["plug_di_any"] = plug_di_any;
            v["plug_any"] = plug_any;

            // ---- 指示灯期望态（不是 GPIO 回读）----
            v["lamp_do1_estop"] = ctx.logic_faults.system_estop ? 1 : 0;
            v["lamp_do2_plug_any"] = plug_any;
            v["lamp_do3_plug_di_any"] = plug_di_any;
            v["lamp_do4_adc1_plug"] = adc1_plug_detected;
            v["lamp_do5_adc2_plug"] = adc2_plug_detected;
            v["lamp_do6_io_alive"] = (ctx.last_io_ts != 0) ? 1 : 0;
            v["lamp_do7_any_fault"] = ctx.logic_faults.any_fault ? 1 : 0;
        }


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

        /*
         * 补齐 normal_map_logic.v4.with_path.jsonl 所需的兼容别名。
         *
         * 注意：
         * - 这里只补 logic_view 别名，不直接写 HMI；
         * - HMI 实际写入仍由 NormalHmiWriter 根据 jsonl 的 path 完成；
         * - 旧字段不删除，避免影响已有调试页和业务逻辑。
         */
        addNormalMapCompatibilityAliases_(v);

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