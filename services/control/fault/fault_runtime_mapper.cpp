//
// Created by ChatGPT on 2026/4/8.
//

#include "fault_runtime_mapper.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <regex>
#include <sstream>

#include <nlohmann/json.hpp>

#include "fault_catalog.h"
#include "fault_center.h"
#include "logger.h"

namespace control
{
    using json = nlohmann::json;

    namespace
    {
        static bool parseJsonLine_(const std::string& line, json& out, std::string* err)
        {
            try
            {
                out = json::parse(line);
                return true;
            }
            catch (const std::exception& e)
            {
                if (err) *err = e.what();
                return false;
            }
        }

        static uint32_t parseU32Loose_(const json& v, uint32_t defv = 0)
        {
            try
            {
                if (v.is_number_unsigned()) return static_cast<uint32_t>(v.get<uint64_t>());
                if (v.is_number_integer())
                {
                    auto x = v.get<int64_t>();
                    return x >= 0 ? static_cast<uint32_t>(x) : defv;
                }
                if (v.is_string())
                {
                    const std::string s = v.get<std::string>();
                    if (s.empty()) return defv;
                    return static_cast<uint32_t>(std::stoul(s, nullptr, 0));
                }
            }
            catch (...)
            {
            }
            return defv;
        }

        static bool parseBoolLooseJson_(const json& v, bool defv = false)
        {
            try
            {
                if (v.is_boolean()) return v.get<bool>();
                if (v.is_number_integer()) return v.get<int64_t>() != 0;
                if (v.is_number_unsigned()) return v.get<uint64_t>() != 0;
                if (v.is_string())
                {
                    std::string s = v.get<std::string>();
                    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (s == "1" || s == "true" || s == "yes" || s == "y") return true;
                    if (s == "0" || s == "false" || s == "no" || s == "n") return false;
                }
            }
            catch (...)
            {
            }
            return defv;
        }

        static const control::bms::BmsPerInstanceCache* findBmsInst_(
            const control::bms::BmsLogicCache& cache,
            uint32_t inst)
        {
            if (inst < 1 || inst > 4) return nullptr;

            const std::string key = "BMS_" + std::to_string(inst);
            auto it = cache.items.find(key);
            if (it == cache.items.end()) return nullptr;
            return &it->second;
        }
    } // namespace

    void FaultRuntimeMapper::clear()
    {
        rules_.clear();
        stats_ = LoadStats{};
    }

    std::string FaultRuntimeMapper::trim_(const std::string& s)
    {
        size_t b = 0;
        size_t e = s.size();

        while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;

        return s.substr(b, e - b);
    }

    std::string FaultRuntimeMapper::normalizeToken_(const std::string& s)
    {
        std::string out;
        out.reserve(s.size());

        for (char ch : s)
        {
            const unsigned char c = static_cast<unsigned char>(ch);

            if (std::isalnum(c))
            {
                out.push_back(static_cast<char>(std::tolower(c)));
            }
            else
            {
                if (out.empty() || out.back() == '_') continue;
                out.push_back('_');
            }
        }

        while (!out.empty() && out.front() == '_') out.erase(out.begin());
        while (!out.empty() && out.back() == '_') out.pop_back();

        return out;
    }

    std::string FaultRuntimeMapper::normalizeSource_(const std::string& s)
    {
        const std::string t = normalizeToken_(s);

        if (t == "bms") return "bms";

        if (t == "pcu" || t == "pcu1" || t == "pcu2")
            return "pcu";

        if (t == "ups")
            return "ups";

        if (t == "smoke" || t == "tss" || t == "smokesensor")
            return "smoke";

        if (t == "gas" || t == "cgs" || t == "gasdetector")
            return "gas";

        if (t == "air" || t == "aircon" || t == "airconditioner" || t == "ac")
            return "air";

        if (t == "logic" || t == "vcu" || t == "system")
            return "logic";

        // 第十批：system/comm 类来源统一收口到 logic
        if (t == "hmi" || t == "screen" || t == "display")
            return "logic";

        if (t == "remote" || t == "remoteio" || t == "remote_io")
            return "logic";

        if (t == "sd" || t == "sdcard" || t == "tf" || t == "tfcard")
            return "logic";

        return t;
    }

    std::string FaultRuntimeMapper::normalizeSignal_(const std::string& s)
    {
        const std::string t = normalizeToken_(s);

        // ---- offline / online ----
        if (t == "offline" || t == "runtime_offline") return "offline";
        if (t == "online" || t == "runtime_online") return "online";

        // ---- PCU comm fault ----
        // 这里不再统一收成 "offline"，而是保留实例粒度，
        // 交给 logic source / evalLogicSignal_() 去识别。
        if (t == "pcu1_comm_fault" || t == "pcu_1_comm_fault") return "pcu0_offline";
        if (t == "pcu2_comm_fault" || t == "pcu_2_comm_fault") return "pcu1_offline";

        // ---- UPS / smoke / gas / air comm fault ----
        if (t == "ups_comm_fault") return "ups_offline";
        if (t == "tss_comm_fault" || t == "smoke_comm_fault") return "smoke_offline";
        if (t == "cgs_comm_fault" || t == "gas_comm_fault") return "gas_offline";
        if (t == "aircon_comm_fault" || t == "air_comm_fault" || t == "airconditioner_comm_fault") return "air_offline";

        // ---- HMI / remote / sdcard ----
        if (t == "hmi_comm_fault" || t == "screen_comm_fault" || t == "display_comm_fault")
            return "hmi_comm_fault";

        if (t == "remote_comm_fault" || t == "remoteio_comm_fault" || t == "remote_io_comm_fault")
            return "remote_comm_fault";

        if (t == "sdcard_fault" || t == "sd_fault" || t == "tfcard_fault" || t == "tf_fault")
            return "sdcard_fault";

        // ---- 通用聚合 ----
        if (t == "alarm_any" || t == "env_alarm_any") return "alarm_any";
        if (t == "fault_any") return "fault_any";
        if (t == "logic_any_fault") return "any_fault";
        if (t == "env_any_alarm") return "env_any_alarm";
        if (t == "system_estop" || t == "estop") return "system_estop";

        // ---- UPS ----
        if (t == "ups_alarm_any") return "alarm_any";
        if (t == "ups_fault_any") return "fault_any";
        if (t == "ups_fault_code_nonzero") return "fault_code_nonzero";
        if (t == "ups_battery_low") return "battery_low";
        if (t == "ups_bypass" || t == "ups_bypass_active") return "bypass_active";

        // ---- Smoke / TSS ----
        if (t == "smoke_alarm_any" || t == "tss_alarm_any") return "alarm_any";
        if (t == "smoke_fault_any" || t == "tss_fault_any") return "fault_any";

        if (t == "tss_smoke_alarm") return "smoke_alarm";
        if (t == "tss_temp_alarm") return "temp_alarm";
        if (t == "tss_ss_alarm" || t == "tss_smoke_sensor_fault" || t == "tss_smoke_fault")
            return "smoke_sensor_fault";
        if (t == "tss_spollution_alarm" || t == "tss_smoke_pollution_fault" || t == "tss_pollution_fault")
            return "smoke_pollution_fault";
        if (t == "tss_temp_fault") return "temp_sensor_fault";

        // ---- Gas / CGS ----
        if (t == "gas_alarm_any" || t == "cgs_alarm_any") return "alarm_any";
        if (t == "gas_fault_any" || t == "cgs_fault_any") return "fault_any";
        if (t == "gas_status_nonzero" || t == "cgs_status_nonzero") return "status_nonzero";

        if (t == "cgs_sensor_fault" || t == "cgs_fault") return "sensor_fault";
        if (t == "cgs_sensor_low" || t == "cgs_low_alarm") return "low_alarm";
        if (t == "cgs_sensor_high" || t == "cgs_high_alarm") return "high_alarm";

        // ---- Air / AirConditioner ----
        if (t == "air_alarm_any" || t == "aircon_alarm_any" || t == "aircon_alarm")
            return "alarm_any";
        if (t == "air_fault_any" || t == "aircon_fault_any" || t == "aircon_fault")
            return "fault_any";

        if (t == "aircon_hightemp_alarm") return "high_temp_alarm";
        if (t == "aircon_lowtemp_alarm") return "low_temp_alarm";
        if (t == "aircon_highhumi_alarm") return "high_humidity_alarm";
        if (t == "aircon_lowhumi_alarm") return "low_humidity_alarm";
        if (t == "aircon_coil_freeze_protect") return "coil_freeze_protect";
        if (t == "aircon_exhaust_hightemp_alarm") return "exhaust_high_temp_alarm";

        if (t == "aircon_coil_temp_sensor_fault") return "coil_temp_sensor_fault";
        if (t == "aircon_outdoor_temp_sensor_fault") return "outdoor_temp_sensor_fault";
        if (t == "aircon_condenser_temp_sensor_fault") return "condenser_temp_sensor_fault";
        if (t == "aircon_indoor_temp_sensor_fault") return "indoor_temp_sensor_fault";
        if (t == "aircon_exhaust_temp_sensor_fault") return "exhaust_temp_sensor_fault";
        if (t == "aircon_humidity_sensor_fault") return "humidity_sensor_fault";

        if (t == "aircon_internal_fan_fault") return "internal_fan_fault";
        if (t == "aircon_external_fan_fault") return "external_fan_fault";
        if (t == "aircon_compressor_fault") return "compressor_fault";
        if (t == "aircon_heater_fault") return "heater_fault";
        if (t == "aircon_emergency_fan_fault") return "emergency_fan_fault";

        if (t == "aircon_high_pressure_alarm") return "high_pressure_alarm";
        if (t == "aircon_low_pressure_alarm") return "low_pressure_alarm";
        if (t == "aircon_water_alarm" || t == "aircon_water_leak_alarm") return "water_alarm";
        if (t == "aircon_smoke_alarm" || t == "aircon_smoke") return "smoke_alarm";
        if (t == "aircon_gating_alarm" || t == "aircon_door_alarm" || t == "aircon_door")
            return "gating_alarm";

        if (t == "aircon_high_pressure_lock") return "high_pressure_lock";
        if (t == "aircon_low_pressure_lock") return "low_pressure_lock";
        if (t == "aircon_exhaust_lock") return "exhaust_lock";

        if (t == "aircon_ac_over_voltage_alarm") return "ac_over_voltage_alarm";
        if (t == "aircon_ac_under_voltage_alarm") return "ac_under_voltage_alarm";
        if (t == "aircon_ac_power_loss") return "ac_power_loss";
        if (t == "aircon_lose_phase_alarm" || t == "aircon_phase_loss") return "lose_phase_alarm";
        if (t == "aircon_freq_fault") return "freq_fault";
        if (t == "aircon_anti_phase_alarm" || t == "aircon_phase_reverse") return "anti_phase_alarm";
        if (t == "aircon_dc_over_voltage_alarm") return "dc_over_voltage_alarm";
        if (t == "aircon_dc_under_voltage_alarm") return "dc_under_voltage_alarm";

        // ---- UPS 额外别名 ----
        if (t == "ups_overload") return "overload_warning";
        if (t == "ups_overload_alarm") return "overload_fail";
        if (t == "ups_fan_fault") return "fan_lock_warning";
        if (t == "ups_battery_overtemp") return "bat_overtemp";
        if (t == "ups_sys_over_capacity") return "sys_over_capacity_warning";

        // ---- BMS 额外别名 ----
        if (t == "bms_runtime_offline") return "offline";
        if (t == "bms_runtime_online") return "online";
        if (t == "bms_runtime_stale") return "runtime_stale";
        if (t == "bms_st2_stale") return "st2_stale";
        if (t == "bms_current_limit_stale") return "current_limit_stale";
        if (t == "bms_fault1_stale") return "fault1_stale";
        if (t == "bms_fault2_stale") return "fault2_stale";

        return t;
    }
    bool FaultRuntimeMapper::parseBoolLoose_(const std::string& s, bool defv)
    {
        std::string t = normalizeToken_(s);
        if (t == "1" || t == "true" || t == "yes" || t == "y") return true;
        if (t == "0" || t == "false" || t == "no" || t == "n") return false;
        return defv;
    }

    bool FaultRuntimeMapper::tryParseInstanceFromSignal_(const std::string& signal,
                                                         uint32_t& out_inst)
    {
        const std::string t = normalizeToken_(signal);

        // --------------------------
        // BMS: BMS_1 ~ BMS_4 风格
        // --------------------------
        if (t.find("bms1") != std::string::npos || t.find("bms_1") != std::string::npos) {
            out_inst = 1;
            return true;
        }
        if (t.find("bms2") != std::string::npos || t.find("bms_2") != std::string::npos) {
            out_inst = 2;
            return true;
        }
        if (t.find("bms3") != std::string::npos || t.find("bms_3") != std::string::npos) {
            out_inst = 3;
            return true;
        }
        if (t.find("bms4") != std::string::npos || t.find("bms_4") != std::string::npos) {
            out_inst = 4;
            return true;
        }

        // --------------------------
        // PCU:
        // 约定内部实例：
        // inst=1 <-> pcu0_state
        // inst=2 <-> pcu1_state
        //
        // 兼容两套外部命名：
        // 1) pcu1_comm_fault / pcu2_comm_fault
        // 2) 归一化后的 pcu0_offline / pcu1_offline
        // --------------------------

        // 外部规则常见写法
        if (t.find("pcu1commfault") != std::string::npos || t.find("pcu_1_comm_fault") != std::string::npos) {
            out_inst = 1;
            return true;
        }
        if (t.find("pcu2commfault") != std::string::npos || t.find("pcu_2_comm_fault") != std::string::npos) {
            out_inst = 2;
            return true;
        }

        // 归一化后实例别名
        if (t == "pcu0offline" || t == "pcu_0_offline") {
            out_inst = 1;
            return true;
        }
        if (t == "pcu1offline" || t == "pcu_1_offline") {
            out_inst = 2;
            return true;
        }

        // 兼容直接写 pcu1_offline / pcu2_offline 的规则表
        if (t.find("pcu1offline") != std::string::npos || t.find("pcu_1_offline") != std::string::npos) {
            out_inst = 1;
            return true;
        }
        if (t.find("pcu2offline") != std::string::npos || t.find("pcu_2_offline") != std::string::npos) {
            out_inst = 2;
            return true;
        }

        return false;
    }

    bool FaultRuntimeMapper::loadJsonl(const std::string& path, std::string* err)
    {
        clear();

        std::ifstream ifs(path);
        if (!ifs.is_open())
        {
            if (err) *err = "open failed: " + path;
            return false;
        }

        std::string line;
        int lineno = 0;

        while (std::getline(ifs, line))
        {
            ++lineno;

            line = trim_(line);
            if (line.empty()) continue;
            if (!line.empty() && line[0] == '#') continue;

            json j;
            std::string jerr;
            if (!parseJsonLine_(line, j, &jerr))
            {
                if (err)
                {
                    std::ostringstream oss;
                    oss << "jsonl parse error at line " << lineno << ": " << jerr;
                    *err = oss.str();
                }
                return false;
            }

            if (!j.contains("items") || !j["items"].is_array())
            {
                continue;
            }

            for (const auto& it : j["items"])
            {
                if (!it.is_object()) continue;

                std::string code_s = it.value("code_hex", "");
                if (code_s.empty()) code_s = it.value("path", "");
                if (code_s.empty()) continue;

                uint16_t code = 0;
                if (!FaultCatalog::parseCode(code_s, code))
                {
                    continue;
                }

                ++stats_.total_items;

                Rule r;
                r.code = code;
                r.code_hex = it.value("code_hex", code_s);
                r.name = it.value("name", "");

                r.source = trim_(it.value("source", ""));
                r.signal = trim_(it.value("signal", ""));
                r.source_norm = normalizeSource_(r.source);
                r.signal_norm = normalizeSignal_(r.signal);

                if (it.contains("instance"))
                {
                    r.instance = parseU32Loose_(it["instance"], 0);
                }

                if (it.contains("show_hmi_current"))
                {
                    r.show_hmi_current = parseBoolLooseJson_(it["show_hmi_current"], false);
                }
                if (it.contains("show_hmi_history"))
                {
                    r.show_hmi_history = parseBoolLooseJson_(it["show_hmi_history"], false);
                }

                // 只收真正带 source/signal 的运行态规则
                if (r.source_norm.empty() || r.signal_norm.empty())
                {
                    ++stats_.skipped_no_source_or_signal;
                    continue;
                }

                // 支持多 source
                const bool supported_source =
                    (r.source_norm == "bms") ||
                    (r.source_norm == "pcu") ||
                    (r.source_norm == "ups") ||
                    (r.source_norm == "smoke") ||
                    (r.source_norm == "gas") ||
                    (r.source_norm == "air") ||
                    (r.source_norm == "logic");

                if (!supported_source)
                {
                    ++stats_.skipped_unsupported_source;
                    continue;
                }

                // BMS / PCU 若 instance 未填，可尝试从 signal 推导
                if ((r.source_norm == "bms" || r.source_norm == "pcu") && r.instance == 0)
                {
                    uint32_t inferred = 0;
                    if (tryParseInstanceFromSignal_(r.signal_norm, inferred))
                    {
                        r.instance = inferred;
                    }
                }

                    LOGINFO("[FAULT][MAP][LOAD] code=0x%04X name=%s source=%s signal=%s -> source_norm=%s signal_norm=%s inst=%u", // 20260414
                        (unsigned)r.code,
                        r.name.c_str(),
                        r.source.c_str(),
                        r.signal.c_str(),
                        r.source_norm.c_str(),
                        r.signal_norm.c_str(),
                        (unsigned)r.instance);
                    ++stats_.accepted_rules;
                    rules_.push_back(std::move(r));

            }
        }

        return true;
    }

    bool FaultRuntimeMapper::evalBmsSignal_(const control::bms::BmsPerInstanceCache& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" || s == "bms_offline") return !x.online;
        if (s == "online" || s == "runtime_online" || s == "bms_online") return x.online;

        if (s == "runtime_stale" || s == "runtime_fault_stale" || s == "bms_runtime_stale")
            return x.runtime_fault_stale;

        if (s == "fault_block_hv" || s == "bms_fault_block_hv")
            return x.hv_should_open ||
                x.f1_hvil_fault ||
                x.f1_over_chg ||
                (x.f1_low_ins_res >= 3) ||
                x.f2_pack_self_protect ||
                x.f2_main_loop_prechg_err ||
                x.f2_aux_loop_prechg_err ||
                x.f2_chrg_ins_low_err;

        if (s == "ins_low_any" || s == "bms_ins_low_any")
            return (x.f1_low_ins_res != 0) || x.f2_chrg_ins_low_err;

        if (s == "alarm_any" || s == "bms_alarm_any")
            return x.alarm_any;

        if (s == "fault_any" || s == "bms_fault_any")
            return (x.fault_level > 0) || x.rq_hv_power_off ||
                (x.fire_fault_level > 0) || (x.tms_fault_level > 0);

        if (s == "st2_stale" || s == "bms_st2_stale")
            return (x.last_st2_ms > 0) && !x.st2_online;

        if (s == "current_limit_stale" || s == "bms_current_limit_stale")
            return (x.last_current_limit_ms > 0) && !x.current_limit_online;

        if (s == "fault1_stale" || s == "bms_fault1_stale")
            return (x.last_fault1_ms > 0) && !x.fault1_online;

        if (s == "fault2_stale" || s == "bms_fault2_stale")
            return (x.last_fault2_ms > 0) && !x.fault2_online;

        if (s == "rq_hv_power_off" || s == "bms_rq_hv_power_off")
            return x.rq_hv_power_off;

        if (s == "fault_level_ge_1") return x.fault_level >= 1;
        if (s == "fault_level_ge_2") return x.fault_level >= 2;
        if (s == "tms_fault_level_ge_1") return x.tms_fault_level >= 1;
        if (s == "tms_fault_level_ge_2") return x.tms_fault_level >= 2;
        if (s == "fire_fault_level_ge_1") return x.fire_fault_level >= 1;
        if (s == "fire_fault_level_ge_2") return x.fire_fault_level >= 2;

        if (s == "f1_del_temp" || s == "bms_f1_del_temp") return x.f1_del_temp != 0;
        if (s == "f1_over_temp" || s == "bms_f1_over_temp") return x.f1_over_temp >= 3;
        if (s == "f1_over_ucell" || s == "bms_f1_over_ucell") return x.f1_over_ucell >= 3;
        if (s == "f1_low_ucell" || s == "bms_f1_low_ucell") return x.f1_low_ucell >= 3;
        if (s == "f1_low_ins_res" || s == "bms_f1_low_ins_res") return x.f1_low_ins_res >= 3;
        if (s == "f1_ucell_uniformity" || s == "bms_f1_ucell_uniformity") return x.f1_ucell_uniformity;
        if (s == "f1_over_chg" || s == "bms_f1_over_chg") return x.f1_over_chg;
        if (s == "f1_over_soc" || s == "bms_f1_over_soc") return x.f1_over_soc;
        if (s == "f1_soc_change_fast" || s == "bms_f1_soc_change_fast") return x.f1_soc_change_fast;
        if (s == "f1_bat_sys_not_match" || s == "bms_f1_bat_sys_not_match") return x.f1_bat_sys_not_match;
        if (s == "f1_hvil_fault" || s == "bms_f1_hvil_fault") return x.f1_hvil_fault;

        if (s == "f2_tms_err" || s == "bms_f2_tms_err") return x.f2_tms_err;
        if (s == "f2_pack_self_protect" || s == "bms_f2_pack_self_protect") return x.f2_pack_self_protect;
        if (s == "f2_main_loop_prechg_err" || s == "bms_f2_main_loop_prechg_err") return x.f2_main_loop_prechg_err;
        if (s == "f2_aux_loop_prechg_err" || s == "bms_f2_aux_loop_prechg_err") return x.f2_aux_loop_prechg_err;
        if (s == "f2_chrg_ins_low_err" || s == "bms_f2_chrg_ins_low_err") return x.f2_chrg_ins_low_err;
        if (s == "f2_acan_lost" || s == "bms_f2_acan_lost") return x.f2_acan_lost;
        if (s == "f2_inner_comm_err" || s == "bms_f2_inner_comm_err") return x.f2_inner_comm_err;
        if (s == "f2_dcdc_err" || s == "bms_f2_dcdc_err") return x.f2_dcdc_err;
        if (s == "f2_branch_break_err" || s == "bms_f2_branch_break_err") return x.f2_branch_break_err;
        if (s == "f2_heat_relay_open_err" || s == "bms_f2_heat_relay_open_err") return x.f2_heat_relay_open_err;
        if (s == "f2_heat_relay_weld_err" || s == "bms_f2_heat_relay_weld_err") return x.f2_heat_relay_weld_err;
        if (s == "f2_main_pos_open_err" || s == "bms_f2_main_pos_open_err") return x.f2_main_pos_open_err;
        if (s == "f2_main_pos_weld_err" || s == "bms_f2_main_pos_weld_err") return x.f2_main_pos_weld_err;
        if (s == "f2_main_neg_open_err" || s == "bms_f2_main_neg_open_err") return x.f2_main_neg_open_err;
        if (s == "f2_main_neg_weld_err" || s == "bms_f2_main_neg_weld_err") return x.f2_main_neg_weld_err;

        return false;
    }

    bool FaultRuntimeMapper::evalPcuSignal_(const PcuOnlineState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeSignal_(signal);

        // 基础在线状态
        if (s == "offline" || s == "runtime_offline") return !x.online;
        if (s == "online"  || s == "runtime_online")  return x.online;

        // 第12批：兼容实例化后的 offline token
        // 一旦路由到本实例，实例化 offline token 语义就等同于 offline。
        if (s == "pcu0_offline" || s == "pcu1_offline") return !x.online;

        // 通信 / 心跳类
        if (s == "rx_timeout") return x.seen_once && !x.rx_alive;
        if (s == "heartbeat_stale") return x.seen_once && x.rx_alive && !x.hb_alive;
        if (s == "heartbeat_missing") return !x.has_last_heartbeat;
        if (s == "estop") return x.estop;

        // 兼容规则表里直接写 comm fault
        if (s == "pcu1_comm_fault" || s == "pcu2_comm_fault") return !x.online;

        return false;
    }

    bool FaultRuntimeMapper::evalUpsSignal_(const UpsFaultState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" || s == "ups_offline") return !x.online;
        if (s == "online" || s == "runtime_online" || s == "ups_online") return x.online;

        if (s == "alarm_any") return x.alarm_any;
        if (s == "fault_any") return x.fault_any;
        if (s == "battery_low") return x.battery_low != 0;
        if (s == "bypass_active" || s == "bypass") return x.bypass_active != 0;
        if (s == "fault_code_nonzero") return x.ups_fault_code != 0;

        if (s == "mains_abnormal") return x.mains_abnormal;
        if (s == "battery_low_state") return x.battery_low_state;
        if (s == "bypass_mode") return x.bypass_mode;
        if (s == "ups_fault_state") return x.ups_fault_state;
        if (s == "backup_mode") return x.backup_mode;
        if (s == "self_test_active") return x.self_test_active;

        if (s == "internal_warning") return x.internal_warning;
        if (s == "epo_active") return x.epo_active;
        if (s == "module_unlock") return x.module_unlock;
        if (s == "line_loss") return x.line_loss;
        if (s == "ipn_loss") return x.ipn_loss;
        if (s == "line_phase_err") return x.line_phase_err;
        if (s == "site_fail") return x.site_fail;
        if (s == "bypass_loss") return x.bypass_loss;
        if (s == "bypass_phase_err") return x.bypass_phase_err;
        if (s == "bat_open") return x.bat_open;
        if (s == "bat_low_warning") return x.bat_low_warning;
        if (s == "over_chg_warning") return x.over_chg_warning;
        if (s == "bat_reverse") return x.bat_reverse;
        if (s == "overload_warning") return x.overload_warning;
        if (s == "overload_fail") return x.overload_fail;
        if (s == "fan_lock_warning") return x.fan_lock_warning;
        if (s == "maintain_on") return x.maintain_on;
        if (s == "chg_fail") return x.chg_fail;
        if (s == "error_location") return x.error_location;
        if (s == "turn_on_abnormal") return x.turn_on_abnormal;
        if (s == "redundant_loss") return x.redundant_loss;
        if (s == "module_hotswap_active") return x.module_hotswap_active;
        if (s == "battery_inform") return x.battery_inform;
        if (s == "inspection_inform") return x.inspection_inform;
        if (s == "guarantee_inform") return x.guarantee_inform;
        if (s == "temp_low_warning") return x.temp_low_warning;
        if (s == "temp_high_warning") return x.temp_high_warning;
        if (s == "bat_overtemp") return x.bat_overtemp;
        if (s == "fan_maint_inform") return x.fan_maint_inform;
        if (s == "bus_cap_maint_inform") return x.bus_cap_maint_inform;
        if (s == "sys_over_capacity_warning") return x.sys_over_capacity_warning;
        if (s == "high_external_warning") return x.high_external_warning;

        if (s == "bus_soft_timeout") return x.bus_soft_timeout;
        if (s == "bus_over") return x.bus_over;
        if (s == "bus_under") return x.bus_under;
        if (s == "bus_unbalance") return x.bus_unbalance;
        if (s == "bus_short") return x.bus_short;
        if (s == "inv_soft_timeout") return x.inv_soft_timeout;
        if (s == "inv_volt_high") return x.inv_volt_high;
        if (s == "inv_volt_low") return x.inv_volt_low;
        if (s == "op_volt_short") return x.op_volt_short;
        if (s == "over_load_fault") return x.over_load_fault;
        if (s == "over_temperature") return x.over_temperature;
        if (s == "comm_line_loss") return x.comm_line_loss;
        if (s == "power_fault") return x.power_fault;
        if (s == "ups_all_fault") return x.ups_all_fault;
        if (s == "battery_abnormal") return x.battery_abnormal;
        if (s == "battery_over_charge_fault") return x.battery_over_charge_fault;
        if (s == "epo_fault") return x.epo_fault;

        return false;
    }

    bool FaultRuntimeMapper::evalSmokeSignal_(const SmokeFaultState& x,
                                              const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" || s == "smoke_offline") return !x.online;
        if (s == "online" || s == "runtime_online" || s == "smoke_online") return x.online;

        if (s == "alarm_any") return x.alarm_any;
        if (s == "fault_any") return x.fault_any;

        if (s == "smoke_alarm") return x.smoke_alarm;
        if (s == "temp_alarm") return x.temp_alarm;
        if (s == "smoke_sensor_fault") return x.smoke_sensor_fault;
        if (s == "smoke_pollution_fault") return x.smoke_pollution_fault;
        if (s == "temp_sensor_fault") return x.temp_sensor_fault;

        return false;
    }

    bool FaultRuntimeMapper::evalGasSignal_(const GasFaultState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" || s == "gas_offline") return !x.online;
        if (s == "online" || s == "runtime_online" || s == "gas_online") return x.online;

        if (s == "alarm_any") return x.alarm_any;
        if (s == "fault_any") return x.fault_any;
        if (s == "status_nonzero") return x.status_code != 0;

        if (s == "sensor_fault") return x.sensor_fault;
        if (s == "low_alarm") return x.low_alarm;
        if (s == "high_alarm") return x.high_alarm;

        return false;
    }

    bool FaultRuntimeMapper::evalAirSignal_(const AirFaultState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" || s == "air_offline") return !x.online;
        if (s == "online" || s == "runtime_online" || s == "air_online") return x.online;

        if (s == "alarm_any") return x.alarm_any;
        if (s == "fault_any") return x.fault_any;
        if (s == "run_state_zero") return x.run_state == 0;
        if (s == "power_off") return x.power_state == 0;

        if (s == "high_temp_alarm") return x.high_temp_alarm;
        if (s == "low_temp_alarm") return x.low_temp_alarm;
        if (s == "high_humidity_alarm") return x.high_humidity_alarm;
        if (s == "low_humidity_alarm") return x.low_humidity_alarm;
        if (s == "coil_freeze_protect") return x.coil_freeze_protect;
        if (s == "exhaust_high_temp_alarm") return x.exhaust_high_temp_alarm;

        if (s == "coil_temp_sensor_fault") return x.coil_temp_sensor_fault;
        if (s == "outdoor_temp_sensor_fault") return x.outdoor_temp_sensor_fault;
        if (s == "condenser_temp_sensor_fault") return x.condenser_temp_sensor_fault;
        if (s == "indoor_temp_sensor_fault") return x.indoor_temp_sensor_fault;
        if (s == "exhaust_temp_sensor_fault") return x.exhaust_temp_sensor_fault;
        if (s == "humidity_sensor_fault") return x.humidity_sensor_fault;

        if (s == "internal_fan_fault") return x.internal_fan_fault;
        if (s == "external_fan_fault") return x.external_fan_fault;
        if (s == "compressor_fault") return x.compressor_fault;
        if (s == "heater_fault") return x.heater_fault;
        if (s == "emergency_fan_fault") return x.emergency_fan_fault;

        if (s == "high_pressure_alarm") return x.high_pressure_alarm;
        if (s == "low_pressure_alarm") return x.low_pressure_alarm;
        if (s == "water_alarm") return x.water_alarm;
        if (s == "smoke_alarm") return x.smoke_alarm;
        if (s == "gating_alarm") return x.gating_alarm;

        if (s == "high_pressure_lock") return x.high_pressure_lock;
        if (s == "low_pressure_lock") return x.low_pressure_lock;
        if (s == "exhaust_lock") return x.exhaust_lock;

        if (s == "ac_over_voltage_alarm") return x.ac_over_voltage_alarm;
        if (s == "ac_under_voltage_alarm") return x.ac_under_voltage_alarm;
        if (s == "ac_power_loss") return x.ac_power_loss;
        if (s == "lose_phase_alarm") return x.lose_phase_alarm;
        if (s == "freq_fault") return x.freq_fault;
        if (s == "anti_phase_alarm") return x.anti_phase_alarm;
        if (s == "dc_over_voltage_alarm") return x.dc_over_voltage_alarm;
        if (s == "dc_under_voltage_alarm") return x.dc_under_voltage_alarm;

        return false;
    }

    bool FaultRuntimeMapper::evalLogicSignal_(const LogicFaultState& x,
                                              const std::string& signal)
    {
        const std::string s = normalizeSignal_(signal);

        if (s == "any_fault") return x.any_fault;
        if (s == "pcu_any_offline") return x.pcu_any_offline;
        if (s == "bms_any_offline") return x.bms_any_offline;
        if (s == "ups_offline") return x.ups_offline;
        if (s == "smoke_offline") return x.smoke_offline;
        if (s == "gas_offline") return x.gas_offline;
        if (s == "air_offline") return x.air_offline;
        if (s == "env_any_alarm") return x.env_any_alarm;
        if (s == "system_estop") return x.system_estop;

        // system / comm 类
        if (s == "hmi_comm_fault") return x.hmi_comm_fault;
        if (s == "remote_comm_fault") return x.remote_comm_fault;
        if (s == "sdcard_fault") return x.sdcard_fault;

        // 第13批：实例化 PCU comm alias
        if (s == "pcu0_offline" || s == "pcu1_comm_fault" || s == "pcu1_offline")
            return x.pcu_any_offline;

        if (s == "pcu1_offline" || s == "pcu2_comm_fault" || s == "pcu2_offline")
            return x.pcu_any_offline;

        // 第13批：设备 comm alias 继续收口到 logic 聚合 offline
        if (s == "ups_comm_fault") return x.ups_offline;
        if (s == "tss_comm_fault" || s == "smoke_comm_fault") return x.smoke_offline;
        if (s == "cgs_comm_fault" || s == "gas_comm_fault") return x.gas_offline;
        if (s == "aircon_comm_fault" || s == "air_comm_fault") return x.air_offline;

        return false;
    }

    bool FaultRuntimeMapper::isKnownSignalForSource_(const std::string& source_norm,
                                                     const std::string& signal_norm)
    {
        if (source_norm == "bms")
        {
            static const char* known[] = {
                "offline", "online", "runtime_stale", "fault_block_hv", "ins_low_any", "alarm_any", "fault_any",
                "st2_stale", "current_limit_stale", "fault1_stale", "fault2_stale",
                "rq_hv_power_off",
                "fault_level_ge_1", "fault_level_ge_2",
                "tms_fault_level_ge_1", "tms_fault_level_ge_2",
                "fire_fault_level_ge_1", "fire_fault_level_ge_2",
                "f1_del_temp", "f1_over_temp", "f1_over_ucell", "f1_low_ucell", "f1_low_ins_res",
                "f1_ucell_uniformity", "f1_over_chg", "f1_over_soc", "f1_soc_change_fast",
                "f1_bat_sys_not_match", "f1_hvil_fault",
                "f2_tms_err", "f2_pack_self_protect", "f2_main_loop_prechg_err",
                "f2_aux_loop_prechg_err", "f2_chrg_ins_low_err", "f2_acan_lost",
                "f2_inner_comm_err", "f2_dcdc_err", "f2_branch_break_err",
                "f2_heat_relay_open_err", "f2_heat_relay_weld_err",
                "f2_main_pos_open_err", "f2_main_pos_weld_err",
                "f2_main_neg_open_err", "f2_main_neg_weld_err"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        if (source_norm == "pcu")
        {
            return signal_norm == "offline" || signal_norm == "online" ||
                   signal_norm == "rx_timeout" || signal_norm == "heartbeat_stale" ||
                   signal_norm == "heartbeat_missing" || signal_norm == "estop" ||
                   signal_norm == "pcu0_offline" || signal_norm == "pcu1_offline";
        }

        if (source_norm == "ups")
        {
            static const char* known[] = {
                "offline", "online",
                "alarm_any", "fault_any", "battery_low", "bypass_active", "fault_code_nonzero",

                "mains_abnormal", "battery_low_state", "bypass_mode", "ups_fault_state",
                "backup_mode", "self_test_active",

                "internal_warning", "epo_active", "module_unlock", "line_loss", "ipn_loss",
                "line_phase_err", "site_fail", "bypass_loss", "bypass_phase_err", "bat_open",
                "bat_low_warning", "over_chg_warning", "bat_reverse", "overload_warning",
                "overload_fail", "fan_lock_warning", "maintain_on", "chg_fail",
                "error_location", "turn_on_abnormal", "redundant_loss",
                "module_hotswap_active", "battery_inform", "inspection_inform",
                "guarantee_inform", "temp_low_warning", "temp_high_warning",
                "bat_overtemp", "fan_maint_inform", "bus_cap_maint_inform",
                "sys_over_capacity_warning", "high_external_warning",

                "bus_soft_timeout", "bus_over", "bus_under", "bus_unbalance",
                "bus_short", "inv_soft_timeout", "inv_volt_high", "inv_volt_low",
                "op_volt_short", "over_load_fault", "over_temperature",
                "comm_line_loss", "power_fault", "ups_all_fault",
                "battery_abnormal", "battery_over_charge_fault", "epo_fault"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        if (source_norm == "smoke")
        {
            static const char* known[] = {
                "offline", "online", "alarm_any", "fault_any",
                "smoke_alarm", "temp_alarm",
                "smoke_sensor_fault", "smoke_pollution_fault", "temp_sensor_fault"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        if (source_norm == "gas")
        {
            static const char* known[] = {
                "offline", "online", "alarm_any", "fault_any", "status_nonzero",
                "sensor_fault", "low_alarm", "high_alarm"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        if (source_norm == "air")
        {
            static const char* known[] = {
                "offline", "online", "alarm_any", "fault_any", "run_state_zero", "power_off",

                "high_temp_alarm", "low_temp_alarm",
                "high_humidity_alarm", "low_humidity_alarm",
                "coil_freeze_protect", "exhaust_high_temp_alarm",

                "coil_temp_sensor_fault", "outdoor_temp_sensor_fault",
                "condenser_temp_sensor_fault", "indoor_temp_sensor_fault",
                "exhaust_temp_sensor_fault", "humidity_sensor_fault",

                "internal_fan_fault", "external_fan_fault", "compressor_fault",
                "heater_fault", "emergency_fan_fault",

                "high_pressure_alarm", "low_pressure_alarm", "water_alarm",
                "smoke_alarm", "gating_alarm",

                "high_pressure_lock", "low_pressure_lock", "exhaust_lock",

                "ac_over_voltage_alarm", "ac_under_voltage_alarm", "ac_power_loss",
                "lose_phase_alarm", "freq_fault", "anti_phase_alarm",
                "dc_over_voltage_alarm", "dc_under_voltage_alarm"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        if (source_norm == "logic")
        {
            static const char* known[] = {
                "any_fault", "pcu_any_offline", "bms_any_offline",
                "ups_offline", "smoke_offline", "gas_offline", "air_offline",
                "env_any_alarm", "system_estop",

                // 第十批：system/comm
                "hmi_comm_fault", "remote_comm_fault", "sdcard_fault",

                // 第十批：实例化 PCU comm alias
                "pcu0_offline", "pcu1_offline"
            };
            for (auto* k : known) if (signal_norm == k) return true;
            return false;
        }

        return false;
    }

    /*
     * @brief 应用 BMS 故障映射规则
     *
     * @param cache BMS 逻辑缓存
     * @param faults 故障中心引用
     * @return void
     */
    void FaultRuntimeMapper::applyBms(const control::bms::BmsLogicCache& cache,
                                      control::FaultCenter& faults) const
    {
        for (const auto& rule : rules_)
        {
            if (rule.source_norm != "bms") continue;

            uint32_t inst = rule.instance;

            if (inst == 0)
            {
                uint32_t inferred = 0;
                if (tryParseInstanceFromSignal_(rule.signal_norm, inferred))
                {
                    inst = inferred;
                }
            }

            if (inst < 1 || inst > 4) continue;

            const auto* x = findBmsInst_(cache, inst);
            const bool active = (x != nullptr) ? evalBmsSignal_(*x, rule.signal_norm) : false;
            faults.setActive(rule.code, active);
        }
    }


    void FaultRuntimeMapper::applyAll(const LogicContext& ctx,
                                      control::FaultCenter& faults) const
    {
        LOG_THROTTLE_MS("fault_runtime_apply_enter", 1000, LOGINFO, // 20260414
    "[PROVE][RUNTIME_APPLY] rules=%zu air{seen=%d online=%d low=%d freeze=%d exhaust=%d} confirmed=%zu",
    rules_.size(),
    ctx.air_faults.seen_once ? 1 : 0,
    ctx.air_faults.online ? 1 : 0,
    ctx.air_faults.low_humidity_alarm ? 1 : 0,
    ctx.air_faults.coil_freeze_protect ? 1 : 0,
    ctx.air_faults.exhaust_high_temp_alarm ? 1 : 0,
    ctx.confirmed_faults.signals.size());
        for (const auto& rule : rules_)
        {
            if (rule.code == 0x102B || rule.code == 0x102C || rule.code == 0x102D) { // 20260414
                LOG_THROTTLE_MS(("fault_rule_probe_" + std::to_string(rule.code)).c_str(), 1000, LOGINFO,
                    "[PROBE][RULE] code=0x%04X src=%s/%s sig=%s/%s inst=%u",
                    (unsigned)rule.code,
                    rule.source.c_str(),
                    rule.source_norm.c_str(),
                    rule.signal.c_str(),
                    rule.signal_norm.c_str(),
                    (unsigned)rule.instance);
            }
            bool active = false;
            bool matched = false;

            // ------------------------------------------------------------
            // 第七批：
            // 优先读取 confirmed signals（来自 FaultLogicEvaluator / BmsFaultEvaluator）
            // 若不存在对应 confirmed key，再回退到旧的原始真源判断逻辑。
            // 这样可保持 fault_map.jsonl 不变结构。
            // ------------------------------------------------------------
            auto tryConfirmedSignal = [&](bool& out_active) -> bool
            {
                std::vector<std::string> keys;

                auto add_if_not_empty = [&](const std::string& s)
                {
                    if (!s.empty()) keys.push_back(s);
                };

                // ---------- BMS confirmed signals ----------
                if (rule.source_norm == "bms")
                {
                    if (rule.instance >= 1 && rule.instance <= 4)
                    {
                        add_if_not_empty("BMS_" + std::to_string(rule.instance) + "." + rule.signal_norm);
                    }
                }

                // ---------- 通用 confirmed signals ----------
                if (rule.source_norm == "pcu")
                {
                    uint32_t inst = rule.instance;
                    if (inst == 0)
                    {
                        uint32_t inferred = 0;
                        if (tryParseInstanceFromSignal_(rule.signal_norm, inferred))
                        {
                            inst = inferred;
                        }
                    }

                    if (inst == 1)
                    {
                        add_if_not_empty("logic.pcu0_" + rule.signal_norm);
                        add_if_not_empty("logic.pcu0_offline");

                        // 兼容规则表直接写 pcu1_comm_fault / pcu1_offline
                        if (rule.signal_norm == "pcu0_offline" ||
                            rule.signal_norm == "pcu1_comm_fault" ||
                            rule.signal_norm == "pcu1_offline" ||
                            rule.signal_norm == "offline" ||
                            rule.signal_norm == "runtime_offline")
                        {
                            add_if_not_empty("logic.pcu0_offline");
                        }
                    }
                    else if (inst == 2)
                    {
                        add_if_not_empty("logic.pcu1_" + rule.signal_norm);
                        add_if_not_empty("logic.pcu1_offline");

                        // 兼容规则表直接写 pcu2_comm_fault / pcu2_offline
                        if (rule.signal_norm == "pcu1_offline" ||
                            rule.signal_norm == "pcu2_comm_fault" ||
                            rule.signal_norm == "pcu2_offline" ||
                            rule.signal_norm == "offline" ||
                            rule.signal_norm == "runtime_offline")
                        {
                            add_if_not_empty("logic.pcu1_offline");
                        }
                    }
                }
                else if (rule.source_norm == "ups")
                {
                    add_if_not_empty("logic.ups_" + rule.signal_norm);

                    if (rule.signal_norm == "fault_any" || rule.signal_norm == "fault_code_nonzero")
                    {
                        add_if_not_empty("logic.ups_fault");
                    }
                    if (rule.signal_norm == "alarm_any")
                    {
                        add_if_not_empty("logic.ups_alarm");
                    }
                    if (rule.signal_norm == "offline" || rule.signal_norm == "runtime_offline" ||
                        rule.signal_norm == "ups_offline" || rule.signal_norm == "ups_comm_fault")
                    {
                        add_if_not_empty("logic.ups_offline");
                    }
                }
                else if (rule.source_norm == "smoke")
                {
                    add_if_not_empty("logic.smoke_" + rule.signal_norm);

                    if (rule.signal_norm == "alarm_any")
                    {
                        add_if_not_empty("logic.smoke_alarm");
                    }
                    if (rule.signal_norm == "offline" || rule.signal_norm == "runtime_offline" ||
                        rule.signal_norm == "smoke_offline" || rule.signal_norm == "tss_comm_fault")
                    {
                        add_if_not_empty("logic.smoke_offline");
                    }
                }
                else if (rule.source_norm == "gas")
                {
                    add_if_not_empty("logic.gas_" + rule.signal_norm);

                    if (rule.signal_norm == "alarm_any")
                    {
                        add_if_not_empty("logic.gas_alarm");
                    }
                    if (rule.signal_norm == "offline" || rule.signal_norm == "runtime_offline" ||
                        rule.signal_norm == "gas_offline" || rule.signal_norm == "cgs_comm_fault")
                    {
                        add_if_not_empty("logic.gas_offline");
                    }
                }
                else if (rule.source_norm == "air")
                {
                    add_if_not_empty("logic.air_" + rule.signal_norm);

                    if (rule.signal_norm == "alarm_any")
                    {
                        add_if_not_empty("logic.air_alarm");
                    }
                    if (rule.signal_norm == "fault_any")
                    {
                        add_if_not_empty("logic.air_fault");
                    }
                    if (rule.signal_norm == "offline" || rule.signal_norm == "runtime_offline" ||
                        rule.signal_norm == "air_offline" || rule.signal_norm == "aircon_comm_fault")
                    {
                        add_if_not_empty("logic.air_offline");
                    }
                }
                else if (rule.source_norm == "logic")
                {
                    add_if_not_empty("logic." + rule.signal_norm);

                    // 第13批：system/comm 规则表兼容 key
                    if (rule.signal_norm == "pcu1_comm_fault" || rule.signal_norm == "pcu1_offline")
                        add_if_not_empty("logic.pcu0_offline");

                    if (rule.signal_norm == "pcu2_comm_fault" || rule.signal_norm == "pcu2_offline")
                        add_if_not_empty("logic.pcu1_offline");

                    if (rule.signal_norm == "ups_comm_fault")
                        add_if_not_empty("logic.ups_offline");

                    if (rule.signal_norm == "tss_comm_fault" || rule.signal_norm == "smoke_comm_fault")
                        add_if_not_empty("logic.smoke_offline");

                    if (rule.signal_norm == "cgs_comm_fault" || rule.signal_norm == "gas_comm_fault")
                        add_if_not_empty("logic.gas_offline");

                    if (rule.signal_norm == "aircon_comm_fault" || rule.signal_norm == "air_comm_fault")
                        add_if_not_empty("logic.air_offline");

                    if (rule.signal_norm == "hmi_comm_fault")
                        add_if_not_empty("logic.hmi_comm_fault");

                    if (rule.signal_norm == "remote_comm_fault")
                        add_if_not_empty("logic.remote_comm_fault");

                    if (rule.signal_norm == "sdcard_fault")
                        add_if_not_empty("logic.sdcard_fault");
                }

                for (const auto& k : keys)
                {
                    auto it = ctx.confirmed_faults.signals.find(k);
                    if (it != ctx.confirmed_faults.signals.end())
                    {
                        out_active = it->second;
                        return true;
                    }
                }

                return false;
            };
            if (rule.code == 0x102B || rule.code == 0x102C || rule.code == 0x102D)
            {
                bool confirmed_probe_active = false;
                const bool confirmed_probe_hit = tryConfirmedSignal(confirmed_probe_active);

                LOG_THROTTLE_MS(("air_rule_result_" + std::to_string(rule.code)).c_str(), 1000, LOGINFO,
                                "[RESULT][AIR_RULE] code=0x%04X src=%s/%s sig=%s/%s inst=%u "
                                "confirmed_hit=%d confirmed_active=%d "
                                "raw{seen=%d online=%d low=%d freeze=%d exhaust=%d} confirmed_size=%zu",
                                (unsigned)rule.code,
                                rule.source.c_str(),
                                rule.source_norm.c_str(),
                                rule.signal.c_str(),
                                rule.signal_norm.c_str(),
                                (unsigned)rule.instance,
                                confirmed_probe_hit ? 1 : 0,
                                confirmed_probe_active ? 1 : 0,
                                ctx.air_faults.seen_once ? 1 : 0,
                                ctx.air_faults.online ? 1 : 0,
                                ctx.air_faults.low_humidity_alarm ? 1 : 0,
                                ctx.air_faults.coil_freeze_protect ? 1 : 0,
                                ctx.air_faults.exhaust_high_temp_alarm ? 1 : 0,
                                ctx.confirmed_faults.signals.size());
            }
            // 1) 优先 confirmed signals
            if (tryConfirmedSignal(active))
            {
                LOG_THROTTLE_MS(("fault_rule_confirmed_" + std::to_string(rule.code)).c_str(), 1000, LOGINFO,
                                "[FAULT][RUNTIME][CONFIRMED] code=0x%04X source=%s signal=%s inst=%u active=%d",
                                (unsigned)rule.code,
                                rule.source_norm.c_str(),
                                rule.signal_norm.c_str(),
                                (unsigned)rule.instance,
                                active ? 1 : 0);
                faults.setActive(rule.code, active);
                continue;
            }
            // 2) 回退到现有原始真源逻辑
            if (rule.source_norm == "bms")
            {
                uint32_t inst = rule.instance;

                if (inst == 0)
                {
                    uint32_t inferred = 0;
                    if (tryParseInstanceFromSignal_(rule.signal_norm, inferred))
                    {
                        inst = inferred;
                    }
                }

                if (inst >= 1 && inst <= 4)
                {
                    const std::string key = "BMS_" + std::to_string(inst);
                    auto it = ctx.bms_cache.items.find(key);
                    if (it != ctx.bms_cache.items.end())
                    {
                        active = evalBmsSignal_(it->second, rule.signal_norm);
                        matched = true;
                    }
                }
            }
            else if (rule.source_norm == "pcu")
            {
                uint32_t inst = rule.instance;

                if (inst == 0)
                {
                    uint32_t inferred = 0;
                    if (tryParseInstanceFromSignal_(rule.signal_norm, inferred))
                    {
                        inst = inferred;
                    }
                }

                if (inst == 1)
                {
                    active = evalPcuSignal_(ctx.pcu0_state, rule.signal_norm);
                    matched = true;
                }
                else if (inst == 2)
                {
                    active = evalPcuSignal_(ctx.pcu1_state, rule.signal_norm);
                    matched = true;
                }
            }
            else if (rule.source_norm == "ups")
            {
                active = evalUpsSignal_(ctx.ups_faults, rule.signal_norm);
                matched = true;
            }
            else if (rule.source_norm == "smoke")
            {
                active = evalSmokeSignal_(ctx.smoke_faults, rule.signal_norm);
                matched = true;
            }
            else if (rule.source_norm == "gas")
            {
                active = evalGasSignal_(ctx.gas_faults, rule.signal_norm);
                matched = true;
            }
            else if (rule.source_norm == "air")
            {
                active = evalAirSignal_(ctx.air_faults, rule.signal_norm);
                matched = true;
            }
            else if (rule.source_norm == "logic")
            {
                active = evalLogicSignal_(ctx.logic_faults, rule.signal_norm);
                matched = true;
            }

            LOG_THROTTLE_MS(("fault_rule_fallback_" + std::to_string(rule.code)).c_str(), 1000, LOGINFO, // 故障结构十批输出
                            "[FAULT][RUNTIME][FALLBACK] code=0x%04X source=%s signal=%s inst=%u active=%d matched=%d",
                            (unsigned)rule.code,
                            rule.source_norm.c_str(),
                            rule.signal_norm.c_str(),
                            (unsigned)rule.instance,
                            active ? 1 : 0,
                            matched ? 1 : 0);

            faults.setActive(rule.code, active);
        }
    }
} // namespace control
