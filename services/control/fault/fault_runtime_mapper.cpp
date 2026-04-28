//
// Created by lxy on 2026/4/8.
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

        // static const control::bms::BmsPerInstanceCache* findBmsInst_(
        //     const control::bms::BmsLogicCache& cache,
        //     uint32_t inst)
        // {
        //     if (inst < 1 || inst > 4) return nullptr;
        //
        //     const std::string key = "BMS_" + std::to_string(inst);
        //     auto it = cache.items.find(key);
        //     if (it == cache.items.end()) return nullptr;
        //     return &it->second;
        // }
    } // namespace

    void FaultRuntimeMapper::clear()
    {
        rules_.clear();
        stats_ = LoadStats{};
        debounce_states_.clear();
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

        if (t == "ups" || t == "ups232" || t == "ups_ascii" || t == "rs232_ups")
            return "ups";

        if (t == "smoke" ||
            t == "smokesensor" ||
            t == "smoke_sensor" ||
            t == "tss" ||
            t == "tss_smoke" ||
            t == "tss_sensor")
            return "smoke";

        if (t == "gas" ||
            t == "gasdetector" ||
            t == "gas_detector" ||
            t == "cgs" ||
            t == "cgs_sensor")
            return "gas";

        if (t == "air" ||
            t == "aircon" ||
            t == "air_conditioner" ||
            t == "airconditioner" ||
            t == "ac")
            return "air";

        if (t == "logic" || t == "vcu" || t == "system")
            return "logic";

        // system / comm 类来源仍统一收口到 logic。
        // 但 UPS / AIR / SMOKE / GAS 的通信故障本批已经要求在 fault_map 中改成设备 source，
        // 所以这些设备通信故障不再依赖 VCU 转发。
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
        if (t == "offline" || t == "runtime_offline")
            return "offline";

        if (t == "online" || t == "runtime_online")
            return "online";

        // ---- PCU comm fault ----
        // PCU 仍暂留 logic / confirmed 链路，因此这里保留实例化 token。
        if (t == "pcu1_comm_fault" || t == "pcu_1_comm_fault")
            return "pcu0_offline";

        if (t == "pcu2_comm_fault" || t == "pcu_2_comm_fault")
            return "pcu1_offline";

        // ---- UPS comm fault ----
        if (t == "ups_comm_fault" ||
            t == "ups_offline" ||
            t == "ups_runtime_offline")
            return "ups_offline";

        // ---- Smoke / TSS comm fault ----
        if (t == "tss_offline" ||
            t == "tss_comm_fault" ||
            t == "smoke_offline" ||
            t == "smoke_comm_fault" ||
            t == "smokesensor_comm_fault" ||
            t == "smoke_sensor_comm_fault")
            return "tss_offline";

        // ---- Gas / CGS comm fault ----
        if (t == "gas_offline" ||
            t == "gas_comm_fault" ||
            t == "cgs_comm_fault" ||
            t == "gasdetector_comm_fault" ||
            t == "gas_detector_comm_fault")
            return "gas_offline";

        // ---- Air / Aircon comm fault ----
        if (t == "air_offline" ||
            t == "air_comm_fault" ||
            t == "aircon_comm_fault" ||
            t == "airconditioner_comm_fault" ||
            t == "air_conditioner_comm_fault")
            return "air_offline";

        // ---- HMI / remote / sdcard ----
        if (t == "hmi_comm_fault" ||
            t == "screen_comm_fault" ||
            t == "display_comm_fault")
            return "hmi_comm_fault";

        if (t == "remote_comm_fault" ||
            t == "remoteio_comm_fault" ||
            t == "remote_io_comm_fault")
            return "remote_comm_fault";

        if (t == "sdcard_fault" ||
            t == "sd_fault" ||
            t == "tfcard_fault" ||
            t == "tf_fault")
            return "sdcard_fault";

        // ---- 通用聚合 ----
        if (t == "alarm_any" || t == "env_alarm_any")
            return "alarm_any";

        if (t == "fault_any")
            return "fault_any";

        if (t == "logic_any_fault")
            return "any_fault";

        if (t == "env_any_alarm")
            return "env_any_alarm";

        if (t == "system_estop" || t == "estop")
            return "system_estop";

        // ---- UPS common aliases ----
        if (t == "ups_alarm_any")
            return "alarm_any";

        if (t == "ups_fault_any")
            return "fault_any";

        if (t == "ups_fault_code_nonzero")
            return "fault_code_nonzero";

        if (t == "ups_battery_low")
            return "battery_low";

        if (t == "ups_bypass" || t == "ups_bypass_active")
            return "bypass_active";

        // ---- Smoke / TSS aliases ----
        if (t == "tss_smoke_alarm")
            return "tss_smoke_alarm";

        if (t == "tss_temp_alarm")
            return "tss_temp_alarm";

        if (t == "tss_ss_alarm")
            return "tss_ss_alarm";

        if (t == "tss_spollution_alarm" ||
            t == "tss_pollution_alarm" ||
            t == "tss_s_pollution_alarm")
            return "tss_spollution_alarm";

        if (t == "tss_temp_fault")
            return "tss_temp_fault";

        // ---- Gas aliases ----
        if (t == "gas_alarm_any")
            return "gas_alarm_any";

        if (t == "gas_fault_any")
            return "gas_fault_any";

        if (t == "gas_sensor_fault")
            return "gas_sensor_fault";

        if (t == "gas_low_alarm")
            return "gas_low_alarm";

        if (t == "gas_high_alarm")
            return "gas_high_alarm";

        if (t == "gas_status_nonzero")
            return "gas_status_nonzero";

        // ---- Air common aliases ----
        if (t == "air_alarm_any" || t == "aircon_alarm_any")
            return "alarm_any";

        if (t == "air_fault_any" || t == "aircon_fault_any")
            return "fault_any";

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
        if (t.find("bms1") != std::string::npos || t.find("bms_1") != std::string::npos)
        {
            out_inst = 1;
            return true;
        }
        if (t.find("bms2") != std::string::npos || t.find("bms_2") != std::string::npos)
        {
            out_inst = 2;
            return true;
        }
        if (t.find("bms3") != std::string::npos || t.find("bms_3") != std::string::npos)
        {
            out_inst = 3;
            return true;
        }
        if (t.find("bms4") != std::string::npos || t.find("bms_4") != std::string::npos)
        {
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
        if (t.find("pcu1commfault") != std::string::npos || t.find("pcu_1_comm_fault") != std::string::npos)
        {
            out_inst = 1;
            return true;
        }
        if (t.find("pcu2commfault") != std::string::npos || t.find("pcu_2_comm_fault") != std::string::npos)
        {
            out_inst = 2;
            return true;
        }

        // 归一化后实例别名
        if (t == "pcu0offline" || t == "pcu_0_offline")
        {
            out_inst = 1;
            return true;
        }
        if (t == "pcu1offline" || t == "pcu_1_offline")
        {
            out_inst = 2;
            return true;
        }

        // 兼容直接写 pcu1_offline / pcu2_offline 的规则表
        if (t.find("pcu1offline") != std::string::npos || t.find("pcu_1_offline") != std::string::npos)
        {
            out_inst = 1;
            return true;
        }
        if (t.find("pcu2offline") != std::string::npos || t.find("pcu_2_offline") != std::string::npos)
        {
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
                    // (r.source_norm == "bms") ||
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
                if ((
                    // r.source_norm == "bms" ||
                    r.source_norm == "pcu") && r.instance == 0)
                {
                    uint32_t inferred = 0;
                    if (tryParseInstanceFromSignal_(r.signal_norm, inferred))
                    {
                        r.instance = inferred;
                    }
                }

                // LOGINFO("[FAULT][MAP][LOAD] code=0x%04X name=%s source=%s signal=%s -> source_norm=%s signal_norm=%s inst=%u", // 20260414
                //     (unsigned)r.code,
                //     r.name.c_str(),
                //     r.source.c_str(),
                //     r.signal.c_str(),
                //     r.source_norm.c_str(),
                //     r.signal_norm.c_str(),
                //     (unsigned)r.instance);
                ++stats_.accepted_rules;
                rules_.push_back(std::move(r));
            }
        }

        return true;
    }

    // bool FaultRuntimeMapper::evalBmsSignal_(const control::bms::BmsPerInstanceCache& x,
    //                                         const std::string& signal)
    // {
    //     const std::string s = normalizeToken_(signal);
    //
    //     if (s == "offline" || s == "runtime_offline" || s == "bms_offline") return !x.online;
    //     if (s == "online" || s == "runtime_online" || s == "bms_online") return x.online;
    //
    //     if (s == "runtime_stale" || s == "runtime_fault_stale" || s == "bms_runtime_stale")
    //         return x.runtime_fault_stale;
    //
    //     if (s == "fault_block_hv" || s == "bms_fault_block_hv")
    //         return x.hv_should_open ||
    //             x.f1_hvil_fault ||
    //             x.f1_over_chg ||
    //             (x.f1_low_ins_res >= 3) ||
    //             x.f2_pack_self_protect ||
    //             x.f2_main_loop_prechg_err ||
    //             x.f2_aux_loop_prechg_err ||
    //             x.f2_chrg_ins_low_err;
    //
    //     if (s == "ins_low_any" || s == "bms_ins_low_any")
    //         return (x.f1_low_ins_res != 0) || x.f2_chrg_ins_low_err;
    //
    //     if (s == "alarm_any" || s == "bms_alarm_any")
    //         return x.alarm_any;
    //
    //     if (s == "fault_any" || s == "bms_fault_any")
    //         return (x.fault_level > 0) || x.rq_hv_power_off ||
    //             (x.fire_fault_level > 0) || (x.tms_fault_level > 0);
    //
    //     if (s == "st2_stale" || s == "bms_st2_stale")
    //         return (x.last_st2_ms > 0) && !x.st2_online;
    //
    //     if (s == "current_limit_stale" || s == "bms_current_limit_stale")
    //         return (x.last_current_limit_ms > 0) && !x.current_limit_online;
    //
    //     if (s == "fault1_stale" || s == "bms_fault1_stale")
    //         return (x.last_fault1_ms > 0) && !x.fault1_online;
    //
    //     if (s == "fault2_stale" || s == "bms_fault2_stale")
    //         return (x.last_fault2_ms > 0) && !x.fault2_online;
    //
    //     if (s == "rq_hv_power_off" || s == "bms_rq_hv_power_off")
    //         return x.rq_hv_power_off;
    //
    //     if (s == "fault_level_ge_1") return x.fault_level >= 1;
    //     if (s == "fault_level_ge_2") return x.fault_level >= 2;
    //     if (s == "tms_fault_level_ge_1") return x.tms_fault_level >= 1;
    //     if (s == "tms_fault_level_ge_2") return x.tms_fault_level >= 2;
    //     if (s == "fire_fault_level_ge_1") return x.fire_fault_level >= 1;
    //     if (s == "fire_fault_level_ge_2") return x.fire_fault_level >= 2;
    //
    //     if (s == "f1_del_temp" || s == "bms_f1_del_temp") return x.f1_del_temp != 0;
    //     if (s == "f1_over_temp" || s == "bms_f1_over_temp") return x.f1_over_temp >= 3;
    //     if (s == "f1_over_ucell" || s == "bms_f1_over_ucell") return x.f1_over_ucell >= 3;
    //     if (s == "f1_low_ucell" || s == "bms_f1_low_ucell") return x.f1_low_ucell >= 3;
    //     if (s == "f1_low_ins_res" || s == "bms_f1_low_ins_res") return x.f1_low_ins_res >= 3;
    //     if (s == "f1_ucell_uniformity" || s == "bms_f1_ucell_uniformity") return x.f1_ucell_uniformity;
    //     if (s == "f1_over_chg" || s == "bms_f1_over_chg") return x.f1_over_chg;
    //     if (s == "f1_over_soc" || s == "bms_f1_over_soc") return x.f1_over_soc;
    //     if (s == "f1_soc_change_fast" || s == "bms_f1_soc_change_fast") return x.f1_soc_change_fast;
    //     if (s == "f1_bat_sys_not_match" || s == "bms_f1_bat_sys_not_match") return x.f1_bat_sys_not_match;
    //     if (s == "f1_hvil_fault" || s == "bms_f1_hvil_fault") return x.f1_hvil_fault;
    //
    //     if (s == "f2_tms_err" || s == "bms_f2_tms_err") return x.f2_tms_err;
    //     if (s == "f2_pack_self_protect" || s == "bms_f2_pack_self_protect") return x.f2_pack_self_protect;
    //     if (s == "f2_main_loop_prechg_err" || s == "bms_f2_main_loop_prechg_err") return x.f2_main_loop_prechg_err;
    //     if (s == "f2_aux_loop_prechg_err" || s == "bms_f2_aux_loop_prechg_err") return x.f2_aux_loop_prechg_err;
    //     if (s == "f2_chrg_ins_low_err" || s == "bms_f2_chrg_ins_low_err") return x.f2_chrg_ins_low_err;
    //     if (s == "f2_acan_lost" || s == "bms_f2_acan_lost") return x.f2_acan_lost;
    //     if (s == "f2_inner_comm_err" || s == "bms_f2_inner_comm_err") return x.f2_inner_comm_err;
    //     if (s == "f2_dcdc_err" || s == "bms_f2_dcdc_err") return x.f2_dcdc_err;
    //     if (s == "f2_branch_break_err" || s == "bms_f2_branch_break_err") return x.f2_branch_break_err;
    //     if (s == "f2_heat_relay_open_err" || s == "bms_f2_heat_relay_open_err") return x.f2_heat_relay_open_err;
    //     if (s == "f2_heat_relay_weld_err" || s == "bms_f2_heat_relay_weld_err") return x.f2_heat_relay_weld_err;
    //     if (s == "f2_main_pos_open_err" || s == "bms_f2_main_pos_open_err") return x.f2_main_pos_open_err;
    //     if (s == "f2_main_pos_weld_err" || s == "bms_f2_main_pos_weld_err") return x.f2_main_pos_weld_err;
    //     if (s == "f2_main_neg_open_err" || s == "bms_f2_main_neg_open_err") return x.f2_main_neg_open_err;
    //     if (s == "f2_main_neg_weld_err" || s == "bms_f2_main_neg_weld_err") return x.f2_main_neg_weld_err;
    //
    //     return false;
    // }

    bool FaultRuntimeMapper::evalPcuSignal_(const PcuOnlineState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        // 通信 / 在线状态
        if (s == "offline" ||
            s == "runtime_offline" ||
            s == "comm_fault" ||
            s == "communication_fault" ||
            s == "pcu_comm_fault" ||
            s == "pcu1_comm_fault" ||
            s == "pcu2_comm_fault")
        {
            return !x.online;
        }

        if (s == "online" ||
            s == "runtime_online")
        {
            return x.online;
        }

        if (s == "rx_timeout") {
            return x.seen_once && !x.rx_alive;
        }

        if (s == "heartbeat_stale") {
            return x.seen_once && x.rx_alive && !x.hb_alive;
        }

        if (s == "heartbeat_missing") {
            return !x.has_last_heartbeat;
        }

        // 急停：只在 online 时承认，避免离线后沿用旧 estop
        if (s == "estop" ||
            s == "emergency_stop" ||
            s == "fault_estop" ||
            s == "pcu_emergency_stop" ||
            s == "pcu1_emergency_stop" ||
            s == "pcu2_emergency_stop")
        {
            return x.online && x.estop;
        }

        return false;
    }

    // ------------------ 替换以下内容 ------------------
    bool FaultRuntimeMapper::evalUpsSignal_(const UpsFaultState& x, const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" || s == "runtime_offline" ||
            s == "ups_offline" || s == "ups_comm_fault")
        {
            return !x.online;
        }

        if (s == "online" || s == "runtime_online" || s == "ups_online")
        {
            return x.online;
        }

        // UPS 离线时，内部 warning/fault 一律不报，只保留通信故障。
        if (!x.online) return false;

        if (s == "alarm_any") return x.alarm_any;
        if (s == "fault_any") return x.fault_any;
        if (s == "battery_low") return x.battery_low_state || x.battery_low_warning;
        if (s == "bypass_active" || s == "bypass") return x.bypass_mode || x.bypass_abnormal;
        if (s == "fault_code_nonzero") return x.fault_bits != 0;

        if (s == "mains_abnormal") return x.mains_abnormal;
        if (s == "battery_low_state") return x.battery_low_state;
        if (s == "bypass_mode") return x.bypass_mode;
        if (s == "ups_fault_state") return x.ups_fault_state;
        if (s == "backup_mode") return x.backup_mode;
        if (s == "self_test_active") return x.self_test_active;

        // Warning bits
        if (s == "internal_warning") return x.internal_warning;
        if (s == "epo_active") return x.epo_active;
        if (s == "module_unlock") return x.module_unlock;
        if (s == "neutral_lost") return x.neutral_lost;
        if (s == "mains_phase_error") return x.mains_phase_error;
        if (s == "ln_reverse") return x.ln_reverse;
        if (s == "bypass_abnormal") return x.bypass_abnormal;
        if (s == "bypass_phase_error") return x.bypass_phase_error;
        if (s == "battery_not_connected") return x.battery_not_connected;
        if (s == "battery_low_warning") return x.battery_low_warning;
        if (s == "battery_overcharge") return x.battery_overcharge;
        if (s == "battery_reverse") return x.battery_reverse;
        if (s == "overload_warning") return x.overload_warning;
        if (s == "overload_alarm") return x.overload_alarm;
        if (s == "fan_fault") return x.fan_fault;
        if (s == "bypass_cover_open") return x.bypass_cover_open;
        if (s == "charger_fault") return x.charger_fault;
        if (s == "position_error") return x.position_error;
        if (s == "boot_condition_not_met") return x.boot_condition_not_met;
        if (s == "redundancy_lost") return x.redundancy_lost;
        if (s == "module_loose") return x.module_loose;
        if (s == "battery_maint_due") return x.battery_maint_due;
        if (s == "inspection_maint_due") return x.inspection_maint_due;
        if (s == "warranty_maint_due") return x.warranty_maint_due;
        if (s == "temp_low_warning") return x.temp_low_warning;
        if (s == "temp_high_warning") return x.temp_high_warning;
        if (s == "battery_overtemp") return x.battery_overtemp;
        if (s == "fan_maint_due") return x.fan_maint_due;
        if (s == "bus_cap_maint_due") return x.bus_cap_maint_due;
        if (s == "system_overload") return x.system_overload;
        if (s == "reserved_warning") return x.reserved_warning;

        // Fault codes
        if (s == "bus_softstart_timeout") return x.bus_softstart_timeout;
        if (s == "bus_overvoltage_fault") return x.bus_overvoltage_fault;
        if (s == "bus_undervoltage_fault") return x.bus_undervoltage_fault;
        if (s == "bus_imbalance_fault") return x.bus_imbalance_fault;
        if (s == "bus_short_circuit") return x.bus_short_circuit;

        if (s == "inv_softstart_timeout") return x.inv_softstart_timeout;
        if (s == "inv_overvoltage_fault") return x.inv_overvoltage_fault;
        if (s == "inv_undervoltage_fault") return x.inv_undervoltage_fault;
        if (s == "output_short_circuit") return x.output_short_circuit;
        if (s == "r_inv_short_circuit") return x.r_inv_short_circuit;
        if (s == "s_inv_short_circuit") return x.s_inv_short_circuit;
        if (s == "t_inv_short_circuit") return x.t_inv_short_circuit;
        if (s == "rs_short_circuit") return x.rs_short_circuit;
        if (s == "st_short_circuit") return x.st_short_circuit;
        if (s == "tr_short_circuit") return x.tr_short_circuit;

        if (s == "reverse_power_fault") return x.reverse_power_fault;
        if (s == "r_reverse_power_fault") return x.r_reverse_power_fault;
        if (s == "s_reverse_power_fault") return x.s_reverse_power_fault;
        if (s == "t_reverse_power_fault") return x.t_reverse_power_fault;
        if (s == "total_reverse_power_fault") return x.total_reverse_power_fault;
        if (s == "current_imbalance_fault") return x.current_imbalance_fault;
        if (s == "overload_fault") return x.overload_fault;
        if (s == "overtemp_fault") return x.overtemp_fault;

        if (s == "inv_relay_fail_close") return x.inv_relay_fail_close;
        if (s == "inv_relay_stuck") return x.inv_relay_stuck;
        if (s == "mains_scr_fault") return x.mains_scr_fault;
        if (s == "battery_scr_fault") return x.battery_scr_fault;
        if (s == "bypass_scr_fault") return x.bypass_scr_fault;
        if (s == "rectifier_fault") return x.rectifier_fault;
        if (s == "input_overcurrent_fault") return x.input_overcurrent_fault;
        if (s == "wiring_error") return x.wiring_error;

        if (s == "comm_cable_disconnected") return x.comm_cable_disconnected;
        if (s == "host_cable_fault") return x.host_cable_fault;
        if (s == "can_comm_fault") return x.can_comm_fault;
        if (s == "sync_signal_fault") return x.sync_signal_fault;
        if (s == "power_supply_fault") return x.power_supply_fault;
        if (s == "all_fan_fault") return x.all_fan_fault;
        if (s == "dsp_error") return x.dsp_error;
        if (s == "charger_softstart_timeout") return x.charger_softstart_timeout;
        if (s == "all_module_fault") return x.all_module_fault;

        if (s == "mains_ntc_open_fault") return x.mains_ntc_open_fault;
        if (s == "mains_fuse_open_fault") return x.mains_fuse_open_fault;
        if (s == "output_imbalance_fault") return x.output_imbalance_fault;
        if (s == "input_mismatch_fault") return x.input_mismatch_fault;
        if (s == "eeprom_data_lost") return x.eeprom_data_lost;
        if (s == "mains_support_failed") return x.mains_support_failed;
        if (s == "power_failed") return x.power_failed;
        if (s == "system_overload_fault") return x.system_overload_fault;
        if (s == "ads7869_error") return x.ads7869_error;
        if (s == "bypass_mode_no_op") return x.bypass_mode_no_op;
        if (s == "op_breaker_off_parallel") return x.op_breaker_off_parallel;

        if (s == "r_bus_fuse_fault") return x.r_bus_fuse_fault;
        if (s == "s_bus_fuse_fault") return x.s_bus_fuse_fault;
        if (s == "t_bus_fuse_fault") return x.t_bus_fuse_fault;
        if (s == "ntc_fault") return x.ntc_fault;
        if (s == "parallel_cable_fault") return x.parallel_cable_fault;
        if (s == "battery_fault") return x.battery_fault;
        if (s == "frequent_overcurrent_fault") return x.frequent_overcurrent_fault;
        if (s == "battery_overcharge_fault") return x.battery_overcharge_fault;
        if (s == "epo_critical_fault") return x.epo_critical_fault;

        return false;
    }

    // --------------------------------------------------

    bool FaultRuntimeMapper::evalSmokeSignal_(const SmokeFaultState& x,
                                              const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "tss_offline" ||
            s == "smoke_offline" ||
            s == "tss_comm_fault" ||
            s == "smoke_comm_fault")
        {
            return !x.online;
        }

        if (!x.online)
        {
            return false;
        }

        if (s == "tss_smoke_alarm")
        {
            return x.smoke_alarm;
        }

        if (s == "tss_temp_alarm")
        {
            return x.temp_alarm;
        }

        if (s == "tss_ss_alarm")
        {
            return x.smoke_sensor_fault;
        }

        if (s == "tss_spollution_alarm")
        {
            return x.smoke_pollution_fault;
        }

        if (s == "tss_temp_fault")
        {
            return x.temp_sensor_fault;
        }

        if (s == "alarm_any")
        {
            return x.alarm_any;
        }

        if (s == "fault_any")
        {
            return x.fault_any;
        }

        return false;
    }

    bool FaultRuntimeMapper::evalGasSignal_(const GasFaultState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        // Gas fault_map 统一使用 gas_ 前缀：
        // gas_offline / gas_online
        // gas_alarm / gas_alarm_any
        // gas_sensor_fault / gas_fault_any
        // gas_low_alarm / gas_high_alarm / gas_status_nonzero

        if (s == "gas_offline" ||
            s == "cgs_comm_fault" ||
            s == "gas_comm_fault")
        {
            return !x.online;
        }

        if (s == "gas_online")
        {
            return x.online;
        }

        if (!x.online)
        {
            return false;
        }

        if (s == "gas_alarm" || s == "gas_alarm_any" || s == "alarm_any")
        {
            return x.alarm_any;
        }

        if (s == "gas_sensor_fault" || s == "gas_fault_any" || s == "fault_any")
        {
            return x.sensor_fault || x.fault_any;
        }

        if (s == "gas_status_nonzero")
        {
            return x.status_code != 0;
        }

        if (s == "gas_low_alarm")
        {
            return x.low_alarm;
        }

        if (s == "gas_high_alarm")
        {
            return x.high_alarm;
        }

        return false;
    }

    bool FaultRuntimeMapper::evalAirSignal_(const AirFaultState& x,
                                            const std::string& signal)
    {
        const std::string s = normalizeToken_(signal);

        if (s == "offline" ||
            s == "runtime_offline" ||
            s == "air_offline" ||
            s == "aircon_comm_fault" ||
            s == "air_comm_fault")
        {
            return !x.online;
        }

        if (s == "online" || s == "runtime_online" || s == "air_online")
        {
            return x.online;
        }

        if (!x.online)
        {
            return false;
        }

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
        // UPS / AIR / SMOKE / GAS 已迁移到 direct runtime mapper。
        //
        // 这里不再对设备协议信号做严格白名单过滤，原因：
        // 1. UPS fault_map 信号很多，硬白名单容易漏；
        // 2. AIR / SMOKE / GAS 后续也可能新增 signal；
        // 3. 真正是否 active 由 evalXxxSignal_() 判断；
        // 4. 未识别 signal 最终只会返回 false，不会误触发故障。
        if (source_norm == "ups" ||
            source_norm == "air" ||
            source_norm == "smoke" ||
            source_norm == "gas")
        {
            return !signal_norm.empty();
        }

        if (source_norm == "bms")
        {
            static const char* known[] = {
                "offline", "online",
                "runtime_stale",
                "fault_block_hv",
                "ins_low_any",
                "alarm_any",
                "fault_any",

                "st2_stale",
                "current_limit_stale",
                "fault1_stale",
                "fault2_stale",

                "rq_hv_power_off",

                "fault_level_ge_1",
                "fault_level_ge_2",
                "tms_fault_level_ge_1",
                "tms_fault_level_ge_2",
                "fire_fault_level_ge_1",
                "fire_fault_level_ge_2",

                "f1_del_temp",
                "f1_over_temp",
                "f1_over_ucell",
                "f1_low_ucell",
                "f1_low_ins_res",
                "f1_ucell_uniformity",
                "f1_over_chg",
                "f1_over_soc",
                "f1_soc_change_fast",
                "f1_bat_sys_not_match",
                "f1_hvil_fault",

                "f2_tms_err",
                "f2_pack_self_protect",
                "f2_main_loop_prechg_err",
                "f2_aux_loop_prechg_err",
                "f2_chrg_ins_low_err",
                "f2_acan_lost",
                "f2_inner_comm_err",
                "f2_dcdc_err",
                "f2_branch_break_err",

                "f2_heat_relay_open_err",
                "f2_heat_relay_weld_err",
                "f2_main_pos_open_err",
                "f2_main_pos_weld_err",
                "f2_main_neg_open_err",
                "f2_main_neg_weld_err"
            };

            for (auto* k : known)
            {
                if (signal_norm == k) return true;
            }

            return false;
        }

        if (source_norm == "pcu")
        {
            return signal_norm == "offline" ||
                   signal_norm == "online" ||
                   signal_norm == "runtime_offline" ||
                   signal_norm == "runtime_online" ||
                   signal_norm == "rx_timeout" ||
                   signal_norm == "heartbeat_stale" ||
                   signal_norm == "heartbeat_missing" ||

                   signal_norm == "estop" ||
                   signal_norm == "emergency_stop" ||
                   signal_norm == "fault_estop" ||
                   signal_norm == "pcu_emergency_stop" ||

                   signal_norm == "pcu1_comm_fault" ||
                   signal_norm == "pcu2_comm_fault" ||
                   signal_norm == "pcu1_emergency_stop" ||
                   signal_norm == "pcu2_emergency_stop";
        }

        if (source_norm == "logic")
        {
            return signal_norm == "any_fault" ||
                signal_norm == "pcu_any_offline" ||
                signal_norm == "bms_any_offline" ||

                signal_norm == "ups_offline" ||
                signal_norm == "smoke_offline" ||
                signal_norm == "gas_offline" ||
                signal_norm == "air_offline" ||

                signal_norm == "env_any_alarm" ||
                signal_norm == "alarm_any" ||
                signal_norm == "fault_any" ||
                signal_norm == "system_estop" ||

                signal_norm == "hmi_comm_fault" ||
                signal_norm == "remote_comm_fault" ||
                signal_norm == "sdcard_fault" ||

                signal_norm == "pcu0_offline" ||
                signal_norm == "pcu1_offline" ||
                signal_norm == "pcu1_comm_fault" ||
                signal_norm == "pcu2_comm_fault";
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
        (void)cache;
        (void)faults;

        // 第五批收敛：
        // BMS 内部故障不再由 FaultRuntimeMapper 直接解释。
        //
        // 正确链路：
        //   BmsFaultEvaluator
        //      -> ctx.bms_confirmed_faults
        //      -> BmsFaultMapper
        //      -> FaultCenter
        //
        // 这里保留空函数只是兼容旧调用点，避免旧接口删除引发联动修改。
    }

    bool FaultRuntimeMapper::isRuntimeDebounceSource_(const std::string& source_norm)
    {
        return source_norm == "ups" ||
            source_norm == "air" ||
            source_norm == "smoke" ||
            source_norm == "gas";
    }

    bool FaultRuntimeMapper::isOfflineSignal_(const Rule& rule)
    {
        const std::string src = normalizeToken_(rule.source_norm);
        const std::string sig = normalizeToken_(rule.signal_norm);

        if (src == "ups")
        {
            return sig == "offline" ||
                sig == "runtime_offline" ||
                sig == "ups_offline" ||
                sig == "ups_comm_fault";
        }

        if (src == "air")
        {
            return sig == "offline" ||
                sig == "runtime_offline" ||
                sig == "air_offline" ||
                sig == "aircon_comm_fault" ||
                sig == "air_comm_fault";
        }

        if (src == "smoke")
        {
            return sig == "tss_offline" ||
                sig == "smoke_offline" ||
                sig == "tss_comm_fault" ||
                sig == "smoke_comm_fault";
        }

        if (src == "gas")
        {
            return sig == "gas_offline" ||
                sig == "cgs_comm_fault" ||
                sig == "gas_comm_fault";
        }

        return false;
    }

    FaultRuntimeMapper::DebouncePolicy
    FaultRuntimeMapper::policyForRule_(const Rule& rule)
    {
        DebouncePolicy p{};

        if (!isRuntimeDebounceSource_(rule.source_norm))
        {
            p.enable = false;
            return p;
        }

        p.enable = true;

        // 通信离线类：沿用旧设备 evaluateXxx_ 风格。
        // 注意：离线前面通常已经过 Scheduler disconnect_window aging，
        // 这里再做 3000ms 是为了避免 HMI 故障页抖动。
        if (isOfflineSignal_(rule))
        {
            p.trigger_ms = 3000;
            p.clear_ms = 0;
            return p;
        }

        // 设备内部故障 / 告警：沿用旧 Smoke/Gas/Air/UPS 的 3000/1000 防抖风格。
        p.trigger_ms = 3000;
        p.clear_ms = 1000;
        return p;
    }

    std::string FaultRuntimeMapper::makeDebounceKey_(const Rule& rule)
    {
        // code 加入 key，避免不同故障码错误共用同一 source/signal 状态。
        return rule.source_norm + "|" +
            rule.signal_norm + "|" +
            std::to_string(static_cast<unsigned>(rule.instance)) + "|" +
            std::to_string(static_cast<unsigned>(rule.code));
    }

    bool FaultRuntimeMapper::debounceRule_(const Rule& rule,
                                           bool raw_active,
                                           uint64_t now_ms) const
    {
        const DebouncePolicy policy = policyForRule_(rule);

        if (!policy.enable)
        {
            return raw_active;
        }

        const std::string key = makeDebounceKey_(rule);
        auto& st = debounce_states_[key];

        if (!st.initialized)
        {
            st.initialized = true;
            st.last_raw = raw_active;
            st.raw_since_ms = now_ms;

            // 初始为 active 时仍需满足 trigger_ms；
            // 初始为 inactive 时直接保持 inactive。
            st.output = raw_active && (policy.trigger_ms == 0);
            return st.output;
        }

        if (st.last_raw != raw_active)
        {
            st.last_raw = raw_active;
            st.raw_since_ms = now_ms;
        }

        if (now_ms < st.raw_since_ms)
        {
            // 防御系统时间异常回跳。
            st.raw_since_ms = now_ms;
        }

        const uint64_t held_ms = now_ms - st.raw_since_ms;

        if (raw_active != st.output)
        {
            const uint32_t need_ms = raw_active ? policy.trigger_ms : policy.clear_ms;
            if (need_ms == 0 || held_ms >= need_ms)
            {
                st.output = raw_active;
            }
        }

        return st.output;
    }


    void FaultRuntimeMapper::applyAll(const LogicContext& ctx,
                                      control::FaultCenter& faults,
                                      uint64_t now_ms) const
    {
        for (const auto& rule : rules_)
        {
            // 第五批收敛：
            // source_norm == "bms" 的 BMS 内部故障，不再由 RuntimeMapper 解释。
            // 否则会在 LogicEngine::applyFaultPages_() 中，
            // 覆盖前一步 BmsFaultMapper 对同一 code 的 setActive 结果。
            //
            // 注意：
            // - VCU 侧的 BMS1~BMS4 通信故障 source_norm 不是 bms，不受影响。
            // - UPS/AIR/Smoke/Gas direct debounce 不受影响。
            // if (rule.source_norm == "bms") {
            //     continue;
            // }

            bool active = false;
            bool matched = false;

            // ------------------------------------------------------------
            // UPS / AIR / Smoke / Gas：
            // 设备协议原始故障统一走 direct runtime mapper。
            //
            // raw_active:
            //   ctx.xxx_faults -> evalXxxSignal_()
            //
            // active:
            //   raw_active -> debounceRule_()
            //
            // 目的：
            //   1. 不再经过 FaultLogicEvaluator confirmed 中转
            //   2. 不恢复 evaluateUps_()
            //   3. AIR/Smoke/Gas 也迁出 evaluateXxx_()
            //   4. 保留旧链路中的 3000/1000 防抖能力
            // ------------------------------------------------------------
            if (rule.source_norm == "ups")
            {
                const bool raw_active = evalUpsSignal_(ctx.ups_faults, rule.signal_norm);
                active = debounceRule_(rule, raw_active, now_ms);
                matched = true;

                LOG_THROTTLE_MS(
                    ("fault_ups_direct_debounced_" + std::to_string(rule.code)).c_str(),
                    500,
                    LOGINFO,
                    "[FAULT][UPS][DIRECT_DEBOUNCE] code=0x%04X sig=%s raw=%d active=%d "
                    "seen=%d online=%d alarm_any=%d fault_any=%d warn=0x%08X fault=0x%08X",
                    static_cast<unsigned>(rule.code),
                    rule.signal_norm.c_str(),
                    raw_active ? 1 : 0,
                    active ? 1 : 0,
                    ctx.ups_faults.seen_once ? 1 : 0,
                    ctx.ups_faults.online ? 1 : 0,
                    ctx.ups_faults.alarm_any ? 1 : 0,
                    ctx.ups_faults.fault_any ? 1 : 0,
                    static_cast<unsigned>(ctx.ups_faults.warning_bits),
                    static_cast<unsigned>(ctx.ups_faults.fault_bits)
                );

                faults.setActive(rule.code, active);
                continue;
            }

            if (rule.source_norm == "air")
            {
                const bool raw_active = evalAirSignal_(ctx.air_faults, rule.signal_norm);
                active = debounceRule_(rule, raw_active, now_ms);
                matched = true;

                LOG_THROTTLE_MS(
                    ("fault_air_direct_debounced_" + std::to_string(rule.code)).c_str(),
                    500,
                    LOGINFO,
                    "[FAULT][AIR][DIRECT_DEBOUNCE] code=0x%04X sig=%s raw=%d active=%d "
                    "seen=%d online=%d alarm_any=%d fault_any=%d",
                    static_cast<unsigned>(rule.code),
                    rule.signal_norm.c_str(),
                    raw_active ? 1 : 0,
                    active ? 1 : 0,
                    ctx.air_faults.seen_once ? 1 : 0,
                    ctx.air_faults.online ? 1 : 0,
                    ctx.air_faults.alarm_any ? 1 : 0,
                    ctx.air_faults.fault_any ? 1 : 0
                );

                faults.setActive(rule.code, active);
                continue;
            }

            if (rule.source_norm == "smoke")
            {
                const bool raw_active = evalSmokeSignal_(ctx.smoke_faults, rule.signal_norm);
                active = debounceRule_(rule, raw_active, now_ms);
                matched = true;

                LOG_THROTTLE_MS(
                    ("fault_smoke_direct_debounced_" + std::to_string(rule.code)).c_str(),
                    500,
                    LOGINFO,
                    "[FAULT][SMOKE][DIRECT_DEBOUNCE] code=0x%04X sig=%s raw=%d active=%d "
                    "seen=%d online=%d alarm_any=%d fault_any=%d",
                    static_cast<unsigned>(rule.code),
                    rule.signal_norm.c_str(),
                    raw_active ? 1 : 0,
                    active ? 1 : 0,
                    ctx.smoke_faults.seen_once ? 1 : 0,
                    ctx.smoke_faults.online ? 1 : 0,
                    ctx.smoke_faults.alarm_any ? 1 : 0,
                    ctx.smoke_faults.fault_any ? 1 : 0
                );

                faults.setActive(rule.code, active);
                continue;
            }

            if (rule.source_norm == "gas")
            {
                const bool raw_active = evalGasSignal_(ctx.gas_faults, rule.signal_norm);
                active = debounceRule_(rule, raw_active, now_ms);
                matched = true;

                LOG_THROTTLE_MS(
                    ("fault_gas_direct_debounced_" + std::to_string(rule.code)).c_str(),
                    500,
                    LOGINFO,
                    "[FAULT][GAS][DIRECT_DEBOUNCE] code=0x%04X sig=%s raw=%d active=%d "
                    "seen=%d online=%d alarm_any=%d fault_any=%d status=0x%04X",
                    static_cast<unsigned>(rule.code),
                    rule.signal_norm.c_str(),
                    raw_active ? 1 : 0,
                    active ? 1 : 0,
                    ctx.gas_faults.seen_once ? 1 : 0,
                    ctx.gas_faults.online ? 1 : 0,
                    ctx.gas_faults.alarm_any ? 1 : 0,
                    ctx.gas_faults.fault_any ? 1 : 0,
                    static_cast<unsigned>(ctx.gas_faults.status_code)
                );

                faults.setActive(rule.code, active);
                continue;
            }

            // ------------------------------------------------------------
            // 其余 source：保持 confirmed 优先。
            // BMS：主要由 BmsFaultEvaluator 写入 confirmed，再由这里落码。
            // PCU / Logic：仍然使用 FaultLogicEvaluator 的 confirmed 防抖。
            // ------------------------------------------------------------
            auto tryConfirmedSignal = [&](bool& out_active) -> bool
            {
                std::vector<std::string> keys;

                auto add_if_not_empty = [&](const std::string& s)
                {
                    if (!s.empty()) keys.push_back(s);
                };

                // ---------- BMS confirmed signals ----------
                // if (rule.source_norm == "bms")
                // {
                //     if (rule.instance >= 1 && rule.instance <= 4)
                //     {
                //         add_if_not_empty("BMS_" + std::to_string(rule.instance) + "." + rule.signal_norm);
                //     }
                // }

                // ---------- PCU confirmed signals ----------
                // else
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

                    /*
                     * fault_map:
                     *   PCU1 -> instance=1 -> 内部 pcu0_state
                     *   PCU2 -> instance=2 -> 内部 pcu1_state
                     */
                    if (inst == 1)
                    {
                        if (rule.signal_norm == "pcu1_comm_fault" ||
                            rule.signal_norm == "offline" ||
                            rule.signal_norm == "runtime_offline")
                        {
                            add_if_not_empty("logic.pcu1_comm_fault");
                            add_if_not_empty("logic.pcu0_offline");
                        }

                        if (rule.signal_norm == "pcu1_emergency_stop" ||
                            rule.signal_norm == "estop" ||
                            rule.signal_norm == "emergency_stop" ||
                            rule.signal_norm == "fault_estop")
                        {
                            add_if_not_empty("logic.pcu1_emergency_stop");
                            add_if_not_empty("logic.pcu0_emergency_stop");
                        }
                    }
                    else if (inst == 2)
                    {
                        if (rule.signal_norm == "pcu2_comm_fault" ||
                            rule.signal_norm == "offline" ||
                            rule.signal_norm == "runtime_offline")
                        {
                            add_if_not_empty("logic.pcu2_comm_fault");
                            add_if_not_empty("logic.pcu1_offline");
                        }

                        if (rule.signal_norm == "pcu2_emergency_stop" ||
                            rule.signal_norm == "estop" ||
                            rule.signal_norm == "emergency_stop" ||
                            rule.signal_norm == "fault_estop")
                        {
                            add_if_not_empty("logic.pcu2_emergency_stop");
                            add_if_not_empty("logic.pcu1_emergency_stop");
                        }
                    }
                }

                // ---------- Logic / VCU / system confirmed signals ----------
                else if (rule.source_norm == "logic")
                {
                    add_if_not_empty("logic." + rule.signal_norm);

                    if (rule.signal_norm == "pcu1_comm_fault" ||
                        rule.signal_norm == "pcu1_offline")
                    {
                        add_if_not_empty("logic.pcu0_offline");
                        add_if_not_empty("logic.pcu1_comm_fault");
                    }

                    if (rule.signal_norm == "pcu2_comm_fault" ||
                        rule.signal_norm == "pcu2_offline")
                    {
                        add_if_not_empty("logic.pcu1_offline");
                        add_if_not_empty("logic.pcu2_comm_fault");
                    }

                    if (rule.signal_norm == "ups_comm_fault")
                        add_if_not_empty("logic.ups_comm_fault");

                    if (rule.signal_norm == "ups_offline")
                        add_if_not_empty("logic.ups_comm_fault");

                    if (rule.signal_norm == "tss_offline" || rule.signal_norm == "tss_comm_fault")
                        add_if_not_empty("logic.tss_comm_fault");

                    if (rule.signal_norm == "cgs_comm_fault" || rule.signal_norm == "gas_comm_fault")
                        add_if_not_empty("logic.cgs_comm_fault");

                    if (rule.signal_norm == "air_offline" ||
                        rule.signal_norm == "air_comm_fault" ||
                        rule.signal_norm == "aircon_comm_fault")
                        add_if_not_empty("logic.air_comm_fault");
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

            // 1) 优先 confirmed signals
            if (tryConfirmedSignal(active))
            {
                LOG_THROTTLE_MS(
                    ("fault_rule_confirmed_" + std::to_string(rule.code)).c_str(),
                    1000,
                    LOGINFO,
                    "[FAULT][RUNTIME][CONFIRMED] code=0x%04X source=%s signal=%s inst=%u active=%d",
                    static_cast<unsigned>(rule.code),
                    rule.source_norm.c_str(),
                    rule.signal_norm.c_str(),
                    static_cast<unsigned>(rule.instance),
                    active ? 1 : 0
                );

                faults.setActive(rule.code, active);
                continue;
            }

            // 2) 回退到现有原始真源逻辑
            // if (rule.source_norm == "bms")
            // {
            //     uint32_t inst = rule.instance;
            //
            //     if (inst == 0)
            //     {
            //         uint32_t inferred = 0;
            //         if (tryParseInstanceFromSignal_(rule.signal_norm, inferred))
            //         {
            //             inst = inferred;
            //         }
            //     }
            //
            //     if (inst >= 1 && inst <= 4)
            //     {
            //         const std::string key = "BMS_" + std::to_string(inst);
            //         auto it = ctx.bms_cache.items.find(key);
            //         if (it != ctx.bms_cache.items.end())
            //         {
            //             active = evalBmsSignal_(it->second, rule.signal_norm);
            //             matched = true;
            //         }
            //     }
            // }
            // else
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
                    active = evalPcuSignal_(ctx.pcu0_state, rule.signal_norm);
                    matched = true;
                }
                else if (inst == 2)
                {
                    active = evalPcuSignal_(ctx.pcu1_state, rule.signal_norm);
                    matched = true;
                }
            }
            else if (rule.source_norm == "logic")
            {
                active = evalLogicSignal_(ctx.logic_faults, rule.signal_norm);
                matched = true;
            }

            LOG_THROTTLE_MS(
                ("fault_rule_fallback_" + std::to_string(rule.code)).c_str(),
                1000,
                LOGINFO,
                "[FAULT][RUNTIME][FALLBACK] code=0x%04X source=%s signal=%s inst=%u active=%d matched=%d",
                static_cast<unsigned>(rule.code),
                rule.source_norm.c_str(),
                rule.signal_norm.c_str(),
                static_cast<unsigned>(rule.instance),
                active ? 1 : 0,
                matched ? 1 : 0
            );

            faults.setActive(rule.code, active);
        }
    }
} // namespace control
