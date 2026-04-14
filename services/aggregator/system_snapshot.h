//
// Created by lxy on 2026/1/12.
//
#ifndef ENERGYSTORAGE_SYSTEM_SNAPSHOT_H
#define ENERGYSTORAGE_SYSTEM_SNAPSHOT_H

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <optional>
#include <nlohmann/json.hpp>

#include "protocol_base.h"
#include "snapshot_device_types.h"

/*
 * SystemSnapshot 设计原则
 * 1. system：系统级抽象（策略 / UI）
 * 2. items ：设备级完整数据（可回放 / 可审计）
 * 3. 复杂设备在 device 级分组（结构体放 snapshot_device_types.h）
 */

enum class DeviceHealth
{
    ONLINE,
    DEGRADED,
    OFFLINE
};

/* ======================= SnapshotItem ======================= */

struct SnapshotItem
{
    uint64_t ts_ms{0};

    // 最后一帧原始数据（复杂设备也保留一份，便于通用逻辑读取）
    DeviceData data;

    // ---- GasDetector ----
    std::map<GasType, GasChannelState> gas_channels;

    // ---- UPS ----
    std::map<std::string /*Q1/Q6/WA*/, UpsGroupData> ups_groups;

    // ---- AirConditioner ----
    std::optional<AirconSnapshot> aircon;

    // ---- PCU ----
    std::optional<PcuSnapshot> pcu;

    // ---- Health ----
    DeviceHealth health{DeviceHealth::OFFLINE};
    bool online{false};

    uint64_t last_ok_ms{0};
    uint32_t disconnect_window_ms{0};
    uint64_t last_offline_ms{0};

    uint32_t disconnect_count{0};
};

namespace agg
{
    /* ======================= SystemSnapshot ======================= */

    struct SystemSnapshot
    {
        uint64_t timestamp_ms{0};

        /* -------- system-level -------- */
        double gas_ppm{0.0};
        double smoke_percent{0.0};

        int smoke_temperature{0};

        double ac_indoor_temp{0.0};
        double ac_humidity{0.0};
        int ac_power{0};
        int ac_run_state{0};
        bool ac_alarm{false};

        int system_temperature{0};

        bool gas_alarm{false};
        bool smoke_alarm{false};

        // ---- BMS summary（轻量投影）----
        uint64_t bms_ts_ms{0};
        double bms_soc{0.0};
        double bms_pack_v{0.0};
        double bms_pack_i{0.0};
        uint32_t bms_fault_num{0};
        bool bms_alarm_any{false};


        /* -------- device items -------- */
        std::map<std::string, SnapshotItem> items;

        /* ================= JSON ================= */

        nlohmann::json toJson() const
        {
            nlohmann::json j;
            j["timestamp_ms"] = timestamp_ms;

            j["system"] = {
                {"gas_ppm", gas_ppm},
                {"smoke_percent", smoke_percent},
                {"smoke_temperature", smoke_temperature},
                {"ac_indoor_temp", ac_indoor_temp},
                {"ac_humidity", ac_humidity},
                {"ac_power", ac_power},
                {"ac_run_state", ac_run_state},
                {"ac_alarm", ac_alarm},
                {"system_temperature", system_temperature},
                {"gas_alarm", gas_alarm},
                {"smoke_alarm", smoke_alarm},

                // ---- BMS summary（轻量投影）----
                {"bms_ts_ms", bms_ts_ms},
                {"bms_soc", bms_soc},
                {"bms_pack_v", bms_pack_v},
                {"bms_pack_i", bms_pack_i},
                {"bms_fault_num", bms_fault_num},
                {"bms_alarm_any", bms_alarm_any},

            };

            nlohmann::json j_items = nlohmann::json::object();

            for (const auto& [name, item] : items)
            {
                nlohmann::json ji;
                ji["ts_ms"] = item.ts_ms;

                /* ===== PCU（按 state/ctrl 分组）===== */
                if (item.pcu.has_value())
                {
                    const auto& p = *item.pcu;
                    nlohmann::json jp = nlohmann::json::object();

                    // ---- state ----
                    jp["state"] = {
                        {"ts_ms", p.state.ts_ms},
                        {"num", p.state.num},
                        {"value", p.state.value},
                        {"status", p.state.status}
                    };

                    // ---- ctrl：没填就不输出（避免空对象占位）----
                    const bool has_ctrl =
                        (p.ctrl.ts_ms != 0) ||
                        (!p.ctrl.num.empty()) ||
                        (!p.ctrl.value.empty()) ||
                        (!p.ctrl.status.empty());

                    if (has_ctrl)
                    {
                        jp["ctrl"] = {
                            {"ts_ms", p.ctrl.ts_ms},
                            {"num", p.ctrl.num},
                            {"value", p.ctrl.value},
                            {"status", p.ctrl.status}
                        };
                    }

                    ji["data"] = jp;
                }

                /* ===== AirConditioner ===== */
                else if (item.aircon.has_value())
                {
                    const auto& ac = *item.aircon;
                    ji["data"] = {
                        {
                            "version", {
                                {"ts_ms", ac.version.ts_ms},
                                {"fields", ac.version.fields}
                            }
                        },
                        {
                            "run_state", {
                                {"ts_ms", ac.run_state.ts_ms},
                                {"fields", ac.run_state.fields}
                            }
                        },
                        {
                            "sensor_state", {
                                {"ts_ms", ac.sensor_state.ts_ms},
                                {"fields", ac.sensor_state.fields}
                            }
                        },
                        {
                            "warn_state", {
                                {"ts_ms", ac.warn_state.ts_ms},
                                {"fields", ac.warn_state.fields}
                            }
                        },
                        {
                            "sys_para", {
                                {"ts_ms", ac.sys_para.ts_ms},
                                {"fields", ac.sys_para.fields}
                            }
                        },
                        {
                            "remote_para", {
                                {"ts_ms", ac.remote_para.ts_ms},
                                {"fields", ac.remote_para.fields}
                            }
                        }
                    };
                }

                /* ===== UPS ===== */
                else if (!item.ups_groups.empty())
                {
                    nlohmann::json ju;
                    for (const auto& [cmd, grp] : item.ups_groups)
                    {
                        ju[cmd] = {
                            {"num", grp.num},
                            {"value", grp.value},
                            {"status", grp.status},
                            {"ts_ms", grp.ts_ms}
                        };
                    }
                    ji["data"] = ju;
                }

                /* ===== Other devices（写入 value，保证 CAN/其他设备字段完整落盘）===== */
                else
                {
                    ji["data"] = {
                        {"num", item.data.num},
                        {"value", item.data.value},
                        {"status", item.data.status}
                    };
                }

                /* ---- gas channels ---- */
                if (!item.gas_channels.empty())
                {
                    nlohmann::json jg;
                    for (const auto& [gt, ch] : item.gas_channels)
                    {
                        jg[std::to_string((uint16_t)gt)] = {
                            {"valid", ch.valid},
                            {"value", ch.value},
                            {"raw", ch.raw},
                            {"status", ch.status},
                            {"decimal", ch.decimal_code},
                            {"type_code", ch.type_code},
                            {"unit_code", ch.unit_code},
                            {"ts_ms", ch.ts_ms}
                        };
                    }
                    ji["gas_channels"] = jg;
                }

                ji["health"] = {
                    {"state", (int)item.health},
                    {"online", item.online},
                    {"last_ok_ms", item.last_ok_ms},
                    {"disconnect_window_ms", item.disconnect_window_ms},
                    {"last_offline_ms", item.last_offline_ms},
                    {"disconnect_count", item.disconnect_count}
                };

                j_items[name] = ji;
            }

            j["items"] = j_items;
            return j;
        }
    };
} // namespace agg

#endif // ENERGYSTORAGE_SYSTEM_SNAPSHOT_H
