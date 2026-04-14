//
// Created by lxy on 2026/2/12.
//
#ifndef ENERGYSTORAGE_SNAPSHOT_DEVICE_TYPES_H
#define ENERGYSTORAGE_SNAPSHOT_DEVICE_TYPES_H

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <optional>

#include "protocol_base.h"

/*
 * 设备级快照结构体（items 的“强类型分组”）
 * - 复杂设备（Gas / UPS / AirConditioner / PCU / future BMS...）的分组结构都放这里
 * - SystemSnapshot 本体保持简洁
 */

/* ======================= GasDetector ======================= */

struct GasChannelState
{
    bool valid{false};
    double value{0.0};

    uint16_t raw{0};
    uint16_t status{0};
    uint16_t decimal_code{0};
    uint16_t unit_code{0};
    uint16_t type_code{0};

    uint64_t ts_ms{0};
};

/* ======================= UPS ======================= */

struct UpsGroupData
{
    std::map<std::string, double> num;
    std::map<std::string, int32_t> value;
    std::map<std::string, uint32_t> status;
    uint64_t ts_ms{0};
};

/* ======================= AirConditioner ======================= */

struct AirconGroup
{
    uint64_t ts_ms{0};
    std::map<std::string, double> fields;
};

struct AirconSnapshot
{
    AirconGroup version;
    AirconGroup run_state;
    AirconGroup sensor_state;
    AirconGroup warn_state;
    AirconGroup sys_para;
    AirconGroup remote_para;
};

/* ======================= PCU ======================= */
/*
 * 你要求：只保存
 *  - PCU->EMU 状态报文（state）
 *  - EMU->PCU 控制报文（ctrl）
 *
 * 这里按 state/ctrl 分组，结构风格与 UPS/Aircon 类似，便于管理。
 */
struct PcuGroupData
{
    std::map<std::string, double> num;
    std::map<std::string, int32_t> value;
    std::map<std::string, uint32_t> status;
    uint64_t ts_ms{0};
};

struct PcuSnapshot
{
    PcuGroupData state; // PCU->EMU 状态反馈帧（0x1801EFA0）
    PcuGroupData ctrl;  // EMU->PCU 控制帧（0x1801A0E0）镜像
};

/* ======================= future: BMS ======================= */
/*
 * 未来 BMS 变量多，建议也放在这里：
 * struct BmsSnapshot { ... }
 */

#endif // ENERGYSTORAGE_SNAPSHOT_DEVICE_TYPES_H
