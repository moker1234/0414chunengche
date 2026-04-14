//
// Created by lxy on 2026/2/28.
//

#ifndef ENERGYSTORAGE_FAULT_IDS_H
#define ENERGYSTORAGE_FAULT_IDS_H
// services/fault/fault_ids.h
#pragma once
#include <cstdint>

namespace fault_ids {


        // =========================================================
        // 非 BMS / logic 侧基础故障码
        // 说明：
        // 1) 这些码优先与 fault_map.jsonl 中现有 code_hex 对齐
        // 2) 当前阶段主要用于：
        //    - 配置文件统一命名
        //    - 后续若某些 logic / mapper 需要直接 setActive(code, on) 时可复用
        // =========================================================

        // ---- 通信/离线类（与 fault_map.jsonl 现有 0x100x 对齐）----
        static constexpr uint16_t HMI_COMM_FAULT       = 0x1000;
        static constexpr uint16_t PCU1_COMM_FAULT      = 0x1005;
        static constexpr uint16_t PCU2_COMM_FAULT      = 0x1006;
        static constexpr uint16_t AIR_COMM_FAULT       = 0x1007;
        static constexpr uint16_t UPS_COMM_FAULT       = 0x1008;
        static constexpr uint16_t TSS_COMM_FAULT       = 0x100A;
        static constexpr uint16_t CGS_COMM_FAULT       = 0x100B;

        // ---- 传感器/环境告警类（与 fault_map.jsonl 现有 0x101x / 0x102x 对齐）----
        static constexpr uint16_t CGS_SENSOR_FAULT     = 0x1016;
        static constexpr uint16_t CGS_SENSOR_LOW       = 0x1017;
        static constexpr uint16_t CGS_SENSOR_HIGH      = 0x1018;

        static constexpr uint16_t TSS_SMOKE_ALARM      = 0x101E;
        static constexpr uint16_t TSS_TEMP_ALARM       = 0x101F;

        static constexpr uint16_t AIR_HIGH_TEMP_ALARM  = 0x1028;
        static constexpr uint16_t AIR_LOW_TEMP_ALARM   = 0x1029;
        static constexpr uint16_t AIR_HIGH_HUMI_ALARM  = 0x102A;
        static constexpr uint16_t AIR_LOW_HUMI_ALARM   = 0x102B;

        // ---- logic 聚合类（当前阶段优先作为运行态映射目标）----
        static constexpr uint16_t SYS_GAS_ALARM        = 0x1001;
        static constexpr uint16_t SYS_SMOKE_ALARM      = 0x1002;
        static constexpr uint16_t SYS_AC_ALARM         = 0x1003;
        static constexpr uint16_t SYS_BMS_ALARM_ANY    = 0x1201;

        static constexpr uint16_t DEV_OFFLINE_BASE     = 0x1800; // + hash/slot

    // =========================================================
    // BMS 4 路故障编码
    // 规则：
    //   0x21xx = BMS_1
    //   0x22xx = BMS_2
    //   0x23xx = BMS_3
    //   0x24xx = BMS_4
    // =========================================================
    constexpr uint16_t bmsBase(uint8_t inst) {
        return static_cast<uint16_t>(0x2000u + (static_cast<uint16_t>(inst) << 8));
    }

    // ---- summary/runtime ----
    constexpr uint16_t BMS_RUNTIME_OFFLINE(uint8_t inst)   { return static_cast<uint16_t>(bmsBase(inst) + 0x01); }
    constexpr uint16_t BMS_FAULT_BLOCK_HV(uint8_t inst)    { return static_cast<uint16_t>(bmsBase(inst) + 0x02); }
    constexpr uint16_t BMS_INS_LOW_ANY(uint8_t inst)       { return static_cast<uint16_t>(bmsBase(inst) + 0x03); }
    constexpr uint16_t BMS_RUNTIME_STALE(uint8_t inst)         { return static_cast<uint16_t>(bmsBase(inst) + 0x04); }
    constexpr uint16_t BMS_ST2_STALE(uint8_t inst)             { return static_cast<uint16_t>(bmsBase(inst) + 0x05); }
    constexpr uint16_t BMS_CURRENT_LIMIT_STALE(uint8_t inst)   { return static_cast<uint16_t>(bmsBase(inst) + 0x06); }
    constexpr uint16_t BMS_FAULT1_STALE(uint8_t inst)          { return static_cast<uint16_t>(bmsBase(inst) + 0x07); }
    constexpr uint16_t BMS_FAULT2_STALE(uint8_t inst)          { return static_cast<uint16_t>(bmsBase(inst) + 0x08); }

    // ---- Fault1 ----
    constexpr uint16_t BMS_F1_DEL_TEMP(uint8_t inst)           { return static_cast<uint16_t>(bmsBase(inst) + 0x10); }
    constexpr uint16_t BMS_F1_OVER_TEMP(uint8_t inst)          { return static_cast<uint16_t>(bmsBase(inst) + 0x11); }
    constexpr uint16_t BMS_F1_OVER_UCELL(uint8_t inst)         { return static_cast<uint16_t>(bmsBase(inst) + 0x12); }
    constexpr uint16_t BMS_F1_LOW_UCELL(uint8_t inst)          { return static_cast<uint16_t>(bmsBase(inst) + 0x13); }
    constexpr uint16_t BMS_F1_LOW_INS_RES(uint8_t inst)        { return static_cast<uint16_t>(bmsBase(inst) + 0x14); }
    constexpr uint16_t BMS_F1_UCELL_UNIFORMITY(uint8_t inst)   { return static_cast<uint16_t>(bmsBase(inst) + 0x15); }
    constexpr uint16_t BMS_F1_OVER_CHG(uint8_t inst)           { return static_cast<uint16_t>(bmsBase(inst) + 0x16); }
    constexpr uint16_t BMS_F1_OVER_SOC(uint8_t inst)           { return static_cast<uint16_t>(bmsBase(inst) + 0x17); }
    constexpr uint16_t BMS_F1_SOC_CHANGE_FAST(uint8_t inst)    { return static_cast<uint16_t>(bmsBase(inst) + 0x18); }
    constexpr uint16_t BMS_F1_BAT_SYS_NOT_MATCH(uint8_t inst)  { return static_cast<uint16_t>(bmsBase(inst) + 0x19); }
    constexpr uint16_t BMS_F1_HVIL_FAULT(uint8_t inst)         { return static_cast<uint16_t>(bmsBase(inst) + 0x1A); }

    // ---- Fault2 ----
    constexpr uint16_t BMS_F2_TMS_ERR(uint8_t inst)                { return static_cast<uint16_t>(bmsBase(inst) + 0x20); }
    constexpr uint16_t BMS_F2_PACK_SELF_PROTECT(uint8_t inst)      { return static_cast<uint16_t>(bmsBase(inst) + 0x21); }
    constexpr uint16_t BMS_F2_MAIN_LOOP_PRECHG_ERR(uint8_t inst)   { return static_cast<uint16_t>(bmsBase(inst) + 0x22); }
    constexpr uint16_t BMS_F2_AUX_LOOP_PRECHG_ERR(uint8_t inst)    { return static_cast<uint16_t>(bmsBase(inst) + 0x23); }
    constexpr uint16_t BMS_F2_CHRG_INS_LOW_ERR(uint8_t inst)       { return static_cast<uint16_t>(bmsBase(inst) + 0x24); }
    constexpr uint16_t BMS_F2_ACAN_LOST(uint8_t inst)              { return static_cast<uint16_t>(bmsBase(inst) + 0x25); }
    constexpr uint16_t BMS_F2_INNER_COMM_ERR(uint8_t inst)         { return static_cast<uint16_t>(bmsBase(inst) + 0x26); }
    constexpr uint16_t BMS_F2_DCDC_ERR(uint8_t inst)               { return static_cast<uint16_t>(bmsBase(inst) + 0x27); }
    constexpr uint16_t BMS_F2_BRANCH_BREAK_ERR(uint8_t inst)       { return static_cast<uint16_t>(bmsBase(inst) + 0x28); }
    constexpr uint16_t BMS_F2_HEAT_RELAY_OPEN_ERR(uint8_t inst)    { return static_cast<uint16_t>(bmsBase(inst) + 0x29); }
    constexpr uint16_t BMS_F2_HEAT_RELAY_WELD_ERR(uint8_t inst)    { return static_cast<uint16_t>(bmsBase(inst) + 0x2A); }
    constexpr uint16_t BMS_F2_MAIN_POS_OPEN_ERR(uint8_t inst)      { return static_cast<uint16_t>(bmsBase(inst) + 0x2B); }
    constexpr uint16_t BMS_F2_MAIN_POS_WELD_ERR(uint8_t inst)      { return static_cast<uint16_t>(bmsBase(inst) + 0x2C); }
    constexpr uint16_t BMS_F2_MAIN_NEG_OPEN_ERR(uint8_t inst)      { return static_cast<uint16_t>(bmsBase(inst) + 0x2D); }
    constexpr uint16_t BMS_F2_MAIN_NEG_WELD_ERR(uint8_t inst)      { return static_cast<uint16_t>(bmsBase(inst) + 0x2E); }

    // ---- confirmed sample faults（第八批正式落码）----
    constexpr uint16_t BMS_SOC_LOW_20_CONFIRM(uint8_t inst)        { return static_cast<uint16_t>(bmsBase(inst) + 0x30); }
    constexpr uint16_t BMS_SOC_GT_100_CONFIRM(uint8_t inst)        { return static_cast<uint16_t>(bmsBase(inst) + 0x31); }
    constexpr uint16_t BMS_TEMP_DELTA_GE_25_CONFIRM(uint8_t inst)  { return static_cast<uint16_t>(bmsBase(inst) + 0x32); }

} // namespace fault_ids
#endif //ENERGYSTORAGE_FAULT_IDS_H