// services/protocol/rs485/hmi/hmi_address_table.h
//
// Created by lxy on 2026/2/4.
//
#ifndef ENERGYSTORAGE_HMI_ADDRESS_TABLE_H
#define ENERGYSTORAGE_HMI_ADDRESS_TABLE_H

#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_map>

// ======================= 地址规划（重要）=======================
//
// bool_rw :  0x0050..0x006F  (FC01/05/0F Coils)
// int_rw  :  0x0070..0x02FF  (FC03/06/10 Holding Registers)  ✅ 放宽：覆盖 0x0200..（逻辑写入）
// bool_rd :  0x0100..0x2FFF  (FC02 Discrete Inputs)
// int_rd  :  0x0080..0x4FFF  (FC04 Input Registers)          ✅ 放宽：覆盖故障区 0x0080..0x0100
//
// 说明：Modbus 的 FC03/FC04 地址空间彼此独立，因此 int_rw 与 int_rd 的地址数值“重叠”不冲突。
// =================================================================

static constexpr uint16_t HMI_BOOL_RW_START  = 0x1000;
static constexpr uint16_t HMI_BOOL_RW_END    = 0x1FFF;

static constexpr uint16_t HMI_INT_RW_START   = 0x2000;
static constexpr uint16_t HMI_INT_RW_END     = 0x2FFF;   //

static constexpr uint16_t HMI_BOOL_RD_START  = 0x3000;
static constexpr uint16_t HMI_BOOL_RD_END    = 0x3FFF;

static constexpr uint16_t HMI_INT_RD_START   = 0x4000;   //
static constexpr uint16_t HMI_INT_RD_END     = 0x4FFF;

class HmiAddressTable {
public:
    HmiAddressTable() = default;

    // ===== 读写 API：RW 区 =====
    bool readBoolRw(uint16_t addr, bool& out) const;
    bool writeBoolRw(uint16_t addr, bool v);

    bool readIntRw(uint16_t addr, uint16_t& out) const;
    bool writeIntRw(uint16_t addr, uint16_t v);

    // ===== 读 API：只读区（给 HMI 读）=====
    bool readBoolRead(uint16_t addr, bool& out) const;
    bool readIntRead(uint16_t addr, uint16_t& out) const;

    // ===== 上层更新：只读区（SystemSnapshot / FaultPage -> table）=====
    void setBoolRead(uint16_t addr, bool v);
    void setIntRead(uint16_t addr, uint16_t v);

    // ===== 映射/范围判断（现在不做映射，只做合法区间判断）=====
    bool mapCoilAddr(uint16_t in, uint16_t& out) const;           // func 01/05/0F
    bool mapDiscreteAddr(uint16_t in, uint16_t& out) const;       // func 02
    bool mapHoldingAddr(uint16_t in, uint16_t& out) const;        // func 03/06/10
    bool mapInputRegAddr(uint16_t in, uint16_t& out) const;       // func 04

private:
    bool inRange(uint16_t addr, uint16_t s, uint16_t e) const { return addr >= s && addr <= e; }

private:
    mutable std::mutex mtx_;

    // 稀疏表：未设置默认 0
    std::unordered_map<uint16_t, bool>      bool_rw_;
    std::unordered_map<uint16_t, uint16_t>  int_rw_;
    std::unordered_map<uint16_t, bool>      bool_rd_;
    std::unordered_map<uint16_t, uint16_t>  int_rd_;
};

#endif //ENERGYSTORAGE_HMI_ADDRESS_TABLE_H