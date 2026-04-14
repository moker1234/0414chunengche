// services/protocol/rs485/hmi/hmi_address_table.cpp
//
// Created by lxy on 2026/2/4.
//
#include "hmi_address_table.h"

// ===== 现在不进行任何地址映射，只做区间合法性判断 =====

bool HmiAddressTable::mapCoilAddr(uint16_t in, uint16_t& out) const {
    if (inRange(in, HMI_BOOL_RW_START, HMI_BOOL_RW_END)) { out = in; return true; }
    return false;
}

bool HmiAddressTable::mapHoldingAddr(uint16_t in, uint16_t& out) const {
    if (inRange(in, HMI_INT_RW_START, HMI_INT_RW_END)) { out = in; return true; }
    return false;
}

bool HmiAddressTable::mapDiscreteAddr(uint16_t in, uint16_t& out) const {
    if (inRange(in, HMI_BOOL_RD_START, HMI_BOOL_RD_END)) { out = in; return true; }
    return false;
}

bool HmiAddressTable::mapInputRegAddr(uint16_t in, uint16_t& out) const {
    if (inRange(in, HMI_INT_RD_START, HMI_INT_RD_END)) { out = in; return true; }
    return false;
}

bool HmiAddressTable::readBoolRw(uint16_t addr, bool& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = bool_rw_.find(addr);
    if (it == bool_rw_.end()) { out = false; return true; }
    out = it->second;
    return true;
}

bool HmiAddressTable::writeBoolRw(uint16_t addr, bool v) {
    if (!inRange(addr, HMI_BOOL_RW_START, HMI_BOOL_RW_END)) return false;
    std::lock_guard<std::mutex> lk(mtx_);
    bool_rw_[addr] = v;
    return true;
}

bool HmiAddressTable::readIntRw(uint16_t addr, uint16_t& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = int_rw_.find(addr);
    if (it == int_rw_.end()) { out = 0; return true; }
    out = it->second;
    return true;
}

bool HmiAddressTable::writeIntRw(uint16_t addr, uint16_t v) {
    if (!inRange(addr, HMI_INT_RW_START, HMI_INT_RW_END)) return false;
    std::lock_guard<std::mutex> lk(mtx_);
    int_rw_[addr] = v;
    return true;
}

bool HmiAddressTable::readBoolRead(uint16_t addr, bool& out) const {

    std::lock_guard<std::mutex> lk(mtx_);
    auto it = bool_rd_.find(addr);
    if (it == bool_rd_.end())
    {
        out = false; return true;
    }
    out = it->second;
    return true;
}

bool HmiAddressTable::readIntRead(uint16_t addr, uint16_t& out) const {
    if (addr >=0x402C && addr <= 0x4041)
    {
        // printf("read reg 0x%04X\n", addr);
    }
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = int_rd_.find(addr);
    if (it == int_rd_.end())
    {
        out = 0;
        // if (addr == 0x4049 || addr == 0x4064 || addr == 0x407F || addr == 0x409A) {
        //     printf("[HMI][READ_INT_RD] addr=0x%04X MISS -> 0\n", addr);
        // }
        return true;
    }
    out = it->second;
    if (addr == 0x4010 || addr == 0x4011){// || addr == 0x407F || addr == 0x409A) {
        printf("[HMI][READ_INT_RD] addr=0x%04X out=%u\n", addr, out);
    }
    return true;
}

void HmiAddressTable::setBoolRead(uint16_t addr, bool v) {
    if (!inRange(addr, HMI_BOOL_RD_START, HMI_BOOL_RD_END)) return;
    std::lock_guard<std::mutex> lk(mtx_);
    bool_rd_[addr] = v;
}

void HmiAddressTable::setIntRead(uint16_t addr, uint16_t v) {
    // if (addr == 0x4049 || addr == 0x4064 || addr == 0x407F || addr == 0x409A) {
    //     printf("[HMI][SET_INT_RD] addr=0x%04X v=%u\n", addr, v);
    // }
    if (!inRange(addr, HMI_INT_RD_START, HMI_INT_RD_END)) return;
    std::lock_guard<std::mutex> lk(mtx_);
    int_rd_[addr] = v;
}