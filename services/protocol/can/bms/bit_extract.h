//
// Created by lxy on 2026/2/13.
//

#ifndef ENERGYSTORAGE_BIT_EXTRACT_H
#define ENERGYSTORAGE_BIT_EXTRACT_H
// services/protocol/can/bms/bit_extract.h
#pragma once
#include <cstdint>
#include <cstring>

namespace bms {

    static inline uint64_t extract_intel_lsb(const uint8_t data[8], int start_lsb, int len) {
        // Intel: startbit_lsb 从 0..63，bit0 是 data[0] 的 LSB
        uint64_t u = 0;
        std::memcpy(&u, data, 8);               // 小端装载
        if (len == 64) return u >> start_lsb;
        return (u >> start_lsb) & ((1ULL << len) - 1ULL);
    }

    // Motorola 的位编号在行业里有多种约定；你表里是 Startbit(LSB) 但 byte_order=Motorola。
    // 最稳的做法：你在生成 JSON 时就把 Motorola startbit 统一成“LSB编号”。如果你已经统一了，
    // 那这里就可直接用 intel 同样的 extract。（否则需要 DBC big-endian 规则转换）
    static inline uint64_t extract_motorola_lsb_compatible(const uint8_t data[8], int start_lsb, int len) {
        // 假设 startbit_lsb 已被预处理为 LSB 编号（推荐）
        return extract_intel_lsb(data, start_lsb, len);
    }

    static inline uint64_t extract_bits(const uint8_t data[8], int start_lsb, int len, int byte_order) {
        return (byte_order == 0) ? extract_intel_lsb(data, start_lsb, len)
                                 : extract_motorola_lsb_compatible(data, start_lsb, len);
    }

} // namespace bms

#endif //ENERGYSTORAGE_BIT_EXTRACT_H