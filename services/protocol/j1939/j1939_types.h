//
// Created by lxy on 2026/2/8.
//

#ifndef ENERGYSTORAGE_J1939_TYPES_H
#define ENERGYSTORAGE_J1939_TYPES_H

// services/protocol/j1939/j1939_types.h
#pragma once

#include <cstdint>
#include <string>

namespace j1939 {

/*
 * J1939 29bit 标识符拆装
 *
 * 29-bit ID (per SAE J1939-21):
 *  Priority (3) | Reserved(1) | DP(1) | PF(8) | PS(8) | SA(8)
 *
 * PGN:
 *  - if PF < 240:  PDU1, PGN = DP:PF:00  (destination in PS)
 *  - if PF >=240:  PDU2, PGN = DP:PF:PS
 */

struct J1939Id {
    uint8_t priority{6};
    uint8_t dp{0};
    uint8_t pf{0};
    uint8_t ps{0};
    uint8_t sa{0};

    bool isPdu1() const { return pf < 240; }   // destination specific
    bool isPdu2() const { return pf >= 240; }  // broadcast / group extension

    /** 计算 PGN（18-bit） */
    uint32_t pgn() const {
        if (isPdu1()) {
            return ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | 0x00;
        }
        return ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | (uint32_t)ps;
    }

    /** 目的地址（仅 PDU1 有意义；PDU2 常为 0xFF） */
    uint8_t dst() const { return isPdu1() ? ps : 0xFF; }
};

/** 从 29-bit 标识符解析（不含 CAN_EFF_FLAG） */
inline J1939Id parseId29(uint32_t id29) {
    J1939Id x{};
    x.priority = (uint8_t)((id29 >> 26) & 0x7);
    x.dp       = (uint8_t)((id29 >> 24) & 0x1);
    x.pf       = (uint8_t)((id29 >> 16) & 0xFF);
    x.ps       = (uint8_t)((id29 >> 8)  & 0xFF);
    x.sa       = (uint8_t)( id29        & 0xFF);
    return x;
}

/** 组装 29-bit 标识符（不含 CAN_EFF_FLAG） */
inline uint32_t buildId29(uint8_t priority, uint8_t dp, uint8_t pf, uint8_t ps, uint8_t sa) {
    uint32_t id29 = 0;
    id29 |= ((uint32_t)(priority & 0x7) << 26);
    id29 |= ((uint32_t)(0)             << 25); // Reserved bit: 0
    id29 |= ((uint32_t)(dp & 0x1)      << 24);
    id29 |= ((uint32_t)pf              << 16);
    id29 |= ((uint32_t)ps              << 8);
    id29 |= ((uint32_t)sa);
    return id29;
}

/**
 * 从 PGN + SA/DST 推导 PF/PS 并组 29-bit ID：
 * - PDU1: PF < 240, PS=dst, PGN 的低 8bit 为 0
 * - PDU2: PF >=240, PS = PGN 低 8bit（组扩展），dst 常为 0xFF
 */
inline uint32_t buildId29FromPgn(uint32_t pgn, uint8_t priority, uint8_t sa, uint8_t dst) {
    uint8_t dp = (uint8_t)((pgn >> 16) & 0x1);
    uint8_t pf = (uint8_t)((pgn >> 8) & 0xFF);
    uint8_t ps = 0;
    if (pf < 240) {
        ps = dst;
    } else {
        ps = (uint8_t)(pgn & 0xFF);
    }
    return buildId29(priority, dp, pf, ps, sa);
}

struct J1939Frame {
    int can_index{0};
    uint32_t id29{0};      // 29-bit ID (no CAN_EFF_FLAG)
    uint8_t dlc{0};
    uint8_t data[8]{};

    J1939Id jid() const { return parseId29(id29); }
};

} // namespace j1939


#endif //ENERGYSTORAGE_J1939_TYPES_H