//
// Created by lxy on 2026/3/4.
//

#ifndef ENERGYSTORAGE_NORMAL_ADDR_LAYOUT_H
#define ENERGYSTORAGE_NORMAL_ADDR_LAYOUT_H

// services/normal/normal_addr_layout.h
#pragma once
#include <cstdint>

namespace normal {

    // 渐进式迁移：普通变量旁路输出地址段（Input Registers / FC04）
    static constexpr uint16_t NORMAL_RD_BASE = 0x3100;

    // 先输出 10 个寄存器（你后续可扩展到更多）
    static constexpr uint16_t NORMAL_RD_COUNT = 10;

    // 定义每个槽位含义（先给一套可观测的系统级变量）
    enum Slot : uint16_t {
        SYS_TEMP = 0,        // system_temperature (int)
        GAS_PPM  = 1,        // gas_ppm (double->u16)
        SMOKE_PCT = 2,       // smoke_percent (double->u16)
        AC_IN_TEMP = 3,      // ac_indoor_temp (double->u16)
        AC_HUM_PCT = 4,      // ac_humidity (double->u16)
        BMS_SOC_X10 = 5,     // bms_soc *10
        BMS_PACK_V_X10 = 6,  // bms_pack_v *10
        BMS_PACK_I_X10 = 7,  // bms_pack_i *10 (带符号可改 s16，这里先做 clamp)
        BMS_FAULT_NUM = 8,   // bms_fault_num
        ALARM_BITS = 9       // bit0=gas_alarm bit1=smoke_alarm bit2=ac_alarm bit3=bms_alarm_any
    };

} // namespace normal

#endif //ENERGYSTORAGE_NORMAL_ADDR_LAYOUT_H