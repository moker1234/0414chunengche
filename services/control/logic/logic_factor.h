//
// Created by lxy on 2026/4/25.
//

#ifndef ENERGYSTORAGE_LOGIC_FACTOR_H
#define ENERGYSTORAGE_LOGIC_FACTOR_H


#pragma once

#include <cstdint>

// Gas 给 HMI 的显示缩放系数
// 说明：
// 1. GasDetectorProto 已经根据 40003 小数点位把 raw 转成物理值 double。
// 2. 这里的 factor 不是协议解析 factor，而是 HMI 寄存器输出 factor。
// 3. 例如 gas_o2 = 20.9，factor=10，则 logic_view 输出 209，HMI 按 1 位小数显示。
inline struct Gas_Factor {
    uint16_t combustible = 1;   // 可燃气体，建议保留 1 位小数
    uint16_t co          = 1;   // CO，暂按 1 位小数
    uint16_t o2          = 1;   // O2，通常需要 1 位小数
    uint16_t temperature = 1;   // 温度，1 位小数
    uint16_t humidity    = 1;   // 湿度，1 位小数
    uint16_t co2         = 1;   // CO2，暂按 1 位小数
} gas_factor;

/*
 * Aircon -> HMI 显示缩放因子
 *
 * 原则：
 * 1. 协议层、Aggregator、SQLite 仍保留工程值 double。
 * 2. logic_view 中面向 HMI 的字段转成整数。
 * 3. HMI 屏幕根据对应 factor 设置小数位显示。
 *
 * 例如：
 *   ac_indoor_temp = 287  表示 28.7 ℃
 *   ac_current     = 12   表示 1.2 A
 *   ac_dc_voltage  = 485  表示 48.5 V
 */
inline struct AirconFactor {
    uint8_t temp_c      = 10;  // ℃ -> x10
    uint8_t humidity    = 1;   // %RH -> x1
    uint8_t current_a   = 10;  // A -> x10
    uint8_t ac_voltage  = 1;   // V -> x1
    uint8_t dc_voltage  = 10;  // V -> x10
} aircon_factor;

#endif //ENERGYSTORAGE_LOGIC_FACTOR_H
