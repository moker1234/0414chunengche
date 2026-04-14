//
// Created by lxy on 2026/1/19.
//

/* 解释整个文件的作用
 * 该文件实现了CRC校验策略的定义，包括构造函数、校验函数和CRC订单枚举类。
 * 构造函数用于初始化CRC校验策略，校验函数用于根据CRC订单校验数据帧的CRC校验值是否正确，CRC订单枚举类定义了CRC校验的字节顺序。
 */
#ifndef ENERGYSTORAGE_CRC_POLICY_H
#define ENERGYSTORAGE_CRC_POLICY_H


#pragma once
#include <vector>
#include <cstdint>

namespace parser {

    enum class CrcOrder {
        LoHi,   // 标准 Modbus
        HiLo    // 你现场 Gas / Smoke
    };

    class CrcPolicy {
    public:
        static bool verify(const std::vector<uint8_t>& frame, CrcOrder order);
    };

} // namespace parser


#endif //ENERGYSTORAGE_CRC_POLICY_H