//
// Created by lxy on 2026/1/19.
//
/*
 * CRC 校验策略实现
 */
/* 解释整个文件的作用
 * 该文件实现了CRC校验策略的定义，包括构造函数、校验函数和CRC订单枚举类。
 * 构造函数用于初始化CRC校验策略，校验函数用于根据CRC订单校验数据帧的CRC校验值是否正确，CRC订单枚举类定义了CRC校验的字节顺序。
 */

#include "crc_policy.h"
#include "crc_policy.h"
#include "../protocol/modbus/modbus_crc.h"

using namespace parser;

bool CrcPolicy::verify(const std::vector<uint8_t>& frame, CrcOrder order) {
    if (frame.size() < 4) return false;

    uint16_t recv_crc = 0;
    if (order == CrcOrder::LoHi) {
        recv_crc =
            frame[frame.size() - 2] |
            (frame[frame.size() - 1] << 8);
    } else {
        recv_crc =
            (frame[frame.size() - 2] << 8) |
            frame[frame.size() - 1];
    }

    uint16_t calc_crc =
        modbusCRC::calc(frame.data(),
                         frame.size() - 2);

    return recv_crc == calc_crc;
}

