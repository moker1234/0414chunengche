//
// Created by forlinx on 2025/12/31.
//

#ifndef ENERGYSTORAGE_MODBUS_CRC_H
#define ENERGYSTORAGE_MODBUS_CRC_H
#pragma once
#include <cstdint>
#include <vector>

class modbusCRC {
public:
    // 对连续内存计算 CRC
    static uint16_t calc(const uint8_t* data, uint16_t len);

    // 对 vector 计算 CRC（最常用）
    static uint16_t calc(const std::vector<uint8_t>& data);

    // 校验一帧 Modbus RTU 数据（最后 2 字节为 CRC）
    static bool verify(const std::vector<uint8_t>& frame);
};



#endif //ENERGYSTORAGE_MODBUS_CRC_H