//
// Created by forlinx on 2025/12/31.
//

#ifndef ENERGYSTORAGE_MODBUS_RTU_H
#define ENERGYSTORAGE_MODBUS_RTU_H


#pragma once
#include <vector>
#include <cstdint>

class ModbusRTU {
public:
    static std::vector<uint8_t> buildReadHolding( // 构建读取保持寄存器请求
        uint8_t slave, // 从站地址
        uint16_t start,
        uint16_t count
    );

    static bool parseHolding(  // 解析读取保持寄存器响应
        const std::vector<uint8_t>& rx, // 收到的原始字节
        std::vector<uint16_t>& regs // 解析出的寄存器值
    );
};


#endif //ENERGYSTORAGE_MODBUS_RTU_H