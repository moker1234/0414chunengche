//
// Created by forlinx on 2025/12/31.
//
/* 解释整个文件的作用
 * 该文件实现了Modbus RTU协议的构建函数和解析函数。
 * 包括构建读取保持寄存器命令和解析保持寄存器响应的函数。
 */
#include "modbus_rtu.h"

#include "modbus_rtu.h"
#include "modbus_crc.h"

std::vector<uint8_t> ModbusRTU::buildReadHolding(
    uint8_t slave, uint16_t start, uint16_t count) {

    std::vector<uint8_t> frame{
        slave,
        0x03,
        uint8_t(start >> 8),
        uint8_t(start & 0xFF),
        uint8_t(count >> 8),
        uint8_t(count & 0xFF)
    };

    uint16_t crc = modbusCRC::calc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    return frame;
}

bool ModbusRTU::parseHolding(
    const std::vector<uint8_t>& rx, std::vector<uint16_t>& regs) {

    if (rx.size() < 4) return false;

    uint8_t slave = rx[0];
    uint8_t func = rx[1];
    if (slave != 0x01 || func != 0x03) return false;

    uint8_t byte_count = rx[2];
    if (byte_count != regs.size() * 2) return false;

    for (uint8_t i = 0; i < byte_count; ++i) {
        regs[i / 2] =
            rx[3 + i] |
            (rx[3 + i + 1] << 8);
    }

    return true;
}
