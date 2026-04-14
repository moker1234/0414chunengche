//
// Created by forlinx on 2025/12/31.
//

#ifndef ENERGYSTORAGE_SMOKE_SENSOR_PROTO_H
#define ENERGYSTORAGE_SMOKE_SENSOR_PROTO_H

#pragma once
#include "../protocol_base.h"
#include "logger.h"
#include <cstdint>
#include <vector>
#include <string>

/*
 * Smoke Sensor (RS485 / Modbus-like)
 *
 * Read Holding Registers (03):
 *  00 00 ALARM   报警状态
 *  00 01 FAULT   故障状态
 *  00 02 LEVEL   报警级别
 *  00 03 SMOKE   烟雾浓度 (%)
 *  00 04 TEMPW   温度
 *
 * Write Single Register (06):
 *  00 0A DEV_ADR     设备地址
 *  00 0B DEV_LEVEL   报警级别
 *  00 0C DEV_RESET  设备复位
 *
 * CRC16: 高字节在前 (Hi, Lo)
 */

class SmokeSensorProto : public ProtocolBase {
public:
    explicit SmokeSensorProto(uint8_t addr);

    // ===== 轮询读取 =====
    std::vector<uint8_t> buildReadCmd() override;
    bool parse(const std::vector<uint8_t>& rx, DeviceData& out) override;

    // ===== 写寄存器（06）=====
    std::vector<uint8_t> buildWriteCmd(uint16_t reg, uint16_t value) override;

    // ===== 文本映射（供上层使用）=====
    static std::string alarmToString(uint16_t v);
    static std::string faultToString(uint16_t v);
    static std::string levelToString(uint16_t v);

    uint8_t slaveAddr() const override { return addr_; }
private:
    uint8_t addr_{1};

    // ===== CRC16 高字节在前 =====
    static std::vector<uint8_t>
    buildReadHolding_CRC_HiLo(uint8_t slave,
                              uint16_t start,
                              uint16_t count);

    static std::vector<uint8_t>
    buildWriteSingle_CRC_HiLo(uint8_t slave,
                              uint16_t reg,
                              uint16_t value);

    static bool verifyCRC_HiLo(const std::vector<uint8_t>& frame);

    // ===== 03 响应解析 =====
    static bool parseHoldingRegs03(const std::vector<uint8_t>& rx,
                                   uint8_t expect_slave,
                                   std::vector<uint16_t>& regs_out);

    // ===== 数值处理 =====
    static int16_t applyTemperature(uint16_t raw);
};


#endif // ENERGYSTORAGE_SMOKE_SENSOR_PROTO_H


