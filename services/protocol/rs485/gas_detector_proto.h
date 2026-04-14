//
// Created by forlinx on 2025/12/31.
//

#ifndef ENERGYSTORAGE_GAS_MBUS_PROTO_H
#define ENERGYSTORAGE_GAS_MBUS_PROTO_H

#pragma once
#include "../protocol_base.h"
#include "logger.h"

#include <cstdint>
#include <vector>
#include <string>

class GasDetectorProto : public ProtocolBase {
public:
    explicit GasDetectorProto(uint8_t addr);

    // 读 40001~40005（浓度/状态/小数点位/单位/种类）
    std::vector<uint8_t> buildReadCmd() override;

    bool parse(const std::vector<uint8_t>& rx, DeviceData& out) override;

    // 一些可复用的映射（上层也可能想用）
    static std::string statusToString(uint16_t s);
    static std::string unitToString(uint16_t u);
    static std::string gasTypeToString(uint16_t t);

    uint8_t slaveAddr() const override { return addr_; }
private:
    uint8_t addr_;

    // ===== 该设备要求：CRC16 高字节在前 =====
    static std::vector<uint8_t> buildReadHolding_CRC_HiLo(
        uint8_t slave, uint16_t start_addr, uint16_t count);

    static bool verifyCRC_HiLo(const std::vector<uint8_t>& frame);

    static bool parseHoldingRegs03(const std::vector<uint8_t>& rx,
                                   uint8_t expect_slave,
                                   std::vector<uint16_t>& regs_out);

    static double applyDecimal(uint16_t raw, uint16_t decimal_code);
};




#endif //ENERGYSTORAGE_GAS_MBUS_PROTO_H