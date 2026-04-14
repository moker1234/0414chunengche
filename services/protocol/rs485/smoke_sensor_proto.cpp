//
// Created by forlinx on 2025/12/31.
//
/*
 * 烟雾传感器协议实现
 */
/* 解释整个文件的作用
 * 该文件实现了烟雾传感器协议的构建函数和解析函数。
 * 包括构建读取命令和解析保持寄存器响应的函数。
 */
#include "smoke_sensor_proto.h"
#include "../modbus/modbus_crc.h"
#include <cstdio>

#include "hex_dump.h"

// 读寄存器：00 00 ~ 00 04
static constexpr uint16_t REG_START = 0x0000;
static constexpr uint16_t REG_COUNT = 5;

// 写寄存器
static constexpr uint16_t REG_DEV_ADDR  = 0x000A;
static constexpr uint16_t REG_DEV_LEVEL = 0x000B;
static constexpr uint16_t REG_DEV_RESET = 0x000C;

SmokeSensorProto::SmokeSensorProto(uint8_t addr)
    : addr_(addr) {
    printf("[SMOKE] SmokeSensorProto ctor addr=%u this=%p\n",
           addr_, this);
}

/* ======================= 读命令 ======================= */

std::vector<uint8_t> SmokeSensorProto::buildReadCmd() {
    return buildReadHolding_CRC_HiLo(addr_, REG_START, REG_COUNT);
}

/* ======================= 写命令 ======================= */

std::vector<uint8_t>
SmokeSensorProto::buildWriteCmd(uint16_t reg, uint16_t value) {
    return buildWriteSingle_CRC_HiLo(addr_, reg, value);
}

/* ======================= CRC 高字节在前 ======================= */

std::vector<uint8_t>
SmokeSensorProto::buildReadHolding_CRC_HiLo(
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
    frame.push_back(uint8_t(crc >> 8)); // CRC_H
    frame.push_back(uint8_t(crc & 0xFF)); // CRC_L
    return frame;
}

std::vector<uint8_t>
SmokeSensorProto::buildWriteSingle_CRC_HiLo(
    uint8_t slave, uint16_t reg, uint16_t value) {

    std::vector<uint8_t> frame{
        slave,
        0x06,
        uint8_t(reg >> 8),
        uint8_t(reg & 0xFF),
        uint8_t(value >> 8),
        uint8_t(value & 0xFF)
    };

    uint16_t crc = modbusCRC::calc(frame);
    frame.push_back(uint8_t(crc >> 8)); // CRC_H
    frame.push_back(uint8_t(crc & 0xFF)); // CRC_L
    return frame;
}

bool SmokeSensorProto::verifyCRC_HiLo(const std::vector<uint8_t>& frame) {
    if (frame.size() < 4) return false;

    uint16_t recv_crc =
        (uint16_t(frame[frame.size() - 2]) << 8) |
        (uint16_t(frame[frame.size() - 1]));

    uint16_t calc_crc =
        modbusCRC::calc(frame.data(),
                        frame.size() - 2);

    return recv_crc == calc_crc;
}

/* ======================= 03 响应解析 ======================= */

bool SmokeSensorProto::parseHoldingRegs03(
    const std::vector<uint8_t>& rx,
    uint8_t expect_slave,
    std::vector<uint16_t>& regs_out) {

    // [slave][03][byte_cnt][data...][crc_h][crc_l]
    if (rx.size() < 5) return false;
    if (rx[0] != expect_slave) return false;
    if (rx[1] != 0x03) return false;
    if (!verifyCRC_HiLo(rx)) return false;

    uint8_t byte_cnt = rx[2];
    if (byte_cnt % 2 != 0) return false;

    size_t expect_len = 3 + byte_cnt + 2;
    if (rx.size() != expect_len) return false;

    size_t reg_cnt = byte_cnt / 2;
    regs_out.clear();
    regs_out.reserve(reg_cnt);

    size_t idx = 3;
    for (size_t i = 0; i < reg_cnt; ++i) {
        uint16_t v =
            (uint16_t(rx[idx]) << 8) |
            (uint16_t(rx[idx + 1]));
        regs_out.push_back(v);
        idx += 2;
    }
    return true;
}

/* ======================= 解析入口 ======================= */

bool SmokeSensorProto::parse(
    const std::vector<uint8_t>& rx,
    DeviceData& out) {

    // LOG_COMM_HEX("RX dev=SmokeSensor ...", rx.data(), rx.size());


    std::vector<uint16_t> regs;
    if (!parseHoldingRegs03(rx, addr_, regs)) return false;
    if (regs.size() < 5) return false;

    uint16_t alarm = regs[0];
    uint16_t fault = regs[1];
    uint16_t warn_level = regs[2];
    uint16_t smoke = regs[3];
    uint16_t tempw = regs[4];

    int16_t temp_c = applyTemperature(tempw);

    out.device_name = "SmokeSensor";

    // ===== 数值 =====
    out.num["alarm"]         = double(alarm);
    out.num["fault"]         = double(fault);
    out.num["warn_level"]    = double(warn_level);
    out.num["smoke_percent"] = double(smoke);   // 0~255 (%)
    out.num["temp"]   = double(temp_c);

    // ===== 文本 =====
    out.str["alarm_text"]      = alarmToString(alarm);
    out.str["fault_text"]      = faultToString(fault);
    out.str["warn_level_text"] = levelToString(warn_level);

    return true;
}

/* ======================= 数值处理 ======================= */

int16_t SmokeSensorProto::applyTemperature(uint16_t raw) {
    uint8_t hi = uint8_t(raw >> 8);
    uint8_t lo = uint8_t(raw & 0xFF);

    if (hi == 0x01) { // 负温度
        return -int16_t(lo);
    }
    return int16_t(lo);
}

/* ======================= 文本映射 ======================= */

std::string SmokeSensorProto::alarmToString(uint16_t v) {
    switch (v & 0x00FF) {
        case 0x00: return "正常";
        case 0x01: return "烟雾报警";
        default:   return "未知报警";
    }
}

std::string SmokeSensorProto::faultToString(uint16_t v) {
    switch (v & 0x00FF) {
        case 0x00: return "无故障";
        case 0x01: return "烟雾传感器故障";
        case 0x02: return "检测室污染";
        case 0x04: return "温度传感器故障";
        default:   return "未知故障";
    }
}

std::string SmokeSensorProto::levelToString(uint16_t v) {
    switch (v & 0x00FF) {
        case 0x01: return "1级(1~2%OBS)";
        case 0x02: return "2级(2~4%OBS)";
        case 0x03: return "3级(4~6%OBS)";
        case 0x04: return "4级(6~8%OBS)";
        default:   return "未知级别";
    }
}
