//
// Created by forlinx on 2025/12/31.
//
/*
 * 气体检测器协议实现
 */
/* 解释整个文件的作用
 * 该文件实现了气体检测器协议的构建函数和解析函数。
 * 包括构建读取命令和解析保持寄存器响应的函数。
 */
#include "gas_detector_proto.h"
#include "../modbus/modbus_crc.h"

#include <cmath>

#include "hex_dump.h"

// 寄存器：40001~40005
// Modbus Holding Register 通常 PDU 地址从 0 开始（即 40001 -> 0x0000）
// 若你现场发现读不到，需要把 start 从 0 改成 1（有些厂家把 40001 当 0x0001）
static constexpr uint16_t REG_START_40001 = 0x0000;
static constexpr uint16_t REG_COUNT       = 5;

GasDetectorProto::GasDetectorProto(uint8_t addr) : addr_(addr) {
    LOGDEBUG("[GAS] GasDetectorProto ctor addr=%u this=%p", addr_, this);}

std::vector<uint8_t> GasDetectorProto::buildReadCmd() {
    // 功能码 0x03：读保持寄存器
    return buildReadHolding_CRC_HiLo(addr_, REG_START_40001, REG_COUNT);
}

/*
 * 解析响应帧
 * @param rx 接收数据
 * @param out 解析结果
 * @return 是否解析成功
 */
bool GasDetectorProto::parse(const std::vector<uint8_t>& rx, DeviceData& out) {
    // LOG_COMM_HEX("RX dev=GasDetector ...", rx.data(), rx.size());

    std::vector<uint16_t> regs;
    if (!parseHoldingRegs03(rx, addr_, regs)) return false; // 校验寄存器数
    if (regs.size() < 5) return false;
    LOGTRACE("[GAS][PARSE] size=%zu", rx.size());
    LOGTRACE("[GAS][PARSE] data=");
    for (auto b : rx) LOGTRACE("%02X ", b);

    const uint16_t gas_raw      = regs[0]; // 40001
    const uint16_t status       = regs[1]; // 40002
    const uint16_t decimal_code = regs[2]; // 40003
    const uint16_t unit_code    = regs[3]; // 40004
    const uint16_t type_code    = regs[4]; // 40005

    const double gas_value = applyDecimal(gas_raw, decimal_code);

    out.device_name = "GasDetector";

    /* ===== 强语义 Gas 数据 ===== */
    out.gas.valid        = true;
    out.gas.type_code   = type_code;
    out.gas.raw         = gas_raw;
    out.gas.status      = status;
    out.gas.decimal_code= decimal_code;
    out.gas.unit_code   = unit_code;
    out.gas.value       = gas_value;

    switch (type_code) {
        case 0x0000: out.gas.type = GasType::Combustible; break;
        case 0x0001: out.gas.type = GasType::CO; break;
        case 0x0002: out.gas.type = GasType::O2; break;
        case 0x0003: out.gas.type = GasType::Temperature; break;
        case 0x0004: out.gas.type = GasType::Humidity; break;
        case 0x0005: out.gas.type = GasType::CO2; break;
        default:     out.gas.type = GasType::Unknown; break;
    }

    out.num["gas_raw"]   = static_cast<double>(gas_raw);
    out.num["gas_value"] = gas_value;
    out.num["status"]    = static_cast<double>(status);
    out.num["decimal"]   = static_cast<double>(decimal_code);
    out.num["unit_code"] = static_cast<double>(unit_code);
    out.num["type_code"] = static_cast<double>(type_code);

    out.str["status_text"] = statusToString(status);
    out.str["unit"]        = unitToString(unit_code);
    out.str["gas_type"]    = gasTypeToString(type_code);

    return true;
}

/* ====================== 设备特化：CRC 高字节在前 ====================== */

std::vector<uint8_t> GasDetectorProto::buildReadHolding_CRC_HiLo(
    uint8_t slave, uint16_t start_addr, uint16_t count) {

    // [slave][03][start_hi][start_lo][count_hi][count_lo][crc_hi][crc_lo]
    std::vector<uint8_t> frame{
        slave,
        0x03,
        static_cast<uint8_t>(start_addr >> 8),
        static_cast<uint8_t>(start_addr & 0xFF),
        static_cast<uint8_t>(count >> 8),
        static_cast<uint8_t>(count & 0xFF),
    };

    const uint16_t crc = modbusCRC::calc(frame);
    // 文档要求：高字节在前，低字节在后
    frame.push_back(static_cast<uint8_t>(crc >> 8));      // CRC_H
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));    // CRC_L
    return frame;
}

bool GasDetectorProto::verifyCRC_HiLo(const std::vector<uint8_t>& frame) {
    if (frame.size() < 4) return false;

    const uint16_t recv_crc =
        (static_cast<uint16_t>(frame[frame.size() - 2]) << 8) |  // CRC_H
        (static_cast<uint16_t>(frame[frame.size() - 1]));        // CRC_L

    const uint16_t calc_crc = modbusCRC::calc(frame.data(),
                                              static_cast<uint16_t>(frame.size() - 2));


    LOGTRACE("[GAS][CRC] recv=%04X calc=%04X", recv_crc, calc_crc);

    return recv_crc == calc_crc;
}

/* ====================== 03 响应解析 ====================== */
/*
 * 解析 0x03 响应帧
 * @param rx 接收数据
 * @param expect_slave 期望从机地址
 * @param regs_out 解析结果（寄存器值）
 * @return 是否解析成功
 */
bool GasDetectorProto::parseHoldingRegs03(const std::vector<uint8_t>& rx,
                                          uint8_t expect_slave,
                                          std::vector<uint16_t>& regs_out) {
    // 标准 03 响应：
    // [slave][03][byte_count][data...][crc..]
    if (rx.size() < 5) return false;
    if (rx[0] != expect_slave) return false;
    if (rx[1] != 0x03) return false;

    if (!verifyCRC_HiLo(rx)) return false;

    const uint8_t byte_count = rx[2];
    const size_t expect_len = 3u + byte_count + 2u; // hdr(3)+data+crc(2)
    if (rx.size() != expect_len) return false;
    if (byte_count % 2 != 0) return false;

    const size_t reg_count = byte_count / 2;
    regs_out.clear();
    regs_out.reserve(reg_count);

    // Modbus 寄存器数据：每个寄存器 2 字节，大端（hi, lo）
    size_t idx = 3;
    for (size_t i = 0; i < reg_count; ++i) {
        uint16_t v = (static_cast<uint16_t>(rx[idx]) << 8) |
                     (static_cast<uint16_t>(rx[idx + 1]));
        regs_out.push_back(v);
        idx += 2;
    }

    return true;
}

/* ====================== 值处理与映射 ====================== */

double GasDetectorProto::applyDecimal(uint16_t raw, uint16_t decimal_code) {
    // 0: 无小数点
    // 1: 0.0
    // 2: 0.00
    switch (decimal_code) {
        case 0x0000: return static_cast<double>(raw);
        case 0x0001: return static_cast<double>(raw) / 10.0;
        case 0x0002: return static_cast<double>(raw) / 100.0;
        default:     return static_cast<double>(raw); // 未知按整数
    }
}

std::string GasDetectorProto::statusToString(uint16_t s) {
    switch (s) {
        case 0x0000: return "工作正常";
        case 0x0001: return "故障";
        case 0x0002: return "低报";
        case 0x0004: return "高报";
        default:     return "未知状态";
    }
}

std::string GasDetectorProto::unitToString(uint16_t u) {
    switch (u) {
        case 0x0000: return "%LEL";
        case 0x0001: return "ppm";
        case 0x0002: return "%VOL";
        case 0x0003: return "℃";
        case 0x0004: return "%RH";
        default:     return "未知单位";
    }
}

std::string GasDetectorProto::gasTypeToString(uint16_t t) {
    switch (t) {
        case 0x0000: return "可燃气体";
        case 0x0001: return "一氧化碳";
        case 0x0002: return "氧气";
        case 0x0003: return "温度";
        case 0x0004: return "湿度";
        case 0x0005: return "二氧化碳";
        default:     return "未知气体";
    }
}
