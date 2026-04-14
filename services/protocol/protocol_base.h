//
// Created by forlinx on 2025/12/17.
//
/*
 * 协议基类定义
 */
/* 解释整个文件的作用
 * 该文件实现了协议基类ProtocolBase的定义，包括构造函数、设备上下文结构体和设备接口函数。
 * 构造函数用于初始化协议名称，设备上下文结构体用于存储协议的上下文信息，设备接口函数用于定义协议的解析和生成命令函数。
 */
#ifndef ENERGYSTORAGE_PROTOCOL_BASE_H
#define ENERGYSTORAGE_PROTOCOL_BASE_H
#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>


/* ===================== Gas 强语义定义 ===================== */
enum class GasType : uint16_t {
    Combustible = 0x0000,
    CO          = 0x0001,
    O2          = 0x0002,
    Temperature = 0x0003,
    Humidity    = 0x0004,
    CO2         = 0x0005,
    Unknown     = 0xFFFF
};
struct GasReading {
    bool     valid{false};

    GasType  type{GasType::Unknown};
    uint16_t type_code{0};

    uint16_t raw{0};           // 40001
    uint16_t status{0};        // 40002
    uint16_t decimal_code{0};  // 40003
    uint16_t unit_code{0};     // 40004

    double   value{0.0};       // applyDecimal 后
};

struct DeviceData {
    std::string device_name;          // 例如 "UPS"
    uint32_t    timestamp{0};          // ms，统一由 Parser 填

    // =========================
    // 数值型（连续量 / 物理量）
    // =========================
    // 电压 / 电流 / 功率 / 温度 / 频率 / 百分比
    std::unordered_map<std::string, double> num;

    // =========================
    // 离散数值 / 枚举 / 整型
    // =========================
    // 模式号 / 剩余时间 / 容量百分比 等
    std::unordered_map<std::string, int32_t> value;

    // =========================
    // 状态位 / bitmask / bool
    // =========================
    // UPS status / fault / warning
    std::unordered_map<std::string, uint32_t> status;

    // =========================
    // 字符串 / 原始字段
    // =========================
    // 原始模式名、调试信息
    std::unordered_map<std::string, std::string> str;

    // ===== Gas Detector 专用 =====
    GasReading gas;
};

enum class ParseResult {
    Ok,
    NeedMoreData,
    CrcError,
    FormatError,
    AddrMismatch,
    Unsupported
};
enum class ProtocolKind {
    Unknown,
    ModbusRtu,
    PrivateBinary,
    Ascii
};


class ProtocolBase {
public:
    virtual ~ProtocolBase() = default;
    virtual uint8_t slaveAddr() const = 0;

    // 解析收到的原始字节
    virtual bool parse(const std::vector<uint8_t>& rx,
                       DeviceData& out) = 0;
    virtual ParseResult parseEx(const std::vector<uint8_t>& rx,
                            DeviceData& out) {
        return parse(rx, out) ? ParseResult::Ok
                              : ParseResult::FormatError;
    }


    // 生成“读取数据”的命令
    virtual std::vector<uint8_t> buildReadCmd() = 0;

    // 生成“控制命令”
    virtual std::vector<uint8_t> buildWriteCmd(uint16_t reg,
                                                uint16_t value) {
        (void)reg; (void)value;
        return {};
    }

    virtual ProtocolKind kind() const {
        return ProtocolKind::Unknown;
    }

    // 处理从站请求,HMI 从站不走 parse 上报（我们在 ProtocolParserThread 里走 handleRs485Slave）
    virtual bool handleSlaveRequest(const std::vector<uint8_t>& frame,
                                std::vector<uint8_t>& resp)
    {
        (void)frame; (void)resp;
        return false;
    }

};

#endif //ENERGYSTORAGE_PROTOCOL_BASE_H