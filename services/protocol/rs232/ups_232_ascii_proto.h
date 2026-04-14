// Created by lxy on 2026/1/20.

#ifndef ENERGYSTORAGE_UPS_232_ASCII_PROTO_H
#define ENERGYSTORAGE_UPS_232_ASCII_PROTO_H

#pragma once
#include "../protocol_base.h"
#include <string>
#include <vector>

class Ups232AsciiProto : public ProtocolBase {
public:
    Ups232AsciiProto();

    uint8_t slaveAddr() const override { return 0; } // RS232 无地址，返回 0 即可

    // 轮询：Q1/Q6/WA 循环
    std::vector<uint8_t> buildReadCmd() override;

    // 解析：输入为一帧 ASCII（不含 CR）
    bool parse(const std::vector<uint8_t>& rx, DeviceData& out) override;

    // 关机：reg/value 在此协议无意义，建议用 value 的低 16 位编码，或直接不用此接口
    std::vector<uint8_t> buildWriteCmd(uint16_t reg, uint16_t value) override;

    // 推荐提供明确接口：S<n>
    static std::vector<uint8_t> buildShutdownCmd(const std::string& n);

private:
    // 轮询命令
    enum class PollKind : int32_t { Q1 = 1, Q6 = 2, WA = 3 };
    PollKind next_{PollKind::Q1};

private:
    static std::string toString(const std::vector<uint8_t>& rx);
    static std::vector<std::string> splitTokens(const std::string& s);

    // 把 "(...." 去掉括号
    static std::string stripWrapper(const std::string& s);

    static bool parseQ1(const std::vector<std::string>& t, DeviceData& out);
    static bool parseQ6(const std::vector<std::string>& t, DeviceData& out);
    static bool parseWA(const std::vector<std::string>& t, DeviceData& out);

    static bool parseDoubleToken(const std::string& tok, double& out);
    static bool parseIntToken(const std::string& tok, int& out);

    static void fillBits8(const std::string& bits01, DeviceData& out, const std::string& prefix);

    // 给上层分组用的内部标记（后续 toJson() 里可以过滤 __ 前缀）
    static void markCmd_(PollKind kind, DeviceData& out);

    // 仅日志/调试
    static const char* cmdToString_(PollKind k);
};

#endif // ENERGYSTORAGE_UPS_232_ASCII_PROTO_H
