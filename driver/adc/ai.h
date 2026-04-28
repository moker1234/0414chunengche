//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_AI_H
#define ENERGYSTORAGE_AI_H

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct AiChannelData {
    int channel_id{0};      // 1..4

    int64_t voltage_raw{0}; // raw file value
    int64_t current_raw{0}; // raw file value

    bool voltage_ok{false};
    bool current_ok{false};

    double voltage_v{0.0};  // V = voltage_raw / 1e6
    double current_a{0.0};  // A = current_raw / 1e6
    double power_w{0.0};    // W = V * A
};

class AiDriver {
public:
    explicit AiDriver(std::string dev_path = "/sys/bus/iio/devices/iio:device2");
    ~AiDriver() = default;

    const std::string& devPath() const { return dev_path_; }

    // 对应 ai_test.cpp 里的 current_channel_enable
    bool setCurrentChannelEnable(const std::string& mask) const;

    // 读取 4 路全量数据（电压/电流/功率）
    bool readAll(std::array<AiChannelData, 4>& out) const;

    // 业务层重点接口：读取单路电压（1~4）
    bool readVoltage(int channel_id, double& out_v) const;

    // 业务层重点接口：读取全部电压
    // out[0]=AI1_V, out[1]=AI2_V, out[2]=AI3_V, out[3]=AI4_V
    bool readAllVoltages(std::vector<double>& out) const;

    // 兼容之前给你的 logic 向量接口：
    // [AI1_V, AI1_A, AI1_W, AI2_V, AI2_A, AI2_W, ... AI4_W]
    bool readLogicVector(std::vector<double>& out) const;

private:
    static bool readInt64_(const std::string& path, int64_t& out);
    static bool writeText_(const std::string& path, const std::string& s);

private:
    std::string dev_path_;
};

#endif // ENERGYSTORAGE_AI_H