//
// Created by lxy on 2026/4/22.
//

#include "ai.h"

#include <fstream>
#include <string>

namespace {

static std::string trim(const std::string& s)
{
    std::size_t b = 0;
    std::size_t e = s.size();

    while (b < e && (s[b] == ' ' || s[b] == '\n' || s[b] == '\r' || s[b] == '\t')) ++b;
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\n' || s[e - 1] == '\r' || s[e - 1] == '\t')) --e;

    return s.substr(b, e - b);
}

} // namespace

AiDriver::AiDriver(std::string dev_path)
    : dev_path_(std::move(dev_path))
{
}

bool AiDriver::readInt64_(const std::string& path, int64_t& out)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string line;
    std::getline(ifs, line);
    line = trim(line);
    if (line.empty()) return false;

    try {
        std::size_t idx = 0;
        long long v = std::stoll(line, &idx, 10);
        if (idx != line.size()) return false;
        out = static_cast<int64_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool AiDriver::writeText_(const std::string& path, const std::string& s)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << s;
    return ofs.good();
}

bool AiDriver::setCurrentChannelEnable(const std::string& mask) const
{
    return writeText_(dev_path_ + "/current_channel_enable", mask);
}

bool AiDriver::readAll(std::array<AiChannelData, 4>& out) const
{
    bool all_ok = true;

    for (int ch = 1; ch <= 4; ++ch) {
        auto& d = out[ch - 1];
        d = AiChannelData{};
        d.channel_id = ch;

        const std::string vpath = dev_path_ + "/in_voltage" + std::to_string(ch) + "_raw";
        const std::string ipath = dev_path_ + "/in_current" + std::to_string(ch) + "_raw";

        d.voltage_ok = readInt64_(vpath, d.voltage_raw);
        d.current_ok = readInt64_(ipath, d.current_raw);

        if (!d.voltage_ok || !d.current_ok) {
            all_ok = false;
        }

        // 参考 ai_test.cpp：
        // voltage_raw = mV * 1000  => V = voltage_raw / 1e6
        // current_raw = mA * 1000  => A = current_raw / 1e6
        if (d.voltage_ok) {
            d.voltage_v = static_cast<double>(d.voltage_raw) / 1e6;
        }
        if (d.current_ok) {
            d.current_a = static_cast<double>(d.current_raw) / 1e6;
        }
        if (d.voltage_ok && d.current_ok) {
            d.power_w = d.voltage_v * d.current_a;
        }
    }

    return all_ok;
}

bool AiDriver::readVoltage(int channel_id, double& out_v) const
{
    out_v = 0.0;

    if (channel_id < 1 || channel_id > 4) {
        return false;
    }

    int64_t raw = 0;
    const std::string vpath = dev_path_ + "/in_voltage" + std::to_string(channel_id) + "_raw";
    if (!readInt64_(vpath, raw)) {
        return false;
    }

    out_v = static_cast<double>(raw) / 1e6;
    return true;
}

bool AiDriver::readAllVoltages(std::vector<double>& out) const
{
    out.assign(4, 0.0);

    bool all_ok = true;
    for (int ch = 1; ch <= 4; ++ch) {
        double v = 0.0;
        const bool ok = readVoltage(ch, v);
        if (!ok) {
            all_ok = false;
        }
        out[ch - 1] = ok ? v : 0.0;
    }

    return all_ok;
}

bool AiDriver::readLogicVector(std::vector<double>& out) const
{
    std::array<AiChannelData, 4> a{};
    const bool ok = readAll(a);

    out.clear();
    out.reserve(12);

    for (const auto& ch : a) {
        out.push_back(ch.voltage_ok ? ch.voltage_v : 0.0);
        out.push_back(ch.current_ok ? ch.current_a : 0.0);
        out.push_back((ch.voltage_ok && ch.current_ok) ? ch.power_w : 0.0);
    }

    return ok;
}