//
// Created by lxy on 2026/4/22.
//

#include "GPIODriver.h"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <chrono>

namespace {

static bool pathExists(const std::string& path)
{
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

} // namespace

GPIODriver::GPIODriver(int gpio_num)
    : gpio_num_(gpio_num)
{
}

std::string GPIODriver::basePath() const
{
    std::ostringstream oss;
    oss << "/sys/class/gpio/gpio" << gpio_num_;
    return oss.str();
}

bool GPIODriver::writeText_(const std::string& path, const std::string& s) const
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << s;
    return ofs.good();
}

bool GPIODriver::readText_(const std::string& path, std::string& out) const
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;
    std::getline(ifs, out);
    return !out.empty();
}

bool GPIODriver::ensureExported_() const
{
    if (gpio_num_ < 0) return false;

    const std::string base = basePath();
    if (pathExists(base)) {
        return true;
    }

    {
        std::ofstream ofs("/sys/class/gpio/export");
        if (!ofs.is_open()) {
            return false;
        }
        ofs << gpio_num_;
        if (!ofs.good()) {
            return false;
        }
    }

    // 给内核一点时间创建设备节点
    for (int i = 0; i < 20; ++i) {
        if (pathExists(base)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return pathExists(base);
}

bool GPIODriver::setDirection_(const char* dir) const
{
    if (!ensureExported_()) return false;
    return writeText_(basePath() + "/direction", dir ? dir : "in");
}

bool GPIODriver::initInput()
{
    return setDirection_("in");
}

bool GPIODriver::initOutput(int initial_raw)
{
    if (!setDirection_("out")) {
        return false;
    }
    return writeRaw(initial_raw);
}

bool GPIODriver::readRaw(int& out) const
{
    if (!ensureExported_()) return false;

    std::string s;
    if (!readText_(basePath() + "/value", s)) {
        return false;
    }

    try {
        out = std::stoi(s);
        out = (out != 0) ? 1 : 0;
        return true;
    } catch (...) {
        return false;
    }
}

bool GPIODriver::writeRaw(int raw) const
{
    if (!ensureExported_()) return false;
    return writeText_(basePath() + "/value", (raw != 0) ? "1" : "0");
}