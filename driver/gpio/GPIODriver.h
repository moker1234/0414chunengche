//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_GPIODRIVER_H
#define ENERGYSTORAGE_GPIODRIVER_H

#pragma once

#include <string>

class GPIODriver {
public:
    explicit GPIODriver(int gpio_num);
    ~GPIODriver() = default;

    bool initInput();
    bool initOutput(int initial_raw = 0);

    bool readRaw(int& out) const;     // raw 0/1 from /sys/class/gpio/gpioN/value
    bool writeRaw(int raw) const;     // write raw 0/1

    int gpioNum() const { return gpio_num_; }
    std::string basePath() const;

private:
    bool ensureExported_() const;
    bool writeText_(const std::string& path, const std::string& s) const;
    bool readText_(const std::string& path, std::string& out) const;
    bool setDirection_(const char* dir) const;

private:
    int gpio_num_{-1};
};

#endif // ENERGYSTORAGE_GPIODRIVER_H