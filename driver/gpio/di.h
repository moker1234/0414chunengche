//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_DI_H
#define ENERGYSTORAGE_DI_H

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

class GPIODriver;

struct DiSample {
    int channel_id{0};     // 1..18
    int gpio_num{0};       // e.g. 291
    int raw_value{-1};     // raw 0/1, -1 means read fail
    bool logical_on{false}; // active_low aware
    bool ok{false};
};

class DigitalInputManager {
public:
    DigitalInputManager();
    ~DigitalInputManager();

    bool init();
    bool readAll(std::vector<DiSample>& out) const;

    // bit0 -> DI1, bit1 -> DI2 ...
    // 逻辑值：ON=1, OFF=0；已按 active_low 处理
    bool sampleBits(uint64_t& di_bits) const;

private:
    std::vector<std::unique_ptr<GPIODriver>> inputs_;
};

#endif // ENERGYSTORAGE_DI_H