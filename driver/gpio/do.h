//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_DO_H
#define ENERGYSTORAGE_DO_H

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

class GPIODriver;

struct DoState {
    int channel_id{0};      // 1..8
    int gpio_num{0};        // e.g. 436
    int raw_value{0};       // actual raw 0/1 written
    bool logical_on{false}; // active_high aware
    bool ok{false};
};

class DigitalOutputManager {
public:
    DigitalOutputManager();
    ~DigitalOutputManager();

    bool init();
    bool setChannel(int channel_id, bool logical_on);
    bool getStates(std::vector<DoState>& out) const;

private:
    std::unordered_map<int, std::unique_ptr<GPIODriver>> outputs_;
    std::unordered_map<int, int> raw_cache_;
};

#endif // ENERGYSTORAGE_DO_H