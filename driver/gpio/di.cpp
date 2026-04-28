//
// Created by lxy on 2026/4/22.
//

#include "di.h"

#include "GPIODriver.h"
#include "../common/io_channel_map.h"

#include <memory>

DigitalInputManager::DigitalInputManager() = default;
DigitalInputManager::~DigitalInputManager() = default;

bool DigitalInputManager::init()
{
    inputs_.clear();
    inputs_.reserve(io_map::kDiDefs.size());

    bool all_ok = true;

    for (const auto& def : io_map::kDiDefs) {
        auto drv = std::make_unique<GPIODriver>(def.gpio_num);
        if (!drv->initInput()) {
            all_ok = false;
        }
        inputs_.push_back(std::move(drv));
    }

    return all_ok;
}

bool DigitalInputManager::readAll(std::vector<DiSample>& out) const
{
    out.clear();
    out.reserve(io_map::kDiDefs.size());

    bool all_ok = true;

    for (std::size_t i = 0; i < io_map::kDiDefs.size(); ++i) {
        const auto& def = io_map::kDiDefs[i];

        int raw = -1;
        const bool ok = inputs_[i] ? inputs_[i]->readRaw(raw) : false;
        if (!ok) all_ok = false;

        bool logical_on = false;
        if (ok) {
            logical_on = def.active_low ? (raw == 0) : (raw != 0);
        }

        DiSample s;
        s.channel_id = def.channel_id;
        s.gpio_num = def.gpio_num;
        s.raw_value = ok ? raw : -1;
        s.logical_on = logical_on;
        s.ok = ok;

        out.push_back(s);
    }

    return all_ok;
}

bool DigitalInputManager::sampleBits(uint64_t& di_bits) const
{
    di_bits = 0;

    std::vector<DiSample> v;
    const bool all_ok = readAll(v);

    for (const auto& s : v) {
        if (!s.ok) continue;
        if (s.channel_id < 1 || s.channel_id > 64) continue;

        if (s.logical_on) {
            di_bits |= (1ULL << (s.channel_id - 1));
        }
    }

    return all_ok;
}