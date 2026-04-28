//
// Created by lxy on 2026/4/22.
//

#include "do.h"

#include "GPIODriver.h"
#include "../common/io_channel_map.h"

#include <memory>

DigitalOutputManager::DigitalOutputManager() = default;
DigitalOutputManager::~DigitalOutputManager() = default;

bool DigitalOutputManager::init()
{
    outputs_.clear();
    raw_cache_.clear();

    bool all_ok = true;

    for (const auto& def : io_map::kDoDefs) {
        auto drv = std::make_unique<GPIODriver>(def.gpio_num);

        // 默认初始化为 OFF
        const int init_raw = def.active_high ? 0 : 1;
        if (!drv->initOutput(init_raw)) {
            all_ok = false;
        }

        outputs_[def.channel_id] = std::move(drv);
        raw_cache_[def.channel_id] = init_raw;
    }

    return all_ok;
}

bool DigitalOutputManager::setChannel(int channel_id, bool logical_on)
{
    const auto* def = io_map::findDo(channel_id);
    if (!def) return false;

    auto it = outputs_.find(channel_id);
    if (it == outputs_.end() || !it->second) return false;

    const int raw = def->active_high
        ? (logical_on ? 1 : 0)
        : (logical_on ? 0 : 1);

    if (!it->second->writeRaw(raw)) {
        return false;
    }

    raw_cache_[channel_id] = raw;
    return true;
}

bool DigitalOutputManager::getStates(std::vector<DoState>& out) const
{
    out.clear();
    out.reserve(io_map::kDoDefs.size());

    bool all_ok = true;

    for (const auto& def : io_map::kDoDefs) {
        DoState s;
        s.channel_id = def.channel_id;
        s.gpio_num = def.gpio_num;

        auto it = outputs_.find(def.channel_id);
        if (it == outputs_.end() || !it->second) {
            s.ok = false;
            all_ok = false;
            out.push_back(s);
            continue;
        }

        int raw = 0;
        const bool ok = it->second->readRaw(raw);
        s.ok = ok;
        if (!ok) {
            all_ok = false;
        }

        if (ok) {
            s.raw_value = raw;
            s.logical_on = def.active_high ? (raw != 0) : (raw == 0);
        } else {
            auto it_cache = raw_cache_.find(def.channel_id);
            if (it_cache != raw_cache_.end()) {
                s.raw_value = it_cache->second;
                s.logical_on = def.active_high
                    ? (s.raw_value != 0)
                    : (s.raw_value == 0);
            }
        }

        out.push_back(s);
    }

    return all_ok;
}