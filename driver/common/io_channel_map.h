//
// Created by lxy on 2026/4/22.
//

#ifndef ENERGYSTORAGE_IO_CHANNEL_MAP_H
#define ENERGYSTORAGE_IO_CHANNEL_MAP_H

#pragma once

#include <array>
#include <cstddef>

namespace io_map
{
    struct DiDef
    {
        int channel_id; // 1..18
        int gpio_num; // sysfs gpio number, e.g. 291
        const char* name; // "DI1"
        bool active_low; // true: raw=0 means logical ON
    };

    struct DoDef
    {
        int channel_id; // 1..8
        int gpio_num; // sysfs gpio number, e.g. 436
        const char* name; // "DO1"
        bool active_high; // true: logical ON => raw write 1
    };

    struct AiDef
    {
        int channel_id; // 1..4
        const char* name; // "AI1"..."AI4"
    };

    static constexpr const char* kDefaultIioDev = "/sys/bus/iio/devices/iio:device2";

    // ------------------------------------------------------------
    // DI：低电平有效（来自 dido_test.cpp）
    // raw=0 => ON, raw=1 => OFF
    // ------------------------------------------------------------
    static constexpr std::array<DiDef, 18> kDiDefs{
        {
            {1, 291, "DI1", true},
            {2, 468, "DI2", true},
            {3, 471, "DI3", true},
            {4, 469, "DI4", true},
            {5, 470, "DI5", true},
            {6, 286, "DI6", true},
            {7, 298, "DI7", true},
            {8, 297, "DI8", true},
            {9, 380, "DI9", true},
            {10, 381, "DI10", true},
            {11, 382, "DI11", true},
            {12, 383, "DI12", true},
            {13, 384, "DI13", true},
            {14, 385, "DI14", true},
            {15, 386, "DI15", true},
            {16, 387, "DI16", true},
            {17, 388, "DI17", true},
            {18, 389, "DI18", true},
        }
    };

    // ------------------------------------------------------------
    // DO：高电平有效（来自 dido_test.cpp）
    // logical ON => write raw 1
    // ------------------------------------------------------------
    static constexpr std::array<DoDef, 8> kDoDefs{
        {
            {1, 436, "DO1", true},
            {2, 437, "DO2", true},
            {3, 438, "DO3", true},
            {4, 439, "DO4", true},
            {5, 440, "DO5", true},
            {6, 442, "DO6", true},
            {7, 447, "DO7", true},
            {8, 448, "DO8", true},
        }
    };

    static constexpr std::array<AiDef, 4> kAiDefs{
        {
            {1, "AI1"},
            {2, "AI2"},
            {3, "AI3"},
            {4, "AI4"},
        }
    };

    /*
     * 查找 DI 定义
     * @param channel_id 1..18
     * @return nullptr if not found
     * @note 仅用于查询，不修改任何状态
     */
    inline const DiDef* findDi(int channel_id)
    {
        for (const auto& x : kDiDefs)
        {
            if (x.channel_id == channel_id) return &x;
        }
        return nullptr;
    }

    inline const DoDef* findDo(int channel_id)
    {
        for (const auto& x : kDoDefs)
        {
            if (x.channel_id == channel_id) return &x;
        }
        return nullptr;
    }

    inline const AiDef* findAi(int channel_id)
    {
        for (const auto& x : kAiDefs)
        {
            if (x.channel_id == channel_id) return &x;
        }
        return nullptr;
    }

    inline constexpr std::size_t diCount() { return kDiDefs.size(); }
    inline constexpr std::size_t doCount() { return kDoDefs.size(); }
    inline constexpr std::size_t aiCount() { return kAiDefs.size(); }
} // namespace io_map

#endif // ENERGYSTORAGE_IO_CHANNEL_MAP_H
