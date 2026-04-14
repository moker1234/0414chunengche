//
// Created by lxy on 2026/1/12.
//
/*
 * 显示数据映射表
 */

/* 解释整个文件的作用
 * 该文件实现了显示数据映射表的定义，包括构造函数、加载函数和数据映射函数。
 * 构造函数用于初始化显示数据映射表，加载函数用于从 JSON 文件加载映射数据，数据映射函数用于根据字段源和值返回映射后的显示数据。
 */

#ifndef ENERGYSTORAGE_DISPLAY_DATA_MAP_H
#define ENERGYSTORAGE_DISPLAY_DATA_MAP_H

#pragma once

#include "../aggregator/system_snapshot.h"   // 方案A：SystemSnapshot.items
#include "../protocol/protocol_base.h"       // DeviceData
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace display {

    /**
     * 数据来源类型：从 DeviceData 的哪个容器取字段
     */
    enum class FieldSrc {
        Auto,   // 自动：num->status->value
        Num,
        Status,
        Value
    };

    /**
     * 一个 Word 映射项（由 JSON 驱动）
     */
    struct WordMapping {
        std::string key;      // snapshot.items 的 key（例如 "sensor.gas.rs485_0_1"）
        std::string field;    // DeviceData 内字段名（例如 "gas_value"）
        uint16_t    addr{0};  // Word 地址

        FieldSrc src{FieldSrc::Auto};

        // 数值转换：word = round(value * scale + offset)
        double scale{10.0};   // 默认 *10（适配你之前的屏幕无浮点习惯）
        double offset{0.0};

        // 缺省值（字段不存在时）
        uint16_t default_word{0};

        // 可选：钳位，避免溢出
        bool     clamp_enable{true};
        int      clamp_min{0};
        int      clamp_max{65535};
    };

    /**
     * 连续写块（用于 0x82 指令）
     */
    struct WordBlock {
        uint16_t start_addr{0};
        std::vector<uint16_t> data;
    };

    /**
     * DisplayDataMap（JSON / 表驱动）
     * - loadFromFile(): 从 JSON 加载 mappings
     * - buildWriteBlocks(): snapshot -> 多个连续写块
     */
    class DisplayDataMap {
    public:
        DisplayDataMap() = default;

        // 从 JSON 文件加载；失败则返回 false（你可以选择继续用默认表）
        bool loadFromFile(const std::string& json_path);

        // 若你希望“无 JSON 时也能跑”，可以调用这个
        void initDefaultMappings();

        std::vector<WordBlock>
        buildWriteBlocks(const agg::SystemSnapshot& snap);

        const std::vector<WordMapping>& mappings() const { return mappings_; }

    private:
        static uint16_t toWord(const DeviceData& d, const WordMapping& m);

    private:
        std::vector<WordMapping> mappings_;
    };

} // namespace display

#endif //ENERGYSTORAGE_DISPLAY_DATA_MAP_H
