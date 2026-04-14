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

#include "display_data_map.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

// 你需要把 nlohmann/json.hpp 放到 third_party/nlohmann/json.hpp
#include "../../third_party/nlohmann/json.hpp"

namespace display {

using nlohmann::json;

static uint16_t clampToWord(int v, int lo, int hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return static_cast<uint16_t>(v);
}

static bool parseU16Addr(const json& jv, uint16_t& out) {
    try {
        if (jv.is_number_unsigned() || jv.is_number_integer()) {
            int v = jv.get<int>();
            if (v < 0 || v > 0xFFFF) return false;
            out = static_cast<uint16_t>(v);
            return true;
        }
        if (jv.is_string()) {
            std::string s = jv.get<std::string>();
            // 支持 "0x0200" 或 "512"
            int base = 10;
            if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) base = 16;
            int v = std::stoi(s, nullptr, base);
            if (v < 0 || v > 0xFFFF) return false;
            out = static_cast<uint16_t>(v);
            return true;
        }
    } catch (...) {
    }
    return false;
}

static FieldSrc parseSrc(const std::string& s) {
    if (s == "num") return FieldSrc::Num;
    if (s == "status") return FieldSrc::Status;
    if (s == "value") return FieldSrc::Value;
    return FieldSrc::Auto;
}

bool DisplayDataMap::loadFromFile(const std::string& json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
        std::printf("[DISPLAY][MAP] open failed: %s\n", json_path.c_str());
        return false;
    }

    json root;
    try {
        ifs >> root;
    } catch (const std::exception& e) {
        std::printf("[DISPLAY][MAP] json parse error: %s\n", e.what());
        return false;
    }

    if (!root.is_object()) return false;
    if (!root.contains("mappings") || !root["mappings"].is_array()) {
        std::printf("[DISPLAY][MAP] missing 'mappings' array\n");
        return false;
    }

    std::vector<WordMapping> out;
    out.reserve(root["mappings"].size());

    // 可选全局默认：default_num_scale
    double default_num_scale = 10.0;
    if (root.contains("defaults") && root["defaults"].is_object()) {
        auto& d = root["defaults"];
        if (d.contains("num_scale") && d["num_scale"].is_number()) {
            default_num_scale = d["num_scale"].get<double>();
        }
    }

    for (const auto& it : root["mappings"]) {
        if (!it.is_object()) continue;

        WordMapping m;

        // key / field / addr 必须
        if (!it.contains("key") || !it["key"].is_string()) continue;
        if (!it.contains("field") || !it["field"].is_string()) continue;
        if (!it.contains("addr")) continue;

        m.key   = it["key"].get<std::string>();
        m.field = it["field"].get<std::string>();

        if (!parseU16Addr(it["addr"], m.addr)) continue;

        // src 可选：num/status/value/auto
        if (it.contains("src") && it["src"].is_string()) {
            m.src = parseSrc(it["src"].get<std::string>());
        } else {
            m.src = FieldSrc::Auto;
        }

        // scale/offset/default/clamp 可选
        m.scale = default_num_scale;
        if (it.contains("scale") && it["scale"].is_number()) m.scale = it["scale"].get<double>();
        if (it.contains("offset") && it["offset"].is_number()) m.offset = it["offset"].get<double>();

        if (it.contains("default") && (it["default"].is_number_integer() || it["default"].is_number_unsigned())) {
            int dv = it["default"].get<int>();
            if (dv < 0) dv = 0;
            if (dv > 65535) dv = 65535;
            m.default_word = static_cast<uint16_t>(dv);
        }

        if (it.contains("clamp") && it["clamp"].is_array() && it["clamp"].size() == 2) {
            if (it["clamp"][0].is_number_integer() && it["clamp"][1].is_number_integer()) {
                m.clamp_enable = true;
                m.clamp_min = it["clamp"][0].get<int>();
                m.clamp_max = it["clamp"][1].get<int>();
            }
        } else if (it.contains("clamp_enable") && it["clamp_enable"].is_boolean()) {
            m.clamp_enable = it["clamp_enable"].get<bool>();
        }

        out.push_back(std::move(m));
    }

    if (out.empty()) {
        std::printf("[DISPLAY][MAP] mappings loaded but empty\n");
        return false;
    }

    mappings_ = std::move(out);
    std::printf("[DISPLAY][MAP] loaded %zu mappings from %s\n", mappings_.size(), json_path.c_str());
    return true;
}

void DisplayDataMap::initDefaultMappings() {
    mappings_ = {
        // 示例默认表（你可以保留当 fallback）
        { "sensor.gas.rs485_0_1", "gas_value",  0x0200, FieldSrc::Num,    10.0, 0.0, 0, true, 0, 65535 },
        { "sensor.gas.rs485_0_1", "alarm",      0x0201, FieldSrc::Status, 1.0,  0.0, 0, true, 0, 1 },

        { "sensor.smoke.rs485_0_2", "smoke_percent", 0x0210, FieldSrc::Num,   1.0, 0.0, 0, true, 0, 255 },
        { "sensor.smoke.rs485_0_2", "temp",          0x0211, FieldSrc::Value, 1.0, 0.0, 0, true, -40, 125 },
        { "sensor.smoke.rs485_0_2", "alarm",         0x0212, FieldSrc::Status,1.0, 0.0, 0, true, 0, 1 },

        { "system", "heartbeat", 0x0000, FieldSrc::Value, 1.0, 0.0, 0, true, 0, 65535 },
        { "system", "uptime",    0x0001, FieldSrc::Value, 1.0, 0.0, 0, true, 0, 65535 },
    };
}

/* ================= 构建写块 ================= */

std::vector<WordBlock>
DisplayDataMap::buildWriteBlocks(const agg::SystemSnapshot& snap) {
    std::map<uint16_t, uint16_t> word_table;

    for (const auto& m : mappings_) {
        auto it = snap.items.find(m.key);
        if (it == snap.items.end()) {
            // key 不存在：填 default（可选：也可以选择跳过）
            word_table[m.addr] = m.default_word;
            continue;
        }

        const DeviceData& d = it->second.data;
        uint16_t v = toWord(d, m);
        word_table[m.addr] = v;
    }

    std::vector<WordBlock> blocks;
    if (word_table.empty()) return blocks;

    auto it = word_table.begin();
    WordBlock cur;
    cur.start_addr = it->first;
    cur.data.push_back(it->second);

    uint16_t last_addr = it->first;
    ++it;

    for (; it != word_table.end(); ++it) {
        if (it->first == static_cast<uint16_t>(last_addr + 1)) {
            cur.data.push_back(it->second);
        } else {
            blocks.push_back(cur);
            cur.start_addr = it->first;
            cur.data.clear();
            cur.data.push_back(it->second);
        }
        last_addr = it->first;
    }

    blocks.push_back(cur);
    return blocks;
}

/* ================= 字段 → Word ================= */

uint16_t DisplayDataMap::toWord(const DeviceData& d, const WordMapping& m) {
    // 1) 指定来源
    if (m.src == FieldSrc::Num) {
        auto itn = d.num.find(m.field);
        if (itn == d.num.end()) return m.default_word;
        double v = itn->second;
        int w = static_cast<int>(std::lround(v * m.scale + m.offset));
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }
    if (m.src == FieldSrc::Status) {
        auto its = d.status.find(m.field);
        if (its == d.status.end()) return m.default_word;
        int w = static_cast<int>(its->second);
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }
    if (m.src == FieldSrc::Value) {
        auto itv = d.value.find(m.field);
        if (itv == d.value.end()) return m.default_word;
        int w = static_cast<int>(itv->second);
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }

    // 2) Auto：num -> status -> value
    auto itn = d.num.find(m.field);
    if (itn != d.num.end()) {
        int w = static_cast<int>(std::lround(itn->second * m.scale + m.offset));
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }
    auto its = d.status.find(m.field);
    if (its != d.status.end()) {
        int w = static_cast<int>(its->second);
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }
    auto itv = d.value.find(m.field);
    if (itv != d.value.end()) {
        int w = static_cast<int>(itv->second);
        return m.clamp_enable ? clampToWord(w, m.clamp_min, m.clamp_max) : static_cast<uint16_t>(w);
    }

    return m.default_word;
}

} // namespace display
