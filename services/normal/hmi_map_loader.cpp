//
// Created by lxy on 2026/4/27.
//

// services/normal/hmi_map_loader.cpp
#include "hmi_map_loader.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace normal {

using json = nlohmann::json;

namespace {

static std::string trim_(const std::string& s)
{
    std::size_t b = 0;
    std::size_t e = s.size();

    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }

    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }

    return s.substr(b, e - b);
}

static std::string lower_(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

static bool startsWith_(const std::string& s, const char* prefix)
{
    if (!prefix) return false;

    const std::string p(prefix);
    return s.size() >= p.size() &&
           s.compare(0, p.size(), p) == 0;
}

static bool jsonToString_(const json& v, std::string& out)
{
    if (v.is_null()) {
        return false;
    }

    try {
        if (v.is_string()) {
            out = v.get<std::string>();
            return true;
        }

        if (v.is_boolean()) {
            out = v.get<bool>() ? "1" : "0";
            return true;
        }

        if (v.is_number_integer()) {
            out = std::to_string(v.get<int64_t>());
            return true;
        }

        if (v.is_number_unsigned()) {
            out = std::to_string(v.get<uint64_t>());
            return true;
        }

        if (v.is_number_float()) {
            out = std::to_string(v.get<double>());
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

static bool readStringAny_(const json& j,
                           std::string& out,
                           const char* k1,
                           const char* k2 = nullptr,
                           const char* k3 = nullptr,
                           const char* k4 = nullptr)
{
    const char* keys[] = {k1, k2, k3, k4};

    for (const char* k : keys) {
        if (!k) continue;
        auto it = j.find(k);
        if (it == j.end()) continue;

        std::string s;
        if (!jsonToString_(*it, s)) {
            continue;
        }

        out = trim_(s);
        return true;
    }

    return false;
}

static bool parseU32Any_(const json& v, uint32_t& out)
{
    try {
        if (v.is_null()) {
            return false;
        }

        if (v.is_number_unsigned()) {
            out = v.get<uint32_t>();
            return true;
        }

        if (v.is_number_integer()) {
            const int64_t x = v.get<int64_t>();
            if (x < 0) return false;
            out = static_cast<uint32_t>(x);
            return true;
        }

        if (v.is_number_float()) {
            const double x = v.get<double>();
            if (x < 0.0 || x > 4294967295.0) return false;
            out = static_cast<uint32_t>(x);
            return true;
        }

        if (v.is_string()) {
            std::string s = trim_(v.get<std::string>());
            if (s.empty()) return false;

            std::size_t idx = 0;
            const unsigned long x = std::stoul(s, &idx, 0);
            if (idx != s.size()) return false;

            out = static_cast<uint32_t>(x);
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

static bool parseU16Any_(const json& v, uint16_t& out)
{
    uint32_t x = 0;
    if (!parseU32Any_(v, x)) {
        return false;
    }

    if (x > 0xFFFFu) {
        return false;
    }

    out = static_cast<uint16_t>(x);
    return true;
}

static bool readU16Any_(const json& j,
                        uint16_t& out,
                        const char* k1,
                        const char* k2 = nullptr,
                        const char* k3 = nullptr,
                        const char* k4 = nullptr)
{
    const char* keys[] = {k1, k2, k3, k4};

    for (const char* k : keys) {
        if (!k) continue;
        auto it = j.find(k);
        if (it == j.end()) continue;

        uint16_t v = 0;
        if (parseU16Any_(*it, v)) {
            out = v;
            return true;
        }
    }

    return false;
}

static bool parseDoubleAny_(const json& v, double& out)
{
    try {
        if (v.is_null()) {
            return false;
        }

        if (v.is_number()) {
            out = v.get<double>();
            return true;
        }

        if (v.is_boolean()) {
            out = v.get<bool>() ? 1.0 : 0.0;
            return true;
        }

        if (v.is_string()) {
            std::string s = trim_(v.get<std::string>());
            if (s.empty()) return false;

            std::size_t idx = 0;
            const double x = std::stod(s, &idx);
            if (idx != s.size()) return false;

            out = x;
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

static bool readDoubleAny_(const json& j,
                           std::optional<double>& out,
                           const char* k1,
                           const char* k2 = nullptr,
                           const char* k3 = nullptr)
{
    const char* keys[] = {k1, k2, k3};

    for (const char* k : keys) {
        if (!k) continue;
        auto it = j.find(k);
        if (it == j.end()) continue;

        double v = 0.0;
        if (parseDoubleAny_(*it, v)) {
            out = v;
            return true;
        }
    }

    return false;
}

static bool parseIntAny_(const json& v, int& out)
{
    try {
        if (v.is_null()) {
            return false;
        }

        if (v.is_number_integer()) {
            out = v.get<int>();
            return true;
        }

        if (v.is_number_unsigned()) {
            out = static_cast<int>(v.get<uint32_t>());
            return true;
        }

        if (v.is_number_float()) {
            out = static_cast<int>(v.get<double>());
            return true;
        }

        if (v.is_string()) {
            std::string s = trim_(v.get<std::string>());
            if (s.empty()) return false;

            std::size_t idx = 0;
            const long x = std::stol(s, &idx, 0);
            if (idx != s.size()) return false;

            out = static_cast<int>(x);
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

static bool readIntAny_(const json& j,
                        std::optional<int>& out,
                        const char* k1,
                        const char* k2 = nullptr)
{
    const char* keys[] = {k1, k2};

    for (const char* k : keys) {
        if (!k) continue;
        auto it = j.find(k);
        if (it == j.end()) continue;

        int v = 0;
        if (parseIntAny_(*it, v)) {
            out = v;
            return true;
        }
    }

    return false;
}

static HmiMapItemKind classifyItem_(const HmiMapItem& item)
{
    if (!item.hasPath()) {
        return HmiMapItemKind::Empty;
    }

    if (startsWith_(item.path, "hmi.fault.num.")) {
        return HmiMapItemKind::FaultNum;
    }

    if (startsWith_(item.path, "hmi.fault.btn.")) {
        return HmiMapItemKind::FaultButton;
    }

    if (isRwMapType(item.type)) {
        return HmiMapItemKind::Rw;
    }

    if (startsWith_(item.path, "hmi.")) {
        return HmiMapItemKind::OtherHmi;
    }

    return HmiMapItemKind::Normal;
}

static void inheritBlockDefaults_(const HmiMapBlock& blk, HmiMapItem& item)
{
    if (!item.def.has_value())       item.def = blk.def;
    if (!item.scale.has_value())     item.scale = blk.scale;
    if (!item.offset.has_value())    item.offset = blk.offset;
    if (!item.minv.has_value())      item.minv = blk.minv;
    if (!item.maxv.has_value())      item.maxv = blk.maxv;
    if (!item.bit_index.has_value()) item.bit_index = blk.bit_index;

    if (item.src.empty()) {
        item.src = blk.src;
    }
}

static void appendItemToModel_(HmiMapModel& model, HmiMapBlock& blk, HmiMapItem item)
{
    item.kind = classifyItem_(item);

    const std::size_t idx = model.items.size();
    model.items.push_back(std::move(item));

    blk.item_indexes.push_back(idx);

    const auto& ref = model.items.back();

    if (ref.hasPath()) {
        model.by_path[ref.path].push_back(idx);
    }

    if (ref.has_addr) {
        model.by_addr[ref.addr].push_back(idx);
    }

    if (ref.kind == HmiMapItemKind::FaultNum) {
        model.fault_num_items.push_back(idx);
    }

    if (ref.kind == HmiMapItemKind::FaultButton) {
        model.fault_button_items.push_back(idx);
    }

    if (isRwMapType(ref.type)) {
        model.rw_items.push_back(idx);
    }

    /*
     * 普通输出视图：
     * - 只包含 read/output 类型
     * - 不包含 hmi.fault.num.*，故障页只能 FaultPageManager 写
     * - path 为空或 "_" 的占位不进入
     */
    if (ref.hasPath() &&
        isReadOutputMapType(ref.type) &&
        ref.kind != HmiMapItemKind::FaultNum)
    {
        model.normal_items.push_back(idx);
    }
}

} // namespace

bool HmiMapLoader::parseType(const std::string& s, HmiMapType& out)
{
    const std::string t = lower_(trim_(s));

    if (t == "bool_read" || t == "bool" || t == "di") {
        out = HmiMapType::BoolRead;
        return true;
    }

    if (t == "bool_rw" || t == "coil" || t == "do") {
        out = HmiMapType::BoolRw;
        return true;
    }

    if (t == "int_read" || t == "input_register" || t == "input_reg") {
        out = HmiMapType::IntRead;
        return true;
    }

    if (t == "int_rw" || t == "holding_register" || t == "holding_reg") {
        out = HmiMapType::IntRw;
        return true;
    }

    if (t == "u16" || t == "uint16") {
        out = HmiMapType::U16;
        return true;
    }

    if (t == "s16" || t == "int16") {
        out = HmiMapType::S16;
        return true;
    }

    if (t == "bit") {
        out = HmiMapType::Bit;
        return true;
    }

    out = HmiMapType::Unknown;
    return false;
}

bool HmiMapLoader::parseU16HexOrDec(const std::string& s, uint16_t& out)
{
    try {
        const std::string x = trim_(s);
        if (x.empty()) {
            return false;
        }

        std::size_t idx = 0;
        const unsigned long v = std::stoul(x, &idx, 0);
        if (idx != x.size()) {
            return false;
        }

        if (v > 0xFFFFul) {
            return false;
        }

        out = static_cast<uint16_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool HmiMapLoader::loadJsonl(const std::string& path,
                             HmiMapModel& out,
                             std::string* err) const
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (err) {
            *err = "open failed: " + path;
        }
        return false;
    }

    HmiMapModel model;

    std::string line;
    uint32_t line_no = 0;

    while (std::getline(ifs, line)) {
        ++line_no;

        line = trim_(line);
        if (line.empty()) {
            continue;
        }

        json j;
        try {
            j = json::parse(line);
        } catch (const std::exception& e) {
            if (err) {
                std::ostringstream oss;
                oss << "json parse failed at line " << line_no << ": " << e.what();
                *err = oss.str();
            }
            return false;
        }

        if (!j.is_object()) {
            if (err) {
                *err = "line " + std::to_string(line_no) + " is not json object";
            }
            return false;
        }

        std::string type_text;
        if (!readStringAny_(j, type_text, "type")) {
            if (err) {
                *err = "line " + std::to_string(line_no) + " missing type";
            }
            return false;
        }

        HmiMapType type = HmiMapType::Unknown;
        if (!parseType(type_text, type)) {
            if (err) {
                *err = "line " + std::to_string(line_no) + " unsupported type: " + type_text;
            }
            return false;
        }

        if (!j.contains("items") || !j["items"].is_array()) {
            if (err) {
                *err = "line " + std::to_string(line_no) + " missing items array";
            }
            return false;
        }

        HmiMapBlock blk;
        blk.type = type;
        blk.type_text = type_text;
        blk.line_no = line_no;

        uint16_t base = 0;
        if (readU16Any_(j, base, "base", "起始地址", "addr_base", "address_base")) {
            blk.has_base = true;
            blk.base = base;
        }

        readStringAny_(j, blk.src, "src", "source", "bit_src");

        readDoubleAny_(j, blk.def,    "def", "default");
        readDoubleAny_(j, blk.scale,  "scale", "缩放");
        readDoubleAny_(j, blk.offset, "offset", "偏移");
        readDoubleAny_(j, blk.minv,   "min", "最小值");
        readDoubleAny_(j, blk.maxv,   "max", "最大值");
        readIntAny_(j, blk.bit_index, "bit", "bit_index");

        const auto& arr = j["items"];

        bool saw_explicit_addr = false;
        uint16_t first_explicit_addr = 0;

        for (std::size_t i = 0; i < arr.size(); ++i) {
            const auto& one = arr[i];

            HmiMapItem item;
            item.type = type;
            item.type_text = type_text;
            item.block_index = model.blocks.size();
            item.item_index = i;
            item.line_no = line_no;
            item.block_has_base = blk.has_base;
            item.block_base = blk.base;
            item.src = blk.src;

            if (one.is_object()) {
                readStringAny_(one, item.path,       "path");
                readStringAny_(one, item.name,       "变量描述", "name", "desc", "description");
                readStringAny_(one, item.value_type, "变量类型", "var_type", "value_type", "type_name");

                uint16_t item_addr = 0;
                if (readU16Any_(one, item_addr, "地址hex", "addr", "address", "地址")) {
                    item.has_addr = true;
                    item.addr = item_addr;

                    if (!saw_explicit_addr) {
                        saw_explicit_addr = true;
                        first_explicit_addr = item_addr;
                    }
                } else if (blk.has_base) {
                    const uint32_t addr =
                        static_cast<uint32_t>(blk.base) + static_cast<uint32_t>(i);

                    if (addr > 0xFFFFu) {
                        if (err) {
                            *err = "line " + std::to_string(line_no) +
                                   " item address overflow at index " + std::to_string(i);
                        }
                        return false;
                    }

                    item.has_addr = true;
                    item.addr = static_cast<uint16_t>(addr);
                }

                uint16_t words = 1;
                if (readU16Any_(one, words, "长度/int16", "words", "length", "长度")) {
                    item.words = (words == 0) ? 1 : words;
                } else {
                    item.words = 1;
                }

                readDoubleAny_(one, item.def,    "def", "default");
                readDoubleAny_(one, item.scale,  "scale", "缩放");
                readDoubleAny_(one, item.offset, "offset", "偏移");
                readDoubleAny_(one, item.minv,   "min", "最小值");
                readDoubleAny_(one, item.maxv,   "max", "最大值");

                readDoubleAny_(one, item.eq, "eq");
                readDoubleAny_(one, item.gt, "gt");
                readDoubleAny_(one, item.ge, "ge");
                readDoubleAny_(one, item.lt, "lt");
                readDoubleAny_(one, item.le, "le");

                readIntAny_(one, item.bit_index, "bit", "bit_index");
            } else {
                // 非 object 的 item 作为占位保留，防止索引错位。
                item.path = "_";
                item.words = 1;

                if (blk.has_base) {
                    const uint32_t addr =
                        static_cast<uint32_t>(blk.base) + static_cast<uint32_t>(i);

                    if (addr <= 0xFFFFu) {
                        item.has_addr = true;
                        item.addr = static_cast<uint16_t>(addr);
                    }
                }
            }

            if (item.words == 0) {
                item.words = 1;
            }

            inheritBlockDefaults_(blk, item);
            appendItemToModel_(model, blk, std::move(item));
        }

        /*
         * 新版 normal_map_logic.jsonl 通常每个 item 自带 地址hex，
         * block 没有 base。
         *
         * 为兼容后续调试和旧结构，这里用第一个显式地址回填 block.base。
         * 真正写屏时仍应优先使用 item.addr。
         */
        if (!blk.has_base && saw_explicit_addr) {
            blk.has_base = true;
            blk.base = first_explicit_addr;
        }

        model.blocks.push_back(std::move(blk));
    }

    out = std::move(model);
    return true;
}

} // namespace normal