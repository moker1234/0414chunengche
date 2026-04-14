#include "hmi_display_map.h"
#include <fstream>
#include <sstream>
#include <cstdlib>

using nlohmann::json;

bool HmiDisplayMapLoader::parseHexU16(const std::string& s, uint16_t& out) {
    std::string t = s;
    if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) t = t.substr(2);
    if (t.empty()) return false;
    char* end = nullptr;
    unsigned long v = std::strtoul(t.c_str(), &end, 16);
    if (!end || *end != '\0') return false;
    if (v > 0xFFFF) return false;
    out = static_cast<uint16_t>(v);
    return true;
}

bool HmiDisplayMapLoader::parseType(const std::string& s, HmiValType& out) {
    // ===== new 4-area types =====
    if (s == "bool_read") { out = HmiValType::BoolRead; return true; }
    if (s == "bool_rw")   { out = HmiValType::BoolRw;   return true; }
    if (s == "int_read")  { out = HmiValType::IntRead;  return true; }
    if (s == "int_rw")    { out = HmiValType::IntRw;    return true; }

    // ===== legacy compatible =====
    if (s == "bool") { out = HmiValType::BoolRead; return true; }
    if (s == "u16")  { out = HmiValType::U16;      return true; }
    if (s == "s16")  { out = HmiValType::S16;      return true; }
    if (s == "bit")  { out = HmiValType::Bit;      return true; }

    return false;
}

void HmiDisplayMapLoader::fillOpt(const json& j, const char* key, std::optional<double>& dst) {
    auto it = j.find(key);
    if (it != j.end() && it->is_number()) dst = it->get<double>();
}

void HmiDisplayMapLoader::fillOptInt(const json& j, const char* key, std::optional<int>& dst) {
    auto it = j.find(key);
    if (it != j.end() && it->is_number_integer()) dst = it->get<int>();
}

static bool parseJsonLine(const std::string& line, json& out, std::string* err) {
    try {
        out = json::parse(line);
        return true;
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
}

bool HmiDisplayMapLoader::loadJsonl(const std::string& path, HmiDisplayMap& out, std::string* err) {
    out.blocks.clear();

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (err) *err = "open failed: " + path;
        return false;
    }

    std::string line;
    int lineno = 0;
    while (std::getline(ifs, line)) {
        ++lineno;

        // trim
        auto is_space = [](unsigned char c){ return std::isspace(c); };
        while (!line.empty() && is_space(line.back())) line.pop_back();
        size_t p = 0;
        while (p < line.size() && is_space((unsigned char)line[p])) ++p;
        if (p > 0) line = line.substr(p);

        if (line.empty()) continue;
        if (line[0] == '#') continue; // comment

        json j;
        std::string jerr;
        if (!parseJsonLine(line, j, &jerr)) { // 在这里会返回false
            if (err) {
                *err = "jsonl parse error at line " + std::to_string(lineno) + ": " + jerr;
            }
            return false;
        }

        // block must have base/type/items
        if (!j.contains("base") || !j.contains("type") || !j.contains("items")) {
            if (err) *err = "missing base/type/items at line " + std::to_string(lineno);
            return false;
        }

        HmiMapBlock blk;

        // base
        if (j["base"].is_string()) {
            if (!parseHexU16(j["base"].get<std::string>(), blk.base)) {
                if (err) *err = "bad base at line " + std::to_string(lineno);
                return false;
            }
        } else if (j["base"].is_number_unsigned()) {
            blk.base = static_cast<uint16_t>(j["base"].get<uint32_t>() & 0xFFFFu);
        } else {
            if (err) *err = "base must be string hex or u32 at line " + std::to_string(lineno);
            return false;
        }

        // type
        if (!j["type"].is_string()) {
            if (err) *err = "type must be string at line " + std::to_string(lineno);
            return false;
        }
        {
            const auto ts = j["type"].get<std::string>();
            if (!parseType(ts, blk.type)) {
                if (err) *err = "unknown type='" + ts + "' at line " + std::to_string(lineno);
                return false;
            }
        }

        // optional src (for bit)
        if (j.contains("src") && j["src"].is_string()) blk.src = j["src"].get<std::string>();

        // block defaults
        fillOpt(j, "def", blk.def);
        fillOpt(j, "scale", blk.scale);
        fillOpt(j, "offset", blk.offset);
        fillOpt(j, "min", blk.minv);
        fillOpt(j, "max", blk.maxv);
        fillOptInt(j, "bit", blk.bit_index);

        // items
        if (!j["items"].is_array()) {
            if (err) *err = "items must be array at line " + std::to_string(lineno);
            return false;
        }

        for (const auto& it : j["items"]) {
            // 支持 {"skip":N} 快速预留
            if (it.is_object() && it.contains("skip") && it["skip"].is_number_integer()) {
                int n = it["skip"].get<int>();
                if (n < 0) n = 0;
                for (int k = 0; k < n; ++k) {
                    HmiPathItem pi;
                    pi.path = "_";
                    blk.items.push_back(std::move(pi));
                }
                continue;
            }

            HmiPathItem pi;

            if (it.is_string()) {
                pi.path = it.get<std::string>();
            } else if (it.is_object()) {
                if (it.contains("path") && it["path"].is_string()) pi.path = it["path"].get<std::string>();
                else pi.path = "_";

                // per-item override
                fillOpt(it, "def", pi.def);
                fillOpt(it, "scale", pi.scale);
                fillOpt(it, "offset", pi.offset);
                fillOpt(it, "min", pi.minv);
                fillOpt(it, "max", pi.maxv);

                // compare
                fillOpt(it, "eq", pi.eq);
                fillOpt(it, "gt", pi.gt);
                fillOpt(it, "ge", pi.ge);
                fillOpt(it, "lt", pi.lt);
                fillOpt(it, "le", pi.le);

                fillOptInt(it, "bit", pi.bit_index);

                if (it.contains("name") && it["name"].is_string()) pi.name = it["name"].get<std::string>();
            } else {
                pi.path = "_";
            }

            blk.items.push_back(std::move(pi));
        }

        out.blocks.push_back(std::move(blk));
    }

    return true;
}
