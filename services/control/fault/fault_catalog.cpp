//
// Created by lxy on 2026/3/2.
//

// services/control/fault_catalog.cpp
#include "../fault/fault_catalog.h"

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
#include <unordered_map>

namespace control {

static bool parseHexU16_(const std::string& s, uint16_t& out) {
    std::string t = s;
    if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) t = t.substr(2);
    if (t.empty()) return false;

    unsigned long v = 0;
    try {
        v = std::stoul(t, nullptr, 16);
    } catch (...) {
        return false;
    }
    if (v > 0xFFFF) return false;
    out = static_cast<uint16_t>(v);
    return true;
}

static bool parseBoolLoose_(const nlohmann::json& v, bool defv = false)
{
    try {
        if (v.is_boolean()) return v.get<bool>();
        if (v.is_number_integer()) return v.get<int64_t>() != 0;
        if (v.is_number_unsigned()) return v.get<uint64_t>() != 0;
        if (v.is_string()) {
            std::string s = v.get<std::string>();
            for (auto& c : s) c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
            if (s == "1" || s == "true" || s == "yes" || s == "y") return true;
            if (s == "0" || s == "false" || s == "no" || s == "n") return false;
        }
    } catch (...) {}
    return defv;
}

static void parseClassificationFlags_(const std::string& classification,
                                          bool& show_cur,
                                          bool& show_his)
{
    show_cur = false;
    show_his = false;

    if (classification == "A1" || classification == "B1") {
        show_cur = true;
        show_his = true;
        return;
    }

    if (classification == "C1") {
        show_cur = false;
        show_his = true;
        return;
    }

    // A2/B2/C2 或其它未识别分类默认都不显示
}



static int parseIntLoose_(const nlohmann::json& v, int defv = 0)
{
    try {
        if (v.is_number_integer()) return static_cast<int>(v.get<int64_t>());
        if (v.is_number_unsigned()) return static_cast<int>(v.get<uint64_t>());
        if (v.is_string()) return std::stoi(v.get<std::string>());
    } catch (...) {}
    return defv;
}
bool FaultCatalog::parseCode(const std::string& s, uint16_t& out) {
    // 优先按 hex 解析（支持 "0x0081" / "0081"）
    if (parseHexU16_(s, out)) return true;

    // 其次按十进制
    try {
        unsigned long v = std::stoul(s, nullptr, 10);
        if (v > 0xFFFF) return false;
        out = static_cast<uint16_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool FaultCatalog::loadJsonl(const std::string& path, std::string* err)
{
    all_codes_.clear();
    current_codes_.clear();
    history_codes_.clear();
    meta_by_code_.clear();

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (err) *err = "open failed: " + path;
        return false;
    }

    std::string line;
    size_t lineno = 0;
    int source_order = 0;

    while (std::getline(ifs, line)) {
        ++lineno;
        if (line.empty()) continue;

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(line);
        } catch (const std::exception& e) {
            if (err) {
                std::ostringstream oss;
                oss << "json parse failed at line " << lineno << ": " << e.what();
                *err = oss.str();
            }
            return false;
        }

        if (!j.contains("items") || !j["items"].is_array()) continue;

        for (const auto& it : j["items"]) {
            if (!it.is_object()) continue;

            uint16_t code = 0;

            std::string code_s = it.value("code_hex", "");
            if (code_s.empty()) code_s = it.value("path", "");
            if (code_s.empty()) continue;

            if (!parseCode(code_s, code)) {
                continue;
            }

            FaultMeta meta;
            meta.code = code;
            meta.code_hex = it.value("code_hex", code_s);
            meta.name = it.value("name", "");
            meta.fault_type = it.value("fault_type", "");
            meta.fault_level = it.value("fault_level", "");
            meta.classification = it.value("classification", "");

            meta.record_db = parseBoolLoose_(it.contains("record_db") ? it["record_db"] : nlohmann::json{}, false);
            meta.reserved  = parseBoolLoose_(it.contains("reserved") ? it["reserved"] : nlohmann::json{}, false);

            meta.priority_rank = parseIntLoose_(it.contains("priority_rank") ? it["priority_rank"] : nlohmann::json{}, 9999);
            meta.source_order  = parseIntLoose_(it.contains("source_order") ? it["source_order"] : nlohmann::json{}, source_order);

            if (it.contains("show_hmi_current"))
                meta.show_hmi_current = parseBoolLoose_(it["show_hmi_current"], false);
            if (it.contains("show_hmi_history"))
                meta.show_hmi_history = parseBoolLoose_(it["show_hmi_history"], false);

            if (!it.contains("show_hmi_current") && !it.contains("show_hmi_history")) {
                parseClassificationFlags_(meta.classification, meta.show_hmi_current, meta.show_hmi_history);
            }

            auto [pos, inserted] = meta_by_code_.emplace(code, meta);
            if (!inserted) {
                const auto& oldv = pos->second;
                const bool better =
                    (meta.priority_rank < oldv.priority_rank) ||
                    (meta.priority_rank == oldv.priority_rank && meta.source_order < oldv.source_order);
                if (better) pos->second = meta;
            }

            ++source_order;
        }
    }

    std::vector<FaultMeta> metas;
    metas.reserve(meta_by_code_.size());
    for (const auto& kv : meta_by_code_) metas.push_back(kv.second);

    std::sort(metas.begin(), metas.end(),
              [](const FaultMeta& a, const FaultMeta& b) {
                  if (a.priority_rank != b.priority_rank) return a.priority_rank < b.priority_rank;
                  if (a.source_order  != b.source_order)  return a.source_order  < b.source_order;
                  return a.code < b.code;
              });

    for (const auto& m : metas) {
        all_codes_.push_back(m.code);
        if (m.show_hmi_current) current_codes_.push_back(m.code);
        if (m.show_hmi_history) history_codes_.push_back(m.code);
    }

    return true;
}

const FaultMeta* FaultCatalog::metaOf(uint16_t code) const
{
    auto it = meta_by_code_.find(code);
    if (it == meta_by_code_.end()) return nullptr;
    return &it->second;
}

bool FaultCatalog::showInCurrent(uint16_t code) const
{
    auto it = meta_by_code_.find(code);
    return (it != meta_by_code_.end()) ? it->second.show_hmi_current : false;
}

bool FaultCatalog::showInHistory(uint16_t code) const
{
    auto it = meta_by_code_.find(code);
    return (it != meta_by_code_.end()) ? it->second.show_hmi_history : false;
}

} // namespace control