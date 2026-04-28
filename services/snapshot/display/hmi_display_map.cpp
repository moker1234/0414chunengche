#include "hmi_display_map.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

using nlohmann::json;

namespace {

static std::string trimCopy_(const std::string& s)
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

static bool parseJsonLine_(const std::string& line, json& out, std::string* err)
{
    try {
        out = json::parse(line);
        return true;
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
}

static bool jsonToDouble_(const json& v, double& out)
{
    try {
        if (v.is_number()) {
            out = v.get<double>();
            return true;
        }

        if (v.is_string()) {
            const std::string s = trimCopy_(v.get<std::string>());
            if (s.empty()) return false;

            std::size_t idx = 0;
            const double d = std::stod(s, &idx);
            if (idx != s.size()) return false;

            out = d;
            return true;
        }
    } catch (...) {
    }

    return false;
}

static bool jsonToInt_(const json& v, int& out)
{
    try {
        if (v.is_number_integer()) {
            out = v.get<int>();
            return true;
        }

        if (v.is_number_float()) {
            out = static_cast<int>(v.get<double>());
            return true;
        }

        if (v.is_string()) {
            const std::string s = trimCopy_(v.get<std::string>());
            if (s.empty()) return false;

            std::size_t idx = 0;
            const long n = std::stol(s, &idx, 0);
            if (idx != s.size()) return false;

            out = static_cast<int>(n);
            return true;
        }
    } catch (...) {
    }

    return false;
}

static bool jsonToU16HexOrDec_(const json& v, uint16_t& out)
{
    try {
        if (v.is_number_unsigned()) {
            const auto u = v.get<uint64_t>();
            if (u > 0xFFFFu) return false;
            out = static_cast<uint16_t>(u);
            return true;
        }

        if (v.is_number_integer()) {
            const auto n = v.get<int64_t>();
            if (n < 0 || n > 0xFFFF) return false;
            out = static_cast<uint16_t>(n);
            return true;
        }

        if (v.is_number_float()) {
            const double d = v.get<double>();
            if (d < 0.0 || d > 65535.0) return false;
            out = static_cast<uint16_t>(d);
            return true;
        }

        if (v.is_string()) {
            return HmiDisplayMapLoader::parseHexU16(v.get<std::string>(), out);
        }
    } catch (...) {
    }

    return false;
}

static bool getStringField_(const json& j,
                            const char* key,
                            std::string& out)
{
    auto it = j.find(key);
    if (it == j.end()) return false;

    if (it->is_string()) {
        out = it->get<std::string>();
        return true;
    }

    if (it->is_number_integer()) {
        out = std::to_string(it->get<int64_t>());
        return true;
    }

    if (it->is_number_unsigned()) {
        out = std::to_string(it->get<uint64_t>());
        return true;
    }

    if (it->is_number_float()) {
        std::ostringstream oss;
        oss << it->get<double>();
        out = oss.str();
        return true;
    }

    return false;
}

static void fillOptAny_(const json& j,
                        const char* key1,
                        const char* key2,
                        std::optional<double>& dst)
{
    auto try_one = [&](const char* key) -> bool {
        auto it = j.find(key);
        if (it == j.end()) return false;

        double v = 0.0;
        if (!jsonToDouble_(*it, v)) return false;

        dst = v;
        return true;
    };

    if (key1 && try_one(key1)) return;
    if (key2) (void)try_one(key2);
}

static void fillOptIntAny_(const json& j,
                           const char* key1,
                           const char* key2,
                           std::optional<int>& dst)
{
    auto try_one = [&](const char* key) -> bool {
        auto it = j.find(key);
        if (it == j.end()) return false;

        int v = 0;
        if (!jsonToInt_(*it, v)) return false;

        dst = v;
        return true;
    };

    if (key1 && try_one(key1)) return;
    if (key2) (void)try_one(key2);
}

static bool parseWords_(const json& j, uint16_t& out_words)
{
    auto it = j.find("长度/int16");
    if (it == j.end()) {
        it = j.find("words");
    }
    if (it == j.end()) {
        it = j.find("word_count");
    }

    if (it == j.end()) {
        out_words = 1;
        return true;
    }

    int n = 0;
    if (!jsonToInt_(*it, n)) {
        return false;
    }

    if (n <= 0) n = 1;
    if (n > 16) n = 16;

    out_words = static_cast<uint16_t>(n);
    return true;
}

static bool parseItemAddress_(const json& it,
                              uint16_t& out_addr)
{
    auto a = it.find("地址hex");
    if (a == it.end()) a = it.find("addr");
    if (a == it.end()) a = it.find("address");
    if (a == it.end()) a = it.find("address_hex");

    if (a == it.end()) {
        return false;
    }

    return jsonToU16HexOrDec_(*a, out_addr);
}

static std::string typeName_(HmiValType t)
{
    switch (t) {
    case HmiValType::BoolRead: return "bool_read";
    case HmiValType::BoolRw:   return "bool_rw";
    case HmiValType::IntRead:  return "int_read";
    case HmiValType::IntRw:    return "int_rw";
    case HmiValType::U16:      return "u16";
    case HmiValType::S16:      return "s16";
    case HmiValType::Bit:      return "bit";
    default:                   return "unknown";
    }
}

} // namespace

bool HmiDisplayMapLoader::parseHexU16(const std::string& s, uint16_t& out)
{
    std::string t = trimCopy_(s);
    if (t.empty()) return false;

    /*
     * 地址列名叫“地址hex”，所以字符串默认按 16 进制解释。
     * 支持：
     *   "0x4000"
     *   "0X4000"
     *   "4000"
     */
    if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) {
        t = t.substr(2);
    }

    if (t.empty()) return false;

    char* end = nullptr;
    const unsigned long v = std::strtoul(t.c_str(), &end, 16);
    if (!end || *end != '\0') return false;
    if (v > 0xFFFFu) return false;

    out = static_cast<uint16_t>(v);
    return true;
}

bool HmiDisplayMapLoader::parseType(const std::string& s, HmiValType& out)
{
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

void HmiDisplayMapLoader::fillOpt(const json& j,
                                  const char* key,
                                  std::optional<double>& dst)
{
    auto it = j.find(key);
    if (it == j.end()) return;

    double v = 0.0;
    if (jsonToDouble_(*it, v)) {
        dst = v;
    }
}

void HmiDisplayMapLoader::fillOptInt(const json& j,
                                     const char* key,
                                     std::optional<int>& dst)
{
    auto it = j.find(key);
    if (it == j.end()) return;

    int v = 0;
    if (jsonToInt_(*it, v)) {
        dst = v;
    }
}

bool HmiDisplayMapLoader::loadJsonl(const std::string& path,
                                    HmiDisplayMap& out,
                                    std::string* err)
{
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

        line = trimCopy_(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        json j;
        std::string jerr;
        if (!parseJsonLine_(line, j, &jerr)) {
            if (err) {
                *err = "jsonl parse error at line " +
                       std::to_string(lineno) + ": " + jerr;
            }
            return false;
        }

        /*
         * 旧版要求 base/type/items。
         * 新版 normal_map_logic.v4.with_path.jsonl 没有 base，
         * 但每个 item 有“地址hex”。
         *
         * 这里放宽为：
         *   - type 必须有
         *   - items 必须有
         *   - base 可选
         */
        if (!j.contains("type") || !j.contains("items")) {
            if (err) {
                *err = "missing type/items at line " +
                       std::to_string(lineno);
            }
            return false;
        }

        if (!j["type"].is_string()) {
            if (err) {
                *err = "type must be string at line " +
                       std::to_string(lineno);
            }
            return false;
        }

        HmiMapBlock blk;

        {
            const std::string ts = j["type"].get<std::string>();
            if (!parseType(ts, blk.type)) {
                if (err) {
                    *err = "unknown type='" + ts +
                           "' at line " + std::to_string(lineno);
                }
                return false;
            }
        }

        // base 可选：旧版有 base；新版没有 base，后面用第一个 item.addr 回填。
        if (j.contains("base")) {
            if (!jsonToU16HexOrDec_(j["base"], blk.base)) {
                if (err) {
                    *err = "bad base at line " +
                           std::to_string(lineno);
                }
                return false;
            }
            blk.has_base = true;
        }

        // optional src (for bit)
        if (j.contains("src") && j["src"].is_string()) {
            blk.src = j["src"].get<std::string>();
        }

        // block defaults：兼容英文旧字段，同时支持中文“缩放”
        fillOpt(j, "def", blk.def);
        fillOptAny_(j, "scale", "缩放", blk.scale);
        fillOpt(j, "offset", blk.offset);
        fillOpt(j, "min", blk.minv);
        fillOpt(j, "max", blk.maxv);
        fillOptInt(j, "bit", blk.bit_index);

        if (!j["items"].is_array()) {
            if (err) {
                *err = "items must be array at line " +
                       std::to_string(lineno);
            }
            return false;
        }

        bool saw_explicit_addr = false;
        uint16_t first_explicit_addr = 0;

        for (const auto& it : j["items"]) {
            // 支持旧版 {"skip":N} 快速预留
            if (it.is_object() &&
                it.contains("skip") &&
                it["skip"].is_number_integer()) {
                int n = it["skip"].get<int>();
                if (n < 0) n = 0;

                for (int k = 0; k < n; ++k) {
                    HmiPathItem pi;
                    pi.path = "_";
                    pi.words = 1;
                    blk.items.push_back(std::move(pi));
                }
                continue;
            }

            HmiPathItem pi;

            if (it.is_string()) {
                // 旧版简写：items: ["system.xxx", "_", ...]
                pi.path = it.get<std::string>();
                pi.words = 1;
            }
            else if (it.is_object()) {
                /*
                 * path：
                 * - 旧版没有 path 时用 "_"
                 * - 新版一般都有 path，即使为空字符串也保持为空字符串
                 */
                if (it.contains("path") && it["path"].is_string()) {
                    pi.path = it["path"].get<std::string>();
                } else {
                    pi.path = "_";
                }

                // name / 变量描述
                if (!getStringField_(it, "name", pi.name)) {
                    (void)getStringField_(it, "变量描述", pi.name);
                }

                // 变量类型
                (void)getStringField_(it, "变量类型", pi.var_type);
                if (pi.var_type.empty()) {
                    (void)getStringField_(it, "var_type", pi.var_type);
                }

                // 地址hex / addr / address
                uint16_t item_addr = 0;
                if (parseItemAddress_(it, item_addr)) {
                    pi.has_addr = true;
                    pi.addr = item_addr;

                    if (!saw_explicit_addr) {
                        saw_explicit_addr = true;
                        first_explicit_addr = item_addr;
                    }
                }

                // 长度/int16 / words
                if (!parseWords_(it, pi.words)) {
                    if (err) {
                        *err = "bad words/长度 at line " +
                               std::to_string(lineno) +
                               ", type=" + typeName_(blk.type) +
                               ", name=" + pi.name;
                    }
                    return false;
                }

                // per-item override：兼容英文旧字段，同时支持中文“缩放”
                fillOpt(it, "def", pi.def);
                fillOptAny_(it, "scale", "缩放", pi.scale);
                fillOpt(it, "offset", pi.offset);
                fillOpt(it, "min", pi.minv);
                fillOpt(it, "max", pi.maxv);

                // compare
                fillOpt(it, "eq", pi.eq);
                fillOpt(it, "gt", pi.gt);
                fillOpt(it, "ge", pi.ge);
                fillOpt(it, "lt", pi.lt);
                fillOpt(it, "le", pi.le);

                fillOptIntAny_(it, "bit", nullptr, pi.bit_index);
            }
            else {
                pi.path = "_";
                pi.words = 1;
            }

            if (pi.words == 0) {
                pi.words = 1;
            }

            blk.items.push_back(std::move(pi));
        }

        /*
         * 新版没有 block base。
         * 为了让第一批替换后旧 NormalHmiWriter 不至于因为 base=0 而完全错段，
         * 这里用第一个显式地址回填 base。
         *
         * 真正按每个 item 的“地址hex”写 HMI，要等第二批修改 NormalHmiWriter。
         */
        if (!blk.has_base) {
            if (saw_explicit_addr) {
                blk.base = first_explicit_addr;
                blk.has_base = true;
            } else {
                if (err) {
                    *err = "line " + std::to_string(lineno) +
                           " has no base and no item 地址hex";
                }
                return false;
            }
        }

        out.blocks.push_back(std::move(blk));
    }

    return true;
}