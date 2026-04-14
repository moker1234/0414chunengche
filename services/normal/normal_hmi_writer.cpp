// services/normal/normal_hmi_writer.cpp
#include "normal_hmi_writer.h"

#include "logger.h"
#include "../protocol/rs485/hmi/hmi_proto.h"

using nlohmann::json;

namespace normal {

bool NormalHmiWriter::loadMapFile(const std::string& path, std::string* err)
{
    map_.blocks.clear();
    std::string e;
    if (!loader_.loadJsonl(path, map_, &e)) {
        if (err) *err = e;
        loaded_ = false;
        return false;
    }
    loaded_ = true;
    return true;
}

const json* NormalHmiWriter::resolvePathCompat(const json& root, const std::string& path)
{
    // '.' split
    std::vector<std::string> tokens;
    {
        std::string cur;
        for (char c : path) {
            if (c == '.') {
                if (!cur.empty()) tokens.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
        if (!cur.empty()) tokens.push_back(cur);
    }

    const json* node = &root;

    for (size_t i = 0; i < tokens.size(); ++i) {
        if (!node->is_object()) return nullptr;
        const auto& tk = tokens[i];

        auto it = node->find(tk);
        if (it != node->end()) {
            node = &(*it);
            continue;
        }

        // compat: join remaining tokens as one key
        std::string joined = tk;
        for (size_t k = i + 1; k < tokens.size(); ++k) {
            joined.push_back('.');
            joined += tokens[k];
        }
        auto it2 = node->find(joined);
        if (it2 != node->end()) {
            node = &(*it2);
            return node;
        }
        return nullptr;
    }
    return node;
}

bool NormalHmiWriter::toNumber(const json& v, double& out)
{
    if (v.is_number()) { out = v.get<double>(); return true; }
    if (v.is_boolean()) { out = v.get<bool>() ? 1.0 : 0.0; return true; }
    return false;
}

bool NormalHmiWriter::toBool(const json& v, bool& out)
{
    if (v.is_boolean()) { out = v.get<bool>(); return true; }
    if (v.is_number()) { out = (v.get<double>() != 0.0); return true; }
    return false;
}

uint16_t NormalHmiWriter::clampU16(int64_t v)
{
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
}

uint16_t NormalHmiWriter::packS16(int32_t v)
{
    return static_cast<uint16_t>(v & 0xFFFF);
}

double NormalHmiWriter::pickOpt(const std::optional<double>& item, const std::optional<double>& blk, double defv)
{
    if (item.has_value()) return *item;
    if (blk.has_value()) return *blk;
    return defv;
}

bool NormalHmiWriter::evalCompare(double x, const HmiPathItem& it, bool& out)
{
    if (it.eq.has_value()) { out = (x == *it.eq); return true; }

    bool has = false;
    bool ok = true;

    if (it.gt.has_value()) { has = true; ok = ok && (x >  *it.gt); }
    if (it.ge.has_value()) { has = true; ok = ok && (x >= *it.ge); }
    if (it.lt.has_value()) { has = true; ok = ok && (x <  *it.lt); }
    if (it.le.has_value()) { has = true; ok = ok && (x <= *it.le); }

    if (has) { out = ok; return true; }
    return false;
}

bool NormalHmiWriter::isBoolType(HmiValType t)
{
    return t == HmiValType::BoolRead || t == HmiValType::BoolRw;
}

bool NormalHmiWriter::isIntType(HmiValType t)
{
    return t == HmiValType::IntRead || t == HmiValType::IntRw || t == HmiValType::U16 || t == HmiValType::S16;
}

bool NormalHmiWriter::isRwType(HmiValType t)
{
    return t == HmiValType::BoolRw || t == HmiValType::IntRw;
}

void NormalHmiWriter::injectDerivedFields(json& j) const
{
    if (!j.is_object()) return;
    if (!j.contains("system") || !j["system"].is_object()) {
        j["system"] = json::object();
    }

    auto& sys = j["system"];

    auto getBool = [&](const char* k) -> bool {
        auto it = sys.find(k);
        if (it == sys.end()) return false;
        if (it->is_boolean()) return it->get<bool>();
        if (it->is_number())  return it->get<double>() != 0.0;
        return false;
    };

    // system.alarm_bits
    uint16_t bits = 0;
    if (getBool("gas_alarm"))     bits |= (1u << 0);
    if (getBool("smoke_alarm"))   bits |= (1u << 1);
    if (getBool("ac_alarm"))      bits |= (1u << 2);
    if (getBool("bms_alarm_any")) bits |= (1u << 3);
    sys["alarm_bits"] = bits;

    // system.normal_seq：每次 flush++（心跳/版本号）
    sys["normal_seq"] = (uint32_t)(++seq_);
}

void NormalHmiWriter::flushFromSnapshot(const agg::SystemSnapshot& snap, HMIProto& hmi) const
{
    // 兼容旧逻辑：不传 logic_view 时给一个空对象
    flushFromModel(snap, json::object(), hmi);
}

void NormalHmiWriter::flushFromModel(const agg::SystemSnapshot& snap,
                                     const nlohmann::json& logic_view,
                                     HMIProto& hmi) const
{
    if (!loaded_) return;

    json j = snap.toJson();

    // 新增：把逻辑显示视图挂到 system.logic_view.*
    if (!j.is_object()) {
        j = json::object();
    }
    if (!j.contains("system") || !j["system"].is_object()) {
        j["system"] = json::object();
    }
    j["system"]["logic_view"] = logic_view.is_object() ? logic_view : json::object();

    // 注入派生字段：alarm_bits + normal_seq
    injectDerivedFields(j);

    for (const auto& blk : map_.blocks) {
        const uint16_t base = blk.base;

        const json* bit_src_node = nullptr;
        if (blk.type == HmiValType::Bit && !blk.src.empty()) {
            bit_src_node = resolvePathCompat(j, blk.src);
        }


        for (size_t i = 0; i < blk.items.size(); ++i) {
            const auto& it = blk.items[i];
            const uint16_t addr = static_cast<uint16_t>(base + static_cast<uint16_t>(i));

            if (it.path == "_" || it.path.empty()) continue;

            const json* node = nullptr;
            if (blk.type == HmiValType::Bit && bit_src_node) node = bit_src_node;
            else node = resolvePathCompat(j, it.path);

            const auto defv = it.def.has_value() ? it.def : blk.def;

            auto writeBool = [&](bool b) {
                if (isRwType(blk.type)) return;
                hmi.setBoolRead(addr, b);
            };

            auto writeInt = [&](uint16_t v) {
                if (isRwType(blk.type)) return;
                // if (addr >= 0x4046 && addr <= 0x4060) {
                //     LOG_COMM_D("addr=0x%04X path=%s value=%u",
                //               addr, it.path.c_str(), (unsigned)v);
                // }
                hmi.setIntRead(addr, v);
            };

            if (!node) {
                if (isBoolType(blk.type)) {
                    bool b = defv.has_value() ? (*defv != 0.0) : false;
                    writeBool(b);
                } else if (isIntType(blk.type)) {
                    uint16_t v = 0;
                    if (defv.has_value()) {
                        if (blk.type == HmiValType::S16) v = packS16(static_cast<int32_t>(*defv));
                        else v = clampU16(static_cast<int64_t>(*defv));
                    }
                    writeInt(v);
                }
                continue;
            }

            if (blk.type == HmiValType::BoolRead || blk.type == HmiValType::BoolRw) {
                double x = 0;
                bool b = false;

                bool ok_num = toNumber(*node, x);
                bool ok_bool = false;
                if (!ok_num) ok_bool = toBool(*node, b);

                if (ok_num) {
                    if (!evalCompare(x, it, b)) b = (x != 0.0);
                } else if (!ok_bool) {
                    b = defv.has_value() ? (*defv != 0.0) : false;
                }
                writeBool(b);
                continue;
            }

            if (blk.type == HmiValType::IntRead || blk.type == HmiValType::IntRw ||
                blk.type == HmiValType::U16 || blk.type == HmiValType::S16) {
                double x = 0;
                if (!toNumber(*node, x)) {
                    uint16_t v = 0;
                    if (defv.has_value()) {
                        if (blk.type == HmiValType::S16) v = packS16(static_cast<int32_t>(*defv));
                        else v = clampU16(static_cast<int64_t>(*defv));
                    }
                    writeInt(v);
                    continue;
                }

                const double scale  = pickOpt(it.scale,  blk.scale,  1.0);
                const double offset = pickOpt(it.offset, blk.offset, 0.0);

                double y = x * scale + offset;

                const auto minv = it.minv.has_value() ? it.minv : blk.minv;
                const auto maxv = it.maxv.has_value() ? it.maxv : blk.maxv;
                if (minv.has_value() && y < *minv) y = *minv;
                if (maxv.has_value() && y > *maxv) y = *maxv;

                uint16_t v = 0;
                if (blk.type == HmiValType::S16) v = packS16(static_cast<int32_t>(y));
                else v = clampU16(static_cast<int64_t>(y));

                writeInt(v);
                continue;
            }

            if (blk.type == HmiValType::Bit) {
                double x = 0;
                if (!toNumber(*node, x)) {
                    bool b = defv.has_value() ? (*defv != 0.0) : false;
                    writeBool(b);
                    continue;
                }

                const int bit = it.bit_index.has_value()
                    ? *it.bit_index
                    : (blk.bit_index.has_value() ? *blk.bit_index : 0);

                const uint32_t u = static_cast<uint32_t>(static_cast<int64_t>(x));
                const bool b = ((u >> bit) & 0x1u) != 0;
                writeBool(b);
                continue;
            }
        }
    }
}

} // namespace normal