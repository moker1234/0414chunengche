#include "hmi_snapshot_mapper.h"
#include <sstream>

#include "logger.h"

using nlohmann::json;

bool HmiSnapshotMapper::loadMapFile(const std::string& path, std::string* err) {
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

static std::vector<std::string> splitDot(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == '.') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

const nlohmann::json* HmiSnapshotMapper::resolvePathCompat(const nlohmann::json& root,
                                               const std::string& path) {
    // if (path.find("items.AirConditioner.data.warn_state.fields.alarm") != std::string::npos) {
    //     LOGD("[HMI_MAP][RESOLVE_HIT] path=%s", path.c_str());
    // }

    // 仍然用 '.' 分割
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

    const nlohmann::json* node = &root;

    for (size_t i = 0; i < tokens.size(); ++i) {
        if (!node->is_object()) return nullptr;

        const auto& tk = tokens[i];

        // 1) 正常按 token 查
        auto it = node->find(tk);
        if (it != node->end()) {
            node = &(*it);
            continue;
        }

        // 2) 兼容：把剩余 token 拼成一个 key 再试一次（用于 fields 下的 "alarm.xxx" 这种扁平 key）
        std::string joined = tk;
        for (size_t k = i + 1; k < tokens.size(); ++k) {
            joined.push_back('.');
            joined += tokens[k];
        }
        auto it2 = node->find(joined);
        if (it2 != node->end()) {
            node = &(*it2);
            return node; // joined 一次吃掉剩余所有 token
        }

        return nullptr;
    }

    return node;
}


bool HmiSnapshotMapper::toNumber(const json& v, double& out) {
    if (v.is_number()) { out = v.get<double>(); return true; }
    if (v.is_boolean()) { out = v.get<bool>() ? 1.0 : 0.0; return true; }
    return false;
}

bool HmiSnapshotMapper::toBool(const json& v, bool& out) {
    if (v.is_boolean()) { out = v.get<bool>(); return true; }
    if (v.is_number()) { out = (v.get<double>() != 0.0); return true; }
    return false;
}

uint16_t HmiSnapshotMapper::clampU16(int64_t v) {
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<uint16_t>(v);
}

uint16_t HmiSnapshotMapper::packS16(int32_t v) {
    // two's complement
    return static_cast<uint16_t>(v & 0xFFFF);
}

double HmiSnapshotMapper::pickOpt(const std::optional<double>& item, const std::optional<double>& blk, double defv) {
    if (item.has_value()) return *item;
    if (blk.has_value()) return *blk;
    return defv;
}

std::optional<double> HmiSnapshotMapper::pickOpt2(const std::optional<double>& item, const std::optional<double>& blk) {
    if (item.has_value()) return item;
    if (blk.has_value()) return blk;
    return std::nullopt;
}

std::optional<int> HmiSnapshotMapper::pickOptInt2(const std::optional<int>& item, const std::optional<int>& blk) {
    if (item.has_value()) return item;
    if (blk.has_value()) return blk;
    return std::nullopt;
}

bool HmiSnapshotMapper::evalCompare(double x, const HmiPathItem& it, bool& out) {
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

static inline bool isBoolType(HmiValType t) {
    return t == HmiValType::BoolRead || t == HmiValType::BoolRw || t == HmiValType::Bit;
}

static inline bool isIntType(HmiValType t) {
    return t == HmiValType::IntRead || t == HmiValType::IntRw || t == HmiValType::U16 || t == HmiValType::S16;
}

static inline bool isRwType(HmiValType t) {
    return t == HmiValType::BoolRw || t == HmiValType::IntRw;
}
#include "getTime.h"
void HmiSnapshotMapper::onSnapshot(const agg::SystemSnapshot& snap) {
    // LOGD("[HMI_MAP][ENTER] onSnapshot ts=%llu", (unsigned long long)snap.timestamp_ms);

    if (!hmi_ || !loaded_) return;

    // 通过 toJson 统一取值（避免 C++ 反射麻烦）
    const json j = snap.toJson();

    // LOGD("[HMI_MAP] blocks=", map_.blocks.size());
    for (const auto& blk : map_.blocks) {
        const uint16_t base = blk.base;

        // Bit 模式：统一 src
        const json* bit_src_node = nullptr;
        if (blk.type == HmiValType::Bit && !blk.src.empty()) {
            bit_src_node = resolvePathCompat(j, blk.src);
        }

        for (size_t i = 0; i < blk.items.size(); ++i) {
            const auto& it = blk.items[i];
            const uint16_t addr = static_cast<uint16_t>(base + static_cast<uint16_t>(i));

            if (it.path == "_" || it.path.empty()) { // 预留占位
                continue;
            }

            // ===== 取节点 =====
            const json* node = nullptr;
            if (blk.type == HmiValType::Bit && bit_src_node) {
                node = bit_src_node;
            } else {
                node = resolvePathCompat(j, it.path);
            }

            // ===== 缺省值处理 =====
            auto defv = pickOpt2(it.def, blk.def);

            auto writeBool = [&](bool b) {
                // ✅ 默认不写 RW 区：避免屏幕写入后，被下一帧 snapshot 覆盖回旧值
                if (isRwType(blk.type)) return;
// 0207
                // if (addr == 0x119) {
                //     uint64_t now = nowMs(); /* 同 onSnapshot 内拿 nowMs/steady_clock */
                //     LOGD("[AC_LAT][T3_MAP] now=%llu addr=0x%04X write=%d",
                //          (unsigned long long)now, addr, (int)b);
                // }
                if (!isBlockedBoolRead_(addr)) {
                    hmi_->setBoolRead(addr, b);
                }
                // hmi_->setBoolRead(addr, b);
            };

            auto writeInt = [&](uint16_t v) {
                // ✅ 默认不写 RW 区：避免覆盖
                if (isRwType(blk.type)) return;
                // 0207
                // if (addr >= 0x4018 && addr <= 0x4041) {
                //     uint64_t now = nowMs(); /* 同 onSnapshot 内拿 nowMs/steady_clock */
                //     LOGD("[AC_LAT][T3_MAP] now=%llu addr=0x%04X write=%d",
                //          (unsigned long long)now, addr, (int)v);
                // }
                if (!isBlockedIntRead_(addr)) {
                    hmi_->setIntRead(addr, v);
                }
                // hmi_->setIntRead(addr, v);
            };


            if (!node) {
                if (isBoolType(blk.type)) {
                    bool b = false;
                    if (defv.has_value()) b = (*defv != 0.0);
                    // 0206
                    // LOGD("[HMI_MAP][MISS] base=0x%04X addr=0x%04X idx=%zu type=%d path=%s def=%s write=%d",
                    //      base, addr, i, (int)blk.type, it.path.c_str(),
                    //      defv.has_value() ? "Y" : "N", (int)b);
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

            // ===== 按类型写入 =====
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
                //0206
                // LOGD("[HMI_MAP][BOOL] base=0x%04X addr=0x%04X idx=%zu path=%s ok_num=%d x=%.3f ok_bool=%d write=%d",
                //      base, addr, i, it.path.c_str(), (int)ok_num, x, (int)ok_bool, (int)b);
                writeBool(b);
            }
            else if (isIntType(blk.type)) {
                double x = 0;
                if (!toNumber(*node, x)) {
                    x = defv.value_or(0.0);
                }

                // scale/offset
                const double scale  = pickOpt(it.scale,  blk.scale,  1.0);
                const double offset = pickOpt(it.offset, blk.offset, 0.0);

                double y = x * scale + offset;

                // clamp
                auto mn = pickOpt2(it.minv, blk.minv);
                auto mx = pickOpt2(it.maxv, blk.maxv);
                if (mn.has_value() && y < *mn) y = *mn;
                if (mx.has_value() && y > *mx) y = *mx;

                // round
                int64_t r = (y >= 0.0) ? static_cast<int64_t>(y + 0.5) : static_cast<int64_t>(y - 0.5);

                uint16_t outv = 0;
                if (blk.type == HmiValType::S16) outv = packS16(static_cast<int32_t>(r));
                else outv = clampU16(r);

                writeInt(outv);
            }
            else if (blk.type == HmiValType::Bit) {
                // Bit：从整数取 bit -> bool
                double x = 0;
                if (!toNumber(*node, x)) x = defv.value_or(0.0);

                const int bit = pickOptInt2(it.bit_index, blk.bit_index).value_or(0);
                const uint32_t ux = static_cast<uint32_t>(static_cast<int64_t>(x));

                bool b = ((ux >> bit) & 0x1u) != 0;

                bool b2 = false;
                if (evalCompare(x, it, b2)) b = b2;

                writeBool(b);
            }
        }
    }
}




// services/snapshot/display/hmi_snapshot_mapper.cpp

void HmiSnapshotMapper::addBlockedIntReadRange(uint16_t start, uint16_t end)
{
    if (end < start) std::swap(start, end);
    blocked_int_read_.push_back({start, end});
}

void HmiSnapshotMapper::addBlockedBoolReadRange(uint16_t start, uint16_t end)
{
    if (end < start) std::swap(start, end);
    blocked_bool_read_.push_back({start, end});
}

void HmiSnapshotMapper::clearBlockedRanges()
{
    blocked_int_read_.clear();
    blocked_bool_read_.clear();
}

bool HmiSnapshotMapper::isBlockedIntRead_(uint16_t addr) const
{
    for (const auto& r : blocked_int_read_) {
        if (addr >= r.start && addr <= r.end) return true;
    }
    return false;
}

bool HmiSnapshotMapper::isBlockedBoolRead_(uint16_t addr) const
{
    for (const auto& r : blocked_bool_read_) {
        if (addr >= r.start && addr <= r.end) return true;
    }
    return false;
}
