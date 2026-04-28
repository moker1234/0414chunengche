// services/normal/normal_hmi_writer.cpp
#include "normal_hmi_writer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "logger.h"
#include "../protocol/rs485/hmi/hmi_proto.h"

using nlohmann::json;

namespace normal {

namespace {

static bool startsWith_(const std::string& s, const char* prefix)
{
    if (!prefix) return false;
    const std::string p(prefix);
    return s.rfind(p, 0) == 0;
}

static bool isEmptyOrPlaceholderPath_(const std::string& path)
{
    return path.empty() || path == "_";
}

static bool isReservedName_(const std::string& name)
{
    return name == "预留" || name.empty();
}

static bool isFaultOwnedPath_(const std::string& path)
{
    /*
     * hmi.fault.* 是故障页专属输出域。
     *
     * normal_map_logic.jsonl 可以保留这些 path，
     * 但 NormalHmiWriter 不能写这些地址。
     *
     * 故障页由 FaultCenter / FaultPageManager 写，
     * 否则会造成故障页数、故障代码、时间等被普通 writer 清零或闪烁。
     */
    return path.rfind("hmi.fault.", 0) == 0;
}

static const char* hmiValTypeName_(HmiValType t)
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

static std::string addrHex4_(uint16_t addr)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(addr));
    return std::string(buf);
}

    static bool endsWith_(const std::string& s, const char* suffix)
{
    if (!suffix) return false;

    const std::string t(suffix);
    if (s.size() < t.size()) return false;

    return s.compare(s.size() - t.size(), t.size(), t) == 0;
}

static std::string lowerAscii_(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

static bool valueTypeContains_(const HmiMapItem& item, const char* key)
{
    if (!key) return false;

    const std::string vt = lowerAscii_(item.value_type);
    const std::string kk = lowerAscii_(key);

    return vt.find(kk) != std::string::npos;
}

static bool isSignedMapItem_(const HmiMapItem& item, HmiValType legacy_type)
{
    if (legacy_type == HmiValType::S16) return true;

    const std::string vt = lowerAscii_(item.value_type);

    if (vt.find("uint") != std::string::npos) return false;
    if (vt.find("udint") != std::string::npos) return false;
    if (vt.find("u16") != std::string::npos) return false;
    if (vt.find("u32") != std::string::npos) return false;

    return vt.find("int16") != std::string::npos ||
           vt.find("s16")   != std::string::npos ||
           vt.find("int32") != std::string::npos ||
           vt.find("s32")   != std::string::npos ||
           vt.find("dint")  != std::string::npos;
}

static bool isDwordMapItem_(const HmiMapItem& item)
{
    if (item.words >= 2) return true;

    return valueTypeContains_(item, "uint32") ||
           valueTypeContains_(item, "int32")  ||
           valueTypeContains_(item, "udint")  ||
           valueTypeContains_(item, "dint")   ||
           valueTypeContains_(item, "u32")    ||
           valueTypeContains_(item, "s32")    ||
           valueTypeContains_(item, "dword");
}

static int64_t roundToI64_(double x)
{
    if (!std::isfinite(x)) {
        return 0;
    }

    if (x > static_cast<double>(std::numeric_limits<int64_t>::max())) {
        return std::numeric_limits<int64_t>::max();
    }

    if (x < static_cast<double>(std::numeric_limits<int64_t>::min())) {
        return std::numeric_limits<int64_t>::min();
    }

    return static_cast<int64_t>(std::llround(x));
}

/*
 * normal_map_logic.jsonl 的“缩放”语义：
 *
 *   工程值 = HMI寄存器整数值 * scale + offset
 *
 * 所以写 HMI 时必须反算：
 *
 *   HMI寄存器整数值 = (工程值 - offset) / scale
 */
static int64_t engineeringToHmiRaw_(double engineering_value,
                                    const HmiMapItem& item)
{
    double v = std::isfinite(engineering_value) ? engineering_value : 0.0;

    // min/max 按工程值理解，先夹工程值，再反算寄存器整数。
    if (item.minv.has_value() && v < *item.minv) {
        v = *item.minv;
    }

    if (item.maxv.has_value() && v > *item.maxv) {
        v = *item.maxv;
    }

    double scale = item.scale.value_or(1.0);
    if (!std::isfinite(scale) || std::fabs(scale) < 1e-12) {
        scale = 1.0;
    }

    const double offset = item.offset.value_or(0.0);
    const double raw = (v - offset) / scale;

    return roundToI64_(raw);
}

static uint16_t packRawU16_(int64_t raw)
{
    if (raw < 0) return 0;
    if (raw > 65535) return 65535;

    return static_cast<uint16_t>(raw);
}

static uint16_t packRawS16_(int64_t raw)
{
    if (raw < -32768) raw = -32768;
    if (raw >  32767) raw =  32767;

    const int32_t s = static_cast<int32_t>(raw);
    return static_cast<uint16_t>(s & 0xFFFF);
}

static void writeU32ToHmi_(HMIProto& hmi,
                           uint16_t addr,
                           uint32_t value)
{
    const uint16_t hi =
        static_cast<uint16_t>((value >> 16) & 0xFFFFu);

    const uint16_t lo =
        static_cast<uint16_t>(value & 0xFFFFu);

    hmi.setIntRead(addr, hi);
    hmi.setIntRead(static_cast<uint16_t>(addr + 1), lo);
}

static void writeS32ToHmi_(HMIProto& hmi,
                           uint16_t addr,
                           int32_t value)
{
    const uint32_t u = static_cast<uint32_t>(value);
    writeU32ToHmi_(hmi, addr, u);
}

static bool isKnownNormalMapPathPrefix_(const std::string& path)
{
    if (isEmptyOrPlaceholderPath_(path)) return true;

    return startsWith_(path, "system.logic_view.") ||
           startsWith_(path, "hmi.ctrl.btn.") ||
           startsWith_(path, "hmi.fault.btn.") ||
           startsWith_(path, "hmi.fault.num.") ||
           startsWith_(path, "hmi.fault.bool.") ||
           startsWith_(path, "hmi.param.");
}

static std::string itemLabel_(const HmiMapItem& item)
{
    std::ostringstream oss;
    oss << hmiValTypeName_(NormalHmiWriter::toLegacyType_(item.type))
        << " "
        << (item.has_addr ? addrHex4_(item.addr) : std::string("<no_addr>"))
        << " name=" << item.name
        << " path=" << item.path;
    return oss.str();
}

struct NormalMapCheckStats
{
    size_t blocks{0};
    size_t items{0};

    size_t bool_read{0};
    size_t bool_rw{0};
    size_t int_read{0};
    size_t int_rw{0};
    size_t legacy{0};

    size_t explicit_addr{0};
    size_t bad_addr{0};

    size_t empty_path{0};
    size_t nonempty_path{0};
    size_t empty_non_reserved{0};

    size_t system_logic_view{0};
    size_t hmi_ctrl_btn{0};
    size_t hmi_fault_btn{0};
    size_t hmi_fault_output{0};
    size_t hmi_param{0};
    size_t unknown_prefix{0};

    size_t rw_items{0};
    size_t rw_with_path{0};
    size_t rw_without_path{0};

    size_t read_items{0};
    size_t read_with_path{0};
    size_t read_without_path{0};

    size_t fault_owned_skipped_by_writer{0};

    size_t words_gt1{0};
    size_t missing_scale_on_int_read{0};

    size_t duplicate_addr_count{0};
};

static void incTypeStats_(NormalMapCheckStats& st, HmiValType type)
{
    switch (type) {
    case HmiValType::BoolRead: ++st.bool_read; break;
    case HmiValType::BoolRw:   ++st.bool_rw;   break;
    case HmiValType::IntRead:  ++st.int_read;  break;
    case HmiValType::IntRw:    ++st.int_rw;    break;
    default:                   ++st.legacy;    break;
    }
}

static void dumpNormalMapSelfCheck_(const HmiMapModel& map,
                                    const std::string& file_path)
{
    NormalMapCheckStats st{};
    st.blocks = map.blocks.size();
    st.items = map.items.size();

    std::map<std::string, std::vector<std::string>> addr_owner;
    std::vector<std::string> duplicate_examples;
    std::vector<std::string> unknown_prefix_examples;
    std::vector<std::string> empty_non_reserved_examples;
    std::vector<std::string> words_gt1_examples;
    std::vector<std::string> int_no_scale_examples;

    auto keepExample = [](std::vector<std::string>& v, const std::string& s) {
        if (v.size() < 10) v.push_back(s);
    };

    for (const auto& item : map.items)
    {
        const HmiValType legacy_type = NormalHmiWriter::toLegacyType_(item.type);
        incTypeStats_(st, legacy_type);

        const bool rw_type = NormalHmiWriter::isRwType(legacy_type);
        const bool read_type = !rw_type;

        if (!item.has_addr) {
            ++st.bad_addr;
            continue;
        }

        ++st.explicit_addr;

        const std::string key =
            std::string(hmiValTypeName_(legacy_type)) + ":" + addrHex4_(item.addr);

        addr_owner[key].push_back(itemLabel_(item));

        const bool empty_path = isEmptyOrPlaceholderPath_(item.path);
        if (empty_path) {
            ++st.empty_path;

            if (!isReservedName_(item.name)) {
                ++st.empty_non_reserved;
                keepExample(empty_non_reserved_examples, itemLabel_(item));
            }
        } else {
            ++st.nonempty_path;
        }

        if (rw_type) {
            ++st.rw_items;
            if (empty_path) ++st.rw_without_path;
            else ++st.rw_with_path;
        } else {
            ++st.read_items;
            if (empty_path) ++st.read_without_path;
            else ++st.read_with_path;
        }

        if (!empty_path)
        {
            if (startsWith_(item.path, "system.logic_view.")) {
                ++st.system_logic_view;
            }
            else if (startsWith_(item.path, "hmi.ctrl.btn.")) {
                ++st.hmi_ctrl_btn;
            }
            else if (startsWith_(item.path, "hmi.fault.btn.")) {
                ++st.hmi_fault_btn;
            }
            else if (startsWith_(item.path, "hmi.fault.num.") ||
                     startsWith_(item.path, "hmi.fault.bool.")) {
                ++st.hmi_fault_output;
            }
            else if (startsWith_(item.path, "hmi.param.")) {
                ++st.hmi_param;
            }
            else {
                ++st.unknown_prefix;
                keepExample(unknown_prefix_examples, itemLabel_(item));
            }

            if (isFaultOwnedPath_(item.path)) {
                ++st.fault_owned_skipped_by_writer;
            }

            if (!isKnownNormalMapPathPrefix_(item.path)) {
                keepExample(unknown_prefix_examples, itemLabel_(item));
            }
        }

        if (item.words > 1) {
            ++st.words_gt1;
            keepExample(words_gt1_examples,
                        itemLabel_(item) + " words=" + std::to_string(item.words));
        }

        if ((legacy_type == HmiValType::IntRead ||
             legacy_type == HmiValType::IntRw ||
             legacy_type == HmiValType::U16 ||
             legacy_type == HmiValType::S16) &&
            !empty_path &&
            !item.scale.has_value())
        {
            ++st.missing_scale_on_int_read;
            keepExample(int_no_scale_examples, itemLabel_(item));
        }

        (void)read_type;
    }

    for (const auto& kv : addr_owner)
    {
        if (kv.second.size() <= 1) continue;

        st.duplicate_addr_count += kv.second.size();

        if (duplicate_examples.size() < 10) {
            std::ostringstream oss;
            oss << kv.first << " owners=" << kv.second.size();
            for (const auto& x : kv.second) {
                oss << " | " << x;
            }
            duplicate_examples.push_back(oss.str());
        }
    }

    LOG_SYS_I("[NORMAL][MAP][CHECK] file=%s blocks=%zu items=%zu normal=%zu fault_num=%zu fault_btn=%zu rw=%zu bad_addr=%zu",
              file_path.c_str(),
              st.blocks,
              st.items,
              map.normal_items.size(),
              map.fault_num_items.size(),
              map.fault_button_items.size(),
              map.rw_items.size(),
              st.bad_addr);

    LOG_SYS_I("[NORMAL][MAP][CHECK] type bool_read=%zu bool_rw=%zu int_read=%zu int_rw=%zu legacy=%zu",
              st.bool_read,
              st.bool_rw,
              st.int_read,
              st.int_rw,
              st.legacy);

    LOG_SYS_I("[NORMAL][MAP][CHECK] path empty=%zu nonempty=%zu empty_non_reserved=%zu unknown_prefix=%zu",
              st.empty_path,
              st.nonempty_path,
              st.empty_non_reserved,
              st.unknown_prefix);

    LOG_SYS_I("[NORMAL][MAP][CHECK] path_dist logic_view=%zu hmi_ctrl_btn=%zu hmi_fault_btn=%zu hmi_fault_output=%zu hmi_param=%zu",
              st.system_logic_view,
              st.hmi_ctrl_btn,
              st.hmi_fault_btn,
              st.hmi_fault_output,
              st.hmi_param);

    LOG_SYS_I("[NORMAL][MAP][CHECK] rw total=%zu with_path=%zu without_path=%zu read total=%zu with_path=%zu without_path=%zu",
              st.rw_items,
              st.rw_with_path,
              st.rw_without_path,
              st.read_items,
              st.read_with_path,
              st.read_without_path);

    LOG_SYS_I("[NORMAL][MAP][CHECK] writer_skip fault_owned=%zu words_gt1=%zu int_missing_scale=%zu duplicate_addr_entries=%zu",
              st.fault_owned_skipped_by_writer,
              st.words_gt1,
              st.missing_scale_on_int_read,
              st.duplicate_addr_count);

    for (std::size_t i = 0; i < duplicate_examples.size(); ++i) {
        LOG_SYS_W("[NORMAL][MAP][DUP_ADDR][%zu] %s",
                  i,
                  duplicate_examples[i].c_str());
    }

    for (std::size_t i = 0; i < unknown_prefix_examples.size(); ++i) {
        LOG_SYS_W("[NORMAL][MAP][UNKNOWN_PATH][%zu] %s",
                  i,
                  unknown_prefix_examples[i].c_str());
    }

    for (std::size_t i = 0; i < empty_non_reserved_examples.size(); ++i) {
        LOG_SYS_W("[NORMAL][MAP][EMPTY_PATH_NON_RESERVED][%zu] %s",
                  i,
                  empty_non_reserved_examples[i].c_str());
    }

    for (std::size_t i = 0; i < words_gt1_examples.size(); ++i) {
        LOG_SYS_I("[NORMAL][MAP][WORDS_GT1][%zu] %s",
                  i,
                  words_gt1_examples[i].c_str());
    }

    for (std::size_t i = 0; i < int_no_scale_examples.size(); ++i) {
        LOG_SYS_W("[NORMAL][MAP][INT_NO_SCALE][%zu] %s",
                  i,
                  int_no_scale_examples[i].c_str());
    }
}

static const nlohmann::json* resolvePathCompatLocal_(const nlohmann::json& root,
                                                     const std::string& path)
{
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

    for (std::size_t i = 0; i < tokens.size(); ++i) {
        if (!node->is_object()) return nullptr;

        const auto& tk = tokens[i];

        auto it = node->find(tk);
        if (it != node->end()) {
            node = &(*it);
            continue;
        }

        // compat: join remaining tokens as one key
        std::string joined = tk;
        for (std::size_t k = i + 1; k < tokens.size(); ++k) {
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

static void dumpNormalMapPathRuntimeCheck_(const HmiMapModel& map,
                                           const nlohmann::json& model_root)
{
    size_t total_system_logic_view = 0;
    size_t found_system_logic_view = 0;
    size_t missing_system_logic_view = 0;

    std::vector<std::string> missing_examples;

    auto keepExample = [](std::vector<std::string>& v, const std::string& s) {
        if (v.size() < 30) v.push_back(s);
    };

    for (std::size_t idx : map.normal_items)
    {
        const HmiMapItem* item = map.itemAt(idx);
        if (!item) continue;

        if (!startsWith_(item->path, "system.logic_view.")) {
            continue;
        }

        ++total_system_logic_view;

        if (!item->has_addr) {
            ++missing_system_logic_view;
            keepExample(missing_examples,
                        std::string(hmiValTypeName_(NormalHmiWriter::toLegacyType_(item->type))) +
                        " bad_addr name=" + item->name +
                        " path=" + item->path);
            continue;
        }

        const nlohmann::json* node =
            resolvePathCompatLocal_(model_root, item->path);

        if (node) {
            ++found_system_logic_view;
        } else {
            ++missing_system_logic_view;
            keepExample(missing_examples, itemLabel_(*item));
        }
    }

    LOG_SYS_I("[NORMAL][MAP][PATH_CHECK] system.logic_view total=%zu found=%zu missing=%zu",
              total_system_logic_view,
              found_system_logic_view,
              missing_system_logic_view);

    for (std::size_t i = 0; i < missing_examples.size(); ++i) {
        LOG_SYS_W("[NORMAL][MAP][PATH_MISSING][%zu] %s",
                  i,
                  missing_examples[i].c_str());
    }

    if (missing_system_logic_view > 0) {
        LOG_SYS_W("[NORMAL][MAP][PATH_CHECK] missing system.logic_view path count=%zu, please check logic_view_builder/bms_logic_adapter/jsonl path",
                  missing_system_logic_view);
    }
}

} // namespace

HmiValType NormalHmiWriter::toLegacyType_(HmiMapType t)
{
    switch (t) {
    case HmiMapType::BoolRead: return HmiValType::BoolRead;
    case HmiMapType::BoolRw:   return HmiValType::BoolRw;
    case HmiMapType::IntRead:  return HmiValType::IntRead;
    case HmiMapType::IntRw:    return HmiValType::IntRw;
    case HmiMapType::U16:      return HmiValType::U16;
    case HmiMapType::S16:      return HmiValType::S16;
    case HmiMapType::Bit:      return HmiValType::Bit;
    default:                   return HmiValType::U16;
    }
}

bool NormalHmiWriter::bindMap(std::shared_ptr<const HmiMapModel> model,
                              std::string* err)
{
    if (!model) {
        if (err) *err = "null HmiMapModel";
        loaded_ = false;
        hmi_map_.reset();
        first_flush_path_check_done_ = false;
        return false;
    }

    if (model->items.empty()) {
        if (err) *err = "empty HmiMapModel";
        loaded_ = false;
        hmi_map_.reset();
        first_flush_path_check_done_ = false;
        return false;
    }

    hmi_map_ = std::move(model);
    loaded_ = true;
    first_flush_path_check_done_ = false;

    LOG_SYS_I("[NORMAL][MAP] bound model blocks=%zu items=%zu normal=%zu fault_num=%zu fault_btn=%zu rw=%zu",
              hmi_map_->blocks.size(),
              hmi_map_->items.size(),
              hmi_map_->normal_items.size(),
              hmi_map_->fault_num_items.size(),
              hmi_map_->fault_button_items.size(),
              hmi_map_->rw_items.size());

    return true;
}

const json* NormalHmiWriter::resolvePathCompat(const json& root,
                                               const std::string& path)
{
    return resolvePathCompatLocal_(root, path);
}

bool NormalHmiWriter::toNumber(const json& v, double& out)
{
    if (v.is_number()) {
        out = v.get<double>();
        return true;
    }

    if (v.is_boolean()) {
        out = v.get<bool>() ? 1.0 : 0.0;
        return true;
    }

    return false;
}

bool NormalHmiWriter::toBool(const json& v, bool& out)
{
    if (v.is_boolean()) {
        out = v.get<bool>();
        return true;
    }

    if (v.is_number()) {
        out = (v.get<double>() != 0.0);
        return true;
    }

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

double NormalHmiWriter::pickOpt(const std::optional<double>& item,
                                const std::optional<double>& blk,
                                double defv)
{
    if (item.has_value()) return *item;
    if (blk.has_value()) return *blk;
    return defv;
}

bool NormalHmiWriter::evalCompare(double x,
                                  const HmiMapItem& it,
                                  bool& out)
{
    if (it.eq.has_value()) {
        out = (x == *it.eq);
        return true;
    }

    bool has = false;
    bool ok = true;

    if (it.gt.has_value()) {
        has = true;
        ok = ok && (x > *it.gt);
    }

    if (it.ge.has_value()) {
        has = true;
        ok = ok && (x >= *it.ge);
    }

    if (it.lt.has_value()) {
        has = true;
        ok = ok && (x < *it.lt);
    }

    if (it.le.has_value()) {
        has = true;
        ok = ok && (x <= *it.le);
    }

    if (has) {
        out = ok;
        return true;
    }

    return false;
}

bool NormalHmiWriter::isBoolType(HmiValType t)
{
    return t == HmiValType::BoolRead ||
           t == HmiValType::BoolRw ||
           t == HmiValType::Bit;
}

bool NormalHmiWriter::isIntType(HmiValType t)
{
    return t == HmiValType::IntRead ||
           t == HmiValType::IntRw ||
           t == HmiValType::U16 ||
           t == HmiValType::S16;
}

bool NormalHmiWriter::isRwType(HmiValType t)
{
    return t == HmiValType::BoolRw ||
           t == HmiValType::IntRw;
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
    if (getBool("gas_alarm"))     bits |= static_cast<uint16_t>(1u << 0);
    if (getBool("smoke_alarm"))   bits |= static_cast<uint16_t>(1u << 1);
    if (getBool("ac_alarm"))      bits |= static_cast<uint16_t>(1u << 2);
    if (getBool("bms_alarm_any")) bits |= static_cast<uint16_t>(1u << 3);

    sys["alarm_bits"] = bits;

    // system.normal_seq：每次 flush++（心跳/版本号）
    sys["normal_seq"] = static_cast<uint32_t>(++seq_);
}

void NormalHmiWriter::flushFromModel(const agg::SystemSnapshot& snap,
                                     const nlohmann::json& logic_view,
                                     HMIProto& hmi) const
{
    if (!loaded() || !hmi_map_) {
        return;
    }

    json j = snap.toJson();

    // 把逻辑显示视图挂到 system.logic_view.*
    if (!j.is_object()) {
        j = json::object();
    }

    if (!j.contains("system") || !j["system"].is_object()) {
        j["system"] = json::object();
    }

    j["system"]["logic_view"] =
        logic_view.is_object() ? logic_view : json::object();

    // 注入派生字段：alarm_bits + normal_seq
    injectDerivedFields(j);

    // 第一次实际 flush 时检查 system.logic_view.* 是否真实存在
    if (!first_flush_path_check_done_) {
        first_flush_path_check_done_ = true;
        dumpNormalMapPathRuntimeCheck_(*hmi_map_, j);
    }

    auto writeBool = [&](uint16_t addr, bool b) {
        hmi.setBoolRead(addr, b);
    };

    auto writeInt16ByMap = [&](uint16_t addr,
                               const HmiMapItem& item,
                               HmiValType legacy_type,
                               int64_t raw) {
        if (isSignedMapItem_(item, legacy_type)) {
            hmi.setIntRead(addr, packRawS16_(raw));
        } else {
            hmi.setIntRead(addr, packRawU16_(raw));
        }
    };

    /*
     * 兼容当前 UPS 备用时间的既有写法：
     *
     * normal_map_logic.jsonl 里可能写：
     *   path = system.logic_view.ups_battery_remain_sec_hi
     *   words = 2
     *
     * 这种情况下，当前 path 只指向高 16 位；
     * 这里自动查同名前缀的 _lo，一次写 addr 和 addr+1。
     *
     * 长期更推荐：
     *   path = system.logic_view.ups_battery_remain_sec
     *   words = 2
     */
    auto tryWriteHiLoPair = [&](const HmiMapItem& item,
                                uint16_t addr,
                                const json* hi_node) -> bool {
        if (item.words < 2) {
            return false;
        }

        if (!endsWith_(item.path, "_hi")) {
            return false;
        }

        if (!hi_node) {
            return false;
        }

        const std::string base_path =
            item.path.substr(0, item.path.size() - 3);

        const std::string lo_path = base_path + "_lo";
        const json* lo_node = resolvePathCompat(j, lo_path);

        double hi_d = 0.0;
        double lo_d = 0.0;

        if (!lo_node || !toNumber(*hi_node, hi_d) || !toNumber(*lo_node, lo_d)) {
            return false;
        }

        const uint16_t hi =
            packRawU16_(roundToI64_(hi_d));

        const uint16_t lo =
            packRawU16_(roundToI64_(lo_d));

        hmi.setIntRead(addr, hi);
        hmi.setIntRead(static_cast<uint16_t>(addr + 1), lo);

        return true;
    };

    for (std::size_t idx : hmi_map_->normal_items) {
        const HmiMapItem* it = hmi_map_->itemAt(idx);
        if (!it) {
            continue;
        }

        if (!it->has_addr) {
            continue;
        }

        if (it->path == "_" || it->path.empty()) {
            continue;
        }

        if (isFaultOwnedPath_(it->path)) {
            continue;
        }

        const HmiValType legacy_type = toLegacyType_(it->type);
        if (isRwType(legacy_type)) {
            continue;
        }

        const uint16_t addr = it->addr;

        const json* node = nullptr;
        if (legacy_type == HmiValType::Bit && !it->src.empty()) {
            node = resolvePathCompat(j, it->src);
        } else {
            node = resolvePathCompat(j, it->path);
        }

        const auto defv = it->def;

        if (!node) {
            if (isBoolType(legacy_type)) {
                const bool b = defv.has_value()
                    ? (*defv != 0.0)
                    : false;

                writeBool(addr, b);
            }
            else if (isIntType(legacy_type)) {
                const double eng = defv.value_or(0.0);
                const int64_t raw = engineeringToHmiRaw_(eng, *it);

                if (isDwordMapItem_(*it)) {
                    if (isSignedMapItem_(*it, legacy_type)) {
                        writeS32ToHmi_(hmi, addr, static_cast<int32_t>(raw));
                    } else {
                        const uint32_t u =
                            raw <= 0 ? 0u :
                            raw >= static_cast<int64_t>(0xFFFFFFFFull) ? 0xFFFFFFFFu :
                            static_cast<uint32_t>(raw);

                        writeU32ToHmi_(hmi, addr, u);
                    }
                } else {
                    writeInt16ByMap(addr, *it, legacy_type, raw);
                }
            }

            continue;
        }

        if (legacy_type == HmiValType::BoolRead ||
            legacy_type == HmiValType::BoolRw) {
            double x = 0.0;
            bool b = false;

            const bool ok_num = toNumber(*node, x);
            bool ok_bool = false;

            if (!ok_num) {
                ok_bool = toBool(*node, b);
            }

            if (ok_num) {
                if (!evalCompare(x, *it, b)) {
                    b = (x != 0.0);
                }
            }
            else if (!ok_bool) {
                b = defv.has_value()
                    ? (*defv != 0.0)
                    : false;
            }

            writeBool(addr, b);
            continue;
        }

        if (legacy_type == HmiValType::IntRead ||
            legacy_type == HmiValType::IntRw ||
            legacy_type == HmiValType::U16 ||
            legacy_type == HmiValType::S16) {
            double engineering_value = 0.0;

            if (!toNumber(*node, engineering_value)) {
                engineering_value = defv.value_or(0.0);
            }

            // UPS 等旧 hi/lo 拆分字段：path 指向 _hi 且 words=2 时，自动补写 _lo。
            if (tryWriteHiLoPair(*it, addr, node)) {
                continue;
            }

            const int64_t raw = engineeringToHmiRaw_(engineering_value, *it);

            if (isDwordMapItem_(*it)) {
                if (isSignedMapItem_(*it, legacy_type)) {
                    int64_t s = raw;
                    if (s < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
                        s = static_cast<int64_t>(std::numeric_limits<int32_t>::min());
                    }
                    if (s > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
                        s = static_cast<int64_t>(std::numeric_limits<int32_t>::max());
                    }

                    writeS32ToHmi_(hmi, addr, static_cast<int32_t>(s));
                } else {
                    uint32_t u = 0;

                    if (raw <= 0) {
                        u = 0;
                    } else if (raw >= static_cast<int64_t>(0xFFFFFFFFull)) {
                        u = 0xFFFFFFFFu;
                    } else {
                        u = static_cast<uint32_t>(raw);
                    }

                    writeU32ToHmi_(hmi, addr, u);
                }
            } else {
                writeInt16ByMap(addr, *it, legacy_type, raw);
            }

            continue;
        }

        if (legacy_type == HmiValType::Bit) {
            double x = 0.0;

            if (!toNumber(*node, x)) {
                const bool b = defv.has_value()
                    ? (*defv != 0.0)
                    : false;

                writeBool(addr, b);
                continue;
            }

            const int bit = it->bit_index.has_value()
                ? *it->bit_index
                : 0;

            const uint32_t u =
                static_cast<uint32_t>(static_cast<int64_t>(x));

            const bool b = ((u >> bit) & 0x1u) != 0;

            writeBool(addr, b);
            continue;
        }
    }
}

} // namespace normal