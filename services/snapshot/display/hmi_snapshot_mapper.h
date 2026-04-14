#pragma once
#include <string>
#include <vector>
#include <optional>
#include <nlohmann/json.hpp>

#include "../aggregator/system_snapshot.h"
#include "hmi_display_map.h"
#include "hmi/hmi_proto.h"


// namespace agg
// {
//     struct SystemSnapshot;
// }

// 把 SystemSnapshot 映射到 HMI 地址表（四区：bool_read/bool_rw/int_read/int_rw）
class HmiSnapshotMapper {
public:
    HmiSnapshotMapper() = default;

    void bind(HMIProto* hmi) { hmi_ = hmi; }
    bool ready() const { return hmi_ != nullptr && loaded_; }

    bool loadMapFile(const std::string& path, std::string* err = nullptr);

    // 每次快照更新调用
    void onSnapshot(const agg::SystemSnapshot& snap);


    struct RangeU16 {
        uint16_t start{0};
        uint16_t end{0}; // inclusive
    };

    // 屏蔽 int_read 写入地址段（例如 0x4018..0x401F）
    void addBlockedIntReadRange(uint16_t start, uint16_t end);

    // （可选）屏蔽 bool_read 写入地址段
    void addBlockedBoolReadRange(uint16_t start, uint16_t end);

    void clearBlockedRanges();

private:
    static const nlohmann::json* resolvePathCompat(const nlohmann::json& root, const std::string& path);

    static bool toNumber(const nlohmann::json& v, double& out);
    static bool toBool(const nlohmann::json& v, bool& out);

    static uint16_t clampU16(int64_t v);
    static uint16_t packS16(int32_t v);

    static double pickOpt(const std::optional<double>& item, const std::optional<double>& blk, double defv);
    static std::optional<double> pickOpt2(const std::optional<double>& item, const std::optional<double>& blk);
    static std::optional<int> pickOptInt2(const std::optional<int>& item, const std::optional<int>& blk);

    static bool evalCompare(double x, const HmiPathItem& it, bool& out);

    bool isBlockedIntRead_(uint16_t addr) const;
    bool isBlockedBoolRead_(uint16_t addr) const;
private:
    HMIProto* hmi_{nullptr};
    HmiDisplayMap map_;
    bool loaded_{false};

    HmiDisplayMapLoader loader_;

    std::vector<RangeU16> blocked_int_read_;
    std::vector<RangeU16> blocked_bool_read_;

};
