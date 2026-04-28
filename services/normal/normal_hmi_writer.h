// services/normal/normal_hmi_writer.h
#pragma once

#ifndef ENERGYSTORAGE_NORMAL_HMI_WRITER_H
#define ENERGYSTORAGE_NORMAL_HMI_WRITER_H

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "../aggregator/system_snapshot.h"
#include "../snapshot/display/hmi_display_map.h"   // 保留 HmiValType，兼容 logic_hmi_input.cpp
#include "hmi_map_model.h"

class HMIProto;

namespace normal {

class NormalHmiWriter {
public:
    NormalHmiWriter() = default;

    /*
     * 第六批后唯一装配入口：
     * HmiMapModel 由 ControlLoop::loadHmiMapFile() 统一加载。
     */
    bool bindMap(std::shared_ptr<const HmiMapModel> model,
                 std::string* err = nullptr);


    // snapshot + logic_view -> HMIProto 地址表
    void flushFromModel(const agg::SystemSnapshot& snap,
                        const nlohmann::json& logic_view,
                        HMIProto& hmi) const;

    bool loaded() const
    {
        return loaded_ && static_cast<bool>(hmi_map_);
    }

    const std::shared_ptr<const HmiMapModel>& mapModel() const
    {
        return hmi_map_;
    }

    static bool isRwType(HmiValType t);

    /*
     * 注意：
     * 这个函数被 normal_hmi_writer.cpp 匿名 namespace 里的自检辅助函数调用，
     * 因此必须是 public。
     */
    static HmiValType toLegacyType_(HmiMapType t);

private:
    static const nlohmann::json* resolvePathCompat(const nlohmann::json& root,
                                                   const std::string& path);

    static bool toNumber(const nlohmann::json& v, double& out);
    static bool toBool(const nlohmann::json& v, bool& out);

    static uint16_t clampU16(int64_t v);
    static uint16_t packS16(int32_t v);

    static double pickOpt(const std::optional<double>& item,
                          const std::optional<double>& blk,
                          double defv);

    static bool evalCompare(double x, const HmiMapItem& it, bool& out);

    static bool isBoolType(HmiValType t);
    static bool isIntType(HmiValType t);

    // 注入派生字段，不改 snapshot 本体
    void injectDerivedFields(nlohmann::json& j) const;

private:
    bool loaded_{false};

    // 第六批后只消费共享 HmiMapModel，不再自己解析 normal_map_logic.jsonl。
    std::shared_ptr<const HmiMapModel> hmi_map_;

    // 心跳序号：在 ControlLoop 单线程里递增即可
    mutable uint32_t seq_{0};

    /*
     * 第一次 flushFromModel() 做 path 真源自检。
     * flushFromModel() 是 const 函数，所以这里必须 mutable。
     */
    mutable bool first_flush_path_check_done_{false};
};

} // namespace normal

#endif // ENERGYSTORAGE_NORMAL_HMI_WRITER_H