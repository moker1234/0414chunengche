//
// Created by lxy on 2026/3/4.
//

#ifndef ENERGYSTORAGE_NORMAL_HMI_WRITER_H
#define ENERGYSTORAGE_NORMAL_HMI_WRITER_H

// services/normal/normal_hmi_writer.h
#pragma once

#include <string>
#include <optional>
#include <cstdint>
#include <nlohmann/json.hpp>

#include "../aggregator/system_snapshot.h"
#include "../snapshot/display/hmi_display_map.h"

class HMIProto;

namespace normal {

    class NormalHmiWriter {
    public:
        NormalHmiWriter() = default;

        bool loadMapFile(const std::string& path, std::string* err = nullptr);

        void flushFromSnapshot(const agg::SystemSnapshot& snap, HMIProto& hmi) const;

        // 新接口：snapshot + logic_view 覆盖
        void flushFromModel(const agg::SystemSnapshot& snap,
                            const nlohmann::json& logic_view,
                            HMIProto& hmi) const;

        bool loaded() const { return loaded_; }

    private:
        static const nlohmann::json* resolvePathCompat(const nlohmann::json& root, const std::string& path);

        static bool toNumber(const nlohmann::json& v, double& out);
        static bool toBool(const nlohmann::json& v, bool& out);

        static uint16_t clampU16(int64_t v);
        static uint16_t packS16(int32_t v);

        static double pickOpt(const std::optional<double>& item, const std::optional<double>& blk, double defv);

        static bool evalCompare(double x, const HmiPathItem& it, bool& out);

        static bool isBoolType(HmiValType t);
        static bool isIntType(HmiValType t);
        static bool isRwType(HmiValType t);

        // ✅ 注入派生字段（不改 snapshot 本体）
        // - system.alarm_bits：bit0..bit3
        // - system.normal_seq：每次 flush++（心跳/版本）
        void injectDerivedFields(nlohmann::json& j) const;

    private:
        bool loaded_{false};
        HmiDisplayMapLoader loader_;
        HmiDisplayMap map_;

        // ✅ 心跳序号：在 ControlLoop 单线程里递增即可，不需要 atomic
        mutable uint32_t seq_{0};
    };

} // namespace normal
#endif //ENERGYSTORAGE_NORMAL_HMI_WRITER_H