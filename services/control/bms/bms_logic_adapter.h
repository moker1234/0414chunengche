#ifndef ENERGYSTORAGE_CONTROL_BMS_LOGIC_ADAPTER_H
#define ENERGYSTORAGE_CONTROL_BMS_LOGIC_ADAPTER_H

#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>

#include "../../protocol/protocol_base.h"
#include "bms_logic_types.h"

namespace control::bms {

    /**
     * BmsLogicAdapter：
     * 1. 吃解析后的 BMS DeviceData
     * 2. 更新 control::bms::BmsLogicCache
     * 3. 把 cache 投影成 logic_view 子树
     *
     * 这一批只做“接入 control + 输出概览”，
     * 不在这里做命令决策 / 复杂保护状态机。
     */
    class BmsLogicAdapter {
    public:
        void onDeviceData(const DeviceData& d, uint64_t ts_ms, BmsLogicCache& cache) const;

        nlohmann::json buildLogicView(const BmsLogicCache& cache) const;

    private:
        static bool isBmsDevice_(const DeviceData& d);

        static uint32_t parseInstanceIndex_(const DeviceData& d);
        static std::string parseInstanceName_(const DeviceData& d, uint32_t idx);

        static const char* parseMsgName_(const DeviceData& d);
        static std::string parseRawHex_(const DeviceData& d);

        static bool readNum_(const DeviceData& d,
                             std::initializer_list<const char*> keys,
                             double& out);

        static bool readInt_(const DeviceData& d,
                             std::initializer_list<const char*> keys,
                             int32_t& out);

        static bool readBool_(const DeviceData& d,
                              std::initializer_list<const char*> keys,
                              bool& out);

        static void updateFromFault1_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromFault2_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt1_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt2_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt3_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt4_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt5_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt6_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromSt7_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromElecEnergy_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromCurrentLimit_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromTm2b_(const DeviceData& d, BmsPerInstanceCache& x);
        static void updateFromFire2b_(const DeviceData& d, BmsPerInstanceCache& x);
    };

} // namespace control::bms

#endif // ENERGYSTORAGE_CONTROL_BMS_LOGIC_ADAPTER_H