//
// Created by lxy on 2026/1/12.
//
#ifndef ENERGYSTORAGE_DATA_AGGREGATOR_H
#define ENERGYSTORAGE_DATA_AGGREGATOR_H

#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "system_snapshot.h"
#include "../../services/device/device_base.h"
#include "bms/bms_snapshot.h"

namespace agg {

    /*
     * BMS runtime health 回写结构
     *
     * 来源：
     *   LogicEngine::updateBmsRuntimeHealth_()
     *
     * 目标：
     *   只回写 BmsSnapshot / SystemSnapshot 的 health 字段；
     *   不增加 BMS 信号级 JSON；
     *   不让 Aggregator 重新计算 BMS online/offline。
     */
    struct BmsRuntimeHealthUpdate {
        std::string instance_name;          // "BMS_1" ~ "BMS_4"
        uint32_t bms_index{0};              // 1 ~ 4

        bool online{false};

        uint64_t last_ok_ms{0};
        uint32_t disconnect_window_ms{0};
        uint64_t last_offline_ms{0};
        uint32_t disconnect_count{0};
    };


    class DataAggregator {
    public:
        DataAggregator();

        void onDeviceData(const DeviceData& d);

        SystemSnapshot snapshot() const;
        snapshot::BmsSnapshot bmsSnapshot() const;

        void updateHealthFromScheduler(
            const std::string& device_name,
            bool online,
            uint64_t last_ok_ms,
            uint32_t disconnect_window_ms,
            uint64_t last_offline_ms,
            uint32_t disconnect_count
            );
        // BMS 专用：由 Control/Logic runtime 真源回写 health。
        // 返回 true 表示至少有一项 health 发生变化，调用方可据此决定是否 dispatchBms。
        bool updateBmsRuntimeHealth(const std::vector<BmsRuntimeHealthUpdate>& updates);


    private:
        void updateSystemTemperature_();

        // ===== BMS helpers =====
        static uint64_t nowMs_();

        static uint32_t extractBmsIndexFromCanId_(uint32_t can_id);
        static std::string makeBmsInstanceName_(uint32_t idx);
        static uint32_t parseBmsIndexFromInstanceName_(const std::string& s);

        void onBmsDeviceData_(const DeviceData& d, uint64_t ts);


    private:
        mutable std::mutex mtx_;
        SystemSnapshot snap_;
        snapshot::BmsSnapshot bms_snap_;
    };

} // namespace agg

#endif // ENERGYSTORAGE_DATA_AGGREGATOR_H