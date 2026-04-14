//
// Created by lxy on 2026/1/12.
//
#ifndef ENERGYSTORAGE_DATA_AGGREGATOR_H
#define ENERGYSTORAGE_DATA_AGGREGATOR_H

#pragma once

#include <mutex>
#include <string>

#include "system_snapshot.h"
#include "../../services/device/device_base.h"
#include "bms/bms_snapshot.h"

namespace agg {



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


    private:
        void updateSystemTemperature_();

        // ===== BMS helpers =====
        static uint64_t nowMs_();
        static bool isBmsInternalKey_(const std::string& k);

        static uint32_t extractBmsIndexFromCanId_(uint32_t can_id);
        static std::string makeBmsInstanceName_(uint32_t idx);
        static uint32_t parseBmsIndexFromInstanceName_(const std::string& s);

        void onBmsDeviceData_(const DeviceData& d, uint64_t ts);
        void updateLegacyBmsProjection_(const DeviceData& d,
                                        const std::string& instance_name,
                                        uint64_t ts);

    private:
        mutable std::mutex mtx_;
        SystemSnapshot snap_;
        snapshot::BmsSnapshot bms_snap_;
    };

} // namespace agg

#endif // ENERGYSTORAGE_DATA_AGGREGATOR_H