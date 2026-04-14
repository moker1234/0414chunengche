#ifndef ENERGYSTORAGE_CONTROL_MODEL_PATH_EXPORTER_H
#define ENERGYSTORAGE_CONTROL_MODEL_PATH_EXPORTER_H

#pragma once

#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

#include "../aggregator/system_snapshot.h"

namespace control {

    class ModelPathExporter {
    public:
        ModelPathExporter() = default;

        void setOutputPath(std::string path) { output_path_ = std::move(path); }
        void setMinIntervalMs(uint64_t ms)   { min_interval_ms_ = ms; }

        bool exportLatest(const agg::SystemSnapshot& snap,
                          const nlohmann::json& logic_view,
                          uint64_t now_ms);

    private:
        static nlohmann::json buildMergedModel_(const agg::SystemSnapshot& snap,
                                               const nlohmann::json& logic_view);

    private:
        std::string output_path_{"/home/zlg/userdata/debug/model_latest_for_csv.json"};
        uint64_t min_interval_ms_{1000};   // 1s 节流，避免频繁刷盘
        uint64_t last_export_ms_{0};
    };

} // namespace control

#endif // ENERGYSTORAGE_CONTROL_MODEL_PATH_EXPORTER_H