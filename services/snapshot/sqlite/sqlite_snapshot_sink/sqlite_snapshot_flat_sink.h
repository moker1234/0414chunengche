//
// Created by lxy on 2026/3/18.
//

#ifndef ENERGYSTORAGE_SQLITE_SNAPSHOT_FLAT_SINK_H
#define ENERGYSTORAGE_SQLITE_SNAPSHOT_FLAT_SINK_H

#pragma once

#include "snapshot_sink.h"

#include <map>
#include <set>
#include <string>
#include <sqlite3.h>
#include <nlohmann/json.hpp>

class SqliteSnapshotFlatSink final : public SnapshotSink {
public:
    struct Config {
        std::string db_path{"/mnt/sqlite_tfcard/json_data.db"};
        uint32_t busy_timeout_ms{3000};

        // 主表每次 snapshot 都写
        bool write_main_every_snapshot{true};
    };

    explicit SqliteSnapshotFlatSink(Config cfg);
    ~SqliteSnapshotFlatSink() override;

    void onSnapshot(const agg::SystemSnapshot& snap) override;

private:
    bool openDb_();
    void closeDb_();
    bool applyPragmas_();
    bool initSchema_();

    bool beginTx_();
    bool commitTx_();
    void rollbackTx_();

    bool insertSnapshotMain_(const agg::SystemSnapshot& snap, int64_t& out_snapshot_id);
    bool insertDeviceHealth_(int64_t snapshot_id,
                             uint64_t ts_ms,
                             const std::string& device_name,
                             const SnapshotItem& item);

    bool insertGasPollEvent_(int64_t snapshot_id,
                             uint64_t ts_ms,
                             const std::string& device_name,
                             const SnapshotItem& item);

    bool insertGasChannelDelta_(int64_t snapshot_id,
                                uint64_t ts_ms,
                                const std::string& device_name,
                                const SnapshotItem& item);

    bool insertSmokeState_(int64_t snapshot_id,
                           uint64_t ts_ms,
                           const std::string& device_name,
                           const SnapshotItem& item);

    bool insertUpsQ1State_(int64_t snapshot_id,
                           uint64_t ts_ms,
                           const std::string& device_name,
                           const SnapshotItem& item);

    bool insertUpsQ6State_(int64_t snapshot_id,
                           uint64_t ts_ms,
                           const std::string& device_name,
                           const SnapshotItem& item);

    bool insertPcuState_(int64_t snapshot_id,
                         uint64_t ts_ms,
                         const std::string& device_name,
                         const SnapshotItem& item);

    bool insertAirconState_(int64_t snapshot_id,
                            uint64_t ts_ms,
                            const std::string& device_name,
                            const SnapshotItem& item);

    std::set<std::string> collectChangedDevices_(const agg::SystemSnapshot& snap);

    bool deviceChanged_(const std::string& name, const SnapshotItem& item);
    bool healthChanged_(const std::string& name, const SnapshotItem& item);

    static bool isBmsShadowItem_(const std::string& name);

    static std::string healthKeyJson_(const SnapshotItem& item);
    static nlohmann::json deviceStateJson_(const std::string& name, const SnapshotItem& item);

    static int32_t scaleX10_(double v);
    static int32_t scaleX100_(double v);
    static int32_t scaleX1000_(double v);

private:
    Config cfg_;

    sqlite3* db_{nullptr};
    bool opened_{false};

    std::map<std::string, std::string> last_device_json_cache_;
    std::map<std::string, std::string> last_device_health_cache_;
};

#endif // ENERGYSTORAGE_SQLITE_SNAPSHOT_FLAT_SINK_H