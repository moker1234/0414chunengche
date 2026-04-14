//
// Created by lxy on 2026/3/19.
//

#ifndef ENERGYSTORAGE_SQLITE_BMS_FLAT_SINK_H
#define ENERGYSTORAGE_SQLITE_BMS_FLAT_SINK_H

#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <sqlite3.h>

#include "bms_snapshot.h"
#include "bms_filesink/bms_snapshot_sink.h"

class SqliteBmsFlatSink final : public BmsSnapshotSink {
public:
    struct Config {
        std::string db_path{"/mnt/sqlite_tfcard/bms_data.db"};
        uint32_t busy_timeout_ms{3000};

        // 主表每个快照都写
        bool write_main_every_snapshot{true};
    };

    explicit SqliteBmsFlatSink(Config cfg);
    ~SqliteBmsFlatSink() override;

    void onBmsSnapshot(const snapshot::BmsSnapshot& snap) override;

private:
    bool openDb_();
    void closeDb_();
    bool applyPragmas_();
    bool initSchema_();

    bool beginTx_();
    bool commitTx_();
    void rollbackTx_();

    bool ensureBmsInstance_(const std::string& instance_name,
                            uint32_t bms_index,
                            int64_t& out_bms_id);

    bool ensureGroupType_(const std::string& group_name,
                          int64_t& out_group_type_id);

    bool insertSnapshotMain_(const snapshot::BmsSnapshot& snap, int64_t& out_snapshot_id);

    bool insertInstanceHealth_(int64_t snapshot_id,
                               uint64_t ts_ms,
                               const std::string& instance_name,
                               const snapshot::BmsInstanceData& inst);

    bool insertGroupRaw_(int64_t snapshot_id,
                         uint64_t ts_ms,
                         const std::string& instance_name,
                         const snapshot::BmsInstanceData& inst,
                         const std::string& msg_name,
                         const snapshot::BmsGroupData& g);

    std::set<std::string> collectChangedGroups_(const snapshot::BmsSnapshot& snap);

    bool groupChanged_(const std::string& instance_name,
                       const std::string& msg_name,
                       const snapshot::BmsGroupData& g);

    bool healthChanged_(const std::string& instance_name,
                        const snapshot::BmsInstanceData& inst);

    static std::string healthKeyJson_(const snapshot::BmsInstanceData& inst);

private:
    Config cfg_;

    sqlite3* db_{nullptr};
    bool opened_{false};

    // key = "BMS_1::B2V_ST2"
    std::map<std::string, std::string> last_group_json_cache_;

    // key = "BMS_1"
    std::map<std::string, std::string> last_health_json_cache_;
};

#endif // ENERGYSTORAGE_SQLITE_BMS_FLAT_SINK_H