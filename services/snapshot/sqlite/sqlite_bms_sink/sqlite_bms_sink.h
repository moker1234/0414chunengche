//
// Created by lxy on 2026/3/18.
//

#ifndef ENERGYSTORAGE_SQLITE_BMS_SINK_H
#define ENERGYSTORAGE_SQLITE_BMS_SINK_H


#pragma once

#include <cstdint>
#include <string>
#include <sqlite3.h>

#include "../../bms_filesink/bms_file_sink.h"

namespace snapshot {
    struct BmsSnapshot;
}

class SqliteBmsSink final : public BmsSnapshotSink {
public:
    struct Config {
        std::string db_path{"/mnt/sqlite_tfcard/json_data.db"};
        uint32_t busy_timeout_ms{3000};

        // 沿用 BmsFileSink 风格：按时间节流入库
        uint32_t history_interval_ms{1000};
    };

    explicit SqliteBmsSink(Config cfg);
    ~SqliteBmsSink() override;

    void onBmsSnapshot(const snapshot::BmsSnapshot& snap) override;

private:
    static uint64_t nowMs();

    bool openDb_();
    void closeDb_();

    bool applyPragmas_();
    bool initSchema_();

    bool prepareInsertStmt_();
    void finalizeInsertStmt_();

    bool insertSnapshotGroups_(const snapshot::BmsSnapshot& snap);

private:
    Config cfg_;

    sqlite3* db_{nullptr};
    sqlite3_stmt* insert_stmt_{nullptr};
    bool opened_{false};

    uint64_t last_hist_ms_{0};
};

#endif //ENERGYSTORAGE_SQLITE_BMS_SINK_H