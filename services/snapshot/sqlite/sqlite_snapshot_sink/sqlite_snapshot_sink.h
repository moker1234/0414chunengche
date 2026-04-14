//
// Created by lxy on 2026/3/17.
//

#ifndef ENERGYSTORAGE_SQLITE_SNAPSHOT_SINK_H
#define ENERGYSTORAGE_SQLITE_SNAPSHOT_SINK_H


#pragma once

#include "snapshot_sink.h"

#include <map>
#include <memory>
#include <string>
#include <sqlite3.h>

class SqliteSnapshotSink final : public SnapshotSink {
public:
    struct Config {
        std::string db_path{"/mnt/sqlite_tfcard/json_data.db"};
        uint32_t busy_timeout_ms{3000};

        // true: 只有 snapshot 变化才入库
        bool only_when_changed{true};

        // 预留：将来如果想做节流可再启用
        uint32_t min_interval_ms{0};
    };

    explicit SqliteSnapshotSink(Config cfg);
    ~SqliteSnapshotSink() override;

    void onSnapshot(const agg::SystemSnapshot& snap) override;

private:
    bool openDb_();
    void closeDb_();
    bool initSchema_();
    bool applyPragmas_();

    bool prepareInsertStmt_();
    void finalizeInsertStmt_();

    bool insertSnapshot_(const agg::SystemSnapshot& snap);

    bool hasChanged_(const agg::SystemSnapshot& snap);
    bool deviceChanged_(const std::string& name, const SnapshotItem& item);
    bool healthChanged_(const std::string& name, const SnapshotItem& item);

    static std::string healthKeyJson_(const SnapshotItem& item);

private:
    Config cfg_;

    sqlite3* db_{nullptr};
    sqlite3_stmt* insert_stmt_{nullptr};
    bool opened_{false};

    // 变化检测 cache（沿用 FileSink 思路）
    std::map<std::string, std::string> last_device_json_cache_;
    std::map<std::string, std::string> last_device_health_cache_;
};


#endif //ENERGYSTORAGE_SQLITE_SNAPSHOT_SINK_H