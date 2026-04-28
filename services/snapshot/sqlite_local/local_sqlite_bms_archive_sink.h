//
// Created by lxy on 2026/4/20.
//

#ifndef ENERGYSTORAGE_LOCAL_SQLITE_BMS_ARCHIVE_SINK_H
#define ENERGYSTORAGE_LOCAL_SQLITE_BMS_ARCHIVE_SINK_H


#pragma once

#include "../bms_filesink/bms_snapshot_sink.h"
#include "../../aggregator/bms/bms_snapshot.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <sqlite3.h>

/**
 * 本地 SQLite 版 BmsFileSink 骨架
 *
 * 第一批目标：
 * 1. 保留 BmsFileSink 的 latest/history 节流入口
 * 2. 建立 BMS 独立 writer thread + 独占 sqlite 连接
 * 3. 第一批只建 meta schema，不开始真实业务表写入
 */
class LocalSqliteBmsArchiveSink final : public BmsSnapshotSink {
public:
    struct Config {
        std::string db_path{"/home/zlg/local_sqlite_archive/bms_file_sink.db"};
        uint32_t busy_timeout_ms{3000};

        uint64_t latest_interval_ms{1000};
        uint64_t history_interval_ms{1000};

        bool enable_latest{true};
        bool enable_history{true};

        size_t queue_capacity{128};
        int schema_version{1};
    };

    explicit LocalSqliteBmsArchiveSink(const Config& cfg);
    ~LocalSqliteBmsArchiveSink() override;

    void onBmsSnapshot(const snapshot::BmsSnapshot& snap) override;

private:
    enum class TaskKind : uint8_t {
        HistorySnapshot = 0,
        LatestRefresh   = 1,
    };

    struct Task {
        TaskKind kind{TaskKind::HistorySnapshot};
        snapshot::BmsSnapshot snap{};
    };

private:
    void startWorker_();
    void stopWorker_();

    void workerMain_();

    bool openDb_();
    void closeDb_();
    bool initSchema_();
    bool processTask_(const Task& task);

    void enqueueTask_(Task&& task);
    bool popTask_(Task& out);

private:
    Config cfg_;

    std::mutex state_mtx_;
    uint64_t last_latest_ms_{0};
    uint64_t last_hist_ms_{0};

    std::mutex q_mtx_;
    std::condition_variable q_cv_;
    std::deque<Task> q_;
    bool stop_{false};

    std::thread worker_;

    sqlite3* db_{nullptr};
    bool db_ready_{false};
};


#endif //ENERGYSTORAGE_LOCAL_SQLITE_BMS_ARCHIVE_SINK_H
