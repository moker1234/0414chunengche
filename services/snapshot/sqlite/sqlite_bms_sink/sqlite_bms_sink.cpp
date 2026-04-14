//
// Created by lxy on 2026/3/18.
//


#include "../sqlite_bms_sink/sqlite_bms_sink.h"

#include <chrono>
#include <filesystem>

#include "../../utils/logger/logger.h"
#include "../../aggregator/bms/bms_snapshot.h"

namespace fs = std::filesystem;

SqliteBmsSink::SqliteBmsSink(Config cfg)
    : cfg_(std::move(cfg))
{
    opened_ = openDb_();
    if (!opened_) {
        LOGERR("[SQLITE_BMS] open db failed: %s", cfg_.db_path.c_str());
        return;
    }

    if (!applyPragmas_()) {
        LOGERR("[SQLITE_BMS] apply pragmas failed");
    }

    if (!initSchema_()) {
        LOGERR("[SQLITE_BMS] init schema failed");
    }

    if (!prepareInsertStmt_()) {
        LOGERR("[SQLITE_BMS] prepare insert stmt failed");
    }
}

SqliteBmsSink::~SqliteBmsSink()
{
    finalizeInsertStmt_();
    closeDb_();
}

uint64_t SqliteBmsSink::nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool SqliteBmsSink::openDb_()
{
    try {
        const fs::path p(cfg_.db_path);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        LOGERR("[SQLITE_BMS] create parent dir failed: %s", e.what());
        return false;
    }

    const int rc = sqlite3_open(cfg_.db_path.c_str(), &db_);
    if (rc != SQLITE_OK || !db_) {
        LOGERR("[SQLITE_BMS] sqlite3_open failed rc=%d err=%s",
               rc, db_ ? sqlite3_errmsg(db_) : "null");
        return false;
    }

    sqlite3_busy_timeout(db_, static_cast<int>(cfg_.busy_timeout_ms));
    return true;
}

void SqliteBmsSink::closeDb_()
{
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool SqliteBmsSink::applyPragmas_()
{
    if (!db_) return false;

    const char* sqls[] = {
        "PRAGMA journal_mode=WAL;",
        "PRAGMA synchronous=FULL;",
        "PRAGMA wal_autocheckpoint=1000;",
        nullptr
    };

    char* err = nullptr;
    for (int i = 0; sqls[i] != nullptr; ++i) {
        const int rc = sqlite3_exec(db_, sqls[i], nullptr, nullptr, &err);
        if (rc != SQLITE_OK) {
            LOGERR("[SQLITE_BMS] pragma failed sql=%s err=%s",
                   sqls[i], err ? err : "unknown");
            if (err) sqlite3_free(err);
            return false;
        }
    }

    return true;
}

bool SqliteBmsSink::initSchema_()
{
    if (!db_) return false;

    const char* create_sql = R"SQL(
CREATE TABLE IF NOT EXISTS bms_history (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    ts_ms         INTEGER NOT NULL,
    instance_name TEXT    NOT NULL,
    bms_index     INTEGER NOT NULL,
    msg_name      TEXT    NOT NULL,
    group_ts_ms   INTEGER NOT NULL,
    rx_count      INTEGER NOT NULL,
    can_id        INTEGER,
    raw_hex       TEXT,
    payload_json  TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_bms_history_ts_ms
ON bms_history(ts_ms);

CREATE INDEX IF NOT EXISTS idx_bms_history_inst_msg_ts
ON bms_history(instance_name, msg_name, ts_ms);
)SQL";

    char* err = nullptr;
    const int rc = sqlite3_exec(db_, create_sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS] initSchema exec failed err=%s",
               err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }

    return true;
}

bool SqliteBmsSink::prepareInsertStmt_()
{
    if (!db_) return false;

    const char* sql =
        "INSERT INTO bms_history("
        "ts_ms, instance_name, bms_index, msg_name, group_ts_ms, rx_count, can_id, raw_hex, payload_json"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?);";

    const int rc = sqlite3_prepare_v2(db_, sql, -1, &insert_stmt_, nullptr);
    if (rc != SQLITE_OK || !insert_stmt_) {
        LOGERR("[SQLITE_BMS] sqlite3_prepare_v2 failed rc=%d err=%s",
               rc, sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

void SqliteBmsSink::finalizeInsertStmt_()
{
    if (insert_stmt_) {
        sqlite3_finalize(insert_stmt_);
        insert_stmt_ = nullptr;
    }
}

bool SqliteBmsSink::insertSnapshotGroups_(const snapshot::BmsSnapshot& snap)
{
    if (!db_ || !insert_stmt_) return false;

    char* err = nullptr;
    int rc = sqlite3_exec(db_, "BEGIN IMMEDIATE TRANSACTION;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS] begin tx failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }

    for (const auto& [inst_name, inst] : snap.items) {
        const int bms_index = static_cast<int>(inst.meta.bms_index);

        for (const auto& [msg_name, g] : inst.groups) {
            const std::string payload_json = g.toJson().dump(0);

            sqlite3_reset(insert_stmt_);
            sqlite3_clear_bindings(insert_stmt_);

            sqlite3_bind_int64(insert_stmt_, 1, static_cast<sqlite3_int64>(snap.ts_ms));
            sqlite3_bind_text (insert_stmt_, 2, inst_name.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int  (insert_stmt_, 3, bms_index);
            sqlite3_bind_text (insert_stmt_, 4, msg_name.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int64(insert_stmt_, 5, static_cast<sqlite3_int64>(g.ts_ms));
            sqlite3_bind_int  (insert_stmt_, 6, static_cast<int>(g.rx_count));
            sqlite3_bind_int64(insert_stmt_, 7, static_cast<sqlite3_int64>(g.can_id));
            sqlite3_bind_text (insert_stmt_, 8, g.raw_hex.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_text (insert_stmt_, 9, payload_json.c_str(), -1, SQLITE_TRANSIENT);

            rc = sqlite3_step(insert_stmt_);
            if (rc != SQLITE_DONE) {
                LOGERR("[SQLITE_BMS] insert step failed inst=%s msg=%s rc=%d err=%s",
                       inst_name.c_str(), msg_name.c_str(), rc, sqlite3_errmsg(db_));
                sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
                return false;
            }
        }
    }

    rc = sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS] commit failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
        return false;
    }

    return true;
}

void SqliteBmsSink::onBmsSnapshot(const snapshot::BmsSnapshot& snap)
{
    if (!opened_ || !db_ || !insert_stmt_) return;

    const uint64_t now = nowMs();

    if (cfg_.history_interval_ms > 0) {
        if (last_hist_ms_ != 0 && (now - last_hist_ms_) < cfg_.history_interval_ms) {
            return;
        }
    }

    if (!insertSnapshotGroups_(snap)) {
        LOGERR("[SQLITE_BMS] insert snapshot groups failed ts_ms=%llu",
               static_cast<unsigned long long>(snap.ts_ms));
        return;
    }

    last_hist_ms_ = now;
}