#include "sqlite_snapshot_sink.h"

#include <cstdio>
#include <filesystem>

#include "../../utils/logger/logger.h"
#include "../../aggregator/system_snapshot.h"

namespace fs = std::filesystem;

/* ================= DeviceData JSON helper ================= */

static nlohmann::json deviceDataJson_(const DeviceData& d)
{
    nlohmann::json j = nlohmann::json::object();
    j["device_name"] = d.device_name;
    j["timestamp"]   = d.timestamp;
    j["num"]         = d.num;
    j["value"]       = d.value;
    j["status"]      = d.status;
    j["str"]         = d.str;
    return j;
}

/* ================= health key ================= */

std::string SqliteSnapshotSink::healthKeyJson_(const SnapshotItem& item)
{
    nlohmann::json j;
    j["state"]                = static_cast<int>(item.health);
    j["online"]               = item.online;
    j["last_ok_ms"]           = item.last_ok_ms;
    j["disconnect_window_ms"] = item.disconnect_window_ms;
    j["last_offline_ms"]      = item.last_offline_ms;
    j["disconnect_count"]     = item.disconnect_count;
    return j.dump();
}

/* ================= ctor / dtor ================= */

SqliteSnapshotSink::SqliteSnapshotSink(Config cfg)
    : cfg_(std::move(cfg))
{
    opened_ = openDb_();
    if (!opened_) {
        LOGERR("[SQLITE_SNAP] open db failed: %s", cfg_.db_path.c_str());
        return;
    }

    if (!applyPragmas_()) {
        LOGERR("[SQLITE_SNAP] apply pragmas failed");
    }

    if (!initSchema_()) {
        LOGERR("[SQLITE_SNAP] init schema failed");
    }

    if (!prepareInsertStmt_()) {
        LOGERR("[SQLITE_SNAP] prepare insert stmt failed");
    }
}

SqliteSnapshotSink::~SqliteSnapshotSink()
{
    finalizeInsertStmt_();
    closeDb_();
}

/* ================= db open / close ================= */

bool SqliteSnapshotSink::openDb_()
{
    try {
        const fs::path p(cfg_.db_path);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        LOGERR("[SQLITE_SNAP] create parent dir failed: %s", e.what());
        return false;
    }

    const int rc = sqlite3_open(cfg_.db_path.c_str(), &db_);
    if (rc != SQLITE_OK || !db_) {
        LOGERR("[SQLITE_SNAP] sqlite3_open failed rc=%d err=%s",
               rc, db_ ? sqlite3_errmsg(db_) : "null");
        return false;
    }

    return true;
}

void SqliteSnapshotSink::closeDb_()
{
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool SqliteSnapshotSink::applyPragmas_()
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
            LOGERR("[SQLITE_SNAP] pragma failed sql=%s err=%s",
                   sqls[i], err ? err : "unknown");
            if (err) sqlite3_free(err);
            return false;
        }
    }

    sqlite3_busy_timeout(db_, static_cast<int>(cfg_.busy_timeout_ms));
    return true;
}

bool SqliteSnapshotSink::initSchema_()
{
    if (!db_) return false;

    const char* create_sql = R"SQL(
CREATE TABLE IF NOT EXISTS snapshot_history (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts_ms     INTEGER NOT NULL,
    json_text TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_snapshot_history_ts_ms
ON snapshot_history(ts_ms);
)SQL";

    char* err = nullptr;
    const int rc = sqlite3_exec(db_, create_sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP] initSchema exec failed err=%s",
               err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }

    return true;
}

bool SqliteSnapshotSink::prepareInsertStmt_()
{
    if (!db_) return false;

    const char* sql =
        "INSERT INTO snapshot_history(ts_ms, json_text) VALUES(?, ?);";

    const int rc = sqlite3_prepare_v2(db_, sql, -1, &insert_stmt_, nullptr);
    if (rc != SQLITE_OK || !insert_stmt_) {
        LOGERR("[SQLITE_SNAP] sqlite3_prepare_v2 failed rc=%d err=%s",
               rc, sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

void SqliteSnapshotSink::finalizeInsertStmt_()
{
    if (insert_stmt_) {
        sqlite3_finalize(insert_stmt_);
        insert_stmt_ = nullptr;
    }
}

/* ================= 变化检测 ================= */

bool SqliteSnapshotSink::deviceChanged_(const std::string& name,
                                        const SnapshotItem& item)
{
    const std::string cur = deviceDataJson_(item.data).dump();

    auto it = last_device_json_cache_.find(name);
    if (it == last_device_json_cache_.end() || it->second != cur) {
        last_device_json_cache_[name] = cur;
        return true;
    }
    return false;
}

bool SqliteSnapshotSink::healthChanged_(const std::string& name,
                                        const SnapshotItem& item)
{
    const std::string cur = healthKeyJson_(item);
    auto it = last_device_health_cache_.find(name);
    if (it == last_device_health_cache_.end() || it->second != cur) {
        last_device_health_cache_[name] = cur;
        return true;
    }
    return false;
}

bool SqliteSnapshotSink::hasChanged_(const agg::SystemSnapshot& snap)
{
    bool changed = false;

    for (const auto& [name, item] : snap.items) {
        if (deviceChanged_(name, item) || healthChanged_(name, item)) {
            changed = true;
        }
    }

    return changed;
}

/* ================= insert ================= */

bool SqliteSnapshotSink::insertSnapshot_(const agg::SystemSnapshot& snap)
{
    if (!db_ || !insert_stmt_) return false;

    const std::string json_text = snap.toJson().dump(0);

    sqlite3_reset(insert_stmt_);
    sqlite3_clear_bindings(insert_stmt_);

    int rc = sqlite3_bind_int64(
        insert_stmt_,
        1,
        static_cast<sqlite3_int64>(snap.timestamp_ms)
    );
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP] bind ts_ms failed rc=%d err=%s",
               rc, sqlite3_errmsg(db_));
        return false;
    }

    rc = sqlite3_bind_text(insert_stmt_,
                           2,
                           json_text.c_str(),
                           static_cast<int>(json_text.size()),
                           SQLITE_TRANSIENT);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP] bind json_text failed rc=%d err=%s",
               rc, sqlite3_errmsg(db_));
        return false;
    }

    rc = sqlite3_step(insert_stmt_);
    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP] insert step failed rc=%d err=%s",
               rc, sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

/* ================= onSnapshot ================= */

void SqliteSnapshotSink::onSnapshot(const agg::SystemSnapshot& snap)
{
    if (!opened_ || !db_ || !insert_stmt_) return;

    if (cfg_.only_when_changed) {
        if (!hasChanged_(snap)) return;
    }

    if (!insertSnapshot_(snap)) {
        LOGERR("[SQLITE_SNAP] insert snapshot failed ts_ms=%llu",
               static_cast<unsigned long long>(snap.timestamp_ms));
    }
}