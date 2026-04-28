//
// Created by lxy on 2026/4/20.
//

#include "local_sqlite_bms_archive_sink.h"

#include "local_sqlite_common.h"
#include "logger.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

static uint64_t nowMs_()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()
    );
}

/* ========================= sqlite helpers ========================= */

static bool bindText_(sqlite3_stmt* stmt, int idx, const std::string& s, std::string* err)
{
    const int rc = sqlite3_bind_text(stmt, idx, s.c_str(), -1, SQLITE_TRANSIENT);
    if (rc != SQLITE_OK) {
        if (err) *err = sqlite3_errstr(rc);
        return false;
    }
    return true;
}

static bool bindInt64_(sqlite3_stmt* stmt, int idx, sqlite3_int64 v, std::string* err)
{
    const int rc = sqlite3_bind_int64(stmt, idx, v);
    if (rc != SQLITE_OK) {
        if (err) *err = sqlite3_errstr(rc);
        return false;
    }
    return true;
}

static bool bindInt_(sqlite3_stmt* stmt, int idx, int v, std::string* err)
{
    const int rc = sqlite3_bind_int(stmt, idx, v);
    if (rc != SQLITE_OK) {
        if (err) *err = sqlite3_errstr(rc);
        return false;
    }
    return true;
}

static bool bindBlob_(sqlite3_stmt* stmt,
                      int idx,
                      const void* data,
                      int bytes,
                      std::string* err)
{
    const int rc = sqlite3_bind_blob(stmt, idx, data, bytes, SQLITE_TRANSIENT);
    if (rc != SQLITE_OK) {
        if (err) *err = sqlite3_errstr(rc);
        return false;
    }
    return true;
}

static bool stepDone_(sqlite3* db, sqlite3_stmt* stmt, std::string* err)
{
    const int rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        if (err) *err = sqlite3_errmsg(db);
        return false;
    }
    return true;
}

static bool prepare_(sqlite3* db, const std::string& sql, sqlite3_stmt** out, std::string* err)
{
    *out = nullptr;
    const int rc = sqlite3_prepare_v2(db, sql.c_str(), -1, out, nullptr);
    if (rc != SQLITE_OK) {
        if (err) *err = sqlite3_errmsg(db);
        return false;
    }
    return true;
}

static bool execPreparedDone_(sqlite3* db, sqlite3_stmt* stmt, std::string* err)
{
    const bool ok = stepDone_(db, stmt, err);
    sqlite3_finalize(stmt);
    return ok;
}

static bool selectSnapshotIdByTs_(sqlite3* db,
                                  uint64_t ts_ms,
                                  sqlite3_int64& snapshot_id,
                                  std::string* err)
{
    snapshot_id = 0;

    sqlite3_stmt* stmt = nullptr;
    if (!prepare_(db,
                  "SELECT snapshot_id FROM bms_snapshot_main WHERE snapshot_ts_ms = ?;",
                  &stmt,
                  err)) {
        return false;
    }

    bool ok = bindInt64_(stmt, 1, static_cast<sqlite3_int64>(ts_ms), err);
    if (!ok) {
        sqlite3_finalize(stmt);
        return false;
    }

    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        snapshot_id = sqlite3_column_int64(stmt, 0);
        sqlite3_finalize(stmt);
        return true;
    }

    if (err) *err = sqlite3_errmsg(db);
    sqlite3_finalize(stmt);
    return false;
}

static int parseCanId_(const std::string& s)
{
    if (s.empty()) return 0;
    return static_cast<int>(std::strtoul(s.c_str(), nullptr, 0));
}

static const char* tableForGroup_(const std::string& g)
{
    if (g == "B2TM_Info")         return "bms_b2tm_info_history";
    if (g == "TM2B_Info")         return "bms_tm2b_info_history";
    if (g == "Fire2B_state")      return "bms_fire2b_state_history";
    if (g == "B2V_CurrentLimit")  return "bms_current_limit_history";
    if (g == "B2V_ElecEnergy")    return "bms_elec_energy_history";
    if (g == "B2V_Fault1")        return "bms_fault1_history";
    if (g == "B2V_Fault2")        return "bms_fault2_history";
    if (g == "B2V_ST1")           return "bms_st1_history";
    if (g == "B2V_ST2")           return "bms_st2_history";
    if (g == "B2V_ST3")           return "bms_st3_history";
    if (g == "B2V_ST4")           return "bms_st4_history";
    if (g == "B2V_ST5")           return "bms_st5_history";
    if (g == "B2V_ST6")           return "bms_st6_history";
    if (g == "B2V_ST7")           return "bms_st7_history";
    return nullptr;
}

static bool upsertBmsSnapshotMain_(sqlite3* db,
                                   const snapshot::BmsSnapshot& snap,
                                   bool mark_history,
                                   bool mark_latest,
                                   sqlite3_int64& snapshot_id,
                                   std::string* err)
{
    static const char* kSql =
        "INSERT INTO bms_snapshot_main ("
        "  snapshot_ts_ms, has_history, has_latest, instance_count"
        ") VALUES (?,?,?,?) "
        "ON CONFLICT(snapshot_ts_ms) DO UPDATE SET "
        "  has_history = (bms_snapshot_main.has_history OR excluded.has_history),"
        "  has_latest  = (bms_snapshot_main.has_latest  OR excluded.has_latest),"
        "  instance_count = excluded.instance_count;";

    sqlite3_stmt* stmt = nullptr;
    if (!prepare_(db, kSql, &stmt, err)) {
        return false;
    }

    int i = 1;
    const bool ok =
        bindInt64_(stmt, i++, static_cast<sqlite3_int64>(snap.ts_ms), err) &&
        bindInt_(stmt,   i++, mark_history ? 1 : 0, err) &&
        bindInt_(stmt,   i++, mark_latest ? 1 : 0, err) &&
        bindInt_(stmt,   i++, static_cast<int>(snap.items.size()), err);

    if (!ok) {
        sqlite3_finalize(stmt);
        return false;
    }

    if (!execPreparedDone_(db, stmt, err)) {
        return false;
    }

    return selectSnapshotIdByTs_(db, snap.ts_ms, snapshot_id, err);
}

static bool upsertInstanceHealth_(sqlite3* db,
                                  sqlite3_int64 snapshot_id,
                                  const std::string& instance_name,
                                  const snapshot::BmsInstanceData& inst,
                                  std::string* err)
{
    static const char* kSql =
        "INSERT INTO bms_instance_health_history ("
        "  snapshot_id, bms_index, instance_name, last_msg_name, online, last_ok_ms, disconnect_window_ms, last_offline_ms, disconnect_count"
        ") VALUES (?,?,?,?,?,?,?,?,?) "
        "ON CONFLICT(snapshot_id, bms_index) DO UPDATE SET "
        "  instance_name = excluded.instance_name,"
        "  last_msg_name = excluded.last_msg_name,"
        "  online = excluded.online,"
        "  last_ok_ms = excluded.last_ok_ms,"
        "  disconnect_window_ms = excluded.disconnect_window_ms,"
        "  last_offline_ms = excluded.last_offline_ms,"
        "  disconnect_count = excluded.disconnect_count;";

    sqlite3_stmt* stmt = nullptr;
    if (!prepare_(db, kSql, &stmt, err)) {
        return false;
    }

    int i = 1;
    const bool ok =
        bindInt64_(stmt, i++, snapshot_id, err) &&
        bindInt_(stmt,   i++, static_cast<int>(inst.meta.bms_index), err) &&
        bindText_(stmt,  i++, instance_name, err) &&
        bindText_(stmt,  i++, inst.meta.last_msg_name, err) &&
        bindInt_(stmt,   i++, inst.health.online ? 1 : 0, err) &&
        bindInt64_(stmt, i++, static_cast<sqlite3_int64>(inst.health.last_ok_ms), err) &&
        bindInt_(stmt,   i++, static_cast<int>(inst.health.disconnect_window_ms), err) &&
        bindInt64_(stmt, i++, static_cast<sqlite3_int64>(inst.health.last_offline_ms), err) &&
        bindInt_(stmt,   i++, static_cast<int>(inst.health.disconnect_count), err);

    if (!ok) {
        sqlite3_finalize(stmt);
        return false;
    }

    return execPreparedDone_(db, stmt, err);
}

static bool upsertGroupRow_(sqlite3* db,
                            const char* table_name,
                            sqlite3_int64 snapshot_id,
                            uint32_t bms_index,
                            const snapshot::BmsGroupData& g,
                            std::string* err)
{
    const std::string sql =
        std::string("INSERT INTO ") + table_name + " ("
        "  snapshot_id, bms_index, group_ts_ms, can_id, rx_count, payload, "
        "  online, last_rx_ms, last_ok_ms, disconnect_window_ms, last_offline_ms, disconnect_count"
        ") VALUES (?,?,?,?,?,?,?,?,?,?,?,?) "
        "ON CONFLICT(snapshot_id, bms_index) DO UPDATE SET "
        "  group_ts_ms = excluded.group_ts_ms,"
        "  can_id = excluded.can_id,"
        "  rx_count = excluded.rx_count,"
        "  payload = excluded.payload,"
        "  online = excluded.online,"
        "  last_rx_ms = excluded.last_rx_ms,"
        "  last_ok_ms = excluded.last_ok_ms,"
        "  disconnect_window_ms = excluded.disconnect_window_ms,"
        "  last_offline_ms = excluded.last_offline_ms,"
        "  disconnect_count = excluded.disconnect_count;";

    std::vector<uint8_t> payload;
    if (!local_sqlite::parseHexBytes(g.raw_hex, payload, err)) {
        return false;
    }

    sqlite3_stmt* stmt = nullptr;
    if (!prepare_(db, sql, &stmt, err)) {
        return false;
    }

    int i = 1;
    bool ok =
        bindInt64_(stmt, i++, snapshot_id, err) &&
        bindInt_(stmt,   i++, static_cast<int>(bms_index), err) &&
        bindInt64_(stmt, i++, static_cast<sqlite3_int64>(g.ts_ms), err) &&
        bindInt_(stmt,   i++, static_cast<int>(g.can_id), err) &&
        bindInt_(stmt,   i++, static_cast<int>(g.rx_count), err);

    if (!ok) {
        sqlite3_finalize(stmt);
        return false;
    }

    if (payload.empty()) {
        ok = bindBlob_(stmt, i++, "", 0, err);
    } else {
        ok = bindBlob_(stmt, i++, payload.data(), static_cast<int>(payload.size()), err);
    }

    ok = ok &&
         bindInt_(stmt,   i++, g.health.online ? 1 : 0, err) &&
         bindInt64_(stmt, i++, static_cast<sqlite3_int64>(g.health.last_rx_ms), err) &&
         bindInt64_(stmt, i++, static_cast<sqlite3_int64>(g.health.last_ok_ms), err) &&
         bindInt_(stmt,   i++, static_cast<int>(g.health.disconnect_window_ms), err) &&
         bindInt64_(stmt, i++, static_cast<sqlite3_int64>(g.health.last_offline_ms), err) &&
         bindInt_(stmt,   i++, static_cast<int>(g.health.disconnect_count), err);

    if (!ok) {
        sqlite3_finalize(stmt);
        return false;
    }

    return execPreparedDone_(db, stmt, err);
}

} // namespace

LocalSqliteBmsArchiveSink::LocalSqliteBmsArchiveSink(const Config& cfg)
    : cfg_(cfg)
{
    startWorker_();
}

LocalSqliteBmsArchiveSink::~LocalSqliteBmsArchiveSink()
{
    stopWorker_();
}

void LocalSqliteBmsArchiveSink::startWorker_()
{
    worker_ = std::thread(&LocalSqliteBmsArchiveSink::workerMain_, this);
}

void LocalSqliteBmsArchiveSink::stopWorker_()
{
    {
        std::lock_guard<std::mutex> lk(q_mtx_);
        stop_ = true;
    }
    q_cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }
}

void LocalSqliteBmsArchiveSink::onBmsSnapshot(const snapshot::BmsSnapshot& snap)
{
    const uint64_t now = nowMs_();

    bool need_latest = false;
    bool need_history = false;

    {
        std::lock_guard<std::mutex> lk(state_mtx_);

        if (cfg_.enable_latest) {
            if (last_latest_ms_ == 0 || (now - last_latest_ms_) >= cfg_.latest_interval_ms) {
                need_latest = true;
                last_latest_ms_ = now;
            }
        }

        if (cfg_.enable_history && cfg_.history_interval_ms > 0) {
            if (last_hist_ms_ == 0 || (now - last_hist_ms_) >= cfg_.history_interval_ms) {
                need_history = true;
                last_hist_ms_ = now;
            }
        }
    }

    if (need_latest) {
        Task t;
        t.kind = TaskKind::LatestRefresh;
        t.snap = snap;
        enqueueTask_(std::move(t));
    }

    if (need_history) {
        Task t;
        t.kind = TaskKind::HistorySnapshot;
        t.snap = snap;
        enqueueTask_(std::move(t));
    }
}

void LocalSqliteBmsArchiveSink::workerMain_()
{
    if (!openDb_()) {
        LOGERR("[LOCAL_SQLITE_BMS] open db failed: %s", cfg_.db_path.c_str());
        return;
    }

    if (!initSchema_()) {
        LOGERR("[LOCAL_SQLITE_BMS] init schema failed: %s", cfg_.db_path.c_str());
        closeDb_();
        return;
    }

    db_ready_ = true;
    LOGINFO("[LOCAL_SQLITE_BMS] worker ready db=%s", cfg_.db_path.c_str());

    while (true) {
        Task task;
        if (!popTask_(task)) {
            break;
        }

        if (!processTask_(task)) {
            LOGERR("[LOCAL_SQLITE_BMS] process task failed");
        }
    }

    db_ready_ = false;
    closeDb_();
    LOGINFO("[LOCAL_SQLITE_BMS] worker exit");
}

bool LocalSqliteBmsArchiveSink::openDb_()
{
    std::string err;
    if (!local_sqlite::openDb(cfg_.db_path, cfg_.busy_timeout_ms, &db_, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] openDb failed: %s", err.c_str());
        return false;
    }

    if (!local_sqlite::applyCommonPragmas(db_, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] applyPragmas failed: %s", err.c_str());
        return false;
    }

    return true;
}

void LocalSqliteBmsArchiveSink::closeDb_()
{
    local_sqlite::closeDb(db_);
}

bool LocalSqliteBmsArchiveSink::initSchema_()
{
    std::string err;
    if (!local_sqlite::initMetaSchema(db_, "LocalSqliteBmsArchiveSink", cfg_.schema_version, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] initMetaSchema failed: %s", err.c_str());
        return false;
    }

    static const char* kSql = R"SQL(
CREATE TABLE IF NOT EXISTS bms_snapshot_main (
    snapshot_id            INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_ts_ms         INTEGER NOT NULL UNIQUE,
    has_history            INTEGER NOT NULL DEFAULT 0,
    has_latest             INTEGER NOT NULL DEFAULT 0,
    instance_count         INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_bms_snapshot_main_ts
ON bms_snapshot_main(snapshot_ts_ms DESC);

CREATE TABLE IF NOT EXISTS bms_instance_health_history (
    id                     INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id            INTEGER NOT NULL,
    bms_index              INTEGER NOT NULL,
    instance_name          TEXT NOT NULL,
    last_msg_name          TEXT,
    online                 INTEGER NOT NULL,
    last_ok_ms             INTEGER,
    disconnect_window_ms   INTEGER,
    last_offline_ms        INTEGER,
    disconnect_count       INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_b2tm_info_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_tm2b_info_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_fire2b_state_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_current_limit_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_elec_energy_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_fault1_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_fault2_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st1_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st2_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st3_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st4_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st5_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st6_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);

CREATE TABLE IF NOT EXISTS bms_st7_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    snapshot_id INTEGER NOT NULL,
    bms_index INTEGER NOT NULL,
    group_ts_ms INTEGER NOT NULL,
    can_id INTEGER NOT NULL,
    rx_count INTEGER NOT NULL,
    payload BLOB NOT NULL,
    online INTEGER,
    last_rx_ms INTEGER,
    last_ok_ms INTEGER,
    disconnect_window_ms INTEGER,
    last_offline_ms INTEGER,
    disconnect_count INTEGER,
    UNIQUE(snapshot_id, bms_index),
    FOREIGN KEY(snapshot_id) REFERENCES bms_snapshot_main(snapshot_id)
);
)SQL";

    if (!local_sqlite::exec(db_, kSql, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] create schema failed: %s", err.c_str());
        return false;
    }

    return true;
}

bool LocalSqliteBmsArchiveSink::processTask_(const Task& task)
{
    if (!db_) return false;

    const bool mark_history = (task.kind == TaskKind::HistorySnapshot);
    const bool mark_latest  = (task.kind == TaskKind::LatestRefresh);

    std::string err;
    if (!local_sqlite::beginImmediate(db_, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] begin tx failed: %s", err.c_str());
        return false;
    }

    sqlite3_int64 snapshot_id = 0;
    if (!upsertBmsSnapshotMain_(db_, task.snap, mark_history, mark_latest, snapshot_id, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] upsert snapshot_main failed: %s", err.c_str());
        local_sqlite::rollback(db_, nullptr);
        return false;
    }

    for (const auto& [instance_name, inst] : task.snap.items) {
        if (!upsertInstanceHealth_(db_, snapshot_id, instance_name, inst, &err)) {
            LOGERR("[LOCAL_SQLITE_BMS] upsert instance health failed inst=%s err=%s",
                   instance_name.c_str(), err.c_str());
            local_sqlite::rollback(db_, nullptr);
            return false;
        }

        for (const auto& [group_name, grp] : inst.groups) {
            const char* table_name = tableForGroup_(group_name);
            if (!table_name) {
                LOGINFO("[LOCAL_SQLITE_BMS] skip unknown group=%s", group_name.c_str());
                continue;
            }

            if (!upsertGroupRow_(db_, table_name, snapshot_id, inst.meta.bms_index, grp, &err)) {
                LOGERR("[LOCAL_SQLITE_BMS] upsert group failed inst=%s group=%s err=%s",
                       instance_name.c_str(), group_name.c_str(), err.c_str());
                local_sqlite::rollback(db_, nullptr);
                return false;
            }
        }
    }

    if (!local_sqlite::commit(db_, &err)) {
        LOGERR("[LOCAL_SQLITE_BMS] commit failed: %s", err.c_str());
        local_sqlite::rollback(db_, nullptr);
        return false;
    }

    return true;
}

void LocalSqliteBmsArchiveSink::enqueueTask_(Task&& task)
{
    std::lock_guard<std::mutex> lk(q_mtx_);

    if (q_.size() >= cfg_.queue_capacity) {
        auto it = q_.begin();
        for (; it != q_.end(); ++it) {
            if (it->kind == TaskKind::LatestRefresh) {
                q_.erase(it);
                break;
            }
        }
        if (q_.size() >= cfg_.queue_capacity && !q_.empty()) {
            q_.pop_front();
        }
    }

    q_.push_back(std::move(task));
    q_cv_.notify_one();
}

bool LocalSqliteBmsArchiveSink::popTask_(Task& out)
{
    std::unique_lock<std::mutex> lk(q_mtx_);
    q_cv_.wait(lk, [&] { return stop_ || !q_.empty(); });

    if (stop_ && q_.empty()) {
        return false;
    }

    out = std::move(q_.front());
    q_.pop_front();
    return true;
}