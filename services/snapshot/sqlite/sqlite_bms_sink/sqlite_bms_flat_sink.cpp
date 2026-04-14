//
// Created by lxy on 2026/3/19.
//

#include "sqlite_bms_flat_sink.h"

#include <filesystem>
#include <utility>

#include "../../utils/logger/logger.h"
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

/* ================= key helpers ================= */

std::string SqliteBmsFlatSink::healthKeyJson_(const snapshot::BmsInstanceData& inst)
{
    nlohmann::json j;
    j["online"]               = inst.health.online;
    j["last_ok_ms"]           = inst.health.last_ok_ms;
    j["disconnect_window_ms"] = inst.health.disconnect_window_ms;
    j["last_offline_ms"]      = inst.health.last_offline_ms;
    j["disconnect_count"]     = inst.health.disconnect_count;
    return j.dump();
}

/* ================= ctor / dtor ================= */

SqliteBmsFlatSink::SqliteBmsFlatSink(Config cfg)
    : cfg_(std::move(cfg))
{
    opened_ = openDb_();
    if (!opened_) {
        LOGERR("[SQLITE_BMS_FLAT] open db failed: %s", cfg_.db_path.c_str());
        return;
    }

    if (!applyPragmas_()) {
        LOGERR("[SQLITE_BMS_FLAT] apply pragmas failed");
    }

    if (!initSchema_()) {
        LOGERR("[SQLITE_BMS_FLAT] init schema failed");
    }
}

SqliteBmsFlatSink::~SqliteBmsFlatSink()
{
    closeDb_();
}

/* ================= db ================= */

bool SqliteBmsFlatSink::openDb_()
{
    try {
        const fs::path p(cfg_.db_path);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        LOGERR("[SQLITE_BMS_FLAT] create parent dir failed: %s", e.what());
        return false;
    }

    const int rc = sqlite3_open(cfg_.db_path.c_str(), &db_);
    if (rc != SQLITE_OK || !db_) {
        LOGERR("[SQLITE_BMS_FLAT] sqlite3_open failed rc=%d err=%s",
               rc, db_ ? sqlite3_errmsg(db_) : "null");
        return false;
    }

    sqlite3_busy_timeout(db_, static_cast<int>(cfg_.busy_timeout_ms));
    return true;
}

void SqliteBmsFlatSink::closeDb_()
{
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool SqliteBmsFlatSink::applyPragmas_()
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
            LOGERR("[SQLITE_BMS_FLAT] pragma failed sql=%s err=%s",
                   sqls[i], err ? err : "unknown");
            if (err) sqlite3_free(err);
            return false;
        }
    }
    return true;
}

bool SqliteBmsFlatSink::initSchema_()
{
    if (!db_) return false;

    const char* sql = R"SQL(
CREATE TABLE IF NOT EXISTS bms_snapshot_main (
    snapshot_id     INTEGER PRIMARY KEY,
    ts_ms           INTEGER NOT NULL,
    instance_count  INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_bms_snapshot_main_ts
ON bms_snapshot_main(ts_ms);

CREATE TABLE IF NOT EXISTS bms_instance (
    id          INTEGER PRIMARY KEY,
    bms_index   INTEGER NOT NULL UNIQUE,
    name        TEXT    NOT NULL UNIQUE
);

CREATE TABLE IF NOT EXISTS bms_group_type (
    id          INTEGER PRIMARY KEY,
    group_name  TEXT NOT NULL UNIQUE
);

CREATE TABLE IF NOT EXISTS bms_instance_health (
    id                  INTEGER PRIMARY KEY,
    snapshot_id         INTEGER NOT NULL,
    ts_ms               INTEGER NOT NULL,
    bms_id              INTEGER NOT NULL,
    online              INTEGER NOT NULL,
    last_ok_ms          INTEGER NOT NULL,
    timeout_count       INTEGER NOT NULL,
    last_msg_group_id   INTEGER,
    FOREIGN KEY (bms_id) REFERENCES bms_instance(id),
    FOREIGN KEY (last_msg_group_id) REFERENCES bms_group_type(id)
);

CREATE INDEX IF NOT EXISTS idx_bms_instance_health_bms_ts
ON bms_instance_health(bms_id, ts_ms DESC);

CREATE TABLE IF NOT EXISTS bms_group_raw (
    id              INTEGER PRIMARY KEY,
    snapshot_id     INTEGER NOT NULL,
    ts_ms           INTEGER NOT NULL,
    bms_id          INTEGER NOT NULL,
    group_type_id   INTEGER NOT NULL,
    group_ts_ms     INTEGER NOT NULL,
    can_id          INTEGER NOT NULL,
    raw_hex         TEXT    NOT NULL,
    rx_count        INTEGER NOT NULL,
    cycle_ms        INTEGER,
    FOREIGN KEY (bms_id) REFERENCES bms_instance(id),
    FOREIGN KEY (group_type_id) REFERENCES bms_group_type(id)
);

CREATE INDEX IF NOT EXISTS idx_bms_group_raw_bms_group_ts
ON bms_group_raw(bms_id, group_type_id, ts_ms DESC);

CREATE INDEX IF NOT EXISTS idx_bms_group_raw_ts
ON bms_group_raw(ts_ms DESC);
)SQL";

    char* err = nullptr;
    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS_FLAT] init schema failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }

    return true;
}

bool SqliteBmsFlatSink::beginTx_()
{
    char* err = nullptr;
    const int rc = sqlite3_exec(db_, "BEGIN IMMEDIATE TRANSACTION;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS_FLAT] begin tx failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

bool SqliteBmsFlatSink::commitTx_()
{
    char* err = nullptr;
    const int rc = sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_BMS_FLAT] commit tx failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

void SqliteBmsFlatSink::rollbackTx_()
{
    sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
}

/* ================= dictionary helpers ================= */

bool SqliteBmsFlatSink::ensureBmsInstance_(const std::string& instance_name,
                                           uint32_t bms_index,
                                           int64_t& out_bms_id)
{
    const char* ins_sql =
        "INSERT OR IGNORE INTO bms_instance(bms_index, name) VALUES(?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, ins_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare ensure bms_instance insert failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int(stmt, 1, static_cast<int>(bms_index));
    sqlite3_bind_text(stmt, 2, instance_name.c_str(), -1, SQLITE_TRANSIENT);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_BMS_FLAT] ensure bms_instance insert failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    const char* sel_sql =
        "SELECT id FROM bms_instance WHERE name=? LIMIT 1;";

    stmt = nullptr;
    rc = sqlite3_prepare_v2(db_, sel_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare ensure bms_instance select failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_text(stmt, 1, instance_name.c_str(), -1, SQLITE_TRANSIENT);

    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        out_bms_id = static_cast<int64_t>(sqlite3_column_int64(stmt, 0));
        sqlite3_finalize(stmt);
        return true;
    }

    sqlite3_finalize(stmt);
    LOGERR("[SQLITE_BMS_FLAT] ensure bms_instance select no row name=%s",
           instance_name.c_str());
    return false;
}

bool SqliteBmsFlatSink::ensureGroupType_(const std::string& group_name,
                                         int64_t& out_group_type_id)
{
    const char* ins_sql =
        "INSERT OR IGNORE INTO bms_group_type(group_name) VALUES(?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, ins_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare ensure group_type insert failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_text(stmt, 1, group_name.c_str(), -1, SQLITE_TRANSIENT);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_BMS_FLAT] ensure group_type insert failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    const char* sel_sql =
        "SELECT id FROM bms_group_type WHERE group_name=? LIMIT 1;";

    stmt = nullptr;
    rc = sqlite3_prepare_v2(db_, sel_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare ensure group_type select failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_text(stmt, 1, group_name.c_str(), -1, SQLITE_TRANSIENT);

    rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        out_group_type_id = static_cast<int64_t>(sqlite3_column_int64(stmt, 0));
        sqlite3_finalize(stmt);
        return true;
    }

    sqlite3_finalize(stmt);
    LOGERR("[SQLITE_BMS_FLAT] ensure group_type select no row name=%s",
           group_name.c_str());
    return false;
}

/* ================= change detection ================= */

bool SqliteBmsFlatSink::groupChanged_(const std::string& instance_name,
                                      const std::string& msg_name,
                                      const snapshot::BmsGroupData& g)
{
    const std::string key = instance_name + "::" + msg_name;
    const std::string cur = g.toJson().dump();

    auto it = last_group_json_cache_.find(key);
    if (it == last_group_json_cache_.end() || it->second != cur) {
        last_group_json_cache_[key] = cur;
        return true;
    }
    return false;
}

bool SqliteBmsFlatSink::healthChanged_(const std::string& instance_name,
                                       const snapshot::BmsInstanceData& inst)
{
    const std::string cur = healthKeyJson_(inst);
    auto it = last_health_json_cache_.find(instance_name);
    if (it == last_health_json_cache_.end() || it->second != cur) {
        last_health_json_cache_[instance_name] = cur;
        return true;
    }
    return false;
}

std::set<std::string> SqliteBmsFlatSink::collectChangedGroups_(const snapshot::BmsSnapshot& snap)
{
    std::set<std::string> out;
    for (const auto& [instance_name, inst] : snap.items) {
        for (const auto& [msg_name, g] : inst.groups) {
            if (groupChanged_(instance_name, msg_name, g)) {
                out.insert(instance_name + "::" + msg_name);
            }
        }
    }
    return out;
}

/* ================= inserts ================= */

bool SqliteBmsFlatSink::insertSnapshotMain_(const snapshot::BmsSnapshot& snap,
                                            int64_t& out_snapshot_id)
{
    const char* sql =
        "INSERT INTO bms_snapshot_main(ts_ms, instance_count) VALUES(?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare snapshot_main failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snap.ts_ms));
    sqlite3_bind_int(stmt, 2, static_cast<int>(snap.items.size()));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_BMS_FLAT] insert snapshot_main failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    out_snapshot_id = static_cast<int64_t>(sqlite3_last_insert_rowid(db_));
    return true;
}

bool SqliteBmsFlatSink::insertInstanceHealth_(int64_t snapshot_id,
                                              uint64_t ts_ms,
                                              const std::string& instance_name,
                                              const snapshot::BmsInstanceData& inst)
{
    int64_t bms_id = 0;
    if (!ensureBmsInstance_(instance_name, inst.meta.bms_index, bms_id)) {
        return false;
    }

    int64_t last_msg_group_id = 0;
    if (!inst.meta.last_msg_name.empty()) {
        if (!ensureGroupType_(inst.meta.last_msg_name, last_msg_group_id)) {
            return false;
        }
    }

    const char* sql =
        "INSERT INTO bms_instance_health("
        "snapshot_id, ts_ms, bms_id, online, last_ok_ms, timeout_count, last_msg_group_id"
        ") VALUES(?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare instance_health failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(bms_id));
    sqlite3_bind_int(stmt,   4, inst.health.online ? 1 : 0);
    sqlite3_bind_int64(stmt, 5, static_cast<sqlite3_int64>(inst.health.last_ok_ms));
    sqlite3_bind_int(stmt,   6, inst.health.online ? 1 : 0);
    sqlite3_bind_int64(stmt, 7, inst.health.last_ok_ms);
    sqlite3_bind_int(stmt,   8, inst.health.disconnect_window_ms);
    sqlite3_bind_int64(stmt, 9, inst.health.last_offline_ms);
    sqlite3_bind_int(stmt,   10, inst.health.disconnect_count);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_BMS_FLAT] insert instance_health failed inst=%s err=%s",
               instance_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteBmsFlatSink::insertGroupRaw_(int64_t snapshot_id,
                                        uint64_t ts_ms,
                                        const std::string& instance_name,
                                        const snapshot::BmsInstanceData& inst,
                                        const std::string& msg_name,
                                        const snapshot::BmsGroupData& g)
{
    int64_t bms_id = 0;
    if (!ensureBmsInstance_(instance_name, inst.meta.bms_index, bms_id)) {
        return false;
    }

    int64_t group_type_id = 0;
    if (!ensureGroupType_(msg_name, group_type_id)) {
        return false;
    }

    const char* sql =
        "INSERT INTO bms_group_raw("
        "snapshot_id, ts_ms, bms_id, group_type_id, group_ts_ms, can_id, raw_hex, rx_count, cycle_ms"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_BMS_FLAT] prepare group_raw failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(bms_id));
    sqlite3_bind_int64(stmt, 4, static_cast<sqlite3_int64>(group_type_id));
    sqlite3_bind_int64(stmt, 5, static_cast<sqlite3_int64>(g.ts_ms));
    sqlite3_bind_int64(stmt, 6, static_cast<sqlite3_int64>(g.can_id));
    sqlite3_bind_text(stmt,  7, g.raw_hex.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   8, static_cast<int>(g.rx_count));
    if (g.cycle_ms >= 0) sqlite3_bind_int(stmt, 9, g.cycle_ms);
    else                 sqlite3_bind_null(stmt, 9);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_BMS_FLAT] insert group_raw failed inst=%s msg=%s err=%s",
               instance_name.c_str(), msg_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

/* ================= main entry ================= */

void SqliteBmsFlatSink::onBmsSnapshot(const snapshot::BmsSnapshot& snap)
{
    if (!opened_ || !db_) return;

    const auto changed_groups = collectChangedGroups_(snap);

    bool any_health_changed = false;
    for (const auto& [instance_name, inst] : snap.items) {
        if (healthChanged_(instance_name, inst)) {
            any_health_changed = true;
        }
    }

    if (!cfg_.write_main_every_snapshot && changed_groups.empty() && !any_health_changed) {
        return;
    }

    if (!beginTx_()) return;

    int64_t snapshot_id = 0;
    if (!insertSnapshotMain_(snap, snapshot_id)) {
        rollbackTx_();
        return;
    }

    // 健康变化
    for (const auto& [instance_name, inst] : snap.items) {
        const auto it = last_health_json_cache_.find(instance_name);
        if (it != last_health_json_cache_.end()) {
            // last_health_json_cache_ 已在 healthChanged_ 里更新过；
            // 这里通过重新比较“本轮变化后缓存是否存在”来决定写入。
            if (!insertInstanceHealth_(snapshot_id, snap.ts_ms, instance_name, inst)) {
                rollbackTx_();
                return;
            }
        }
    }

    // 原始报文变化
    for (const auto& [instance_name, inst] : snap.items) {
        for (const auto& [msg_name, g] : inst.groups) {
            const std::string key = instance_name + "::" + msg_name;
            if (changed_groups.find(key) == changed_groups.end()) continue;

            if (!insertGroupRaw_(snapshot_id, snap.ts_ms, instance_name, inst, msg_name, g)) {
                rollbackTx_();
                return;
            }
        }
    }

    if (!commitTx_()) {
        rollbackTx_();
    }
}