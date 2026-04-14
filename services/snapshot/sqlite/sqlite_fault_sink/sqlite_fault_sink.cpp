//
// Created by lxy on 2026/4/7.
//

#include "sqlite_fault_sink.h"


SqliteFaultSink::SqliteFaultSink(Config cfg)
    : cfg_(std::move(cfg)) {}

SqliteFaultSink::~SqliteFaultSink()
{
    close();
}

bool SqliteFaultSink::open()
{
    if (opened_) return true;

    if (sqlite3_open(cfg_.db_path.c_str(), &db_) != SQLITE_OK) {
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
        return false;
    }

    sqlite3_busy_timeout(db_, static_cast<int>(cfg_.busy_timeout_ms));

    if (!applyPragmas_()) {
        close();
        return false;
    }

    if (!initSchema_()) {
        close();
        return false;
    }

    opened_ = true;
    return true;
}

void SqliteFaultSink::close()
{
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
    opened_ = false;
}

bool SqliteFaultSink::applyPragmas_()
{
    if (!db_) return false;

    char* err = nullptr;
    const char* sql =
        "PRAGMA journal_mode=WAL;"
        "PRAGMA synchronous=NORMAL;"
        "PRAGMA temp_store=MEMORY;";

    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

bool SqliteFaultSink::initSchema_()
{
    if (!db_) return false;

    const char* sql =
        "CREATE TABLE IF NOT EXISTS fault_history ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "code INTEGER NOT NULL,"
        "first_on_ms INTEGER NOT NULL,"
        "clear_ms INTEGER NOT NULL DEFAULT 0,"
        "seq_no INTEGER NOT NULL,"
        "state INTEGER NOT NULL DEFAULT 0,"
        "name TEXT DEFAULT '',"
        "classification TEXT DEFAULT '',"
        "priority_rank INTEGER DEFAULT 9999"
        ");"
        "CREATE INDEX IF NOT EXISTS idx_fault_history_code ON fault_history(code);"
        "CREATE INDEX IF NOT EXISTS idx_fault_history_first_on_ms ON fault_history(first_on_ms);"
        "CREATE INDEX IF NOT EXISTS idx_fault_history_clear_ms ON fault_history(clear_ms);";

    char* err = nullptr;
    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

bool SqliteFaultSink::insertHistoryBegin(const FaultHistoryDbRecord& rec, int64_t& out_row_id)
{
    if (!opened_ || !db_) return false;

    const char* sql =
        "INSERT INTO fault_history "
        "(code, first_on_ms, clear_ms, seq_no, state, name, classification, priority_rank) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }

    sqlite3_bind_int(stmt, 1, static_cast<int>(rec.code));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(rec.first_on_ms));
    sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(rec.clear_ms));
    sqlite3_bind_int(stmt, 4, static_cast<int>(rec.seq_no));
    sqlite3_bind_int(stmt, 5, static_cast<int>(rec.state));
    sqlite3_bind_text(stmt, 6, rec.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 7, rec.classification.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 8, rec.priority_rank);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) return false;

    out_row_id = sqlite3_last_insert_rowid(db_);
    return true;
}

bool SqliteFaultSink::markHistoryCleared(uint16_t code, uint64_t clear_ms)
{
    if (!opened_ || !db_) return false;

    const char* sql =
        "UPDATE fault_history "
        "SET clear_ms = ?, state = 0 "
        "WHERE id = ("
        "  SELECT id FROM fault_history "
        "  WHERE code = ? AND clear_ms = 0 "
        "  ORDER BY id DESC LIMIT 1"
        ");";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(clear_ms));
    sqlite3_bind_int(stmt, 2, static_cast<int>(code));

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    return rc == SQLITE_DONE;
}

bool SqliteFaultSink::loadRecentHistory(std::vector<FaultHistoryDbRecord>& out)
{
    out.clear();
    if (!opened_ || !db_) return false;

    const char* sql =
        "SELECT id, code, first_on_ms, clear_ms, seq_no, state, name, classification, priority_rank "
        "FROM fault_history "
        "ORDER BY id DESC LIMIT ?;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }

    sqlite3_bind_int(stmt, 1, static_cast<int>(cfg_.load_limit));

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        FaultHistoryDbRecord rec;
        rec.id = sqlite3_column_int64(stmt, 0);
        rec.code = static_cast<uint16_t>(sqlite3_column_int(stmt, 1));
        rec.first_on_ms = static_cast<uint64_t>(sqlite3_column_int64(stmt, 2));
        rec.clear_ms = static_cast<uint64_t>(sqlite3_column_int64(stmt, 3));
        rec.seq_no = static_cast<uint16_t>(sqlite3_column_int(stmt, 4));
        rec.state = static_cast<uint16_t>(sqlite3_column_int(stmt, 5));

        const unsigned char* s6 = sqlite3_column_text(stmt, 6);
        const unsigned char* s7 = sqlite3_column_text(stmt, 7);
        rec.name = s6 ? reinterpret_cast<const char*>(s6) : "";
        rec.classification = s7 ? reinterpret_cast<const char*>(s7) : "";
        rec.priority_rank = sqlite3_column_int(stmt, 8);

        out.push_back(rec);
    }

    sqlite3_finalize(stmt);

    std::reverse(out.begin(), out.end());
    return true;
}



























