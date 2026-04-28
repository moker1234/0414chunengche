//
// Created by lxy on 2026/4/20.
//

#include "local_sqlite_common.h"

#include <filesystem>
#include <sstream>
#include <iomanip>
#include <cctype>

#include "logger.h"

namespace fs = std::filesystem;

namespace local_sqlite {

namespace {

static bool isHexChar_(char c)
{
    return std::isxdigit(static_cast<unsigned char>(c)) != 0;
}

static std::string trim_(const std::string& s)
{
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])) != 0) {
        ++b;
    }

    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])) != 0) {
        --e;
    }

    return s.substr(b, e - b);
}

} // namespace

bool ensureParentDir(const std::string& db_path, std::string* err)
{
    try {
        const fs::path p(db_path);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
        return true;
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
}

bool openDb(const std::string& db_path,
            uint32_t busy_timeout_ms,
            sqlite3** out_db,
            std::string* err)
{
    if (!out_db) {
        if (err) *err = "out_db is null";
        return false;
    }

    *out_db = nullptr;

    if (!ensureParentDir(db_path, err)) {
        return false;
    }

    sqlite3* db = nullptr;
    const int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc != SQLITE_OK || !db) {
        if (err) {
            *err = db ? sqlite3_errmsg(db) : "sqlite3_open returned null db";
        }
        if (db) {
            sqlite3_close(db);
            db = nullptr;
        }
        return false;
    }

    sqlite3_busy_timeout(db, static_cast<int>(busy_timeout_ms));
    *out_db = db;
    return true;
}

void closeDb(sqlite3*& db)
{
    if (db) {
        sqlite3_close(db);
        db = nullptr;
    }
}

bool exec(sqlite3* db, const char* sql, std::string* err)
{
    if (!db || !sql) {
        if (err) *err = "db or sql is null";
        return false;
    }

    char* errmsg = nullptr;
    const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &errmsg);
    if (rc != SQLITE_OK) {
        if (err) {
            *err = errmsg ? errmsg : sqlite3_errmsg(db);
        }
        if (errmsg) {
            sqlite3_free(errmsg);
        }
        return false;
    }

    if (errmsg) {
        sqlite3_free(errmsg);
    }
    return true;
}

bool applyCommonPragmas(sqlite3* db, std::string* err)
{
    static const char* kSql =
        "PRAGMA journal_mode=WAL;"
        "PRAGMA synchronous=FULL;"
        "PRAGMA wal_autocheckpoint=1000;"
        "PRAGMA foreign_keys=ON;"
        "PRAGMA temp_store=MEMORY;";

    return exec(db, kSql, err);
}

bool beginImmediate(sqlite3* db, std::string* err)
{
    return exec(db, "BEGIN IMMEDIATE;", err);
}

bool commit(sqlite3* db, std::string* err)
{
    return exec(db, "COMMIT;", err);
}

bool rollback(sqlite3* db, std::string* err)
{
    return exec(db, "ROLLBACK;", err);
}

bool initMetaSchema(sqlite3* db,
                    const std::string& sink_name,
                    int schema_version,
                    std::string* err)
{
    if (!exec(db,
              "CREATE TABLE IF NOT EXISTS archive_meta ("
              "  k TEXT PRIMARY KEY,"
              "  v TEXT NOT NULL"
              ");",
              err)) {
        return false;
    }

    sqlite3_stmt* stmt = nullptr;
    const char* sql =
        "INSERT OR REPLACE INTO archive_meta(k, v) VALUES(?, ?);";

    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        if (err) *err = sqlite3_errmsg(db);
        return false;
    }

    auto bindAndStep = [&](const std::string& k, const std::string& v) -> bool {
        sqlite3_reset(stmt);
        sqlite3_clear_bindings(stmt);

        if (sqlite3_bind_text(stmt, 1, k.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) {
            if (err) *err = sqlite3_errmsg(db);
            return false;
        }
        if (sqlite3_bind_text(stmt, 2, v.c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) {
            if (err) *err = sqlite3_errmsg(db);
            return false;
        }

        const int rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            if (err) *err = sqlite3_errmsg(db);
            return false;
        }
        return true;
    };

    const bool ok =
        bindAndStep("sink_name", sink_name) &&
        bindAndStep("schema_version", std::to_string(schema_version));

    sqlite3_finalize(stmt);
    return ok;
}

bool parseHexBytes(const std::string& text,
                   std::vector<uint8_t>& out,
                   std::string* err)
{
    out.clear();

    const std::string s = trim_(text);
    if (s.empty()) {
        return true;
    }

    std::istringstream iss(s);
    std::string tok;
    while (iss >> tok) {
        if (tok.size() != 2 || !isHexChar_(tok[0]) || !isHexChar_(tok[1])) {
            if (err) *err = "invalid hex token: " + tok;
            out.clear();
            return false;
        }

        unsigned int v = 0;
        std::istringstream hs(tok);
        hs >> std::hex >> v;
        if (hs.fail()) {
            if (err) *err = "parse hex token failed: " + tok;
            out.clear();
            return false;
        }
        out.push_back(static_cast<uint8_t>(v & 0xFFu));
    }

    return true;
}

} // namespace local_sqlite
