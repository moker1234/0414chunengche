//
// Created by lxy on 2026/4/20.
//

#ifndef ENERGYSTORAGE_LOCAL_SQLITE_COMMON_H
#define ENERGYSTORAGE_LOCAL_SQLITE_COMMON_H


#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sqlite3.h>

namespace local_sqlite {

    /**
     * 通用 SQLite 辅助函数
     *
     * 设计约束：
     * 1. 不持有全局状态
     * 2. 只做打开/关闭/事务/PRAGMA/目录/hex辅助
     * 3. 真正的业务 schema / insert 由各 sink 自己实现
     */

    bool ensureParentDir(const std::string& db_path, std::string* err = nullptr);

    bool openDb(const std::string& db_path,
                uint32_t busy_timeout_ms,
                sqlite3** out_db,
                std::string* err = nullptr);

    void closeDb(sqlite3*& db);

    bool exec(sqlite3* db, const char* sql, std::string* err = nullptr);

    bool applyCommonPragmas(sqlite3* db, std::string* err = nullptr);

    bool beginImmediate(sqlite3* db, std::string* err = nullptr);
    bool commit(sqlite3* db, std::string* err = nullptr);
    bool rollback(sqlite3* db, std::string* err = nullptr);

    bool initMetaSchema(sqlite3* db,
                        const std::string& sink_name,
                        int schema_version,
                        std::string* err = nullptr);

    /**
     * "41 00 C7 00 41 00 07 00" -> {0x41,0x00,...}
     * 供后续 BMS payload BLOB 入库使用。
     */
    bool parseHexBytes(const std::string& text,
                       std::vector<uint8_t>& out,
                       std::string* err = nullptr);

} // namespace local_sqlite



#endif //ENERGYSTORAGE_LOCAL_SQLITE_COMMON_H
