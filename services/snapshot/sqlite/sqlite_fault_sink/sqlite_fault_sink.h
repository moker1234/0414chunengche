//
// Created by lxy on 2026/4/7.
//

#ifndef ENERGYSTORAGE_SQLITE_FAULT_SINK_H
#define ENERGYSTORAGE_SQLITE_FAULT_SINK_H

#include <cstdint>
#include <string>
#include <vector>
#include <sqlite3.h>

struct FaultHistoryDbRecord {
    int64_t id{0};

    uint16_t code{0};
    uint64_t first_on_ms{0};
    uint64_t clear_ms{0};

    uint16_t seq_no{0};
    uint16_t state{0};

    std::string name;
    std::string classification;
    int priority_rank{9999};
};

class SqliteFaultSink final {
public:
    struct Config {
        std::string db_path{"/mnt/sqlite_tfcard/json_data.db"};
        uint32_t busy_timeout_ms{3000};
        uint32_t load_limit{1000};
    };

    explicit SqliteFaultSink(Config cfg);
    ~SqliteFaultSink();

    bool open();
    void close();

    bool insertHistoryBegin(const FaultHistoryDbRecord& rec, int64_t& out_row_id);

    // 兼容旧接口：按 code 清除“最新未清除”的那条
    bool markHistoryCleared(uint16_t code, uint64_t clear_ms);

    // 新接口：按 row_id 精确清除
    bool markHistoryClearedById(int64_t row_id, uint64_t clear_ms);

    // 兼容旧接口：启动时加载最近 history
    bool loadRecentHistory(std::vector<FaultHistoryDbRecord>& out);

    // 新接口：历史页缓存用
    bool countHistoryRows(uint32_t& out_total_rows, bool only_cleared = true);
    bool loadLatestHistory(uint32_t limit,
                           std::vector<FaultHistoryDbRecord>& out,
                           bool only_cleared = true);
    bool loadHistoryPage(uint32_t page_no,
                         uint32_t page_size,
                         std::vector<FaultHistoryDbRecord>& out,
                         bool only_cleared = true);
    bool loadHistoryRange(uint32_t offset,
                          uint32_t limit,
                          std::vector<FaultHistoryDbRecord>& out,
                          bool only_cleared = true);

private:
    bool applyPragmas_();
    bool initSchema_();

    bool bindRowsFromStmt_(sqlite3_stmt* stmt,
                           std::vector<FaultHistoryDbRecord>& out);

private:
    Config cfg_;
    sqlite3* db_{nullptr};
    bool opened_{false};
};

#endif // ENERGYSTORAGE_SQLITE_FAULT_SINK_H