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
    bool markHistoryCleared(uint16_t code, uint64_t clear_ms);
    bool loadRecentHistory(std::vector<FaultHistoryDbRecord>& out);

private:
    bool applyPragmas_();
    bool initSchema_();

private:
    Config cfg_;
    sqlite3* db_{nullptr};
    bool opened_{false};
};



#endif //ENERGYSTORAGE_SQLITE_FAULT_SINK_H
