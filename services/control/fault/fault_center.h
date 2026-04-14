//
// Created by lxy on 2026/4/7.
//

#ifndef ENERGYSTORAGE_FAULT_CENTER_H
#define ENERGYSTORAGE_FAULT_CENTER_H

#pragma once

#include <cstdint>
#include <unordered_set>
#include <vector>
#include <mutex>


class HMIProto;
class SqliteFaultSink;
struct FaultHistoryDbRecord;

namespace control {

class FaultCatalog;

struct FaultCenterHistRecord {
    uint16_t code{0};
    uint64_t first_on_ms{0};
    uint64_t clear_ms{0};

    uint16_t seq_no{0};   // 历史显示序号
    uint16_t state{0};    // 0=已清除，1=仍活动
};

struct FaultCenterCurrentRow {
    uint16_t seq_no{0};
    uint16_t code{0};
    uint32_t on_time{0};   // 秒级32位时间戳
};

struct FaultCenterHistoryRow {
    uint16_t seq_no{0};
    uint16_t code{0};
    uint32_t on_time{0};   // 秒级32位时间戳
    uint32_t off_time{0};  // 秒级32位时间戳
    uint16_t state{0};
};

    class FaultCenter {
    public:
        void bindCatalog(const FaultCatalog* cat) { cat_ = cat; }
        void bindFaultDb(SqliteFaultSink* db);

        void restoreHistoryFromDb(const std::vector<FaultHistoryDbRecord>& rows);

        void setActive(uint16_t code, bool on);
        bool isActive(uint16_t code) const;

        void enterHistoryView();
        void enterCurrentView();

        void nextCurrentPage();
        void prevCurrentPage();
        void nextHistoryPage();
        void prevHistoryPage();

        void flushToHmi(HMIProto& hmi) const;

        // ===== 第八批：联调用只读接口 =====
        std::vector<FaultCenterCurrentRow> debugCurrentRows() const;
        std::vector<uint16_t> debugCurrentVisibleCodes() const;
        uint16_t debugCurrentPageIndex() const;
        uint16_t debugCurrentTotalPages() const;
private:
    std::vector<FaultCenterCurrentRow> buildCurrentRows_() const;
    std::vector<FaultCenterHistoryRow> buildHistoryRows_() const;

    std::vector<uint16_t> collectCurrentVisibleCodes_() const;
    std::vector<FaultCenterHistRecord> collectHistoryVisible_() const;

    uint16_t currentTotalPages_() const;
    uint16_t historyTotalPages_() const;

    void trimHistoryIfNeeded_();
    void clampPages_();
    uint64_t lastEventTimeOf_(uint16_t code) const;
    static uint32_t encodeTime_(uint64_t ts_ms);

private:
    const FaultCatalog* cat_{nullptr};
    SqliteFaultSink* fault_db_{nullptr};

    std::unordered_set<uint16_t> active_;

    bool in_history_view_{false};

    uint16_t current_page_{0};
    uint16_t history_page_{0};

    uint16_t next_hist_seq_{1};
    std::vector<FaultCenterHistRecord> history_;

    static constexpr size_t kMaxHistoryRecords = 1000;
        mutable std::mutex mtx_;
};

} // namespace control

#endif // ENERGYSTORAGE_FAULT_CENTER_H