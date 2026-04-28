//
// Created by lxy on 2026/4/18.
//

#ifndef ENERGYSTORAGE_FAULT_HISTORY_CACHE_H
#define ENERGYSTORAGE_FAULT_HISTORY_CACHE_H

#pragma once

#include <cstdint>
#include <string>
#include <vector>

class SqliteFaultSink;
struct FaultHistoryDbRecord;

namespace control {

/**
 * 给 HMI/页管理使用的历史故障显示行
 *
 * 说明：
 * 1. 这里先保留你当前历史页真正需要的字段：
 *    - code
 *    - first_on_ms
 *    - clear_ms
 * 2. name/classification/priority_rank 先一并带上，
 *    后面若你想做排序/调试/扩展显示，不用再改 DB 查询层
 */
struct HistoryFaultViewRow {
    int64_t db_id{0};

    uint16_t code{0};
    uint64_t first_on_ms{0};
    uint64_t clear_ms{0};

    uint16_t seq_no{0};
    uint16_t state{0};

    std::string name;
    std::string classification;
    int priority_rank{9999};
};

/**
 * 历史故障缓存层
 *
 * 设计目标：
 * 1. SQLite 是真源
 * 2. 内存只做热点缓存 + 当前窗口缓存
 * 3. 默认：
 *    - 每页 5 条
 *    - 热点缓存 10 条
 *    - 窗口缓存 3 页（上一页 + 当前页 + 下一页）
 */
class FaultHistoryCache {
public:
    FaultHistoryCache() = default;

    bool bindDb(SqliteFaultSink* db);

    void setPageSize(uint16_t n)      { if (n > 0) page_size_ = n; }
    void setHotCacheRows(uint16_t n)  { if (n > 0) hot_rows_limit_ = n; }
    void setWindowPages(uint16_t n)   { if (n > 0) window_pages_ = n; }

    uint16_t pageSize() const         { return page_size_; }
    uint16_t hotCacheRows() const     { return hot_rows_limit_; }
    uint16_t windowPages() const      { return window_pages_; }

    bool refreshMeta();                     // total_rows / total_pages
    bool refreshHotCache();                 // 最新 N 条
    bool refreshWindow(uint16_t center_page); // 以 center_page 为中心的窗口
    bool ensurePageLoaded(uint16_t page);   // 若目标页不在窗口内则刷新窗口

    void invalidate();
    bool isValid() const { return valid_; }

    uint32_t totalRows() const { return total_rows_; }
    uint16_t totalPages() const { return total_pages_; }

    uint16_t windowStartPage() const { return window_start_page_; }
    uint16_t windowEndPage() const { return window_end_page_; }

    bool hasPageInWindow(uint16_t page) const;

    // 返回指定页的 0~page_size_ 条数据（最新在前）
    std::vector<HistoryFaultViewRow> pageRows(uint16_t page) const;

private:
    static HistoryFaultViewRow fromDb_(const FaultHistoryDbRecord& r);

    bool sliceFromWindow_(uint16_t page, std::vector<HistoryFaultViewRow>& out) const;
    bool sliceFromHot_(uint16_t page, std::vector<HistoryFaultViewRow>& out) const;

    static uint16_t calcTotalPages_(uint32_t total_rows, uint16_t page_size);
    uint16_t clampPage_(uint16_t page) const;

private:
    SqliteFaultSink* db_{nullptr};

    uint16_t page_size_{5};
    uint16_t hot_rows_limit_{10};
    uint16_t window_pages_{3};   // 当前按奇数页窗口设计，默认 3 页

    bool valid_{false};

    uint32_t total_rows_{0};
    uint16_t total_pages_{0};

    // 热点缓存：最新 N 条
    std::vector<HistoryFaultViewRow> hot_rows_;

    // 窗口缓存：连续若干页
    std::vector<HistoryFaultViewRow> window_rows_;
    uint16_t window_start_page_{0};
    uint16_t window_end_page_{0};
};

} // namespace control

#endif // ENERGYSTORAGE_FAULT_HISTORY_CACHE_H