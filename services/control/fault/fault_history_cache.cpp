//
// Created by lxy on 2026/4/18.
//

#include "fault_history_cache.h"

#include <algorithm>

#include "logger.h"
#include "../../snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"

namespace control {

bool FaultHistoryCache::bindDb(SqliteFaultSink* db)
{
    db_ = db;
    invalidate();
    valid_ = (db_ != nullptr);
    return valid_;
}

void FaultHistoryCache::invalidate()
{
    total_rows_ = 0;
    total_pages_ = 0;

    hot_rows_.clear();

    window_rows_.clear();
    window_start_page_ = 0;
    window_end_page_ = 0;

    valid_ = (db_ != nullptr);
}

uint16_t FaultHistoryCache::calcTotalPages_(uint32_t total_rows, uint16_t page_size)
{
    if (page_size == 0 || total_rows == 0) return 0;
    return static_cast<uint16_t>((total_rows + page_size - 1u) / page_size);
}

uint16_t FaultHistoryCache::clampPage_(uint16_t page) const
{
    if (total_pages_ == 0) return 1;
    if (page < 1) return 1;
    if (page > total_pages_) return total_pages_;
    return page;
}

HistoryFaultViewRow FaultHistoryCache::fromDb_(const FaultHistoryDbRecord& r)
{
    HistoryFaultViewRow row;
    row.db_id = r.id;
    row.code = r.code;
    row.first_on_ms = r.first_on_ms;
    row.clear_ms = r.clear_ms;
    row.seq_no = r.seq_no;
    row.state = r.state;
    row.name = r.name;
    row.classification = r.classification;
    row.priority_rank = r.priority_rank;
    return row;
}

    bool FaultHistoryCache::refreshMeta()
{
    if (!db_) return false;

    uint32_t total = 0;
    if (!db_->countHistoryRows(total, true)) {
        // 20260418
        LOG_THROTTLE_MS("fault_history_cache_meta_fail", 1000, LOGINFO,
                        "[FAULT][HIS_CACHE][META] countHistoryRows failed");
        return false;
    }

    total_rows_ = total;
    total_pages_ = calcTotalPages_(total_rows_, page_size_);
    valid_ = true;

    // 20260418
    LOG_THROTTLE_MS("fault_history_cache_meta", 1000, LOGINFO,
                    "[FAULT][HIS_CACHE][META] total_rows=%u total_pages=%u page_size=%u hot=%u window_pages=%u",
                    (unsigned)total_rows_,
                    (unsigned)total_pages_,
                    (unsigned)page_size_,
                    (unsigned)hot_rows_limit_,
                    (unsigned)window_pages_);
    return true;
}

bool FaultHistoryCache::refreshHotCache()
{
    if (!db_) return false;

    std::vector<FaultHistoryDbRecord> rows;
    if (!db_->loadLatestHistory(hot_rows_limit_, rows, true)) {
        // 20260418
        LOG_THROTTLE_MS("fault_history_cache_hot_fail", 1000, LOGINFO,
                        "[FAULT][HIS_CACHE][HOT] loadLatestHistory failed limit=%u",
                        (unsigned)hot_rows_limit_);
        return false;
    }

    hot_rows_.clear();
    hot_rows_.reserve(rows.size());
    for (const auto& r : rows) {
        hot_rows_.push_back(fromDb_(r));
    }

    valid_ = true;
    // 20260418
    LOG_THROTTLE_MS("fault_history_cache_hot", 1000, LOGINFO,
                    "[FAULT][HIS_CACHE][HOT] rows=%zu first_code=0x%04X first_id=%lld",
                    hot_rows_.size(),
                    (unsigned)(hot_rows_.empty() ? 0 : hot_rows_.front().code),
                    (long long)(hot_rows_.empty() ? 0 : hot_rows_.front().db_id));
    return true;
}

bool FaultHistoryCache::refreshWindow(uint16_t center_page)
{
    if (!db_) return false;

    if (!refreshMeta()) {
        return false;
    }

    if (total_pages_ == 0) {
        window_rows_.clear();
        window_start_page_ = 0;
        window_end_page_ = 0;
        valid_ = true;
        // 20260418
        LOG_THROTTLE_MS("fault_history_cache_window_empty", 1000, LOGINFO,
                        "[FAULT][HIS_CACHE][WINDOW] empty total_pages=0");
        return true;
    }

    const uint16_t page = clampPage_(center_page);

    // 当前按 3 页窗口设计：
    // center=5 -> [4,5,6]
    // center=1 -> [1,2,3]
    // center=last -> [last-2,last-1,last]
    uint16_t start_page = 1;
    uint16_t end_page = 1;

    if (window_pages_ <= 1) {
        start_page = page;
        end_page = page;
    } else {
        const uint16_t half = static_cast<uint16_t>(window_pages_ / 2);

        if (page <= 1 + half) {
            start_page = 1;
            end_page = std::min<uint16_t>(total_pages_, window_pages_);
        } else if (page + half >= total_pages_) {
            end_page = total_pages_;
            start_page = (total_pages_ > window_pages_ - 1)
                       ? static_cast<uint16_t>(total_pages_ - window_pages_ + 1)
                       : 1;
        } else {
            start_page = static_cast<uint16_t>(page - half);
            end_page = static_cast<uint16_t>(start_page + window_pages_ - 1);
        }
    }

    const uint32_t offset = static_cast<uint32_t>(start_page - 1u) * page_size_;
    const uint32_t limit =
        static_cast<uint32_t>(end_page - start_page + 1u) * page_size_;

    std::vector<FaultHistoryDbRecord> rows;
    if (!db_->loadHistoryRange(offset, limit, rows, true)) {
        // 20260418
        LOG_THROTTLE_MS("fault_history_cache_window_fail", 1000, LOGINFO,
                        "[FAULT][HIS_CACHE][WINDOW] loadHistoryRange failed center=%u start=%u end=%u offset=%u limit=%u",
                        (unsigned)page,
                        (unsigned)start_page,
                        (unsigned)end_page,
                        (unsigned)offset,
                        (unsigned)limit);
        return false;
    }

    window_rows_.clear();
    window_rows_.reserve(rows.size());
    for (const auto& r : rows) {
        window_rows_.push_back(fromDb_(r));
    }

    window_start_page_ = start_page;
    window_end_page_ = end_page;
    valid_ = true;
    // 20260418
    LOG_THROTTLE_MS("fault_history_cache_window", 1000, LOGINFO,
                    "[FAULT][HIS_CACHE][WINDOW] center=%u start=%u end=%u rows=%zu offset=%u limit=%u first_code=0x%04X first_id=%lld",
                    (unsigned)page,
                    (unsigned)window_start_page_,
                    (unsigned)window_end_page_,
                    window_rows_.size(),
                    (unsigned)offset,
                    (unsigned)limit,
                    (unsigned)(window_rows_.empty() ? 0 : window_rows_.front().code),
                    (long long)(window_rows_.empty() ? 0 : window_rows_.front().db_id));
    return true;
}

bool FaultHistoryCache::hasPageInWindow(uint16_t page) const
{
    if (window_start_page_ == 0 || window_end_page_ == 0) return false;
    return page >= window_start_page_ && page <= window_end_page_;
}

bool FaultHistoryCache::ensurePageLoaded(uint16_t page)
{
    if (!db_) return false;

    if (!valid_) {
        if (!refreshMeta()) return false;
    }

    if (total_pages_ == 0) {
        window_rows_.clear();
        window_start_page_ = 0;
        window_end_page_ = 0;
        return true;
    }

    const uint16_t p = clampPage_(page);
    if (hasPageInWindow(p)) {
        return true;
    }

    return refreshWindow(p);
}

bool FaultHistoryCache::sliceFromWindow_(uint16_t page,
                                         std::vector<HistoryFaultViewRow>& out) const
{
    out.clear();

    if (!hasPageInWindow(page)) {
        return false;
    }

    const uint16_t local_page_index =
        static_cast<uint16_t>(page - window_start_page_);
    const size_t begin =
        static_cast<size_t>(local_page_index) * page_size_;

    if (begin >= window_rows_.size()) {
        return true;
    }

    const size_t end =
        std::min(window_rows_.size(), begin + static_cast<size_t>(page_size_));

    out.insert(out.end(), window_rows_.begin() + static_cast<long long>(begin),
                          window_rows_.begin() + static_cast<long long>(end));
    return true;
}

bool FaultHistoryCache::sliceFromHot_(uint16_t page,
                                      std::vector<HistoryFaultViewRow>& out) const
{
    out.clear();

    if (page == 0 || page_size_ == 0) {
        return false;
    }

    const uint32_t offset = static_cast<uint32_t>(page - 1u) * page_size_;
    if (offset >= hot_rows_.size()) {
        return false;
    }

    const size_t begin = static_cast<size_t>(offset);
    const size_t end =
        std::min(hot_rows_.size(), begin + static_cast<size_t>(page_size_));

    out.insert(out.end(), hot_rows_.begin() + static_cast<long long>(begin),
                          hot_rows_.begin() + static_cast<long long>(end));
    return true;
}

std::vector<HistoryFaultViewRow> FaultHistoryCache::pageRows(uint16_t page) const
{
    std::vector<HistoryFaultViewRow> out;

    if (!valid_) {
        return out;
    }

    if (total_pages_ == 0) {
        return out;
    }

    const uint16_t p = clampPage_(page);

    // 优先从窗口缓存切页
    if (sliceFromWindow_(p, out)) {
        return out;
    }

    // 若窗口尚未覆盖，但页号仍落在热点缓存范围内，则允许直接用热点缓存
    // 默认 10 条热点缓存，正好覆盖前 2 页（page_size=5）
    if (sliceFromHot_(p, out)) {
        return out;
    }

    // 否则返回空，由上层先调用 ensurePageLoaded(page)
    return out;
}

} // namespace control