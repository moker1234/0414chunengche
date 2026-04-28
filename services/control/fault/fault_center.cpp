//
// Created by lxy on 2026/4/7.
//

#include "fault_center.h"

#include <algorithm>
#include <cstdio>
#include  "sstream"
#include <ctime>
#include <string>

// #include "./fault_addr_layout.h"
#include "fault_catalog.h"
#include "getTime.h"
#include "logger.h"

#include "../../protocol/rs485/hmi/hmi_proto.h"
#include "../../snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"

namespace control
{
    namespace
    {
        // 当前项目故障页一阶段仍按 5 行组织。
        // 注意：这里不是 HMI 地址定义，只是当前页内行数。
        // HMI 地址已经迁移到 fault_hmi_layout.jsonl。
        static constexpr uint16_t kFaultPageRows = 5;
    }
    bool FaultCenter::shouldRecordToDb_(uint16_t code) const
    {
        // 没有目录时，保守策略：不写库
        if (!cat_) return false;

        const FaultMeta* meta = cat_->metaOf(code);
        if (!meta) return false;

        return meta->record_db;
    }

    void FaultCenter::bindFaultDb(SqliteFaultSink* db)
    {
        fault_db_ = db;

        // DB 绑定变化后，后续历史缓存层应至少刷新一次
        history_dirty_ = true;
        ++history_version_;
    }

    void FaultCenter::restoreHistoryFromDb(const std::vector<FaultHistoryDbRecord>& rows)
    {
        history_.clear();
        active_db_rowid_by_code_.clear();
        active_begin_ms_by_code_.clear();
        next_hist_seq_ = 1;

        for (const auto& r : rows)
        {
            FaultCenterHistRecord rec;
            rec.code = r.code;
            rec.first_on_ms = r.first_on_ms;
            rec.clear_ms = r.clear_ms;
            rec.seq_no = r.seq_no;
            rec.state = r.state;

            history_.push_back(rec);

            if (rec.seq_no >= next_hist_seq_)
            {
                next_hist_seq_ = static_cast<uint16_t>(rec.seq_no + 1);
            }

            // 第十一批：
            // 启动恢复时，把“仍未清除”的那条记录 row_id 也恢复进映射。
            // 若同码出现多条未清除（理论上不该发生），保留最后一条。
            if (r.clear_ms == 0) {
                active_db_rowid_by_code_[r.code] = r.id;
                active_begin_ms_by_code_[r.code] = r.first_on_ms;
            }
        }

        clampPages_();
        // 20240418
        LOGINFO("[FAULT][HISTORY][RESTORE] rows=%zu next_seq=%u active_rowids=%zu",
        rows.size(),
        (unsigned)next_hist_seq_,
        active_db_rowid_by_code_.size());
        for (const auto& r : rows)
        {
            LOG_THROTTLE_MS("fault_history_restore_rows", 1000, LOGINFO,
                            "[FAULT][HISTORY][RESTORE_ROW] id=%lld code=0x%04X on_ms=%llu clear_ms=%llu seq=%u state=%u",
                            (long long)r.id,
                            (unsigned)r.code,
                            (unsigned long long)r.first_on_ms,
                            (unsigned long long)r.clear_ms,
                            (unsigned)r.seq_no,
                            (unsigned)r.state);
        }

        // 第二批已有：恢复后通知历史缓存刷新
        history_dirty_ = true;
        ++history_version_;
    }

void FaultCenter::setActive(uint16_t code, bool on)
{
    if (code == 0x2002 || code == 0x2102 || code == 0x2202 || code == 0x2302) {
        LOGINFO("[DEBUG_CENTER] Receive setActive -> code: 0x%X | on: %d", code, on);
    }
    if (on)
    {
        LOGINFO("[FAULT][CENTER] code=0x%04X active=%d",
                (unsigned)code,
                on ? 1 : 0);
    }

    const bool was_on = (active_.find(code) != active_.end());
    if (code == 0x102B || code == 0x103C) {
        LOGINFO("[TRACE][FAULT_EDGE] code=0x%04X old=%d new=%d",
                (unsigned)code,
                was_on ? 1 : 0,
                on ? 1 : 0);
    }

    const bool record_db = shouldRecordToDb_(code);
    bool history_changed = false;

    if (on)
    {
        active_.insert(code);

        if (!was_on)
        {
            const uint64_t ts = unixNowMs();

            // 第十五批：
            // 现存故障页时间不再依赖 history_，
            // 所有新激活故障都要记录本次开始时间。
            active_begin_ms_by_code_[code] = ts;

            // 只有 record_db=true 的故障，才进入内存 history_ / SQLite
            if (record_db)
            {
                FaultCenterHistRecord rec;
                rec.code = code;
                rec.first_on_ms = ts;
                rec.clear_ms = 0;
                rec.seq_no = next_hist_seq_++;
                rec.state = 1;
                history_.push_back(rec);
                trimHistoryIfNeeded_();

                int64_t saved_row_id = 0;

                if (fault_db_)
                {
                    FaultHistoryDbRecord dbrec;
                    dbrec.code = rec.code;
                    dbrec.first_on_ms = rec.first_on_ms;
                    dbrec.clear_ms = rec.clear_ms;
                    dbrec.seq_no = rec.seq_no;
                    dbrec.state = rec.state;

                    if (cat_)
                    {
                        if (const FaultMeta* meta = cat_->metaOf(code))
                        {
                            dbrec.name = meta->name;
                            dbrec.classification = meta->classification;
                            dbrec.priority_rank = meta->priority_rank;
                        }
                    }

                    if (fault_db_->insertHistoryBegin(dbrec, saved_row_id))
                    {
                        if (saved_row_id > 0) {
                            active_db_rowid_by_code_[code] = saved_row_id;
                        }
                    }
                }

                LOGINFO("[FAULT][HISTORY][BEGIN] code=0x%04X on_ms=%llu seq=%u row_id=%lld record_db=%d active_rowids=%zu",
                        (unsigned)code,
                        (unsigned long long)ts,
                        (unsigned)rec.seq_no,
                        (long long)saved_row_id,
                        record_db ? 1 : 0,
                        active_db_rowid_by_code_.size());

                history_changed = true;
            }
            else
            {
                LOGINFO("[FAULT][HISTORY][SKIP_BEGIN] code=0x%04X reason=record_db_false",
                        (unsigned)code);
            }
        }
    }
    else
    {
        active_.erase(code);

        if (was_on)
        {
            if (record_db)
            {
                const uint64_t ts = unixNowMs();
                int64_t cleared_row_id = 0;
                bool used_row_id = false;

                for (auto it = history_.rbegin(); it != history_.rend(); ++it)
                {
                    if (it->code == code && it->clear_ms == 0)
                    {
                        it->clear_ms = ts;
                        it->state = 0;
                        break;
                    }
                }

                if (fault_db_)
                {
                    auto it_row = active_db_rowid_by_code_.find(code);
                    if (it_row != active_db_rowid_by_code_.end() && it_row->second > 0)
                    {
                        cleared_row_id = it_row->second;
                        used_row_id = true;
                        fault_db_->markHistoryClearedById(it_row->second, ts);
                        active_db_rowid_by_code_.erase(it_row);
                    }
                    else
                    {
                        fault_db_->markHistoryCleared(code, ts);
                    }
                }

                LOGINFO("[FAULT][HISTORY][CLEAR] code=0x%04X clear_ms=%llu row_id=%lld used_row_id=%d active_rowids=%zu",
                        (unsigned)code,
                        (unsigned long long)ts,
                        (long long)cleared_row_id,
                        used_row_id ? 1 : 0,
                        active_db_rowid_by_code_.size());

                history_changed = true;
            }
            else
            {
                LOGINFO("[FAULT][HISTORY][SKIP_CLEAR] code=0x%04X reason=record_db_false",
                        (unsigned)code);
            }

            // 第十五批：
            // 故障结束后，无论是否 record_db，都要清掉“当前激活开始时间”
            active_begin_ms_by_code_.erase(code);

            // 原有 row_id 清理逻辑继续保留
            active_db_rowid_by_code_.erase(code);
        }
    }

    clampPages_();

    if (history_changed)
    {
        history_dirty_ = true;
        ++history_version_;

        LOGINFO("[FAULT][HISTORY][DIRTY] version=%llu history_size=%zu cur_total=%u his_total=%u",
                (unsigned long long)history_version_,
                history_.size(),
                (unsigned)currentTotalPages_(),
                (unsigned)historyTotalPages_());
    }
}
    bool FaultCenter::consumeHistoryDirty(uint64_t* out_version)
    {
        if (out_version) {
            *out_version = history_version_;
        }

        const bool dirty = history_dirty_;
        history_dirty_ = false;
        return dirty;
    }
    bool FaultCenter::isActive(uint16_t code) const
    {
        return active_.find(code) != active_.end();
    }

    void FaultCenter::enterHistoryView()
    {
        in_history_view_ = true;
        history_page_ = 0;
        clampPages_();
    }

    void FaultCenter::enterCurrentView()
    {
        in_history_view_ = false;
        current_page_ = 0;
        clampPages_();
    }

    void FaultCenter::nextCurrentPage()
    {
        const uint16_t total = currentTotalPages_();
        if (total == 0)
        {
            current_page_ = 0;
            return;
        }
        if (current_page_ + 1 < total) ++current_page_;
    }

    void FaultCenter::prevCurrentPage()
    {
        if (current_page_ > 0) --current_page_;
    }

    void FaultCenter::nextHistoryPage()
    {
        const uint16_t total = historyTotalPages_();
        if (total == 0)
        {
            history_page_ = 0;
            return;
        }
        if (history_page_ + 1 < total) ++history_page_;
    }

    void FaultCenter::prevHistoryPage()
    {
        if (history_page_ > 0) --history_page_;
    }

    void FaultCenter::toFirstCurrentPage()
    {
        current_page_ = 0;
    }

    void FaultCenter::toFirstHistoryPage()
    {
        history_page_ = 0;
    }

    uint32_t FaultCenter::encodeTime_(uint64_t ts_ms)
    {
        if (ts_ms == 0) return 0u;
        return static_cast<uint32_t>(ts_ms / 1000ULL); // 秒级32位时间戳
    }
    uint64_t FaultCenter::currentBeginTimeOf_(uint16_t code) const
    {
        std::lock_guard<std::mutex> lk(mtx_);

        auto it = active_begin_ms_by_code_.find(code);
        if (it == active_begin_ms_by_code_.end()) {
            return 0;
        }
        return it->second;
    }

    void FaultCenter::trimHistoryIfNeeded_()
    {
        while (history_.size() > kMaxHistoryRecords)
        {
            auto it = std::find_if(history_.begin(), history_.end(),
                                   [](const FaultCenterHistRecord& r)
                                   {
                                       return r.clear_ms != 0;
                                   });

            if (it != history_.end())
            {
                history_.erase(it);
            }
            else
            {
                history_.erase(history_.begin());
            }
        }
    }

    uint64_t FaultCenter::lastEventTimeOf_(uint16_t code) const
    {
        for (auto it = history_.rbegin(); it != history_.rend(); ++it)
        {
            if (it->code != code) continue;
            if (it->clear_ms != 0) return it->clear_ms;
            return it->first_on_ms;
        }
        return 0;
    }

    std::vector<uint16_t> FaultCenter::collectCurrentVisibleCodes_() const
    {
        std::vector<uint16_t> visible;
        if (!cat_) return visible;

        for (const auto code : cat_->currentCodes())
        {
            if (isActive(code)) visible.push_back(code);
        }

        // 现存故障排序规则：
        // 1) 只按“本次触发时间”倒序，最新触发的排第一
        // 2) 若时间相同，再按故障码升序做稳定兜底
        std::sort(visible.begin(), visible.end(),
                  [this](uint16_t a, uint16_t b)
                  {
                      const uint64_t ta = currentBeginTimeOf_(a);
                      const uint64_t tb = currentBeginTimeOf_(b);

                      if (ta != tb) return ta > tb;   // 最新触发优先
                      return a < b;                   // 同时触发时按 code 稳定兜底
                  });

        return visible;
    }

    std::vector<FaultCenterHistRecord> FaultCenter::collectHistoryVisible_() const
    {
        std::lock_guard<std::mutex> lk(mtx_);

        std::vector<FaultCenterHistRecord> visible;
        if (!cat_) return visible;

        visible.reserve(history_.size());

        for (const auto& rec : history_)
        {
            if (cat_->showInHistory(rec.code))
            {
                visible.push_back(rec); // 复制，不存指针
            }
        }

        std::sort(visible.begin(), visible.end(),
                  [this](const FaultCenterHistRecord& a, const FaultCenterHistRecord& b)
                  {
                      const bool a_alive = (a.clear_ms == 0);
                      const bool b_alive = (b.clear_ms == 0);
                      if (a_alive != b_alive) return a_alive > b_alive;

                      const uint64_t ac = a.clear_ms;
                      const uint64_t bc = b.clear_ms;
                      if (ac != bc) return ac > bc; // 最近消失优先

                      if (a.first_on_ms != b.first_on_ms) return a.first_on_ms > b.first_on_ms;

                      const FaultMeta* ma = cat_->metaOf(a.code);
                      const FaultMeta* mb = cat_->metaOf(b.code);

                      const int pra = ma ? ma->priority_rank : 9999;
                      const int prb = mb ? mb->priority_rank : 9999;
                      if (pra != prb) return pra < prb;

                      return a.seq_no < b.seq_no;
                  });

        return visible;
    }

    std::vector<FaultCenterCurrentRow> FaultCenter::buildCurrentRows_() const
    {
        std::vector<FaultCenterCurrentRow> out;
        if (!cat_) return out;

        const auto visible = collectCurrentVisibleCodes_();

        for (auto c : visible) {
            if (c == 0x1073 || c == 0x10AE) {
                LOGINFO("[DEBUG_HMI] Target fault 0x%X is in the visible list!", c);
            }
        }

        const uint32_t start =
            static_cast<uint32_t>(current_page_) * kFaultPageRows;

        for (uint16_t i = 0; i < kFaultPageRows; ++i)
        {
            const uint32_t idx = start + i;
            if (idx >= visible.size()) break;

            FaultCenterCurrentRow row;
            row.seq_no = static_cast<uint16_t>(idx + 1);
            row.code = visible[idx];

            // 现存故障页 on_time 只看“本次激活开始时间”，
            // 不再依赖 history_ / record_db。
            row.on_time = encodeTime_(currentBeginTimeOf_(row.code));

            out.push_back(row);
        }

        return out;
    }

    std::vector<FaultCenterHistoryRow> FaultCenter::buildHistoryRows_() const
    {
        std::vector<FaultCenterHistoryRow> out;
        if (!cat_) return out;

        const auto visible = collectHistoryVisible_();

        const uint32_t start = static_cast<uint32_t>(history_page_) * kFaultPageRows;
        for (uint16_t i = 0; i < kFaultPageRows; ++i)
        {
            const uint32_t idx = start + i;
            if (idx >= visible.size()) break;

            const auto& rec = visible[idx];
            FaultCenterHistoryRow row;
            row.seq_no = rec.seq_no;
            row.code = rec.code;
            row.on_time = encodeTime_(rec.first_on_ms);
            row.off_time = encodeTime_(rec.clear_ms);
            row.state = (rec.clear_ms == 0) ? 1 : 0;
            out.push_back(row);
        }

        return out;
    }

    uint16_t FaultCenter::currentTotalPages_() const
    {
        const auto visible = collectCurrentVisibleCodes_();
        if (visible.empty()) return 0;
        return static_cast<uint16_t>((visible.size() + kFaultPageRows - 1) / kFaultPageRows);
    }

    uint16_t FaultCenter::historyTotalPages_() const
    {
        const auto visible = collectHistoryVisible_();
        if (visible.empty()) return 0;
        return static_cast<uint16_t>((visible.size() + kFaultPageRows - 1) / kFaultPageRows);
    }

    void FaultCenter::clampPages_()
    {
        const uint16_t cur_total = currentTotalPages_();
        const uint16_t his_total = historyTotalPages_();

        if (cur_total == 0) current_page_ = 0;
        else if (current_page_ >= cur_total) current_page_ = static_cast<uint16_t>(cur_total - 1);

        if (his_total == 0) history_page_ = 0;
        else if (history_page_ >= his_total) history_page_ = static_cast<uint16_t>(his_total - 1);
    }

    static inline void writeU32To2Regs(HMIProto& hmi, uint16_t base, uint16_t row_index, uint32_t value)
    {
        const uint16_t addr_hi = static_cast<uint16_t>(base + row_index * 2);
        const uint16_t addr_lo = static_cast<uint16_t>(base + row_index * 2 + 1);

        hmi.setIntRead(addr_hi, static_cast<uint16_t>((value >> 16) & 0xFFFFu));
        hmi.setIntRead(addr_lo, static_cast<uint16_t>(value & 0xFFFFu));
    }

    std::vector<FaultCenterCurrentRow> FaultCenter::debugCurrentRows() const
    {
        return buildCurrentRows_();
    }

    std::vector<uint16_t> FaultCenter::debugCurrentVisibleCodes() const
    {
        return collectCurrentVisibleCodes_();
    }

    uint16_t FaultCenter::debugCurrentPageIndex() const
    {
        return current_page_;
    }

    // 显示全故障用的,当前页总页数
    uint16_t FaultCenter::debugCurrentTotalPages() const
    {
        return currentTotalPages_();
    }

    static std::string joinCodesU16_(const std::vector<uint16_t>& codes)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < codes.size(); ++i)
        {
            if (i != 0) oss << ",";
            oss << "0x" << std::hex << std::uppercase << codes[i] << std::dec;
        }
        if (codes.empty())
        {
            oss << "<none>";
        }
        return oss.str();
    }

    // 显示全故障用的,当前页故障行
    static std::string joinCurrentRows_(const std::vector<control::FaultCenterCurrentRow>& rows)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < rows.size(); ++i)
        {
            if (i != 0) oss << " | ";
            oss << "#"
                << rows[i].seq_no
                << ":0x" << std::hex << std::uppercase << rows[i].code << std::dec
                << "@on=" << rows[i].on_time;
        }
        if (rows.empty())
        {
            oss << "<empty>";
        }
        return oss.str();
    }

    // 显示当前页故障用的,格式化时间
    static std::string formatUnixSecToDateTime_(uint32_t sec)
    {
        if (sec == 0)
        {
            return "0000-00-00 00:00:00";
        }

        std::time_t t = static_cast<std::time_t>(sec);
        std::tm tm_buf{};
        localtime_r(&t, &tm_buf);

        char buf[32] = {0};
        std::snprintf(buf, sizeof(buf),
                      "%04d-%02d-%02d %02d:%02d:%02d",
                      tm_buf.tm_year + 1900,
                      tm_buf.tm_mon + 1,
                      tm_buf.tm_mday,
                      tm_buf.tm_hour,
                      tm_buf.tm_min,
                      tm_buf.tm_sec);
        return std::string(buf);
    }

    void FaultCenter::flushToHmi(HMIProto& hmi) const
    {
        (void)hmi;

    }

    // ===== 历史故障页 =====
    // 第十二批（方案 A）：
    // 这里不再写任何 ADDR_HIS_* 地址。
    // 历史页由 LogicEngine::applyFaultHmi_() 中的 FaultHistoryCache 覆盖输出。
} // namespace control
