//
// Created by lxy on 2026/4/7.
//

#include "fault_center.h"

#include <algorithm>
#include <cstdio>
#include  "sstream"
#include <ctime>
#include <string>

#include "./fault_addr_layout.h"
#include "fault_catalog.h"
#include "getTime.h"
#include "logger.h"

#include "../../protocol/rs485/hmi/hmi_proto.h"
#include "../../snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"

namespace control
{
    void FaultCenter::bindFaultDb(SqliteFaultSink* db)
    {
        fault_db_ = db;
    }

    void FaultCenter::restoreHistoryFromDb(const std::vector<FaultHistoryDbRecord>& rows)
    {
        history_.clear();
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
        }

        clampPages_();
    }

    void FaultCenter::setActive(uint16_t code, bool on)
    {
        if (on)
        {
            LOGINFO("[FAULT][CENTER] code=0x%04X active=%d", // 故障结构十批输出
                    (unsigned)code,
                    on ? 1 : 0);
        }
        const bool was_on = (active_.find(code) != active_.end());
        if (code == 0x102B || code == 0x103C) { // 20260414
            LOGINFO("[TRACE][FAULT_EDGE] code=0x%04X old=%d new=%d",
                    (unsigned)code,
                    was_on ? 1 : 0,
                    on ? 1 : 0);
        }

        if (on)
        {
            active_.insert(code);

            if (!was_on)
            {
                const uint64_t ts = unixNowMs();

                FaultCenterHistRecord rec;
                rec.code = code;
                rec.first_on_ms = ts;
                rec.clear_ms = 0;
                rec.seq_no = next_hist_seq_++;
                rec.state = 1;
                history_.push_back(rec);
                trimHistoryIfNeeded_();

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

                    int64_t row_id = 0;
                    fault_db_->insertHistoryBegin(dbrec, row_id);
                    // LOGINFO("[FAULT][HISTORY] begin code=0x%04X ts=%llu", // 故障结构十批输出
                    //         (unsigned)code,
                    //         (unsigned long long)ts);
                }
            }
        }
        else
        {
            active_.erase(code);

            if (was_on)
            {
                const uint64_t ts = unixNowMs();

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
                    fault_db_->markHistoryCleared(code, ts);
                    // LOGINFO("[FAULT][HISTORY] clear code=0x%04X ts=%llu", // 故障结构十批输出
                    //         (unsigned)code,
                    //         (unsigned long long)ts);
                }
            }
        }

        clampPages_();
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

    uint32_t FaultCenter::encodeTime_(uint64_t ts_ms)
    {
        if (ts_ms == 0) return 0u;
        return static_cast<uint32_t>(ts_ms / 1000ULL); // 秒级32位时间戳
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

        std::sort(visible.begin(), visible.end(),
                  [this](uint16_t a, uint16_t b)
                  {
                      const FaultMeta* ma = cat_->metaOf(a);
                      const FaultMeta* mb = cat_->metaOf(b);

                      const int pra = ma ? ma->priority_rank : 9999;
                      const int prb = mb ? mb->priority_rank : 9999;
                      if (pra != prb) return pra < prb;

                      const uint64_t ta = lastEventTimeOf_(a);
                      const uint64_t tb = lastEventTimeOf_(b);
                      if (ta != tb) return ta > tb; // 最近优先

                      const int soa = ma ? ma->source_order : 9999;
                      const int sob = mb ? mb->source_order : 9999;
                      if (soa != sob) return soa < sob;

                      return a < b;
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

        const uint32_t start = static_cast<uint32_t>(current_page_) * fault::FAULTS_PER_PAGE;
        for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i)
        {
            const uint32_t idx = start + i;
            if (idx >= visible.size()) break;

            FaultCenterCurrentRow row;
            row.seq_no = static_cast<uint16_t>(idx + 1);
            row.code = visible[idx];
            row.on_time = encodeTime_(lastEventTimeOf_(row.code)); // 当前故障触发/最近事件时间
            out.push_back(row);
        }

        return out;
    }

    std::vector<FaultCenterHistoryRow> FaultCenter::buildHistoryRows_() const
    {
        std::vector<FaultCenterHistoryRow> out;
        if (!cat_) return out;

        const auto visible = collectHistoryVisible_();

        const uint32_t start = static_cast<uint32_t>(history_page_) * fault::FAULTS_PER_PAGE;
        for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i)
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
        return static_cast<uint16_t>((visible.size() + fault::FAULTS_PER_PAGE - 1) / fault::FAULTS_PER_PAGE);
    }

    uint16_t FaultCenter::historyTotalPages_() const
    {
        const auto visible = collectHistoryVisible_();
        if (visible.empty()) return 0;
        return static_cast<uint16_t>((visible.size() + fault::FAULTS_PER_PAGE - 1) / fault::FAULTS_PER_PAGE);
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
        LOG_THROTTLE_MS("fault_flush_hmi", 1000, LOGINFO, // 故障结构十批输出
                        "[FAULT][HMI] cur_total=%u cur_page=%u his_total=%u his_page=%u in_history=%d",
                        (unsigned)currentTotalPages_(),
                        (unsigned)(currentTotalPages_() == 0 ? 0 : (current_page_ + 1)),
                        (unsigned)historyTotalPages_(),
                        (unsigned)(historyTotalPages_() == 0 ? 0 : (history_page_ + 1)),
                        in_history_view_ ? 1 : 0);
// 20260414
        {
            const auto visible = collectCurrentVisibleCodes_();
            const uint16_t total_pages = currentTotalPages_();

            for (uint16_t p = 0; p < total_pages; ++p)
            {
                const size_t begin = static_cast<size_t>(p) * fault::FAULTS_PER_PAGE;
                const size_t end = std::min(begin + static_cast<size_t>(fault::FAULTS_PER_PAGE),
                                            visible.size());

                std::ostringstream oss;
                for (size_t i = begin; i < end; ++i)
                {
                    if (i != begin) oss << ",";
                    oss << "0x" << std::hex << std::uppercase << visible[i] << std::dec;
                }
                if (begin >= end)
                {
                    oss << "<empty>";
                }
                LOGINFO("[FAULT][HMI][ALL_PAGES] page=%u/%u codes=%s",
                        (unsigned)(p + 1),
                        (unsigned)total_pages,
                        oss.str().c_str());
            }
        }

        const auto cur_rows = buildCurrentRows_();
        const auto his_rows = buildHistoryRows_();

        const uint16_t cur_total = currentTotalPages_();
        const uint16_t his_total = historyTotalPages_();

        // const auto all_cur_codes = collectCurrentVisibleCodes_();
        //
        // LOG_THROTTLE_MS("fault_page_dump", 500, LOG_COMM_D,
        //     "[FAULT][PAGE] cur_page=%u/%u all_cur_count=%zu all_cur_codes=[%s] cur_rows=[%s]",
        //     (unsigned)(cur_total == 0 ? 0 : (current_page_ + 1)),
        //     (unsigned)cur_total,
        //     all_cur_codes.size(),
        //     joinCodesU16_(all_cur_codes).c_str(),
        //     joinCurrentRows_(cur_rows).c_str());

        hmi.setIntRead(fault::ADDR_CUR_TOTAL_PAGES, cur_total);
        hmi.setIntRead(fault::ADDR_CUR_PAGE_INDEX,
                       cur_total == 0 ? 0 : static_cast<uint16_t>(current_page_ + 1));

        for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i)
        {
            const auto row = (i < cur_rows.size()) ? cur_rows[i] : FaultCenterCurrentRow{};
            hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_CUR_SEQ_BASE + i), row.seq_no);
            hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_CUR_CODE_BASE + i), row.code);
            writeU32To2Regs(hmi, fault::ADDR_CUR_ON_TIME_BASE, i, row.on_time);
        }

        {
            LOG_THROTTLE_MS(
                "fault_hmi_current_rows_fmt", 1000, LOG_SYS_I,
                "[FAULT][HMI][CUR] page=%u/%u "
                "r1{seq=%u code=0x%04X on=%u on_str=%s} "
                "r2{seq=%u code=0x%04X on=%u on_str=%s} "
                "r3{seq=%u code=0x%04X on=%u on_str=%s} "
                "r4{seq=%u code=0x%04X on=%u on_str=%s} "
                "r5{seq=%u code=0x%04X on=%u on_str=%s}",
                static_cast<unsigned>(cur_total == 0 ? 0 : current_page_ + 1),
                static_cast<unsigned>(cur_total),

                static_cast<unsigned>(cur_rows.size() > 0 ? cur_rows[0].seq_no : 0),
                static_cast<unsigned>(cur_rows.size() > 0 ? cur_rows[0].code : 0),
                static_cast<unsigned>(cur_rows.size() > 0 ? cur_rows[0].on_time : 0),
                formatUnixSecToDateTime_(cur_rows.size() > 0 ? cur_rows[0].on_time : 0).c_str(),

                static_cast<unsigned>(cur_rows.size() > 1 ? cur_rows[1].seq_no : 0),
                static_cast<unsigned>(cur_rows.size() > 1 ? cur_rows[1].code : 0),
                static_cast<unsigned>(cur_rows.size() > 1 ? cur_rows[1].on_time : 0),
                formatUnixSecToDateTime_(cur_rows.size() > 1 ? cur_rows[1].on_time : 0).c_str(),

                static_cast<unsigned>(cur_rows.size() > 2 ? cur_rows[2].seq_no : 0),
                static_cast<unsigned>(cur_rows.size() > 2 ? cur_rows[2].code : 0),
                static_cast<unsigned>(cur_rows.size() > 2 ? cur_rows[2].on_time : 0),
                formatUnixSecToDateTime_(cur_rows.size() > 2 ? cur_rows[2].on_time : 0).c_str(),

                static_cast<unsigned>(cur_rows.size() > 3 ? cur_rows[3].seq_no : 0),
                static_cast<unsigned>(cur_rows.size() > 3 ? cur_rows[3].code : 0),
                static_cast<unsigned>(cur_rows.size() > 3 ? cur_rows[3].on_time : 0),
                formatUnixSecToDateTime_(cur_rows.size() > 3 ? cur_rows[3].on_time : 0).c_str(),

                static_cast<unsigned>(cur_rows.size() > 4 ? cur_rows[4].seq_no : 0),
                static_cast<unsigned>(cur_rows.size() > 4 ? cur_rows[4].code : 0),
                static_cast<unsigned>(cur_rows.size() > 4 ? cur_rows[4].on_time : 0),
                formatUnixSecToDateTime_(cur_rows.size() > 4 ? cur_rows[4].on_time : 0).c_str()
            );
        }


        hmi.setIntRead(fault::ADDR_HIS_TOTAL_PAGES, his_total);
        hmi.setIntRead(fault::ADDR_HIS_PAGE_INDEX,
                       his_total == 0 ? 0 : static_cast<uint16_t>(history_page_ + 1));

        for (uint16_t i = 0; i < fault::FAULTS_PER_PAGE; ++i)
        {
            const auto row = (i < his_rows.size()) ? his_rows[i] : FaultCenterHistoryRow{};
            hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_SEQ_BASE + i), row.seq_no);
            hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_CODE_BASE + i), row.code);
            writeU32To2Regs(hmi, fault::ADDR_HIS_ON_TIME_BASE, i, row.on_time);
            writeU32To2Regs(hmi, fault::ADDR_HIS_OFF_TIME_BASE, i, row.off_time);
            hmi.setIntRead(static_cast<uint16_t>(fault::ADDR_HIS_STATE_BASE + i), row.state);
        }

        // 第八批：联调用节流日志，直接对应 HMI 读取 0x4125~0x4129
        uint16_t c0 = (cur_rows.size() > 0) ? cur_rows[0].code : 0;
        uint16_t c1 = (cur_rows.size() > 1) ? cur_rows[1].code : 0;
        uint16_t c2 = (cur_rows.size() > 2) ? cur_rows[2].code : 0;
        uint16_t c3 = (cur_rows.size() > 3) ? cur_rows[3].code : 0;
        uint16_t c4 = (cur_rows.size() > 4) ? cur_rows[4].code : 0;

        // LOG_THROTTLE_MS(
        //     "fault_flush_to_hmi", 1000, LOG_SYS_I,
        //     "[FAULT][HMI] cur_total=%u cur_page=%u codes=[0x%04X,0x%04X,0x%04X,0x%04X,0x%04X] addrs=0x4125..0x4129",
        //     static_cast<unsigned>(cur_total),
        //     static_cast<unsigned>(cur_total == 0 ? 0 : current_page_ + 1),
        //     c0, c1, c2, c3, c4
        // );
    }
} // namespace control
