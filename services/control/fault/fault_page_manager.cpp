//
// Created by lxy on 2026/3/2.
//

//
// Created by lxy on 2026/3/2.
//

#include "fault_page_manager.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "fault_center.h"
#include "fault_history_cache.h"
#include "logger.h"

#include "../../normal/hmi_map_model.h"
#include "../../protocol/rs485/hmi/hmi_proto.h"

namespace control {

    namespace {


static uint16_t clampRowsByBlock_(uint16_t want,
                                  const FaultHmiBlock& a,
                                  const FaultHmiBlock& b,
                                  const FaultHmiBlock& c)
{
    uint16_t out = want;
    if (a.valid()) out = std::min<uint16_t>(out, a.count);
    if (b.valid()) out = std::min<uint16_t>(out, b.count);
    if (c.valid()) out = std::min<uint16_t>(out, c.count);
    return out == 0 ? want : out;
}

static uint16_t clampRowsByBlock_(uint16_t want,
                                  const FaultHmiBlock& a,
                                  const FaultHmiBlock& b,
                                  const FaultHmiBlock& c,
                                  const FaultHmiBlock& d,
                                  const FaultHmiBlock& e)
{
    uint16_t out = want;
    if (a.valid()) out = std::min<uint16_t>(out, a.count);
    if (b.valid()) out = std::min<uint16_t>(out, b.count);
    if (c.valid()) out = std::min<uint16_t>(out, c.count);
    if (d.valid()) out = std::min<uint16_t>(out, d.count);
    if (e.valid()) out = std::min<uint16_t>(out, e.count);
    return out == 0 ? want : out;
}

static bool startsWith_(const std::string& s, const char* prefix)
{
    if (!prefix) return false;
    const std::string p(prefix);
    return s.rfind(p, 0) == 0;
}

/*
 * 从形如：
 *   hmi.fault.num.cur_row1_code
 *   hmi.fault.num.his_row12_disapptime
 *
 * 中解析 row number。
 */
static int rowNoBetween_(const std::string& s,
                         const std::string& prefix,
                         const std::string& suffix)
{
    if (s.empty()) return 0;
    if (s.rfind(prefix, 0) != 0) return 0;
    if (s.size() <= prefix.size() + suffix.size()) return 0;
    if (s.compare(s.size() - suffix.size(), suffix.size(), suffix) != 0) return 0;

    const std::string mid =
        s.substr(prefix.size(), s.size() - prefix.size() - suffix.size());

    if (mid.empty()) return 0;

    for (char c : mid) {
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            return 0;
        }
    }

    try {
        const int n = std::stoi(mid);
        return (n > 0 && n <= 999) ? n : 0;
    } catch (...) {
        return 0;
    }
}

static bool faultMapItemUsable_(const normal::HmiMapItem& it)
{
    return it.has_addr &&
           it.words != 0 &&
           !it.path.empty() &&
           startsWith_(it.path, "hmi.fault.num.");
}

static bool buildSingleBlockFromMap_(const normal::HmiMapModel& model,
                                     const char* path,
                                     FaultHmiBlock& out)
{
    out = FaultHmiBlock{};

    if (!path || !*path) {
        return false;
    }

    const std::vector<std::size_t>* owners = model.findByPath(path);
    if (!owners || owners->empty()) {
        return false;
    }

    for (std::size_t idx : *owners) {
        const normal::HmiMapItem* it = model.itemAt(idx);
        if (!it || !faultMapItemUsable_(*it)) {
            continue;
        }

        out.base = it->addr;
        out.count = 1;
        out.words = it->words == 0 ? 1 : it->words;
        return out.valid();
    }

    return false;
}

static bool buildRowBlockFromMap_(const normal::HmiMapModel& model,
                                  const char* path_prefix,
                                  const char* path_suffix,
                                  FaultHmiBlock& out)
{
    out = FaultHmiBlock{};

    if (!path_prefix || !path_suffix) {
        return false;
    }

    std::map<int, const normal::HmiMapItem*> rows;

    /*
     * 优先使用 loader 已经分类出的 fault_num_items。
     * 如果后续某次 loader 没有分类，也 fallback 扫描全量 items。
     */
    auto collectOne = [&](const normal::HmiMapItem& it) {
        if (!faultMapItemUsable_(it)) {
            return;
        }

        const int row = rowNoBetween_(it.path, path_prefix, path_suffix);
        if (row <= 0) {
            return;
        }

        if (rows.find(row) == rows.end()) {
            rows[row] = &it;
        }
    };

    if (!model.fault_num_items.empty()) {
        for (std::size_t idx : model.fault_num_items) {
            const normal::HmiMapItem* it = model.itemAt(idx);
            if (!it) continue;
            collectOne(*it);
        }
    } else {
        for (const auto& it : model.items) {
            collectOne(it);
        }
    }

    auto it1 = rows.find(1);
    if (it1 == rows.end() || !it1->second) {
        return false;
    }

    uint16_t count = 0;
    for (int i = 1; i <= 999; ++i) {
        if (rows.find(i) == rows.end()) {
            break;
        }
        ++count;
    }

    if (count == 0) {
        return false;
    }

    const auto* first = it1->second;

    out.base = first->addr;
    out.count = count;
    out.words = first->words == 0 ? 1 : first->words;

    return out.valid();
}

static std::string blockDesc_(const FaultHmiBlock& b)
{
    char buf[96];
    std::snprintf(buf, sizeof(buf),
                  "base=0x%04X count=%u words=%u valid=%d",
                  (unsigned)b.base,
                  (unsigned)b.count,
                  (unsigned)b.words,
                  b.valid() ? 1 : 0);
    return std::string(buf);
}

static void appendMissing_(std::ostringstream& oss,
                           const char* name,
                           const FaultHmiBlock& b)
{
    if (b.valid()) {
        return;
    }

    if (oss.tellp() > 0) {
        oss << "; ";
    }

    oss << name << "(" << blockDesc_(b) << ")";
}

} // namespace


bool FaultPageManager::buildLayoutFromHmiMap(const normal::HmiMapModel& model,
                                             std::string* err)
{
    FaultHmiLayout next{};

    // ===== 当前故障页 =====
    buildSingleBlockFromMap_(model,
                             "hmi.fault.num.cur_total_page",
                             next.cur_total_pages);

    buildSingleBlockFromMap_(model,
                             "hmi.fault.num.cur_show_page",
                             next.cur_page_index);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.cur_row",
                          "_snum",
                          next.cur_seq);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.cur_row",
                          "_code",
                          next.cur_code);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.cur_row",
                          "_triggertime",
                          next.cur_on_time);

    // ===== 历史故障页 =====
    buildSingleBlockFromMap_(model,
                             "hmi.fault.num.his_total_page",
                             next.his_total_pages);

    buildSingleBlockFromMap_(model,
                             "hmi.fault.num.his_show_page",
                             next.his_page_index);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.his_row",
                          "_snum",
                          next.his_seq);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.his_row",
                          "_code",
                          next.his_code);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.his_row",
                          "_triggertime",
                          next.his_on_time);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.his_row",
                          "_disapptime",
                          next.his_off_time);

    buildRowBlockFromMap_(model,
                          "hmi.fault.num.his_row",
                          "_state",
                          next.his_state);

    next.current_rows = clampRowsByBlock_(next.current_rows,
                                          next.cur_seq,
                                          next.cur_code,
                                          next.cur_on_time);

    next.history_rows = clampRowsByBlock_(next.history_rows,
                                          next.his_seq,
                                          next.his_code,
                                          next.his_on_time,
                                          next.his_off_time,
                                          next.his_state);

    if (!next.currentValid()) {
        std::ostringstream oss;
        appendMissing_(oss, "cur_total_pages", next.cur_total_pages);
        appendMissing_(oss, "cur_page_index", next.cur_page_index);
        appendMissing_(oss, "cur_seq", next.cur_seq);
        appendMissing_(oss, "cur_code", next.cur_code);
        appendMissing_(oss, "cur_on_time", next.cur_on_time);

        if (err) {
            *err = "HmiMapModel missing current fault layout: " + oss.str();
        }

        layout_.loaded = false;
        return false;
    }

    if (!next.historyValid()) {
        std::ostringstream oss;
        appendMissing_(oss, "his_total_pages", next.his_total_pages);
        appendMissing_(oss, "his_page_index", next.his_page_index);
        appendMissing_(oss, "his_seq", next.his_seq);
        appendMissing_(oss, "his_code", next.his_code);
        appendMissing_(oss, "his_on_time", next.his_on_time);
        appendMissing_(oss, "his_off_time", next.his_off_time);
        appendMissing_(oss, "his_state", next.his_state);

        if (err) {
            *err = "HmiMapModel missing history fault layout: " + oss.str();
        }

        layout_.loaded = false;
        return false;
    }

    next.loaded = true;
    layout_ = next;

    LOG_SYS_I("[FAULT][LAYOUT] built from HmiMapModel cur_rows=%u his_rows=%u "
              "cur{total=0x%04X page=0x%04X seq=0x%04X/%u code=0x%04X/%u on=0x%04X/%uw/%u} "
              "his{total=0x%04X page=0x%04X seq=0x%04X/%u code=0x%04X/%u on=0x%04X/%uw/%u off=0x%04X/%uw/%u state=0x%04X/%u}",
              (unsigned)layout_.current_rows,
              (unsigned)layout_.history_rows,

              (unsigned)layout_.cur_total_pages.base,
              (unsigned)layout_.cur_page_index.base,
              (unsigned)layout_.cur_seq.base,
              (unsigned)layout_.cur_seq.count,
              (unsigned)layout_.cur_code.base,
              (unsigned)layout_.cur_code.count,
              (unsigned)layout_.cur_on_time.base,
              (unsigned)layout_.cur_on_time.words,
              (unsigned)layout_.cur_on_time.count,

              (unsigned)layout_.his_total_pages.base,
              (unsigned)layout_.his_page_index.base,
              (unsigned)layout_.his_seq.base,
              (unsigned)layout_.his_seq.count,
              (unsigned)layout_.his_code.base,
              (unsigned)layout_.his_code.count,
              (unsigned)layout_.his_on_time.base,
              (unsigned)layout_.his_on_time.words,
              (unsigned)layout_.his_on_time.count,
              (unsigned)layout_.his_off_time.base,
              (unsigned)layout_.his_off_time.words,
              (unsigned)layout_.his_off_time.count,
              (unsigned)layout_.his_state.base,
              (unsigned)layout_.his_state.count);

    return true;
}

void FaultPageManager::writeU32ToBlockRow_(HMIProto& hmi,
                                           const FaultHmiBlock& blk,
                                           uint16_t row,
                                           uint32_t value)
{
    if (!blk.valid() || row >= blk.count) {
        return;
    }

    const uint16_t base = static_cast<uint16_t>(blk.base + row * blk.words);

    if (blk.words <= 1) {
        hmi.setIntRead(base, static_cast<uint16_t>(value & 0xFFFFu));
        return;
    }

    hmi.setIntRead(base, static_cast<uint16_t>((value >> 16) & 0xFFFFu));
    hmi.setIntRead(static_cast<uint16_t>(base + 1u), static_cast<uint16_t>(value & 0xFFFFu));

    // 如果配置了超过 2 个 word，本项目不用，但清零避免脏数据。
    for (uint16_t i = 2; i < blk.words; ++i) {
        hmi.setIntRead(static_cast<uint16_t>(base + i), 0);
    }
}

void FaultPageManager::clearU32BlockRow_(HMIProto& hmi,
                                         const FaultHmiBlock& blk,
                                         uint16_t row)
{
    writeU32ToBlockRow_(hmi, blk, row, 0);
}

void FaultPageManager::clearCurrentToHmi_(HMIProto& hmi) const
{
    if (!layout_.currentValid()) {
        return;
    }

    hmi.setIntRead(layout_.cur_total_pages.base, 0);
    hmi.setIntRead(layout_.cur_page_index.base, 0);

    for (uint16_t i = 0; i < layout_.current_rows; ++i) {
        hmi.setIntRead(static_cast<uint16_t>(layout_.cur_seq.base + i), 0);
        hmi.setIntRead(static_cast<uint16_t>(layout_.cur_code.base + i), 0);
        clearU32BlockRow_(hmi, layout_.cur_on_time, i);
    }
}

void FaultPageManager::clearHistoryToHmi_(HMIProto& hmi) const
{
    if (!layout_.historyValid()) {
        return;
    }

    hmi.setIntRead(layout_.his_total_pages.base, 0);
    hmi.setIntRead(layout_.his_page_index.base, 0);

    for (uint16_t i = 0; i < layout_.history_rows; ++i) {
        hmi.setIntRead(static_cast<uint16_t>(layout_.his_seq.base + i), 0);
        hmi.setIntRead(static_cast<uint16_t>(layout_.his_code.base + i), 0);
        clearU32BlockRow_(hmi, layout_.his_on_time, i);
        clearU32BlockRow_(hmi, layout_.his_off_time, i);
        hmi.setIntRead(static_cast<uint16_t>(layout_.his_state.base + i), 0);
    }
}

void FaultPageManager::flushCurrentToHmi_(HMIProto& hmi) const
{
    if (!layout_.currentValid()) {
        return;
    }

    if (!center_) {
        clearCurrentToHmi_(hmi);
        return;
    }

    const auto rows = center_->debugCurrentRows();
    const uint16_t total_pages = center_->debugCurrentTotalPages();
    const uint16_t page_index =
        (total_pages == 0) ? 0 : static_cast<uint16_t>(center_->debugCurrentPageIndex() + 1u);

    hmi.setIntRead(layout_.cur_total_pages.base, total_pages);
    hmi.setIntRead(layout_.cur_page_index.base, page_index);

    for (uint16_t i = 0; i < layout_.current_rows; ++i) {
        if (i < rows.size()) {
            const auto& r = rows[i];

            hmi.setIntRead(static_cast<uint16_t>(layout_.cur_seq.base + i), r.seq_no);
            hmi.setIntRead(static_cast<uint16_t>(layout_.cur_code.base + i), r.code);
            writeU32ToBlockRow_(hmi, layout_.cur_on_time, i, r.on_time);
        } else {
            hmi.setIntRead(static_cast<uint16_t>(layout_.cur_seq.base + i), 0);
            hmi.setIntRead(static_cast<uint16_t>(layout_.cur_code.base + i), 0);
            clearU32BlockRow_(hmi, layout_.cur_on_time, i);
        }
    }

    LOG_THROTTLE_MS("fault_hmi_current_by_layout", 1000, LOG_SYS_I,
                    "[FAULT][HMI][CUR_LAYOUT] total=%u page=%u rows=%zu cur_rows=%u",
                    (unsigned)total_pages,
                    (unsigned)page_index,
                    rows.size(),
                    (unsigned)layout_.current_rows);
}

void FaultPageManager::flushHistoryToHmi_(HMIProto& hmi,
                                          const FaultHistoryCache* history_cache,
                                          uint16_t history_page_no) const
{
    if (!layout_.historyValid()) {
        return;
    }

    if (!history_cache) {
        clearHistoryToHmi_(hmi);
        return;
    }

    const uint16_t total_pages = history_cache->totalPages();
    const uint16_t page_index = (total_pages == 0) ? 0 : history_page_no;
    const auto rows = history_cache->pageRows(history_page_no);

    hmi.setIntRead(layout_.his_total_pages.base, total_pages);
    hmi.setIntRead(layout_.his_page_index.base, page_index);

    for (uint16_t i = 0; i < layout_.history_rows; ++i) {
        const uint16_t seq_addr = static_cast<uint16_t>(layout_.his_seq.base + i);
        const uint16_t code_addr = static_cast<uint16_t>(layout_.his_code.base + i);
        const uint16_t state_addr = static_cast<uint16_t>(layout_.his_state.base + i);

        if (i < rows.size()) {
            const auto& r = rows[i];

            hmi.setIntRead(seq_addr, r.seq_no);
            hmi.setIntRead(code_addr, r.code);

            const uint32_t on_sec = static_cast<uint32_t>(r.first_on_ms / 1000ULL);
            const uint32_t off_sec = static_cast<uint32_t>(r.clear_ms / 1000ULL);

            writeU32ToBlockRow_(hmi, layout_.his_on_time, i, on_sec);
            writeU32ToBlockRow_(hmi, layout_.his_off_time, i, off_sec);
            hmi.setIntRead(state_addr, r.state);
        } else {
            hmi.setIntRead(seq_addr, 0);
            hmi.setIntRead(code_addr, 0);
            clearU32BlockRow_(hmi, layout_.his_on_time, i);
            clearU32BlockRow_(hmi, layout_.his_off_time, i);
            hmi.setIntRead(state_addr, 0);
        }
    }

    LOG_THROTTLE_MS("fault_hmi_history_by_layout", 1000, LOG_SYS_I,
                    "[FAULT][HMI][HIS_LAYOUT] total=%u page=%u rows=%zu his_rows=%u window=%u~%u",
                    (unsigned)total_pages,
                    (unsigned)page_index,
                    rows.size(),
                    (unsigned)layout_.history_rows,
                    (unsigned)history_cache->windowStartPage(),
                    (unsigned)history_cache->windowEndPage());
}

void FaultPageManager::flushToHmi(HMIProto& hmi,
                                  const FaultHistoryCache* history_cache,
                                  uint16_t history_page_no) const
{
    if (!layout_.loaded) {
        LOG_THROTTLE_MS("fault_layout_not_loaded", 1000, LOG_SYS_W,
                        "[FAULT][HMI] fault layout not built from HmiMapModel, skip fault page output");
        return;
    }

    flushCurrentToHmi_(hmi);
    flushHistoryToHmi_(hmi, history_cache, history_page_no);
}

} // namespace control