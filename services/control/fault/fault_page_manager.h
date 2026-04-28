//
// Created by lxy on 2026/3/2.
//

#ifndef ENERGYSTORAGE_FAULT_PAGE_MANAGER_H
#define ENERGYSTORAGE_FAULT_PAGE_MANAGER_H

#pragma once

#include <cstdint>
#include <string>

class HMIProto;

namespace normal {
    struct HmiMapModel;
}

namespace control {

    class FaultCenter;
    class FaultHistoryCache;

    struct FaultHmiBlock {
        uint16_t base{0};
        uint16_t count{0};   // items 数量
        uint16_t words{1};   // 每行占用寄存器数，时间字段一般为 2

        bool valid() const
        {
            return base != 0 && count != 0 && words != 0;
        }
    };

    struct FaultHmiLayout {
        // 本项目当前 HMI 一阶段仍按 5 行显示。
        // normal_map_logic.jsonl 里可以有 8/12 行，但当前显示行数仍由这里限制。
        uint16_t current_rows{5};
        uint16_t history_rows{5};

        // ===== 当前故障页 =====
        FaultHmiBlock cur_total_pages;
        FaultHmiBlock cur_page_index;
        FaultHmiBlock cur_seq;
        FaultHmiBlock cur_code;
        FaultHmiBlock cur_on_time;

        // ===== 历史故障页 =====
        FaultHmiBlock his_total_pages;
        FaultHmiBlock his_page_index;
        FaultHmiBlock his_seq;
        FaultHmiBlock his_code;
        FaultHmiBlock his_on_time;
        FaultHmiBlock his_off_time;
        FaultHmiBlock his_state;

        bool loaded{false};

        bool currentValid() const
        {
            return cur_total_pages.valid() &&
                   cur_page_index.valid() &&
                   cur_seq.valid() &&
                   cur_code.valid() &&
                   cur_on_time.valid();
        }

        bool historyValid() const
        {
            return his_total_pages.valid() &&
                   his_page_index.valid() &&
                   his_seq.valid() &&
                   his_code.valid() &&
                   his_on_time.valid() &&
                   his_off_time.valid() &&
                   his_state.valid();
        }
    };

    /**
     * FaultPageManager
     *
     * 第六批后职责：
     * 1. 不再解析 fault_hmi_layout.jsonl；
     * 2. 只从 HmiMapModel 生成 FaultHmiLayout；
     * 3. 根据 FaultCenter 当前故障页数据写 HMI；
     * 4. 根据 FaultHistoryCache 历史故障页数据写 HMI。
     */
    class FaultPageManager {
    public:
        void bindCenter(const FaultCenter* center) { center_ = center; }

        // 唯一布局入口：来自 normal_map_logic.jsonl 的统一解析模型。
        bool buildLayoutFromHmiMap(const normal::HmiMapModel& model,
                                   std::string* err = nullptr);

        bool layoutLoaded() const { return layout_.loaded; }
        const FaultHmiLayout& layout() const { return layout_; }

        uint16_t currentPageRows() const { return layout_.current_rows; }
        uint16_t historyPageRows() const { return layout_.history_rows; }

        void flushToHmi(HMIProto& hmi,
                        const FaultHistoryCache* history_cache,
                        uint16_t history_page_no) const;

    private:
        static void writeU32ToBlockRow_(HMIProto& hmi,
                                        const FaultHmiBlock& blk,
                                        uint16_t row,
                                        uint32_t value);

        static void clearU32BlockRow_(HMIProto& hmi,
                                      const FaultHmiBlock& blk,
                                      uint16_t row);

        void clearCurrentToHmi_(HMIProto& hmi) const;
        void clearHistoryToHmi_(HMIProto& hmi) const;

        void flushCurrentToHmi_(HMIProto& hmi) const;
        void flushHistoryToHmi_(HMIProto& hmi,
                                const FaultHistoryCache* history_cache,
                                uint16_t history_page_no) const;

    private:
        const FaultCenter* center_{nullptr};
        FaultHmiLayout layout_;
    };

} // namespace control

#endif // ENERGYSTORAGE_FAULT_PAGE_MANAGER_H