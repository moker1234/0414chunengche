//
// Created by lxy on 2026/3/2.
//

#ifndef ENERGYSTORAGE_FAULT_CATALOG_H
#define ENERGYSTORAGE_FAULT_CATALOG_H


// services/control/fault_catalog.h
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace control {

    /**
     * FaultCatalog：故障目录（由 fault_map.jsonl 驱动）
     *
     * 目标：
     * - 提供稳定的故障排序（分页依据）
     * - fault_map.jsonl 格式“与 display_map.jsonl 相同”，但这里我们只使用：
     *   - base / type / items
     *   - item.path：存放故障码（如 "0x1001" 或 "1001"）
     *   - item.name：注释/显示名（可选）
     *
     * 你后续可以扩展：按 group/page 分类、按 severity 排序等。
     */
    struct FaultMeta {
        uint16_t code{0};

        std::string code_hex;
        std::string name;
        std::string fault_type;
        std::string fault_level;
        std::string classification;

        bool record_db{false};
        bool show_hmi_current{false};
        bool show_hmi_history{false};
        bool reserved{false};

        int priority_rank{9999};
        int source_order{9999};
    };
    class FaultCatalog {
    public:
        bool loadJsonl(const std::string& path, std::string* err = nullptr);

        const std::vector<uint16_t>& all() const { return all_codes_; }
        const std::vector<uint16_t>& currentCodes() const { return current_codes_; }
        const std::vector<uint16_t>& historyCodes() const { return history_codes_; }

        const FaultMeta* metaOf(uint16_t code) const;

        bool showInCurrent(uint16_t code) const;
        bool showInHistory(uint16_t code) const;

        static bool parseCode(const std::string& s, uint16_t& out);

    private:
        std::vector<uint16_t> all_codes_;
        std::vector<uint16_t> current_codes_;
        std::vector<uint16_t> history_codes_;

        std::unordered_map<uint16_t, FaultMeta> meta_by_code_;
    };

} // namespace control

#endif //ENERGYSTORAGE_FAULT_CATALOG_H