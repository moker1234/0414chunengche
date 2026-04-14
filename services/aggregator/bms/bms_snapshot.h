#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <nlohmann/json.hpp>

namespace snapshot {

    using json = nlohmann::json;

    /**
     * 单条报文健康状态（第一批先保留最小字段）
     */
    struct BmsGroupHealth {
        bool online{false};
        uint64_t last_rx_ms{0};
        uint64_t last_ok_ms{0};

        uint32_t disconnect_window_ms{0};
        uint64_t last_offline_ms{0};
        uint32_t disconnect_count{0};

        json toJson() const;
    };

    /**
     * 单条报文原始存档
     * 这一批先按你的目标 JSON：
     *  - ts_ms
     *  - rx_count
     *  - can_id
     *  - raw_hex
     */
    struct BmsGroupData {
        uint64_t ts_ms{0};
        uint32_t rx_count{0};

        uint32_t can_id{0};           // 29bit ID（不含 CAN_EFF_FLAG）
        int32_t cycle_ms{-1};         // 可选：协议周期
        std::string raw_hex;          // "01 6A 62 20 FB A3 03 0F"

        BmsGroupHealth health;

        json toJson() const;
    };

    /**
     * 单实例元信息
     */
    struct BmsInstanceMeta {
        uint32_t bms_index{0};
        std::string last_msg_name;

        json toJson() const;
    };

    /**
     * 单实例健康
     */
    struct BmsInstanceHealth {
        bool online{false};
        uint64_t last_ok_ms{0};

        uint32_t disconnect_window_ms{0};
        uint64_t last_offline_ms{0};
        uint32_t disconnect_count{0};

        json toJson() const;
    };

    /**
     * 单实例数据
     */
    struct BmsInstanceData {
        BmsInstanceMeta meta;
        BmsInstanceHealth health;
        std::map<std::string, BmsGroupData> groups;

        json toJson() const;
        json toJsonGroupsOnly() const;
    };

    /**
     * BMS 总快照
     * 目标结构：
     * {
     *   "ts_ms": ...,
     *   "items": {
     *     "BMS_1": { ... },
     *     "BMS_2": { ... },
     *     ...
     *   }
     * }
     */
    struct BmsSnapshot {
        uint64_t ts_ms{0};
        std::map<std::string, BmsInstanceData> items;

        json toJson() const;
        json toJsonItemsOnly() const;

        BmsInstanceData& ensureInstance(const std::string& instance_name, uint32_t bms_index);
    };

} // namespace snapshot