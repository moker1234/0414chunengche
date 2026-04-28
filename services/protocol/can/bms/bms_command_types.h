#pragma once

#include <cstdint>
#include <string>

namespace proto::bms {

    struct V2bCmdFields {
        // 1~4，对应 BMS_1 ~ BMS_4
        uint32_t instance_index{1};

        // 控制器侧实际发送的 can 口
        int can_index{2};

        // ===== V2B_CMD（按 msg1.2.xlsx 对齐）=====
        uint8_t life_signal{0};

        // 0 Reserved / 1 PowerOn / 2 PowerOff / 3 Invalid
        uint32_t hv_onoff{2};

        uint32_t aux1_onoff{0};
        uint32_t aux2_onoff{0};
        uint32_t aux3_onoff{0};

        uint32_t vehicle_speed{0};

        uint32_t main_pos_relay_st{0};
        uint32_t main_pos_relay_flt{0};
        uint32_t main_neg_relay_st{0};
        uint32_t main_neg_relay_flt{0};

        uint32_t chrg_pos_relay_st{0};
        uint32_t chrg_pos_relay_flt{0};

        uint32_t heat_pos_relay_st{0};
        uint32_t heat_pos_relay_flt{0};
        uint32_t heat_neg_relay_st{0};
        uint32_t heat_neg_relay_flt{0};

        uint32_t aux1_relay_st{0};
        uint32_t aux1_relay_flt{0};
        uint32_t aux2_relay_st{0};
        uint32_t aux2_relay_flt{0};
        uint32_t aux3_relay_st{0};
        uint32_t aux3_relay_flt{0};
        uint32_t aux4_relay_st{0};
        uint32_t aux4_relay_flt{0};

        uint32_t reserved1{0};
        uint32_t reserved2{0};

        // ===== 兼容旧字段：当前协议不再落位，仅保留避免别处编译报错 =====
        uint32_t prechg_relay_st{0};
        uint32_t prechg_relay_flt{0};
        uint32_t system_enable{0};

        std::string source{"logic"};
        uint64_t request_ts_ms{0};
        bool valid{true};
    };

} // namespace proto::bms