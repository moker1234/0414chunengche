#pragma once

#include <cstdint>
#include <string>

namespace proto::bms {

    /**
     * V2B_CMD 结构化命令对象
     *
     * 设计目标：
     * 1. logic 不直接拼 bit
     * 2. control/bms/bms_command_manager 只操作这个结构
     * 3. bms_tx 负责把这个结构 pack 成 can_frame
     */
    struct V2bCmdFields {
        // 实例号：1~4
        uint32_t instance_index{1};

        // 发送到哪个 can 口（默认 can2）
        int can_index{2};

        // ===== V2B_CMD 信号 =====
        // 默认值尽量贴近“安全、保守、可运行”
        uint8_t life_signal{0};

        // HV 上下电：
        // 0 Reserved
        // 1 PowerOn
        // 2 PowerOff
        // 3 Invalid
        uint32_t hv_onoff{2};

        // 2bit：若协议存在这些字段，就按表 pack；若协议没有则自动忽略
        uint32_t aux1_onoff{0};
        uint32_t aux2_onoff{0};
        uint32_t aux3_onoff{0};

        // 车速
        uint32_t vehicle_speed{0};

        // 继电器状态 / 故障状态（若协议表有这些字段，就 pack）
        uint32_t main_pos_relay_st{0};
        uint32_t main_pos_relay_flt{0};
        uint32_t main_neg_relay_st{0};
        uint32_t main_neg_relay_flt{0};
        uint32_t prechg_relay_st{0};
        uint32_t prechg_relay_flt{0};

        // 某些版本协议可能有该字段；没有则忽略
        uint32_t system_enable{1};

        // 来源信息（方便日志/追踪）
        std::string source{"logic"};

        // 时间戳（非协议字段）
        uint64_t request_ts_ms{0};

        // 是否有效
        bool valid{true};
    };

} // namespace proto::bms