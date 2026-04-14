//
// Created by lxy on 2026/3/11.
//

#ifndef ENERGYSTORAGE_BMS_COMMAND_MANAGER_H
#define ENERGYSTORAGE_BMS_COMMAND_MANAGER_H


#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "bms_logic_types.h"
#include "../../control/control_commands.h"
#include "../../protocol/can/bms/bms_command_types.h"
#include "../../protocol/can/bms/bms_tx.h"

class DriverManager;

namespace control::bms {
    struct BmsCommandView {
        uint32_t instance_index{0};

        int can_index{2};

        int hv_onoff{0};
        int system_enable{0};
        int life_signal{0};

        uint64_t last_build_ts_ms{0};
        uint64_t last_send_ts_ms{0};

        bool has_last_sent{false};
        bool tx_alive{false};
        bool valid{false};

        std::string source;

        int reason_code{0};
        std::string reason_text;
        std::string hv_text;        // PowerOn / PowerOff / Reserved / Invalid
        std::string tx_state_text;  // Idle / Active / Stale

        int dlc{0};
        double last_send_age_ms{0.0};

        std::string can_id_hex;
        std::string frame_hex;
    };

    // 命令来源
    enum class BmsCmdReason : uint8_t {
        Init = 0,
        NoData = 1,
        Offline = 2,
        RqHvPowerOff = 3,
        FaultLevelBlock = 4,
        FireFaultBlock = 5,
        TmsFaultBlock = 6,
        AllowClose = 7,
        FallbackPowerOff = 8,
    };

    /**
     * 每台 BMS 的命令状态
     */
    struct BmsCommandState {
        uint8_t last_life{0};

        proto::bms::V2bCmdFields desired{};
        proto::bms::V2bCmdFields last_sent{};

        uint64_t last_build_ts_ms{0};
        uint64_t last_send_ts_ms{0};

        bool has_last_sent{false};
        BmsCmdReason reason{BmsCmdReason::Init};

        can_frame last_frame{};
        bool has_last_frame{false};
    };

    /**
     * BMS 命令管理器
     *
     * 设计目标：
     * 1. 不直接发包，只输出 control::Command
     * 2. 由 logic 根据 bms_cache 驱动 desired
     * 3. 每台 BMS 独立维护 life_signal
     */
    class BmsCommandManager {
    public:
        BmsCommandManager() = default;

        bool init(DriverManager& drv_mgr, int default_can_index = 2);

        void setDefaultCanIndex(int can_index) { default_can_index_ = can_index; }

        void rebuildDesiredFromCache(const BmsLogicCache& cache, uint64_t ts_ms);

        void emitPeriodicCommands(uint64_t ts_ms,
                                  std::vector<control::Command>& out_cmds,
                                  uint32_t period_ms = 100);

        std::map<uint32_t, BmsCommandView> buildCommandView(uint64_t now_ms,
                                                   uint32_t alive_timeout_ms = 500) const;

    private:
        BmsCommandState& ensureState_(uint32_t instance_index);
        static std::string makeName_(uint32_t instance_index);

        static const char* reasonText_(BmsCmdReason r);
        static const char* hvText_(uint32_t hv_onoff);
        static std::string canIdHex_(uint32_t can_id);
        static std::string frameHex_(const can_frame& fr);

    private:
        proto::bms::BmsTx tx_;
        int default_can_index_{2};

        std::map<uint32_t, BmsCommandState> states_; // key: 1~4

    };

} // namespace control::bms


#endif //ENERGYSTORAGE_BMS_COMMAND_MANAGER_H