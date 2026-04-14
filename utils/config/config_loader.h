//
// Created by lxy on 2026/2/3.
//

#ifndef ENERGYSTORAGE_CONFIG_LOADER_H
#define ENERGYSTORAGE_CONFIG_LOADER_H

#pragma once
#include <string>
#include <vector>
#include <cstdint>

struct SerialPortCfg {
    std::string name;
    std::string device;
    int baudrate{9600};
    bool enable{true};
};

struct CanPortCfg {
    std::string name;
    std::string ifname{"can0"};
    int bitrate{500000};
    bool enable{true};
};

struct PollCfg {
    bool enable{false};

    // 轮询周期
    uint32_t interval_ms{1000};

    // 设备响应超时
    uint32_t timeout_ms{300};

    // 退避/降频相关（DevicePollCtx 用）
    uint32_t backoff_ms{1000};

    uint32_t disconnect_window_ms{0};

    // 重发策略（PollTask 用）
    uint32_t resend_after_ms{300};
    uint32_t max_retries{2};
};

enum class LinkRole {
    MasterPoll,
    SlaveHmi,
    Unknown
};

enum class Rs485ProtoType {
    Gas,
    Smoke,
    AirConditioner,
    Hmi,
    Unknown
};

struct Rs485LinkCfg {
    int link_index{0};
    std::string name;
    LinkRole role{LinkRole::Unknown};
    Rs485ProtoType type{Rs485ProtoType::Unknown};
    uint8_t slave_id{1};     // modbus addr
    std::string crc_order;   // "hilo" / "lohi"
    PollCfg poll;
};

struct Rs232LinkCfg {
    int link_index{0};
    std::string name;
    LinkRole role{LinkRole::Unknown};
    std::string type;        // "ups_ascii"
    PollCfg poll;
};

struct IoMapConfig {
    std::vector<CanPortCfg>   can;
    std::vector<SerialPortCfg> rs485;
    std::vector<SerialPortCfg> rs232;
};

struct CanLinkCfg {
    int can_index{0};
    std::string name;
    std::string role;       // 先字符串即可，如 "periodic_tx_rx"
    bool enable{false};

    // protocol
    std::string protocol_type; // "emu_pcu_v1"

    // IDs (29-bit, not including CAN_EFF_FLAG)
    uint32_t id_emu_ctrl{0};
    uint32_t id_emu_status{0};
    uint32_t id_pcu_status{0};

    // tx
    bool tx_enable{false};
    uint32_t interval_ms{200};
    bool send_ctrl{true};
    bool send_status{true};

    // ctrl defaults (bytes)
    uint8_t ctrl_enable_default{1};
    uint8_t plug_default{0};
    uint8_t estop_default{0};
    uint8_t batt1_estop_default{0};
    uint8_t batt2_estop_default{0};

    // rx (可选扩展)
    bool rx_enable{true};
    uint32_t rx_timeout_ms{1000};
    bool require_heartbeat_increment{false};
};
struct SystemConfig {
    std::vector<Rs485LinkCfg> rs485_links;
    std::vector<Rs232LinkCfg> rs232_links;
    std::vector<CanLinkCfg> can_links;
};

class ConfigLoader {
public:
    static bool loadIoMap(const std::string& path, IoMapConfig& out, std::string& err);
    static bool loadSystem(const std::string& path, SystemConfig& out, std::string& err);
};


#endif //ENERGYSTORAGE_CONFIG_LOADER_H