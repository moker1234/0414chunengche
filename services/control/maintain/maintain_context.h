//
// Created by lxy on 2026/4/28.
//

#ifndef ENERGYSTORAGE_MAINTAIN_CONTEXT_H
#define ENERGYSTORAGE_MAINTAIN_CONTEXT_H


// services/control/maintain/maintain_context.h
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "../../normal/hmi_map_model.h"

namespace control::maintain {

struct MaintainEndpoint
{
    bool valid{false};

    // 来自 normal_map_logic.jsonl，不允许在业务代码里硬编码 HMI 地址
    uint16_t addr{0};
    uint16_t words{1};

    normal::HmiMapType type{normal::HmiMapType::Unknown};
    normal::HmiMapItemKind kind{normal::HmiMapItemKind::Empty};

    std::string path;
    std::string name;
    std::string value_type;

    void clear()
    {
        valid = false;
        addr = 0;
        words = 1;
        type = normal::HmiMapType::Unknown;
        kind = normal::HmiMapItemKind::Empty;
        path.clear();
        name.clear();
        value_type.clear();
    }
};

struct MaintainStorageState
{
    std::string config_path;

    // 是否真的从 maintain.json 读取成功。
    // false 代表使用默认值，不代表模块不可用。
    bool loaded_from_file{false};

    uint32_t current_password{123456};

    uint16_t box_id{0};

    // 0 表示不主动下发 UPS 关机命令，只保存参数。
    uint16_t ups_shutdown_time{0};

    int16_t aircon_set_temp{25};
    uint16_t aircon_set_humidity{60};

    uint64_t last_update_ts_ms{0};
};

struct MaintainLinkState
{
    bool system_loaded{false};

    int air_rs485_index{-1};
    uint8_t air_slave_id{1};

    int ups_rs232_index{-1};

    void clear()
    {
        system_loaded = false;
        air_rs485_index = -1;
        air_slave_id = 1;
        ups_rs232_index = -1;
    }
};

struct MaintainMapState
{
    bool bound{false};

    std::shared_ptr<const normal::HmiMapModel> model;

    // 维护界面相关变量，全部从 normal_map_logic.jsonl 的 path 识别
    MaintainEndpoint maintain_password;     // hmi.param.maintain_password
    MaintainEndpoint box_num;               // hmi.param.box_num
    MaintainEndpoint ups_shutdown_time;     // hmi.param.ups_shutdown_time
    MaintainEndpoint aircon_set_temp;       // hmi.param.aircon_set_temp
    MaintainEndpoint aircon_set_humidity;   // hmi.param.aircon_set_humidity
    MaintainEndpoint password_new_1;        // hmi.param.password_new_1

    void clear()
    {
        bound = false;
        model.reset();

        maintain_password.clear();
        box_num.clear();
        ups_shutdown_time.clear();
        aircon_set_temp.clear();
        aircon_set_humidity.clear();
        password_new_1.clear();
    }
};

struct MaintainPasswordWriteState
{
    bool pending_hi_valid{false};
    uint16_t pending_hi{0};
    uint64_t pending_hi_ts_ms{0};

    uint32_t pending_timeout_ms{2000};

    void clearPending()
    {
        pending_hi_valid = false;
        pending_hi = 0;
        pending_hi_ts_ms = 0;
    }
};

struct MaintainContext
{
    MaintainStorageState storage;
    MaintainLinkState links;
    MaintainMapState map;
    MaintainPasswordWriteState password_write;
};

} // namespace control::maintain

#endif //ENERGYSTORAGE_MAINTAIN_CONTEXT_H
