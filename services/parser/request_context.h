//
// Created by lxy on 2026/1/28.
//

#ifndef ENERGYSTORAGE_REQUEST_CONTEXT_H
#define ENERGYSTORAGE_REQUEST_CONTEXT_H
#pragma once

#include <cstdint>
#include <string>

#include "../device/device_types.h"

namespace parser {

    struct RequestContext {
        dev::LinkType link_type;   // RS485 / RS232
        int link_index;            // 串口索引

        //  “真实 pending”：每次发送/重发都会刷新
        uint64_t send_ts_ms;       // 最近一次发送时间（ms）
        uint32_t timeout_ms;       // 超时阈值（ms）

        // 重试次数（0=首次发送；每次 Resend +1）
        uint8_t retry_count{0};

        // 用于 Timeout 事件统一命名来源
        std::string device_name;
    };

} // namespace parser

#endif //ENERGYSTORAGE_REQUEST_CONTEXT_H
