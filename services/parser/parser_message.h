//
// Created by lxy on 2026/1/19.
//
/*
 * 解析器消息定义
 */
#ifndef ENERGYSTORAGE_PARSER_MESSAGE_H
#define ENERGYSTORAGE_PARSER_MESSAGE_H
#pragma once

#include <string>

#include "protocol_base.h"
#include "../../services/device/device_base.h"

namespace parser {
    // Timeout 保留仅作兼容残留；当前设备健康状态不再由 timeout 驱动
    enum class ParsedType {
        DeviceData,
        ParseError,
        Timeout,
    };

    struct ParserMessage {
        ParsedType type{ParsedType::ParseError};

        dev::LinkType link_type{dev::LinkType::Unknown};
        int link_index{-1};

        // 统一设备命名：所有事件都尽量填（Timeout 必填）
        std::string device_name;

        DeviceData device_data;

        int error_code{0};
        std::string error_text;
        uint64_t rx_ts_ms{0};   // T1

    };

} // namespace parser

#endif //ENERGYSTORAGE_PARSER_MESSAGE_H
