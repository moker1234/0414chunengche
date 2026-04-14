//
// Created by forlinx on 2025/12/28.
//

#ifndef ENERGYSTORAGE_CAN_PROTOCOL_BASE_H
#define ENERGYSTORAGE_CAN_PROTOCOL_BASE_H

#pragma once
#include <linux/can.h>
#include <cstdint>
#include <string>

#include "../../protocol/protocol_base.h" // DeviceData

namespace proto {

    class CanProtocolBase {
    public:
        virtual ~CanProtocolBase() = default;

        virtual std::string name() const = 0;

        // 解析 RX 帧，成功返回 true 并填 out
        virtual bool parse(const can_frame& fr, DeviceData& out) = 0;
    };

} // namespace proto

#endif // ENERGYSTORAGE_CAN_PROTOCOL_BASE_H
