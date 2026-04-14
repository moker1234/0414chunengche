//
// Created by forlinx on 2025/12/26.
//

#ifndef ENERGYSTORAGE_EVENT_H
#define ENERGYSTORAGE_EVENT_H
// app/event.h
#pragma once
#include <linux/can.h>
#include <string>
#include <vector>

// #include "protocol_base.h"
#include "../services/protocol/protocol_base.h"
struct Event {
    enum class Type {
        Boot = 0,
        Tick,

        // ===== CAN =====
        CanRx,
        CanDown,
        CanUp,


        SerialRx,      // bytes 有效（RS485/RS232 统一）
        SerialDown,
        SerialUp,

        CmdStart,
        CmdStop,
        Error,
    }  type{Type::Boot};

    can_frame e_can_frame{};   // RX / TX 都用它
    int link_index{0};                // 串口/总线索引（例如 ttyS2=0, ttyS3=1）
    std::vector<uint8_t> bytes;       // 串口原始字节


    int code = 0;
    std::string text;
};

#endif //ENERGYSTORAGE_EVENT_H