//
// Created by forlinx on 2025/12/20.
//

#ifndef ENERGYSTORAGE_SERIAL_EVENT_H
#define ENERGYSTORAGE_SERIAL_EVENT_H
// driver/serial/serial_event.h
#pragma once
#include <vector>
#include <string>

enum class SerialEventType {
    RX_DATA,
    TX_ERROR,
    RX_ERROR
};

// 串口事件结构体
struct SerialEvent {
    SerialEventType type;
    std::string     port;   // "/dev/ttyS4"
    std::vector<uint8_t> data;  // RX bytes
    int error{0};               // errno
};

#endif //ENERGYSTORAGE_SERIAL_EVENT_H