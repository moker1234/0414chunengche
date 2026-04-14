//
// Created by forlinx on 2025/12/20.
//

#ifndef ENERGYSTORAGE_SERIAL_CONFIG_H
#define ENERGYSTORAGE_SERIAL_CONFIG_H
// driver/common/serial_config.h
#pragma once
#include <termios.h>
#include <cstdint>

struct SerialConfig {
    int baudrate;      // 9600 / 115200 ...
    int data_bits;     // 7 / 8
    char parity;       // 'N' 'E' 'O'
    int stop_bits;     // 1 / 2

    bool non_block = true;
};

#endif //ENERGYSTORAGE_SERIAL_CONFIG_H