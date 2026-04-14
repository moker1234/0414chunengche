//
// Created by lxy on 2026/1/6.
//

#include "GPIODriver.h"


#include <gpiod.h>
#include <stdio.h>

GPIODriver::GPIODriver(const std::string& chip, int line)
    : chip_(chip), line_(line) {}

GPIODriver::~GPIODriver() {
    if (fd_ >= 0) {
        gpiod_line_release(reinterpret_cast<gpiod_line*>(fd_));
        fd_ = -1;
    }
}

bool GPIODriver::init() {
    gpiod_chip* chip = gpiod_chip_open(chip_.c_str());
    if (!chip) {
        perror("gpiod_chip_open");
        return false;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, line_);
    if (!line) {
        perror("gpiod_chip_get_line");
        return false;
    }

    if (gpiod_line_request_output(line, "rs485_de", 0) < 0) {
        perror("gpiod_line_request_output");
        return false;
    }

    fd_ = reinterpret_cast<intptr_t>(line);
    return true;
}

void GPIODriver::setValue(bool high) {
    if (fd_ < 0) return;
    gpiod_line_set_value(reinterpret_cast<gpiod_line*>(fd_), high ? 1 : 0);
}
