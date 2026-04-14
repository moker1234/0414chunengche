//
// Created by lxy on 2026/1/6.
//

#ifndef ENERGYSTORAGE_GPIODRIVER_H
#define ENERGYSTORAGE_GPIODRIVER_H


#pragma once
#include <string>

class GPIODriver {
public:
    GPIODriver(const std::string& chip, int line);
    ~GPIODriver();

    bool init();
    void setValue(bool high);

private:
    std::string chip_; // "/dev/gpiochip0"
    int line_;         // 22
    int fd_{-1};
};


#endif //ENERGYSTORAGE_GPIODRIVER_H