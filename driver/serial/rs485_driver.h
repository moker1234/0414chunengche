//
// Created by forlinx on 2025/12/17.
//

#ifndef ENERGYSTORAGE_RS485_DRIVER_H
#define ENERGYSTORAGE_RS485_DRIVER_H


// rs485_driver.h
#pragma once
#include <memory>

#include "serial_driver_base.h"
// #include "gpio/GPIODriver.h"

class Rs485Driver : public SerialDriverBase {
public:
    Rs485Driver(std::string dev,
                int de);

    const std::vector<int>& getAllFds() const override;
protected:
    void beforeWrite() override;
    void afterWrite() override;

private:
    int de_gpio_;
    std::vector<int> fds_;
};


#endif //ENERGYSTORAGE_RS485_DRIVER_H