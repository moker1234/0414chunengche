//
// Created by forlinx on 2025/12/17.
//

// rs485_driver.cpp
#include "rs485_driver.h"
#include <unistd.h>
// #include "gpio/GPIODriver.h"

Rs485Driver::Rs485Driver(std::string dev,
                         int de)
    : SerialDriverBase(std::move(dev)),
      de_gpio_(std::move(de)) {}

void Rs485Driver::beforeWrite() {
    // if (de_gpio_) de_gpio_->setValue(true);
}

void Rs485Driver::afterWrite() {
    // 等待移位寄存器清空（经验值）
    usleep(100);   // 100~200us，和波特率有关
    // if (de_gpio_) de_gpio_->setValue(false);
}
const std::vector<int>& Rs485Driver::getAllFds() const {
    return fds_;
}