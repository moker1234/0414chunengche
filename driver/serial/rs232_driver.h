//
// Created by lxy on 2026/1/20.
//

#ifndef ENERGYSTORAGE_RS232_DRIVER_H
#define ENERGYSTORAGE_RS232_DRIVER_H


#pragma once
#include "serial_driver_base.h"
#include <vector>

class Rs232Driver : public SerialDriverBase {
public:
    explicit Rs232Driver(std::string dev)
        : SerialDriverBase(std::move(dev)) {}

    const std::vector<int>& getAllFds() const override {
        fds_.clear();
        if (fd_ >= 0) fds_.push_back(fd_);
        return fds_;
    }

private:
    mutable std::vector<int> fds_;
};



#endif //ENERGYSTORAGE_RS232_DRIVER_H