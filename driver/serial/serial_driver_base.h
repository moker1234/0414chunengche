//
// Created by forlinx on 2025/12/17.
//

#ifndef ENERGYSTORAGE_SERIAL_BASE_H
#define ENERGYSTORAGE_SERIAL_BASE_H

#include <termios.h>
#include <fcntl.h>

#pragma once

#include <string>
#include <termios.h>
#include <unistd.h>
#include "./common/serial_config.h"
#include <vector>

class SerialDriverBase {
public:
    explicit SerialDriverBase(std::string dev);
    virtual ~SerialDriverBase();

    bool init(const SerialConfig& cfg);
    void close();
    bool reopen();

    int  getFd() const { return fd_; }
    virtual const std::vector<int>& getAllFds() const = 0;

    ssize_t read(uint8_t* buf, size_t len);
    ssize_t write(const uint8_t* buf, size_t len);

protected:
    virtual void beforeWrite() {}
    virtual void afterWrite() {}

    bool setupTermios(const SerialConfig& cfg);
    bool setNonBlocking();

protected:
    int fd_{-1};
    std::string device_;
    struct termios old_opt_{};
};


#endif //ENERGYSTORAGE_SERIAL_BASE_H