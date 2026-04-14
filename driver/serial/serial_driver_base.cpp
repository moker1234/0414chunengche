//
// Created by forlinx on 2025/12/17.
//

#include "serial_driver_base.h"
#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include <stdio.h>

#include "logger.h"

SerialDriverBase::SerialDriverBase(std::string dev)
    : device_(std::move(dev)) {}

SerialDriverBase::~SerialDriverBase() {
    close();
}

bool SerialDriverBase::init(const SerialConfig& cfg) {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
        perror("serial open failed");
        return false;
    }

    if (tcgetattr(fd_, &old_opt_) != 0) {
        perror("tcgetattr");
        close();
        return false;
    }

    if (!setupTermios(cfg)) {
        close();
        return false;
    }

    if (cfg.non_block && !setNonBlocking()) {
        close();
        return false;
    }

    return true;
}

void SerialDriverBase::close() {
    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &old_opt_);
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialDriverBase::reopen() {
    close();
    return true; // 由外部重新 init
}

bool SerialDriverBase::setNonBlocking() {
    int flags = fcntl(fd_, F_GETFL, 0);
    return (flags >= 0) &&
           (fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == 0);
}

bool SerialDriverBase::setupTermios(const SerialConfig& cfg) {
    struct termios opt{};
    tcgetattr(fd_, &opt);

    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    {}
    opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    opt.c_oflag &= ~(ONLCR);
{}
    opt.c_oflag &= ~OPOST;

    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= (cfg.data_bits == 7 ? CS7 : CS8);

    if (cfg.parity == 'N') opt.c_cflag &= ~PARENB;
    else {
        opt.c_cflag |= PARENB;
        if (cfg.parity == 'O') opt.c_cflag |= PARODD;
        else opt.c_cflag &= ~PARODD;
    }

    if (cfg.stop_bits == 2) opt.c_cflag |= CSTOPB;
    else opt.c_cflag &= ~CSTOPB;

    speed_t baud;
    switch (cfg.baudrate) {
    case 2400:   baud = B2400;   break;
    case 4800:   baud = B4800;   break;
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    default:
        LOGERR("[SERIAL] unsupported baudrate %d", cfg.baudrate);
        return false;
    }


    cfsetispeed(&opt, baud);
    cfsetospeed(&opt, baud);

    opt.c_cc[VMIN]  = 0;//1;
    opt.c_cc[VTIME] = 0;

    return tcsetattr(fd_, TCSANOW, &opt) == 0;
}

ssize_t SerialDriverBase::read(uint8_t* buf, size_t len) {
    return ::read(fd_, buf, len);
}

ssize_t SerialDriverBase::write(const uint8_t* buf, size_t len) {
    beforeWrite();
    ssize_t n = ::write(fd_, buf, len);
    afterWrite();
    return n;
}
