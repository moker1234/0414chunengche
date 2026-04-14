//
// Created by forlinx on 2025/12/17.
//

#ifndef ENERGYSTORAGE_CAN_DRIVER_H
#define ENERGYSTORAGE_CAN_DRIVER_H
#pragma once

#include <linux/can.h>
#include <string>

class CanDriver {
public:
    explicit CanDriver(const std::string& ifname);
    ~CanDriver();

    bool init();     // 创建/绑定 socket
    void close();    // 关闭 socket

    // 供 CANThread 在重连时使用
    bool reopen();               // close + init
    const std::string& ifname() const { return ifname_; }

    int  getFd() const;

    // 非阻塞语义：
    // - send/recv: 成功 true；无数据/不可写(EAGAIN/EINTR) false（可重试）；致命错误 false（errno 指示）
    bool send(const can_frame& frame_tx);
    bool recv(can_frame& frame_rx);

    // 判断 errno 是否属于“需要重建 socket/重连”的致命错误（供线程层决策）
    static bool isFatalErrno(int err);

private:
    bool setNonBlocking();
    bool setSocketOption();

private:
    int socket_fd_; // CAN 套接字文件描述符
    std::string ifname_;
};

#endif // ENERGYSTORAGE_CAN_DRIVER_H
