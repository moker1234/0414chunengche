//
// Created by forlinx on 2025/12/17.
//

#include "can_driver.h"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "logger.h"

CanDriver::CanDriver(const std::string& ifname)
    : socket_fd_(-1), ifname_(ifname) {}

CanDriver::~CanDriver() {
    close();
}

bool CanDriver::init() {
    struct ifreq ifr{};
    struct sockaddr_can addr{};

    printf("[CAN] init %s\n", ifname_.c_str());

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (socket_fd_ < 0) {
        printf("[CAN][ERROR] socket failed: %s\n", strerror(errno));
        return false;
    }

    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        printf("[CAN][ERROR] ioctl(SIOCGIFINDEX) failed, if=%s, err=%s\n",
               ifname_.c_str(), strerror(errno));
        close();
        return false;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        printf("[CAN][ERROR] bind failed, if=%s, err=%s\n",
               ifname_.c_str(), strerror(errno));
        close();
        return false;
    }

    if (!setNonBlocking()) {
        printf("[CAN][ERROR] setNonBlocking failed: %s\n", strerror(errno));
        close();
        return false;
    }

    if (!setSocketOption()) {
        printf("[CAN][ERROR] setSocketOption failed\n");
        close();
        return false;
    }

    printf("[CAN] %s init success, fd=%d\n", ifname_.c_str(), socket_fd_);
    return true;
}

void CanDriver::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanDriver::reopen() {
    close();
    return init();
}

// 设置 CAN 套接字为非阻塞模式
bool CanDriver::setNonBlocking() {
    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0) return false;
    if (::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) return false;
    return true;
}
// 设置 CAN 套接字选项
bool CanDriver::setSocketOption() {
    int recv_buf = 256 * 1024;
    int send_buf = 256 * 1024;

    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf)) < 0) {
        printf("[CAN][ERROR] setsockopt SO_SNDBUF failed: %s\n", strerror(errno));
        return false;
    }
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &recv_buf, sizeof(recv_buf)) < 0) {
        printf("[CAN][ERROR] setsockopt SO_RCVBUF failed: %s\n", strerror(errno));
        return false;
    }

    // 禁用回环（按你原需求）
    int loopback = 0;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        printf("[CAN][ERROR] setsockopt CAN_RAW_LOOPBACK failed: %s\n", strerror(errno));
        return false;
    }

    // ✅ 建议：打开错误帧接收，便于识别 BUS-OFF 等（不影响正常帧）
    can_err_mask_t err_mask =
        CAN_ERR_TX_TIMEOUT |
        CAN_ERR_BUSOFF |
        CAN_ERR_CRTL |
        CAN_ERR_PROT |
        CAN_ERR_RESTARTED;
    (void)::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    return true;
}

int CanDriver::getFd() const {
    return socket_fd_;
}

// bool CanDriver::send(const can_frame& frame_tx) {
//     if (socket_fd_ < 0) {
//         errno = EBADF;
//         return false;
//     }
//     ssize_t n = ::write(socket_fd_, &frame_tx, sizeof(frame_tx));
//     if (n < 0) {
//         // 非阻塞常见：EAGAIN；信号打断：EINTR
//         if (errno == EAGAIN || errno == EINTR) return false;
//         return false;
//     }
//     return n == static_cast<ssize_t>(sizeof(frame_tx));
// }
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <cinttypes>   // PRIu32 / PRIX32

bool CanDriver::send(const can_frame& frame_tx) {
    if (socket_fd_ < 0) {
        errno = EBADF;
        LOG_COMM_D("[CAN][TX] fd=%d EBADF\n", socket_fd_);
        return false;
    }

    // 发送前打印
    // LOG_COMM_D("[CAN][TX][WRITE] fd=%d can_id=0x%08" PRIX32 " dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
    //             socket_fd_,
    //             (uint32_t)frame_tx.can_id,
    //             (unsigned)frame_tx.can_dlc,
    //             frame_tx.data[0], frame_tx.data[1], frame_tx.data[2], frame_tx.data[3],
    //             frame_tx.data[4], frame_tx.data[5], frame_tx.data[6], frame_tx.data[7]);

    errno = 0;
    ssize_t n = ::write(socket_fd_, &frame_tx, sizeof(frame_tx));

    // 发送后打印
    if (n < 0) { // 发送失败
        const int e = errno;
        LOG_THROTTLE_MS("can_tx_error", 500, LOG_COMM_D,"[CAN][TX][WRITE][FAIL] fd=%d ret=%zd errno=%d(%s)\n",
                    socket_fd_, n, e, std::strerror(e));

        // 非阻塞常见：EAGAIN；信号打断：EINTR
        if (e == EAGAIN || e == EINTR) return false;
        return false;
    }

    if (n != (ssize_t)sizeof(frame_tx)) { // 发送不完整
        LOG_COMM_D("[CAN][TX][WRITE][SHORT] fd=%d ret=%zd expect=%zu\n",
                    socket_fd_, n, sizeof(frame_tx));
        return false;
    }

    // LOG_COMM_D("[CAN][TX][WRITE][OK] fd=%d ret=%zd\n", socket_fd_, n);
    return true;
}


bool CanDriver::recv(can_frame& frame_rx) {
    if (socket_fd_ < 0) {
        errno = EBADF;
        return false;
    }

    ssize_t n = ::read(socket_fd_, &frame_rx, sizeof(frame_rx));
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) return false;
        printf("[CAN][ERROR] recv failed: %s\n", strerror(errno));
        return false;
    }
    return n == static_cast<ssize_t>(sizeof(frame_rx));
}

bool CanDriver::isFatalErrno(int err) {
    // 这些错误通常意味着：接口 down、设备不存在、socket 已坏，需要重建
    switch (err) {
        case ENETDOWN:
        case ENETUNREACH:
        case ENODEV:
        case EIO:
        case EBADF:
        case EPIPE:
        case ECONNRESET:
        case ECONNREFUSED:
        case EADDRNOTAVAIL:
        case EINVAL:
            return true;
        default:
            return false;
    }
}
