//
// Created by forlinx on 2026/1/1.
//
/*
 * @brief RS485 会话类
 * @details 负责管理多个 RS485 设备的通信，包括添加设备、发送命令、接收响应等。
 */
/* 解释整个文件的作用
 * 该文件实现了 RS485 会话类的定义，包括构造函数、添加设备、周期调用和尝试发送命令等。
 * 构造函数用于初始化会话，添加设备用于添加 RS485 设备，周期调用用于发送命令和处理响应，
 * 尝试发送命令用于检查设备状态并发送读取命令。
 */
#include "rs485_session.h"
// rs485_session.cpp
#include "rs485_session.h"
#include <chrono>
#define RS485_SESSION_DEBUG

using SteadyClock = std::chrono::steady_clock;


/**
 * @brief 构造函数
 * @param serial 串口线程引用，用于发送和接收数据
 */
Rs485Session::Rs485Session(SerialThread& serial)
    : serial_(serial) {}

/**
 * @brief 添加 RS485 设备
 * @param dev RS485 设备上下文
 */
void Rs485Session::addDevice(Rs485DeviceCtx dev) {
    devices_.push_back(std::move(dev));
}

/**
 * @brief 周期调用，用于发送命令和处理响应
 * @details 检查每个设备的状态，如果是 Idle 则发送命令，
 *          如果是 WaitingResp 则检查是否超时。
 */
void Rs485Session::onPeriodic() {
    auto now = SteadyClock::now();

    for (auto& dev : devices_) {
        if (dev.state == Rs485TxState::Idle) {
            trySend(dev);
        } else {
            auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - dev.last_tx).count();

            if (elapsed > dev.timeout_ms) {
                // 超时，清理状态
                dev.rx_buffer.clear();
                dev.state = Rs485TxState::Idle;
            }
        }
    }
}
/**
 * @brief 尝试发送命令
 * @param dev RS485 设备上下文引用
 * @details 检查设备状态是否为 Idle，
 *          如果是则构建读取命令并发送，
 *          同时更新状态为 WaitingResp。
 */
void Rs485Session::trySend(Rs485DeviceCtx& dev) {
    auto cmd = dev.proto->buildReadCmd();
    if (cmd.empty()) return;

    serial_.send(cmd);
    dev.last_tx = SteadyClock::now();
    dev.state   = Rs485TxState::WaitingResp;
    printf("[RS485][SESSION] state -> WaitingResp\n");
#ifdef RS485_SESSION_DEBUG
    printf("[RS485][TX] ");
    for (auto b : cmd) printf("%02X ", b);
    printf("\n");
#endif

}
/**
 * @brief 接收字节回调
 * @param bytes 接收到的字节向量
 * @details 检查每个设备的状态是否为 WaitingResp，
 *          如果是则将字节添加到接收缓冲区，
 *          并尝试解析响应。
 */
void Rs485Session::onRxBytes(const std::vector<uint8_t>& bytes) {
    printf("[RS485][SESSION] onRxBytes size=%zu devs=%zu\n",
           bytes.size(), devices_.size());
    for (size_t i=0; i<devices_.size(); ++i) {
        printf("[RS485][SESSION] dev[%zu] state=%d buf=%zu\n",
               i, (int)devices_[i].state, devices_[i].rx_buffer.size());
    }
    //  简化：先全部丢给第一个 Waiting 的设备
    for (auto& dev : devices_) {
        if (dev.state == Rs485TxState::WaitingResp) {
            dev.rx_buffer.insert(
                dev.rx_buffer.end(),
                bytes.begin(),
                bytes.end()
            );
            printf("[RS485][SESSION] calling tryParse, buf size=%zu\n", dev.rx_buffer.size());
            tryParse(dev);
            break;
        }
    }
}
/**
 * @brief 尝试解析响应
 * @param dev RS485 设备上下文引用
 * @details 检查接收缓冲区是否包含完整的响应，
 *          如果是则调用协议解析器解析，
 *          并根据解析结果更新设备状态。
 */
void Rs485Session::tryParse(Rs485DeviceCtx& dev) {
    if (dev.rx_buffer.size() < 5) return;
    DeviceData out;
    printf("[RS485][SESSION] proto->parse() about to call, proto=%p\n", dev.proto.get());

    if (dev.proto->parse(dev.rx_buffer, out)) {
        dev.rx_buffer.clear();
        dev.state = Rs485TxState::Idle;

        // 解析成功，上抛 DeviceData
        if (on_device_data_) {
            on_device_data_(out);   //  真正上抛
        }else
        {
            printf("[RS485][SESSION] no on_device_data_ callback\n");
        }
    }
}

