//
// Created by forlinx on 2026/1/1.
//

#ifndef ENERGYSTORAGE_RS485_SESSION_H
#define ENERGYSTORAGE_RS485_SESSION_H


// rs485_session.h
#pragma once
#include <vector>
#include <memory>
#include <cstdint>
#include <chrono>

#include "../protocol_base.h"

#include "rs485_session.h"
#include "../../../driver/serial/serial_thread.h"

#include <functional>

enum class Rs485TxState {
    Idle,
    WaitingResp
};

struct Rs485DeviceCtx {
    uint8_t addr;   // 从机地址
    std::unique_ptr<ProtocolBase> proto;

    Rs485TxState state{Rs485TxState::Idle};

    std::vector<uint8_t> rx_buffer;

    std::chrono::steady_clock::time_point last_tx;
    uint32_t timeout_ms{500};

    Rs485DeviceCtx(uint8_t a,
                   std::unique_ptr<ProtocolBase> p)
        : addr(a), proto(std::move(p)) {}
};


class Rs485Session {
public:
    using DeviceDataCallback = std::function<void(const DeviceData&)>;

    explicit Rs485Session(SerialThread& serial);

    void addDevice(Rs485DeviceCtx dev);

    // 定时轮询（由 Timer / AppManager 调用）
    void onPeriodic();

    // 串口 RX 字节输入
    void onRxBytes(const std::vector<uint8_t>& bytes);

    // 设置设备数据回调
    void setOnDeviceData(DeviceDataCallback cb) {
        printf("[RS485][SESSION] setOnDeviceData done\n");
        on_device_data_ = std::move(cb);
    }


private:
    void trySend(Rs485DeviceCtx& dev);
    void tryParse(Rs485DeviceCtx& dev);

private:
    SerialThread& serial_;
    std::vector<Rs485DeviceCtx> devices_; // 从机上下文列表
    DeviceDataCallback on_device_data_;
};


#endif //ENERGYSTORAGE_RS485_SESSION_H