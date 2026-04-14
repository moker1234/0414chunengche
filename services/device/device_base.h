//
// Created by lxy on 2026/1/11.
//


/* 解释整个文件的作用
 * 该文件实现了设备基类DeviceBase的定义，包括构造函数、设备上下文结构体和设备接口函数。
 * 构造函数用于初始化设备名称，设备上下文结构体用于存储设备的上下文信息，设备接口函数用于定义设备的定时驱动、RX 输入和可选输出。
 */

#ifndef ENERGYSTORAGE_DEVICE_BASE_H
#define ENERGYSTORAGE_DEVICE_BASE_H

#pragma once

#include "device_types.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace dev {

    // Scheduler 给 Device 的能力集合：Device 不直接接触线程/FD
    struct DeviceContext {
        // 发 CAN：can_index=0..2
        std::function<void(int /*can_index*/, const can_frame&)> send_can;

        // 发串口：type=RS485/RS232/DISPLAY_RS485, index 对应通道号
        std::function<void(LinkType /*type*/, int /*index*/, const std::vector<uint8_t>&)> send_serial;

        // 发以太网（payload 由你定义 JSON/Protobuf/二进制）
        std::function<void(const std::vector<uint8_t>& /*payload*/)> send_eth;
    };

    // Device 只关心：定时驱动 + RX 输入 +（可选）输出数据给 Aggregator
    class DeviceBase {
    public:
        virtual ~DeviceBase() = default;

        virtual const char* name() const = 0;

        // 设备挂载在哪些链路（例如 BMS 挂 CAN0；气体探测器挂 RS485_1）
        virtual std::vector<LinkId> links() const = 0;

        // Scheduler 定时驱动（设备自己决定要不要发）
        virtual void onTick(DeviceContext& ctx, TickKind kind) { (void)ctx; (void)kind; }

        // RX 输入（Scheduler 根据 links()/路由规则投递）
        virtual void onCanRx(DeviceContext& ctx, const CanRxPacket& rx) { (void)ctx; (void)rx; }
        virtual void onSerialRx(DeviceContext& ctx, const SerialRxPacket& rx) { (void)ctx; (void)rx; }
    };

    using DevicePtr = std::unique_ptr<DeviceBase>;

} // namespace dev


#endif //ENERGYSTORAGE_DEVICE_BASE_H