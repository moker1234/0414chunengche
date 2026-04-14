//
// Created by lxy on 2026/1/11.
//

/* 解释整个文件的作用
 * 该文件实现了设备类型定义的枚举类LinkType、设备上下文结构体LinkId、上行输入结构体CanRxPacket和SerialRxPacket、定时类型枚举类TickKind。
 * 枚举类LinkType定义了物理通道的类型，包括CAN、RS485、RS232、DISPLAY_RS485和ETH。
 * 结构体LinkId用于匹配设备的输入/输出端口，包含通道类型、索引和端口名称。
 * 结构体CanRxPacket用于表示CAN通道的接收数据包，包含通道索引和CAN帧。
 * 结构体SerialRxPacket用于表示串口通道的接收数据包，包含通道类型、索引、端口名称和字节向量。
 * 枚举类TickKind定义了定时类型，包括快速能量、慢速能量和传感器轮询。
 */

#ifndef ENERGYSTORAGE_DEVICE_TYPES_H
#define ENERGYSTORAGE_DEVICE_TYPES_H

#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <vector>

namespace dev {

    // 物理通道标识（你会有 3 CAN、3xRS485、1xRS232、1xDisplay485、1xEth）
    enum class LinkType {
        CAN,
        RS485,
        RS232,
        DISPLAY_RS485,
        ETH,
        Unknown,
    };
    /*
     * 物理通道标识：用于匹配设备的输入/输出端口
     */
    struct LinkId {
        LinkType type{};            // 通道类型
        int index{-1};              // CAN0..2, RS485_0..2, RS232_0, DISPLAY_0, ETH_0
        std::string port_name;      // 对串口可填 "rs485_1" 或 "/dev/ttyS7"（由你统一口径）

        static LinkId Can(int i) { return LinkId{LinkType::CAN, i, {}}; }
        static LinkId Rs485(int i, std::string name = {}) { return LinkId{LinkType::RS485, i, std::move(name)}; }
        static LinkId Rs232(int i, std::string name = {}) { return LinkId{LinkType::RS232, i, std::move(name)}; }
        static LinkId Display(int i, std::string name = {}) { return LinkId{LinkType::DISPLAY_RS485, i, std::move(name)}; }
        static LinkId Eth(int i) { return LinkId{LinkType::ETH, i, {}}; }
    };

    // 上行输入（从 Thread → Scheduler → Device）
    struct CanRxPacket {
        int can_index{-1};       // 0..2
        can_frame frame{};        // 标准帧（11bit ID）
    };

    struct SerialRxPacket {
        LinkType type{LinkType::RS485}; // RS485 / RS232 / DISPLAY_RS485(一般不收)
        int serial_index{-1};     // 0..2 for rs485, 0 for rs232, 0 for display
        std::string port_name;    // "rs485_1" / "rs232_0" ...
        std::vector<uint8_t> bytes;
    };

    // 定时类型（Scheduler 统一驱动）
    enum class TickKind {
        EnergyFast,       // 例如 100ms
        EnergySlow,       // 例如 1s
        SensorPoll,       // 例如 1s
        DisplayRefresh,   // 例如 200~500ms
        UploadTick        // 例如 2s/5s/10s
    };

} // namespace dev

#endif //ENERGYSTORAGE_DEVICE_TYPES_H