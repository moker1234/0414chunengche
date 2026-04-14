//
// Created by lxy on 2026/2/8.
//

#ifndef ENERGYSTORAGE_OPENSAE_PORT_H
#define ENERGYSTORAGE_OPENSAE_PORT_H

// services/protocol/j1939/opensae_port.h
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <linux/can.h>

/*
 * OpenSAE 端口层（Port Layer）
 *
 * 目标：
 *  - 复用你工程的 CanThread/CanDriver（不让 OpenSAE 自己开 SocketCAN）
 *  - 为 OpenSAE（或你自己的 J1939 栈）提供统一的“发送/时间/存储”等平台能力
 *
 * 当前阶段（第一步：收发跑通）：
 *  - 实现：nowMs + sendCanFrame
 *  - 存储接口先留 stub（后续如需保存 NAME/address 等再补）
 */

namespace j1939 {

    using CanTxFunc = std::function<void(const can_frame&)>;

    /** 绑定某一路 CAN（can_index）对应的发送函数（通常是 CanThread::send 的包装） */
    void bindCanTx(int can_index, CanTxFunc tx);

    /** 解绑某一路 CAN */
    void unbindCanTx(int can_index);

    /** 获取单调时钟毫秒（steady_clock） */
    uint64_t nowMs();

    /**
     * 发送 CAN 帧（J1939 通常为 29bit 扩展帧）
     * @return true 表示成功“投递到发送函数”；不代表总线层已发送完成
     */
    bool sendCanFrame(int can_index, uint32_t can_id, const uint8_t* data, uint8_t dlc);

    /** 便利：发送已构造好的 can_frame */
    bool sendCanFrame(int can_index, const can_frame& fr);

    /* =====（可选）后续接 OpenSAE 时可能需要的“持久化”接口：先 stub ===== */

    /**
     * 读取持久化结构（stub）
     * @return true 表示读取成功（当前默认 false）
     */
    bool loadBlob(const std::string& key, std::vector<uint8_t>& out);

    /**
     * 写入持久化结构（stub）
     * @return true 表示写入成功（当前默认 false）
     */
    bool saveBlob(const std::string& key, const uint8_t* data, size_t len);

} // namespace j1939


#endif //ENERGYSTORAGE_OPENSAE_PORT_H