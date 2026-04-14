//
// Created by lxy on 2026/2/8.
//

#ifndef ENERGYSTORAGE_J1939_STACK_H
#define ENERGYSTORAGE_J1939_STACK_H


// services/protocol/j1939/j1939_stack.h
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>

#include <linux/can.h>

#include "j1939_types.h"

extern "C" {
// OpenSAE 头文件（third_party/opensae_j1939 已加入 include path）
#include "openSAE_j1939/Open_SAE_J1939/Open_SAE_J1939.h"
#include "openSAE_j1939/Hardware/Hardware.h"
}

namespace j1939 {

/*
 * J1939Stack：挂接 OpenSAE 的 C++ 包装层（多 CAN 通道可并存）
 *
 * 运行机制：
 *  - onCanFrame(): CanThread RX 回调 -> 把帧入队
 *  - tick():      设置 TLS active stack -> 反复调用 Open_SAE_J1939_Listen_For_Messages()
 *                -> OpenSAE 内部通过 INTERNAL_CALLBACK 的 CAN_Read_Message() 回调从本栈队列取帧
 *
 * OpenSAE API 参考：
 *  - Open_SAE_J1939_Listen_For_Messages(J1939*):contentReference[oaicite:3]{index=3}
 *  - Open_SAE_J1939_Startup_ECU(J1939*) / Closedown:contentReference[oaicite:4]{index=4}
 *  - CAN_Set_Callback_Functions(...):contentReference[oaicite:5]{index=5}
 */

class J1939Stack {
public:
    using RxCallback = std::function<void(const J1939Frame&)>;

    explicit J1939Stack(int can_index);
    ~J1939Stack();

    int canIndex() const { return can_index_; }

    /** 设置本 ECU 源地址（0~253），建议启动前设置 */
    void setSourceAddress(uint8_t sa);

    /** 启动 OpenSAE（可重复调用；只会执行一次） */
    bool start();

    /** 关闭 OpenSAE（可选） */
    void stop();

    /** CanThread RX 回调入口 */
    void onCanFrame(int can_index, const can_frame& fr);

    /** 周期处理：驱动 OpenSAE 消费队列并解析 */
    void tick(uint32_t budget = 64);

    /** 发送：29-bit ID（不含 CAN_EFF_FLAG） */
    bool sendRaw29(uint32_t id29, const uint8_t* data, uint8_t dlc);

    /** 发送：单帧 PGN（dlc<=8） */
    bool sendSingleFramePgn(uint32_t pgn, const uint8_t* data, uint8_t dlc,
                            uint8_t priority = 6,
                            uint8_t sa = 0x80,
                            uint8_t dst = 0xFF);

    /** 设置接收回调（解析成 J1939Frame 后回调） */
    void setRxCallback(RxCallback cb);

    /** 访问 OpenSAE 的 J1939 结构（后续你要读 DM1/地址表等时会用） */
    J1939* raw() { return &j1939_; }
    const J1939* raw() const { return &j1939_; }

private:
    struct RxItem {
        uint32_t id29{0};      // 29-bit ID（不含 CAN_EFF_FLAG）
        uint8_t  dlc{0};
        uint8_t  data[8]{};
    };

    // === OpenSAE 全局回调（INTERNAL_CALLBACK） ===
    static void cbSend_(uint32_t ID, uint8_t dlc, uint8_t data[]);
    static void cbRead_(uint32_t* ID, uint8_t data[], bool* is_new_message);
    static void cbTraffic_(uint32_t ID, uint8_t dlc, uint8_t data[], bool is_tx);
    static void cbDelayMs_(uint8_t ms);

    static void ensureCallbacksInstalled_();

    // 从队列弹出一帧（给 cbRead_ 使用）
    std::optional<RxItem> popRx_();

private:
    int can_index_{0};

    // OpenSAE 的核心状态
    J1939 j1939_{};

    bool started_{false};

    // RX 队列：由 onCanFrame 入队，由 tick->cbRead_ 出队
    std::mutex rx_mtx_;
    std::deque<RxItem> rx_q_;

    // 用户侧回调（调试/上层事件用）
    std::mutex cb_mtx_;
    RxCallback rx_cb_{};
};

} // namespace j1939



#endif //ENERGYSTORAGE_J1939_STACK_H