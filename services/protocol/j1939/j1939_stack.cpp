//
// Created by lxy on 2026/2/8.
//

// services/protocol/j1939/j1939_stack.cpp
#include "j1939_stack.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

#include <linux/can.h>

#include "opensae_adapter.h"
#include "opensae_port.h"

namespace j1939 {

// tick() 期间指向当前正在运行的 stack（因为 OpenSAE 的回调是全局函数）
static thread_local J1939Stack* g_tls_active = nullptr;

// 只安装一次回调
static std::once_flag g_cb_once;

J1939Stack::J1939Stack(int can_index)
    : can_index_(can_index) {
    std::memset(&j1939_, 0, sizeof(j1939_));

    // OpenSAE 示例里建议把 other_ECU_address 全部初始化为 0xFF（broadcast）:contentReference[oaicite:6]{index=6}
    for (int i = 0; i < 255; ++i) {
        j1939_.other_ECU_address[i] = 0xFF;
    }

    // 默认给一个 SA（建议你后续从 system.json 配）
    j1939_.information_this_ECU.this_ECU_address = static_cast<uint8_t>(0x80 + (can_index_ & 0x1F));
}

J1939Stack::~J1939Stack() {
    stop();
}

void J1939Stack::setSourceAddress(uint8_t sa) {
    j1939_.information_this_ECU.this_ECU_address = sa;
}

void J1939Stack::ensureCallbacksInstalled_() {
    std::call_once(g_cb_once, [] {
        // 注册 OpenSAE 的 INTERNAL_CALLBACK 回调入口:contentReference[oaicite:7]{index=7}
        CAN_Set_Callback_Functions(
            &J1939Stack::cbSend_,
            &J1939Stack::cbRead_,
            &J1939Stack::cbTraffic_,
            &J1939Stack::cbDelayMs_
        );
    });
}

bool J1939Stack::start() {
    if (started_) return true;

    ensureCallbacksInstalled_();

    // 启动时 OpenSAE 会 Load_Struct，然后做 Address Claimed 广播等:contentReference[oaicite:8]{index=8}
    const bool ok = Open_SAE_J1939_Startup_ECU(&j1939_);
    started_ = ok;
    return ok;
}

void J1939Stack::stop() {
    if (!started_) return;

    // 关闭时保存信息（可选）:contentReference[oaicite:9]{index=9}
    (void)Open_SAE_J1939_Closedown_ECU(&j1939_);
    started_ = false;

    // 清空队列
    {
        std::lock_guard<std::mutex> lk(rx_mtx_);
        rx_q_.clear();
    }
}

void J1939Stack::setRxCallback(RxCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mtx_);
    rx_cb_ = std::move(cb);
}

void J1939Stack::onCanFrame(int can_index, const can_frame& fr) {
    if (can_index != can_index_) return;

    // J1939 使用扩展帧
    if ((fr.can_id & CAN_EFF_FLAG) == 0) return;

    RxItem it{};
    it.id29 = (fr.can_id & CAN_EFF_MASK);
    it.dlc  = (fr.can_dlc > 8) ? 8 : fr.can_dlc;
    if (it.dlc > 0) {
        std::memcpy(it.data, fr.data, it.dlc);
    }

    {
        std::lock_guard<std::mutex> lk(rx_mtx_);
        rx_q_.push_back(it);
        // 简单限长（避免堆积爆内存）；后续可改成统计丢包
        if (rx_q_.size() > 4096) {
            rx_q_.pop_front();
        }
    }

    // 同时把“原始帧信息”回调出去（用于你上层日志/调试）
    RxCallback cb;
    {
        std::lock_guard<std::mutex> lk(cb_mtx_);
        cb = rx_cb_;
    }
    if (cb) {
        J1939Frame jf{};
        jf.can_index = can_index_;
        jf.id29 = it.id29;
        jf.dlc  = it.dlc;
        if (jf.dlc > 0) std::memcpy(jf.data, it.data, jf.dlc);
        cb(jf);
    }
}

std::optional<J1939Stack::RxItem> J1939Stack::popRx_() {
    std::lock_guard<std::mutex> lk(rx_mtx_);
    if (rx_q_.empty()) return std::nullopt;
    RxItem it = rx_q_.front();
    rx_q_.pop_front();
    return it;
}

void J1939Stack::tick(uint32_t budget) {
    if (budget == 0) return;

    if (!started_) {
        // 允许“懒启动”
        if (!start()) return;
    }

    // 在本线程内把 TLS 指向当前 stack，供 OpenSAE 的回调使用
    g_tls_active = this;

    // 关键：告诉 adapter 当前通道是谁（OpenSAE 的 CAN_Read/Send 没有通道参数）
    opensae_set_active_can_index(can_index_);

    // OpenSAE：每调用一次 Listen_For_Messages()，最多消费一条 CAN 消息
    // 所以用 budget 控制每次 tick 最多处理多少帧
    for (uint32_t i = 0; i < budget; ++i) {
        // 注意：Open_SAE_J1939_Listen_For_Messages 的参数取决于你 OpenSAE 版本。
        // 常见是传入 ECU 结构体指针（比如 open_sae_ctx_）。
        // 下面假设你 stack 内部有 open_sae_（或类似）句柄：
        (void)Open_SAE_J1939_Listen_For_Messages(&j1939_);

        // 可选优化：如果你在 adapter 里提供 “当前通道队列是否为空” 的函数，就能提前 break。
        // 例如：if (opensae_rx_queue_empty(can_index_)) break;
    }

    // 可选：避免误用（一般不需要清空 TLS）
    // g_tls_active = nullptr;
}

bool J1939Stack::sendRaw29(uint32_t id29, const uint8_t* data, uint8_t dlc) {
    if (dlc > 8) return false;
    const uint32_t can_id = id29;//(id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
    return sendCanFrame(can_index_, can_id, data, dlc);
}

bool J1939Stack::sendSingleFramePgn(uint32_t pgn, const uint8_t* data, uint8_t dlc,
                                    uint8_t priority, uint8_t sa, uint8_t dst) {
    if (dlc > 8) return false;
    const uint32_t id29 = buildId29FromPgn(pgn, priority, sa, dst);
    return sendRaw29(id29, data, dlc);
}

/* ===================== OpenSAE INTERNAL_CALLBACK 回调实现 ===================== */

void J1939Stack::cbSend_(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    // OpenSAE 的 ID 传的是 29-bit（不含 CAN_EFF_FLAG），直接封装成 Linux can_frame 发送
    if (!g_tls_active) return;
    if (dlc > 8) dlc = 8;

    const uint32_t can_id = (ID & CAN_EFF_MASK) | CAN_EFF_FLAG;
    (void)sendCanFrame(opensae_get_active_can_index(), can_id, data, dlc); // 这里使用opensae_get_active_can_index获取idx，原来的g_tls_active->can_index_获取到的始终是0
}

void J1939Stack::cbRead_(uint32_t* ID, uint8_t data[], bool* is_new_message) {
    if (!ID || !data || !is_new_message) return;
    *is_new_message = false;

    if (!g_tls_active) return;

    auto opt = g_tls_active->popRx_();
    if (!opt.has_value()) return;

    *ID = opt->id29;
    // OpenSAE 期望总是给 8 字节数组（它内部 memcpy 8 字节）:contentReference[oaicite:11]{index=11}
    std::memset(data, 0, 8);
    if (opt->dlc > 0) std::memcpy(data, opt->data, opt->dlc);

    *is_new_message = true;
}

void J1939Stack::cbTraffic_(uint32_t /*ID*/, uint8_t /*dlc*/, uint8_t /*data*/[], bool /*is_tx*/) {
    // 可选：你想要 OpenSAE 的“总线流量回调”时在这里打日志
}

void J1939Stack::cbDelayMs_(uint8_t ms) {
    // OpenSAE 某些路径会调用 CAN_Delay（例如示例/内部流程），这里给一个轻量 sleep
    if (ms == 0) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace j1939
