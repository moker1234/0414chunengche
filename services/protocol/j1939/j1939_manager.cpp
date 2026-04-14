// services/protocol/j1939/j1939_manager.cpp
#include "j1939_manager.h"

#include <chrono>
#include <cstring>

#include "logger.h"
#include "opensae_adapter.h"   // 我们刚新增的 adapter（active_can_index + rx queue）
#include "Open_SAE_J1939.h"    // OpenSAE Listen/Startup 等（third_party include path 已加）

namespace j1939 {

static J1939Manager* g_mgr = nullptr;

J1939Manager::J1939Manager() {
    g_mgr = this;
}

J1939Manager::~J1939Manager() {
    stop();
    if (g_mgr == this) g_mgr = nullptr;
}

void J1939Manager::bindChannel(int can_index, CanTxFunc tx) {
    if (can_index < 0) return;

    bindCanTx(can_index, tx);

    std::lock_guard<std::mutex> lk(mtx_);

    tx_map_[can_index] = std::move(tx);
    channels_.insert(can_index);

    if (stacks_.find(can_index) == stacks_.end()) {
        stacks_[can_index] = std::make_shared<J1939Stack>(can_index);
    }

    // 绑定/创建时清空队列，避免历史残帧干扰验证
    opensae_clear_rx_queue(can_index);

    LOG_SYS_I("[J1939][MGR] bindChannel idx=%d", can_index);
}

void J1939Manager::unbindChannel(int can_index) {
    unbindCanTx(can_index);
    std::lock_guard<std::mutex> lk(mtx_);
    stacks_.erase(can_index);
    tx_map_.erase(can_index);
    channels_.erase(can_index);

    opensae_clear_rx_queue(can_index);

    LOG_SYS_I("[J1939][MGR] unbindChannel idx=%d", can_index);
}

std::shared_ptr<J1939Stack> J1939Manager::getStack(int can_index) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = stacks_.find(can_index);
    if (it == stacks_.end()) return {};
    return it->second;
}

void J1939Manager::onCanFrame(int can_index, const can_frame& fr) {
    // 1) 推入 OpenSAE adapter 的 RX 队列（OpenSAE Listen 从 CAN_Read_Message 取）
    {
        uint8_t data[8] = {0};
        const uint8_t dlc = (fr.can_dlc > 8) ? 8 : fr.can_dlc;
        std::memcpy(data, fr.data, dlc);

        // 你工程 CanDriver 打印的 id 类似 0x9801EFA0，本质上应是 29bit 扩展帧ID
        // 这里直接原样推入
        opensae_push_rx_frame(can_index, (uint32_t)fr.can_id, dlc, data);
    }

    // 2) 目前阶段“不解析”，stack->onCanFrame 可留着（如果你内部做了统计/日志）
    std::shared_ptr<J1939Stack> st;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = stacks_.find(can_index);
        if (it == stacks_.end()) return;
        st = it->second;
    }
    if (st) st->onCanFrame(can_index, fr);
}

void J1939Manager::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;

    th_ = std::thread([this] { this->threadMain_(); });
    LOG_SYS_I("[J1939][MGR] start");
}

void J1939Manager::stop() {
    if (!running_.exchange(false)) return;
    if (th_.joinable()) th_.join();
    LOG_SYS_I("[J1939][MGR] stop");
}

void J1939Manager::tickOnce() {
    // 复制一份通道集合，避免 tick 时长持锁
    std::unordered_set<int> chans;
    std::unordered_map<int, std::shared_ptr<J1939Stack>> stacks;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        chans = channels_;
        stacks = stacks_;
    }

    for (int idx : chans) {
        auto it = stacks.find(idx);
        if (it == stacks.end() || !it->second) continue;

        // 让 OpenSAE 的 CAN_Read/CAN_Send 都落到当前 idx
        opensae_set_active_can_index(idx);
        // 接收测试
        uint32_t rid = 0;
        uint8_t  rdata[8] = {0};
        if (CAN_Read_Message(&rid, rdata)) {
            LOG_SYS_I("[J1939][TEST][RX] idx=%d id=0x%08X d0=%02X", idx, rid, rdata[0]);
        }

        // 这里的 J1939Stack::tick() 你应该内部调用 Open_SAE_J1939_Listen_For_Messages(j1939*)
        it->second->tick();

        // 如果你暂时还没在 stack->tick() 里接 OpenSAE，那么也可以在这里直接调用 Listen：
        // Open_SAE_J1939_Listen_For_Messages(it->second->raw()); // 取决于你 stack 的封装
    }
}

void J1939Manager::threadMain_() {
    using namespace std::chrono_literals;

    // 先给一个较高频率，确保收发验证“看得见”
    // 后续可以调成 5~20ms 或者由系统 tick 驱动
    while (running_.load()) {
        tickOnce();
        std::this_thread::sleep_for(5ms);
    }
}
    // J1939发送测试程序
//     void J1939Manager::threadMain_() {
//     using namespace std::chrono_literals;
//
//     uint64_t last_tx_ms = 0;
//
//     while (running_.load()) {
//         tickOnce();
//
//         // ===== 每 1 秒发一帧测试帧（最小闭环验证）=====
//         const uint64_t now_ms = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
//                                     std::chrono::steady_clock::now().time_since_epoch()
//                                 ).count();
//
//         if (now_ms - last_tx_ms >= 1000) {
//             last_tx_ms = now_ms;
//
//             // 选择一个已绑定的通道（优先 can0）
//             int tx_idx = -1;
//             {
//                 std::lock_guard<std::mutex> lk(mtx_);
//                 if (!channels_.empty()) tx_idx = 2;
//             }
//             if (tx_idx >= 0) {
//                 // 让 OpenSAE 的 CAN_Send_Message 走到这个 can_index
//                 opensae_set_active_can_index(tx_idx);
//
//                 // 组一帧固定数据（8字节）
//                 uint8_t data[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
//
//                 // 固定 29-bit 扩展帧 ID（你也可以用你设备实际 ID）
//                 // 注意：你的 CanDriver/SocketCAN 里如果要求设置 CAN_EFF_FLAG，
//                 // 那就在 adapter->sendRaw 或 CanDriver 里处理；此处先原样发。
//                 uint32_t test_id = 0x18FF50E5; // 常见的扩展帧格式示例
//
//                 LOG_SYS_I("[J1939][TEST][TX] idx=%d id=0x%08X", tx_idx, test_id);
//
//                 // 关键：调用 OpenSAE 的发送接口，验证闭环
//                 CAN_Send_Message(test_id, data);
//             }
//         }
//
//         std::this_thread::sleep_for(5ms);
//     }
// }


void J1939Manager::sendRaw_(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    CanTxFunc tx;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = tx_map_.find(can_index);
        if (it == tx_map_.end()) {
            LOGERR("[J1939][TX] drop: idx=%d not bound (tx_map_ miss)", can_index);
            return;
        }
        tx = it->second;
    }

    can_frame fr{};
    fr.can_id = can_id;//(can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    fr.can_dlc = (dlc > 8) ? 8 : dlc;
    if (data && fr.can_dlc > 0) {
        std::memcpy(fr.data, data, fr.can_dlc);
    }

    // 你可以在这里加一条日志，验证 OpenSAE 的 TX 是否真的走到了工程的 send
    LOG_SYS_I("[J1939][sendRaw_] can_index=%d id=0x%08X (tx_map keys: ...)", can_index, can_id);


    // 注意：你的 CanTxFunc 形态来自 opensae_port.h（工程里已有）
    // 一般是 tx(can_index, can_frame) 或 tx(can_frame)
    // 下面按“bindChannel 传入的是可直接发送该帧”的约定调用：
    tx(fr);
}

} // namespace j1939

extern "C" void opensae_adapter_tx_dispatch(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    if (!j1939::g_mgr) return;
    j1939::g_mgr->sendRaw_(can_index, can_id, dlc, data);
}
