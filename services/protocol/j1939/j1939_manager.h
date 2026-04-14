//
// Created by lxy on 2026/2/8.
//

#ifndef ENERGYSTORAGE_J1939_MANAGER_H
#define ENERGYSTORAGE_J1939_MANAGER_H

// services/protocol/j1939/j1939_manager.h
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <linux/can.h>

#include "j1939_stack.h"
#include "opensae_port.h"

namespace j1939 {

    /*
     * J1939Manager：管理多路 CAN（can0/can1/can2）
     *
     * - bindChannel(can_index, tx_func): 绑定某路 CAN 的发送函数，并创建 stack
     * - onCanFrame(): CanThread RX 入口，推入 OpenSAE adapter RX 队列 + 交给 stack（可选）
     * - start/stop: pump 线程，轮询 stacks 调 tick（内部会设置 active_can_index 并调用 OpenSAE Listen）
     *
     * 目前阶段：不解析协议，仅验证收发闭环。
     */
    class J1939Manager {
    public:
        J1939Manager();
        ~J1939Manager();

        /** 注册/绑定某路 CAN：创建 stack + 保存 tx */
        void bindChannel(int can_index, CanTxFunc tx);

        /** 解绑某路 CAN：移除 stack + tx */
        void unbindChannel(int can_index);

        /** CanThread RX 统一入口 */
        void onCanFrame(int can_index, const can_frame& fr);

        /** 启动/停止 pump（建议在 AppManager::start/stop 调用） */
        void start();
        void stop();

        /** 手动 tick（如果你不想开线程，也可以外部周期调用） */
        void tickOnce();

        /** 获取某路 stack（可能为空） */
        std::shared_ptr<J1939Stack> getStack(int can_index);

        void sendRaw_(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data);
    private:
        void threadMain_();

    private:
        std::mutex mtx_;
        std::unordered_map<int, std::shared_ptr<J1939Stack>> stacks_;
        std::unordered_map<int, CanTxFunc> tx_map_;

        std::atomic<bool> running_{false};
        std::thread th_;

        // 轮询 tick 时的通道集合（避免每次复制 map）
        std::unordered_set<int> channels_;
    };

} // namespace j1939

/*
 * 供 opensae_adapter.c 调用的 C 接口：真正把 OpenSAE 发送的数据发到对应 can_index
 * 注意：OpenSAE 的 CAN_Send_Message 没带通道，所以 adapter 会用 active_can_index 作为 can_index。
 */
extern "C" void opensae_adapter_tx_dispatch(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data);


#endif //ENERGYSTORAGE_J1939_MANAGER_H