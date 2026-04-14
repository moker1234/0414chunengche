// services/control/command_dispatcher.h
//
// 工业级控制：命令下发器（所有发送统一串行化）
// Created by ChatGPT on 2026/02/24.
//

#ifndef ENERGYSTORAGE_COMMAND_DISPATCHER_H
#define ENERGYSTORAGE_COMMAND_DISPATCHER_H

#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

#include "control_commands.h"

class DriverManager;

namespace parser {
    class ProtocolParserThread;
}

namespace control {

    /**
     * @brief CommandDispatcher：把 Command 发送到正确的通道
     *
     * 依赖：
     * - DriverManager：CAN send / 串口 raw send
     * - ProtocolParserThread：sendPoll（复用 pending/timeout/retry）
     *
     * 工业级做法：
     * - 在 ControlLoop 单线程调用 dispatch()，Dispatcher 内部尽量少锁
     * - 对“重复写同一 DO / 同一 HMI RW 地址”等可做合并（这里先留骨架）
     */
    class CommandDispatcher {
    public:
        CommandDispatcher(DriverManager& drv, parser::ProtocolParserThread& parser);
        ~CommandDispatcher() = default;

        void dispatch(const std::vector<Command>& cmds);

        // 可选：提供统计/观测接口
        uint64_t lastDispatchTs() const { return last_dispatch_ts_; }

    private:
        void dispatchOne_(const Command& c);

    private:
        DriverManager& drv_;
        parser::ProtocolParserThread& parser_;

        uint64_t last_dispatch_ts_{0};

        // 合并缓存（可选增强）：避免同一周期重复写同一个 DO
        std::unordered_map<int, bool> last_do_cache_;
    };

} // namespace control

#endif // ENERGYSTORAGE_COMMAND_DISPATCHER_H