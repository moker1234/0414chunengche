// services/control/control_loop.h
//
// 工业级控制：单线程控制环（事件队列 -> 逻辑 -> 命令下发）
// Created by ChatGPT on 2026/02/24.
//
#ifndef ENERGYSTORAGE_CONTROL_LOOP_H
#define ENERGYSTORAGE_CONTROL_LOOP_H

#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <mutex>

#include "../utils/queue/msg_queue.h"
#include "../utils/time/getTime.h"

#include "control_events.h"
#include "control_commands.h"
#include "./logic/logic_context.h"
#include "./logic/logic_engine.h"
#include "command_dispatcher.h"

class DriverManager;
namespace parser { class ProtocolParserThread; }
class HMIProto;
class SqliteFaultSink;
struct FaultHistoryDbRecord;

namespace control {

    class ControlLoop {
    public:
        ControlLoop(::DriverManager& drv, parser::ProtocolParserThread& parser);
        ~ControlLoop();

        void start();
        void stop();

        void post(const Event& e);
        void post(Event&& e);

        void bindHmi(HMIProto* hmi);

        bool loadNormalMapFile(const std::string& path, std::string* err = nullptr);
        bool loadFaultMapFile(const std::string& path, std::string* err = nullptr);

        void bindFaultDb(SqliteFaultSink* db);
        void restoreFaultHistory(const std::vector<FaultHistoryDbRecord>& rows);

        void refreshFromLatestSnapshot(const agg::SystemSnapshot& snap, uint64_t ts_ms);

        void refreshFaultPagesOnly(uint64_t ts_ms);
    private:
        void threadMain_();

        // Snapshot 去积压：只保留最新一份
        void postSnapshotLatest_(const Event& e);
        void postSnapshotLatest_(Event&& e);
        bool takeLatestSnapshot_(Event& out);

    private:
        std::atomic<bool> running_{false};
        std::thread th_;
        MsgQueue<Event> q_;

        // ===== Snapshot latest-only buffer =====
        mutable std::mutex snapshot_mtx_;
        Event latest_snapshot_event_{};
        bool has_latest_snapshot_{false};
        bool snapshot_marker_enqueued_{false};

        LogicContext ctx_;
        LogicEngine engine_;
        CommandDispatcher dispatcher_;
        mutable std::mutex logic_mtx_;
    };

} // namespace control

#endif // ENERGYSTORAGE_CONTROL_LOOP_H