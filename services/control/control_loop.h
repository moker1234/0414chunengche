// services/control/control_loop.h
//
// 工业级控制：单线程控制环（事件队列 -> 逻辑 -> 命令下发）
// Created by lxy on 2026/02/24.
//
#ifndef ENERGYSTORAGE_CONTROL_LOOP_H
#define ENERGYSTORAGE_CONTROL_LOOP_H

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <functional>

#include "../utils/queue/msg_queue.h"
#include "../utils/time/getTime.h"
#include "../aggregator/data_aggregator.h"

#include "control_events.h"
#include "control_commands.h"
#include "./logic/logic_context.h"
#include "./logic/logic_engine.h"
#include "command_dispatcher.h"
#include "./maintain/maintain_service.h"

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

        // 控制面 TX 镜像回写入口。
        // 典型用途：PCU TX 镜像 -> AppManager -> Aggregator -> SnapshotDispatcher
        void setTxMirrorCallback(std::function<void(const DeviceData&)> cb);

        // 新统一入口：
        // normal_map_logic.jsonl 只在这里解析一次，
        // 然后同一份 HmiMapModel 同时供 NormalHmiWriter / FaultPageManager 使用。
        bool loadHmiMapFile(const std::string& path, std::string* err = nullptr);

        bool loadFaultMapFile(const std::string& path, std::string* err = nullptr);

        void bindFaultDb(SqliteFaultSink* db);
        void restoreFaultHistory(const std::vector<FaultHistoryDbRecord>& rows);

        void refreshFaultPagesOnly(uint64_t ts_ms);

        // BMS runtime health 只读快照。
        // AppManager 的独立低频线程读取它，再回写 Aggregator。
        // 注意：这里不调用 Aggregator，不做落盘，不做 HMI 刷新。
        std::vector<agg::BmsRuntimeHealthUpdate> latestBmsRuntimeHealth() const;
    private:
        void threadMain_();

        void refreshCachedBmsRuntimeHealth_();

        // Snapshot 去积压：只保留最新一份
        void postSnapshotLatest_(const Event& e);
        void postSnapshotLatest_(Event&& e);
        bool takeLatestSnapshot_(Event& out);

    private:
        std::atomic<bool> running_{false};
        std::thread th_;

        // ===== ControlLoop 事件队列 =====
        //
        // HMI 写入事件必须优先处理，否则 Tick/Snapshot/DeviceData 堆积时，
        // 屏幕按键释放、翻页、反馈会出现明显延迟。
        mutable std::mutex q_mtx_;
        std::condition_variable q_cv_;

        std::deque<Event> q_high_;    // HmiWrite / HmiHeartbeat / Stop
        std::deque<Event> q_normal_;  // DeviceData / Snapshot / IoSample / Tick

        static bool isHighPriorityEvent_(const Event& e);

        void pushEvent_(const Event& e);
        void pushEvent_(Event&& e);
        bool popEvent_(Event& out);

        // ===== Snapshot latest-only buffer =====
        mutable std::mutex snapshot_mtx_;
        Event latest_snapshot_event_{};
        bool has_latest_snapshot_{false};
        bool snapshot_marker_enqueued_{false};

        LogicContext ctx_;
        LogicEngine engine_;
        CommandDispatcher dispatcher_;

        maintain::MaintainService maintain_;

        mutable std::mutex logic_mtx_;

        std::function<void(const DeviceData&)> tx_mirror_cb_;

        // 只用于跨线程导出 BMS runtime health。
        // 写入：ControlLoop 线程内部，在 engine_.onEvent() 后复制 ctx_.bms_cache 的轻量结果。
        // 读取：AppManager 独立低频同步线程。
        mutable std::mutex bms_health_mtx_;
        std::vector<agg::BmsRuntimeHealthUpdate> latest_bms_health_;
    };

} // namespace control

#endif // ENERGYSTORAGE_CONTROL_LOOP_H