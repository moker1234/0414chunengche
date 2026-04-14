// app_manager.h
#pragma once
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>

#include "event.h"
#include "scheduler/device_scheduler.h"
#include "state_machine/state_base.h"

#include "../services/parser/protocol_parser_thread.h"
#include "../services/snapshot/snapshot_dispatcher.h"
#include "j1939/j1939_manager.h"
#include "../services/protocol/can/bms/bms_thread/bms_queue.h"
#include "../services/protocol/can/bms/bms_thread/bms_worker.h"
#include "../services/protocol/can/bms/bms_proto.h"

#include "../services/control/control_loop.h"

namespace agg
{
    class DataAggregator;
}

class DisplayRs485Session;
class DriverManager;   // 前置声明
class SqliteFaultSink;

class AppManager {
public:
    AppManager();
    ~AppManager();     // 非 inline

    bool init();
    void start();
    void stop();

    void post(const Event& e);
    void pumpOnce();

    void transitionTo(std::unique_ptr<StateBase> next);

    DriverManager& getDriverManager();   // 不再 inline

    void onParsed(const parser::ParserMessage& msg);

    bool sendRs485(int index, const std::vector<uint8_t>& bytes);

    DriverManager& driver() { return *driver_manager_; }

    DeviceScheduler& scheduler() { return *scheduler_; }
    const DeviceScheduler& scheduler() const { return *scheduler_; }

    agg::DataAggregator& aggregator() { return *aggregator_; }
    const agg::DataAggregator& aggregator() const { return *aggregator_; }

    void postSnapshotToControl_(
        uint64_t ts_ms);

private:
    void onLogicDeviceData(const DeviceData& d);

private:
    bool running_{false};

    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Event> q_;

    std::unique_ptr<StateBase> state_;
    std::unique_ptr<DeviceScheduler> scheduler_;
    std::unique_ptr<parser::ProtocolParserThread> parser_;
    std::unique_ptr<DriverManager> driver_manager_;

    // ❌ 不再暴露 DriverManager 的存储方式
    // DriverManager* driver_mgr_{nullptr};   // 仅声明指针
    std::unique_ptr<DriverManager> driver_mgr_;


    std::unique_ptr<SnapshotDispatcher> snapshot_dispatcher_;
    std::unique_ptr<agg::DataAggregator> aggregator_;
    std::unique_ptr<DisplayRs485Session> display_session_;


    std::unique_ptr<j1939::J1939Manager> j1939_mgr_;

    // ===== BMS async pipeline =====
    proto::bms::BmsQueue bms_queue_;
    std::unique_ptr<proto::bms::BmsWorker> bms_worker_;
    int bms_can_index_{2}; // 当前 BMS 走 can2，后续也可从 system.json 读

    proto::bms::BmsProto bms_proto_{"BMS"};


    std::unique_ptr<control::ControlLoop> control_;

    std::unique_ptr<SchedulerTimer> app_tick_timer_;

    std::unique_ptr<SqliteFaultSink> fault_db_;


    // ===== latest snapshot 100ms 刷新线程 =====
    std::thread logic_refresh_thread_;
    std::atomic<bool> logic_refresh_running_{false};
    void logicRefreshThreadMain_();

    std::thread fault_refresh_thread_;
    std::atomic<bool> fault_refresh_running_{false};
    void faultRefreshThreadMain_();
};
