// app_manager.h
#pragma once
#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>
#include <thread>
#include <atomic>

#include "event.h"
#include "scheduler/device_scheduler.h"
#include "state_machine/state_base.h"

#include "../services/parser/protocol_parser_thread.h"
#include "../services/snapshot/snapshot_dispatcher.h"

#include "../services/protocol/protocol_base.h"
#include "../services/protocol/can/bms/bms_thread/bms_queue.h"
#include "../services/protocol/can/bms/bms_thread/bms_worker.h"
#include "../services/protocol/can/bms/bms_proto.h"

#include "../services/control/control_loop.h"

namespace agg
{
    class DataAggregator;
}

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

private:
    bool loadBmsCanIndexFromSystem_(int& out_index);
    bool loadHmiRs485IndexFromSystem_(int& out_index);
    void onControlTxMirrorDeviceData_(const DeviceData& d);

private:
    bool running_{false};

    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Event> q_;

    std::unique_ptr<StateBase> state_;
    std::unique_ptr<DeviceScheduler> scheduler_;
    std::unique_ptr<parser::ProtocolParserThread> parser_;
    std::unique_ptr<DriverManager> driver_manager_;


    std::unique_ptr<SnapshotDispatcher> snapshot_dispatcher_;
    std::unique_ptr<agg::DataAggregator> aggregator_;


    // ===== BMS async pipeline =====
    proto::bms::BmsQueue bms_queue_;
    std::unique_ptr<proto::bms::BmsWorker> bms_worker_;
    int bms_can_index_{0}; // 当前 BMS 走 can2，后续也可从 system.json 读

    proto::bms::BmsProto bms_proto_{"BMS"};

    // ===== HMI RS485 index =====
    int hmi_rs485_index_{0};


    std::unique_ptr<control::ControlLoop> control_;

    std::unique_ptr<SchedulerTimer> app_tick_timer_;

    std::unique_ptr<SqliteFaultSink> fault_db_;


    // ===== 故障页周期刷新线程 =====
    std::thread fault_refresh_thread_;
    std::atomic<bool> fault_refresh_running_{false};
    void faultRefreshThreadMain_();


    // ===== BMS runtime health 低频回写线程 =====
    // 注意：不在 ControlLoop 线程中回写 Aggregator，避免卡住 HMI / Fault 翻页。
    std::thread bms_health_sync_thread_;
    std::atomic<bool> bms_health_sync_running_{false};
    void bmsHealthSyncThreadMain_();
};
