// app/scheduler/device_scheduler.h

#pragma once

#include <map>
#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <mutex>

#include "protocol_base.h"
#include "../../services/device/device_base.h"
#include "scheduler_timer.h"

// 前置声明（避免 include 循环）
namespace parser { class ProtocolParserThread; }

enum class PollState {
    ONLINE,
    OFFLINE
};

struct DevicePollCtx {
    PollState state{PollState::OFFLINE};

    // 最近一次收到有效反馈的时刻
    uint64_t last_ok_ms{0};

    // 最近一次发送查询的时刻（仅调试/观察用）
    uint64_t last_send_ms{0};

    // 断连判断窗口：优先取 system.json 中 poll.disconnect_window_ms
    // 若未配置，则由调度器按 timeout_ms / interval_ms 推导兜底值
    uint32_t disconnect_window_ms{2000};

    // 最近一次从 ONLINE -> OFFLINE 的时刻
    uint64_t last_offline_ms{0};

    // 在线状态变化为离线的累计次数
    uint32_t disconnect_count{0};

    // 便于外部直接读取
    bool online{false};
};
// 一个“轮询任务”：Scheduler 用它来决定什么时候触发一次 sendPoll
// ✅ 每设备轮询任务：携带 timeout_ms（以及可选的重发策略）
struct PollTask {
    dev::LinkType type{dev::LinkType::RS485};
    int index{0};
    std::string device_name;

    // 固定发送周期
    uint32_t period_ms{1000};
    uint64_t next_due_ms{0};

    // 断连判断窗口：优先取 system.json 中 poll.disconnect_window_ms
    // 若未配置，则由调度器按 timeout_ms / interval_ms 推导兜底值
    uint32_t disconnect_window_ms{1000};

    bool enable{true};
};
// ===== BMS periodic TX (V2B_CMD) =====
struct BmsTxTask {
    int can_index{2};
    uint32_t period_ms{100};
    uint64_t next_due_ms{0};

    uint32_t id_v2b_cmd{0x1802F3EF}; // 29bit，不含 CAN_EFF_FLAG

    // LifeSignal rolling counter
    uint8_t life{0};

    // 可先只保留一个字段，后续再补更多控制位
    uint8_t hv_onoff{0};   // 0 Reserved / 1 PowerOn / 2 PowerOff / 3 Invalid（如果你后续要发）
    uint8_t enable{1};     // 预留默认一直发1（如协议需要）
};

static constexpr uint32_t SCHED_TICK_MS = 50;

class DeviceScheduler {
public:
    DeviceScheduler();
    ~DeviceScheduler();

    void start();
    void stop();

    // AppManager 注入 Parser（执行发送在 Parser）
    void setParser(parser::ProtocolParserThread* p);

    // Parser → AppManager → Scheduler（成功数据：用于恢复）
    void onDeviceData(const DeviceData& d);

    // Scheduler → AppManager（把设备数据回给上层/聚合器）
    void setOnDeviceData(std::function<void(const DeviceData&)> cb);

    void setOnHealthChanged(std::function<void(const std::string& device_name)> cb);

    // 轮询 gate：由 Scheduler 自己的状态决定
    bool allowPoll(const std::string& device_name);

    // 线程安全：返回拷贝（输出参数）
    bool getPollCtx(const std::string& device_name, DevicePollCtx& out) const;

    void setSendCan(std::function<void(int /*can_index*/, const can_frame&)> cb);

    // 轮询 gate：由 Scheduler 自己的状态决定
    void setBmsTxEnabled(bool en);
    bool bmsTxEnabled() const;


private:
    void pollTick(); // 定时触发轮询任务

    // 查找任务（必须在锁内调用）
    PollTask* findTaskLocked(const std::string& device_name);
    const PollTask* findTaskLocked(const std::string& device_name) const;

private:
    std::atomic<bool> running_{false};

    SchedulerTimer timer_;
    parser::ProtocolParserThread* parser_{nullptr};

    // ====== 被多个线程访问的共享状态 ======
    mutable std::mutex mtx_;
    std::vector<PollTask> tasks_;
    std::map<std::string, DevicePollCtx> poll_ctx_;

    // 回调（也会被多个线程读写，纳入同一把锁保护）
    std::function<void(const DeviceData&)> on_data_;


    // ===== HEALTH summary (every 5s, throttled) =====
    // PollState gas_st   = PollState::OFFLINE;
    // PollState smoke_st = PollState::OFFLINE;
    // PollState ac_st    = PollState::OFFLINE;
    // PollState ups_st   = PollState::OFFLINE;

    struct CanTask {
        int can_index{-1};
        std::string name;
        uint32_t period_ms{200};
        uint64_t next_due_ms{0};

        // 第一个协议三帧 ID（29bit，不含 CAN_EFF_FLAG）
        uint32_t id_emu_ctrl{0};
        uint32_t id_emu_status{0};
        uint32_t id_pcu_status{0}; // 目前只用来过滤 RX，可选

        // 发送使能（方便先只发 ctrl 测试）
        bool send_ctrl{true};
        bool send_status{true};

        // 控制帧字段默认值
        uint8_t ctrl_enable_default{1};
        uint8_t plug_default{0};
        uint8_t estop_default{0};
        uint8_t batt1_estop_default{0};
        uint8_t batt2_estop_default{0};

        // 心跳
        uint8_t hb{0};
    };

    std::vector<CanTask> can_tasks_;
    std::function<void(int, const can_frame&)> send_can_;

    // 给 DeviceScheduler 增加一组BMS 定时发送任务的状态
    BmsTxTask bms_tx_;
    bool bms_tx_enable_{false};  // 交给 control/bms 命令管理器，默认关闭 scheduler 的旧 BMS TX
    std::function<void(const std::string&)> on_health_changed_;
};
