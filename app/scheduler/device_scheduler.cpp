// app/scheduler/device_scheduler.cpp

#include "device_scheduler.h"
#include "logger.h"
#include "../../services/parser/protocol_parser_thread.h"
#include "config_loader.h"
#include "../../services/protocol/can/pcu/proto_pcu.h"
#include <cstring>   // memset/memcpy
#include <linux/can.h> // CAN_EFF_FLAG

#include "hex_dump.h"




DeviceScheduler::DeviceScheduler() = default;
DeviceScheduler::~DeviceScheduler() { stop(); }

void DeviceScheduler::setParser(parser::ProtocolParserThread* p) {
    std::lock_guard<std::mutex> lk(mtx_);
    parser_ = p;
}

void DeviceScheduler::start() {
    if (running_.exchange(true)) return;

    {
        std::lock_guard<std::mutex> lk(mtx_);

        if (tasks_.empty()) {
            SystemConfig sys{};
            std::string err;

            const bool ok = ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, err);
            if (!ok) {
                LOGERR("[SCHED] load system.json failed: %s (fallback hardcode)", err.c_str());

                // ===== fallback: 原硬编码（不含 HMI）=====
                auto addTask = [this](dev::LinkType type, int idx,
                      const std::string& name,
                      uint32_t period_ms,
                      uint32_t disconnect_window_ms)
                {
                    PollTask t;
                    t.type = type;
                    t.index = idx;
                    t.device_name = name;
                    t.period_ms = period_ms;
                    t.next_due_ms = 0;
                    t.disconnect_window_ms = disconnect_window_ms;
                    t.enable = true;
                    tasks_.push_back(t);

                    DevicePollCtx ctx{};
                    ctx.disconnect_window_ms = disconnect_window_ms;
                    ctx.state = PollState::OFFLINE;
                    ctx.online = false;
                    poll_ctx_[t.device_name] = ctx;
                };

                // addTask(dev::LinkType::RS485, 0, "GasDetector",    200, 200, 1000, 300, 2);
                // addTask(dev::LinkType::RS485, 1, "SmokeSensor",    200, 200, 1000, 300, 2);
                // addTask(dev::LinkType::RS485, 2, "AirConditioner", 200, 200, 1200, 300, 2);
                // addTask(dev::LinkType::RS232, 0, "UPS",            200, 200, 1000, 300, 2);
                addTask(dev::LinkType::RS485, 0, "GasDetector",    200, 1500);
                addTask(dev::LinkType::RS485, 1, "SmokeSensor",    200, 1500);
                addTask(dev::LinkType::RS485, 2, "AirConditioner", 200, 1500);
                addTask(dev::LinkType::RS232, 0, "UPS",            200, 2000);

            } else {
                // ===== 从 system.json 生成任务 =====

                auto addTaskFromCfg = [this](dev::LinkType type,
                             int link_index,
                             const std::string& devName,
                             const PollCfg& poll)
                {
                    PollTask t;
                    t.type = type;
                    t.index = link_index;
                    t.device_name = devName;
                    t.period_ms = (poll.interval_ms == 0) ? 1000 : poll.interval_ms;
                    t.next_due_ms = 0;

                    const uint32_t req_timeout_ms = (poll.timeout_ms == 0) ? 300 : poll.timeout_ms;
                    const uint32_t base_period_ms = (poll.interval_ms == 0) ? 1000 : poll.interval_ms;
                    uint32_t disconnect_window_ms = poll.disconnect_window_ms;
                    if (disconnect_window_ms == 0) {
                        // 兜底策略：
                        // 1) 不小于 timeout_ms
                        // 2) 不小于 3 个周期
                        // 3) 至少 1000ms，避免慢设备抖动
                        disconnect_window_ms = std::max<uint32_t>(
                            req_timeout_ms,
                            std::max<uint32_t>(base_period_ms * 3, 1000)
                        );
                    }
                    t.period_ms = base_period_ms;
                    t.next_due_ms = 0;
                    t.disconnect_window_ms = disconnect_window_ms;
                    t.enable = poll.enable;

                    tasks_.push_back(t);

                    DevicePollCtx ctx{};
                    ctx.disconnect_window_ms = t.disconnect_window_ms;
                    ctx.state = PollState::OFFLINE;
                    ctx.online = false;
                    poll_ctx_[t.device_name] = ctx;
                };

                // ---- RS485 ----
                for (const auto& l : sys.rs485_links) {
                    // 只对 master_poll + enable 建任务；HMI (slave_hmi) 自动跳过
                    if (l.role != LinkRole::MasterPoll) continue;
                    if (!l.poll.enable) continue;

                    // 设备名：你可以继续沿用原有固定命名，方便 onDeviceTimeout / health summary
                    std::string devName;
                    switch (l.type) {
                        case Rs485ProtoType::Gas:            devName = "GasDetector";    break;
                        case Rs485ProtoType::Smoke:          devName = "SmokeSensor";    break;
                        case Rs485ProtoType::AirConditioner: devName = "AirConditioner"; break;
                        default:
                            devName = !l.name.empty() ? l.name : "RS485Device";
                            break;
                    }

                    addTaskFromCfg(dev::LinkType::RS485, l.link_index, devName, l.poll);
                }

                // ---- RS232 ----
                LOG_SYS_I("[SCHED][RS232] sys.rs232_links size=%zu", sys.rs232_links.size());
                for (const auto& l : sys.rs232_links) {
                    LOG_SYS_I("[SCHED][RS232] link_index=%d name=%s type=%s role=%d poll.enable=%d interval=%u timeout=%u",
                              l.link_index,
                              l.name.c_str(),
                              l.type.c_str(),
                              (int)l.role,
                              (int)l.poll.enable,
                              (unsigned)l.poll.interval_ms,
                              (unsigned)l.poll.timeout_ms);
                }

                // ---- RS232 ----
                for (const auto& l : sys.rs232_links) {
                    if (l.role != LinkRole::MasterPoll) {
                        LOG_SYS_I("[SCHED][RS232] skip by role=%d", (int)l.role);
                        continue;
                    }
                    if (!l.poll.enable) {
                        LOG_SYS_I("[SCHED][RS232] skip by poll.enable=0");
                        continue;
                    }

                    std::string devName = (l.type == "ups_ascii") ? "UPS"
                                       : (!l.name.empty() ? l.name : "RS232Device");

                    LOG_SYS_I("[SCHED][RS232] ADD before tasks=%zu dev=%s", tasks_.size(), devName.c_str());
                    addTaskFromCfg(dev::LinkType::RS232, l.link_index, devName, l.poll);
                    LOG_SYS_I("[SCHED][RS232] ADD after  tasks=%zu", tasks_.size());
                }

                // ---- CAN ----
                // system.json: sys.can_links（你需要在 ConfigLoader/SystemConfig 增加）
                // 这里先只支持协议 type="emu_pcu_v1"
                for (const auto& l : sys.can_links) {
                    if (!l.enable) continue;

                    // 先只做 can0 测试：你也可以通过配置 enable/disable 控制
                    // if (l.can_index != 0) continue;

                    if (l.protocol_type != "emu_pcu_v1") {
                        LOGERR("[SCHED][CAN] unknown protocol_type=%s on can_index=%d",
                               l.protocol_type.c_str(), l.can_index);
                        continue;
                    }

                    CanTask t;
                    t.can_index = l.can_index;
                    t.name = !l.name.empty() ? l.name : ("can" + std::to_string(l.can_index));
                    t.period_ms = (l.interval_ms == 0) ? 200 : l.interval_ms;
                    t.next_due_ms = 0;

                    t.id_emu_ctrl   = l.id_emu_ctrl;
                    t.id_emu_status = l.id_emu_status;
                    t.id_pcu_status = l.id_pcu_status;

                    t.send_ctrl   = l.send_ctrl;
                    t.send_status = l.send_status;

                    t.ctrl_enable_default = l.ctrl_enable_default;
                    t.plug_default        = l.plug_default;
                    t.estop_default       = l.estop_default;
                    t.batt1_estop_default = l.batt1_estop_default;
                    t.batt2_estop_default = l.batt2_estop_default;

                    can_tasks_.push_back(t);

                    LOG_SYS_I("[SCHED][CAN] add can_index=%d period=%u id_ctrl=0x%08X id_st=0x%08X",
                              t.can_index, t.period_ms, t.id_emu_ctrl, t.id_emu_status);
                }

                LOG_SYS_I("[SCHED] tasks loaded from system.json, count=%zu", tasks_.size());
            }
        }
    }

    // DeviceScheduler::start() 中初始化 bms_tx_
    {
        std::lock_guard<std::mutex> lk(mtx_);
        // BMS V2B_CMD 默认：can2，100ms
        bms_tx_.can_index = 2;
        bms_tx_.period_ms = 100;
        bms_tx_.id_v2b_cmd = 0x1802F3EF;
        bms_tx_.next_due_ms = 0;
        bms_tx_.life = 0;
        bms_tx_.hv_onoff = 0;
        bms_tx_.enable = 1;
    }


    timer_.addPeriodic(SCHED_TICK_MS, [this] {
        this->pollTick();
    });

    timer_.start();
    LOG_SYS_I("SCHED started tick=%ums", SCHED_TICK_MS);
}





void DeviceScheduler::stop() {
    if (!running_.exchange(false)) return;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_) return;
        running_ = false;
    }
    timer_.stop();
    LOG_SYS_I("SCHED stopped");
}

void DeviceScheduler::setOnDeviceData(std::function<void(const DeviceData&)> cb) {
    std::lock_guard<std::mutex> lk(mtx_);
    on_data_ = std::move(cb);
}

void DeviceScheduler::onDeviceData(const DeviceData& d) {
    std::function<void(const DeviceData&)> cb;
    std::function<void(const std::string&)> hcb;
    bool health_changed = false;

    {
        std::lock_guard<std::mutex> lk(mtx_);

        uint64_t now = SchedulerTimer::nowMs();
        auto& ctx = poll_ctx_[d.device_name];

        const bool was_online = ctx.online;

        ctx.last_ok_ms = now;
        ctx.state = PollState::ONLINE;
        ctx.online = true;

        if (auto* t = findTaskLocked(d.device_name)) {
            t->next_due_ms = now + t->period_ms;
        }

        cb = on_data_;
        hcb = on_health_changed_;
        health_changed = (!was_online && ctx.online);
    }

    if (cb) cb(d);
    if (health_changed && hcb) hcb(d.device_name);
}
void DeviceScheduler::setOnHealthChanged(std::function<void(const std::string& device_name)> cb) {
    std::lock_guard<std::mutex> lk(mtx_);
    on_health_changed_ = std::move(cb);
}

bool DeviceScheduler::allowPoll(const std::string& device_name) {
    (void)device_name;
    return true;
}


bool DeviceScheduler::getPollCtx(const std::string& device_name, DevicePollCtx& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = poll_ctx_.find(device_name);
    if (it == poll_ctx_.end()) return false;
    out = it->second;
    return true;
}

void DeviceScheduler::pollTick() {
    parser::ProtocolParserThread* parser_ptr = nullptr;

    struct LocalTask {
        dev::LinkType type;
        int index;
        std::string name;
        uint32_t period_ms;
        uint64_t next_due_ms;
        uint32_t disconnect_window_ms;
    };

    std::vector<LocalTask> due_list;
    std::vector<std::string> health_changed_devices;
    uint64_t now = SchedulerTimer::nowMs();

    {
        std::lock_guard<std::mutex> lk(mtx_);
        parser_ptr = parser_;
        if (!parser_ptr) return;

        // A. aging
        for (auto& kv : poll_ctx_) {
            const std::string& device_name = kv.first;
            auto& ctx = kv.second;

            const bool old_online = ctx.online;
            const bool new_online =
                (ctx.last_ok_ms != 0) &&
                (now >= ctx.last_ok_ms) &&
                ((now - ctx.last_ok_ms) <= ctx.disconnect_window_ms);
            if (old_online != new_online) {  // 20260409 检查online
                // LOG_COMM_D("[SCHED][AGING] dev=%s old=%d new=%d now=%llu last_ok=%llu win=%u off=%llu dc=%u",
                //            device_name.c_str(),
                //            old_online ? 1 : 0,
                //            new_online ? 1 : 0,
                //            (unsigned long long)now,
                //            (unsigned long long)ctx.last_ok_ms,
                //            (unsigned)ctx.disconnect_window_ms,
                //            (unsigned long long)ctx.last_offline_ms,
                //            (unsigned)ctx.disconnect_count);
                health_changed_devices.push_back(device_name);
            }
            ctx.online = new_online;
            ctx.state = new_online ? PollState::ONLINE : PollState::OFFLINE;

            if (old_online && !new_online) {
                ctx.last_offline_ms = now;
                ctx.disconnect_count += 1;
            }

            if (old_online != new_online) {
                health_changed_devices.push_back(device_name);
            }
        }

        // B. collect due tasks
        for (auto& t : tasks_) {
            if (!t.enable) continue;
            if (t.next_due_ms == 0) t.next_due_ms = now;
            if (now < t.next_due_ms) continue;

            due_list.push_back(LocalTask{
                t.type,
                t.index,
                t.device_name,
                t.period_ms,
                t.next_due_ms,
                t.disconnect_window_ms
            });

            t.next_due_ms = now + t.period_ms;

            auto& ctx = poll_ctx_[t.device_name];
            ctx.last_send_ms = now;
        }
    }

    if (!parser_ptr) return;

    // C. 每个 due task 只发一次
    for (const auto& lt : due_list) {
        (void)parser_ptr->sendPoll(
            lt.type,
            lt.index,
            lt.name,
            lt.disconnect_window_ms,
            parser::PollSendMode::Normal
        );
    }

    // D. health callback
    std::function<void(const std::string&)> hcb;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        hcb = on_health_changed_;
    }
    if (hcb) {
        for (const auto& name : health_changed_devices) {
            hcb(name);
        }
    }

        // ===== CAN periodic TX (emu_pcu_v1) =====
    {
        std::function<void(int, const can_frame&)> sendCan;
        std::vector<CanTask> local_tasks;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            sendCan = send_can_;
            local_tasks = can_tasks_;
        }

        if (sendCan) {
            const uint64_t now = SchedulerTimer::nowMs();
            bool changed = false;

            for (auto& t : local_tasks) {
                if (t.next_due_ms == 0) t.next_due_ms = now; // 启动即发
                if (now < t.next_due_ms) continue;

                // 组帧并发送
                if (t.send_ctrl) {
                    can_frame fr{};
                    proto::pcu::buildEmuCtrl(fr,
                                            t.id_emu_ctrl,
                                            t.hb,
                                            t.plug_default,
                                            t.estop_default,
                                            t.batt1_estop_default,
                                            t.batt2_estop_default,
                                            t.ctrl_enable_default);
                    sendCan(t.can_index, fr);
                    t.hb = uint8_t(t.hb + 1);
                }

                if (t.send_status) {
                    can_frame fr{};
                    // 这里先用默认 0；后续你可以从 snapshot/控制策略取值
                    proto::pcu::buildEmuStatus(fr, t.id_emu_status,
                                              /*batt1_kw_x10*/0,
                                              /*batt2_kw_x10*/0,
                                              /*batt1_branches*/2,
                                              /*batt2_branches*/2);
                    sendCan(t.can_index, fr);
                }

                t.next_due_ms = now + t.period_ms;
                changed = true;
            }

            if (changed) {
                std::lock_guard<std::mutex> lk(mtx_);
                can_tasks_ = std::move(local_tasks);
            }
        }
    }

    // ===== CAN periodic TX (bms_v2: V2B_CMD) =====
    {
        std::function<void(int, const can_frame&)> sendCan;
        BmsTxTask t{};
        bool enable = false;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            sendCan = send_can_;
            t = bms_tx_;
            enable = bms_tx_enable_;
        }

        if (enable && sendCan) {
            const uint64_t now = SchedulerTimer::nowMs();
            if (t.next_due_ms == 0) t.next_due_ms = now; // 启动即发
            if (now >= t.next_due_ms) {

                // 组帧：V2B_CMD (0x1802F3EF), DLC=8, 扩展帧
                can_frame fr{};
                fr.can_id  = CAN_EFF_FLAG | t.id_v2b_cmd;
                fr.can_dlc = 8;
                std::memset(fr.data, 0, sizeof(fr.data));

                // 最小实现：只发 LifeSignal（Byte0）
                fr.data[0] = t.life;
                t.life = uint8_t(t.life + 1);

                // 如果你要顺带发 HV_OnOff / enable（但你还没确认其 startbit/定义）
                // 可以先按“整字节字段”写在固定 byte 上；更推荐后续用 generated 表的 startbit 来 pack
                // fr.data[?] = t.hv_onoff;
                // fr.data[?] = t.enable;

                sendCan(t.can_index, fr);

                std::vector<uint8_t> v(fr.data, fr.data + fr.can_dlc);
                // LOG_THROTTLE_MS("bms_v2b_cmd_tx", 1000, LOG_SYS_I,
                //     "[BMS][TX] can%d id=0x%08X life=%u data=%s",
                //     t.can_index, t.id_v2b_cmd, (unsigned)t.life, hexDump(v));


                t.next_due_ms = now + t.period_ms;

                // 写回状态
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    bms_tx_.life = t.life;
                    bms_tx_.next_due_ms = t.next_due_ms;
                }
            }
        }
    }

}

PollTask* DeviceScheduler::findTaskLocked(const std::string& device_name) {
    for (auto& t : tasks_) {
        if (t.device_name == device_name) return &t;
    }
    return nullptr;
}

const PollTask* DeviceScheduler::findTaskLocked(const std::string& device_name) const {
    for (auto& t : tasks_) {
        if (t.device_name == device_name) return &t;
    }
    return nullptr;
}

void DeviceScheduler::setSendCan(std::function<void(int, const can_frame&)> cb) {
    std::lock_guard<std::mutex> lk(mtx_);
    send_can_ = std::move(cb);
}

void DeviceScheduler::setBmsTxEnabled(bool en) {
    std::lock_guard<std::mutex> lk(mtx_);
    bms_tx_enable_ = en;
}

bool DeviceScheduler::bmsTxEnabled() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bms_tx_enable_;
}



