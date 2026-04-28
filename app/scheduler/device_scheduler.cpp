// app/scheduler/device_scheduler.cpp

#include "device_scheduler.h"
#include "logger.h"
#include "../../services/parser/protocol_parser_thread.h"
#include "config_loader.h"





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

                addTask(dev::LinkType::RS485, 3, "GasDetector",    200, 1500);
                addTask(dev::LinkType::RS485, 2, "SmokeSensor",    200, 1500);
                addTask(dev::LinkType::RS485, 1, "AirConditioner", 200, 1500);
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

                // PCU 业务帧发送迁移到 ControlLoop / LogicEngine。
                //
                // Scheduler 不再发送：
                //   0x1801A0E0 EMU->PCU 控制帧
                //   0x1802A0E0 EMU->PCU 状态帧
                //
                // 原因：
                //   Scheduler 只能做固定周期调度，无法可靠读取 IO / BMS / Business 真源；
                //   PCU 发送内容应由控制面统一生成，并通过 CommandDispatcher 串行下发。
                for (const auto& l : sys.can_links) {
                    if (!l.enable) continue;

                    if (l.protocol_type == "emu_pcu_v1") {
                        LOG_SYS_I("[SCHED][CAN] skip PCU TX can_index=%d name=%s: moved to ControlLoop",
                                  l.can_index,
                                  l.name.c_str());
                        continue;
                    }

                    // BMS 也不走 Scheduler 旧最小发送器。
                    if (l.protocol_type == "bms_table_v1") {
                        LOG_SYS_I("[SCHED][CAN] skip BMS TX can_index=%d name=%s: managed by control/bms",
                                  l.can_index,
                                  l.name.c_str());
                        continue;
                    }

                    LOG_SYS_W("[SCHED][CAN] ignored protocol_type=%s can_index=%d",
                              l.protocol_type.c_str(),
                              l.can_index);
                }

                LOG_SYS_I("[SCHED] tasks loaded from system.json, count=%zu", tasks_.size());
            }
        }
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
    //
    // PCU TX 已迁移到 ControlLoop / LogicEngine。
    // 这里故意不再发送 PCU 控制帧 / 状态帧，避免双发送。
    //
    // PCU 发送新链路：
    //   LogicEngine::emitPcuPeriodicCommands_()
    //      -> Command::Type::SendCan
    //      -> CommandDispatcher
    //      -> DriverManager::sendCan()

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

// void DeviceScheduler::setSendCan(std::function<void(int, const can_frame&)> cb) {
//     std::lock_guard<std::mutex> lk(mtx_);
//     send_can_ = std::move(cb);
// }



