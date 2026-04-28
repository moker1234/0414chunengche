// app/app_manager.cpp

#include <thread>
#include <atomic>

#include "app_manager.h"

#include "driver_manager.h"          // ✅ 必须在这里 include
#include "../services/aggregator/data_aggregator.h"
#include "state_machine/state_idle.h"
#include "logger.h"
#include <chrono>
#include <cstring>

#include "file_sink.h"
#include "getTime.h"
#include "uplink_sink.h"

#include "../services/snapshot/bms_filesink/bms_file_sink.h"

// 为了 HmiWriteEvent / HMIProto 可见
#include "sqlite_bms_flat_sink.h"
#include "sqlite_snapshot_flat_sink.h"
#include "../services/snapshot/sqlite/sqlite_bms_sink/sqlite_bms_sink.h"
#include "../services/protocol/rs485/hmi/hmi_proto.h"
#include "sqlite/sqlite_snapshot_sink/sqlite_snapshot_sink.h"
#include "../services/snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"

// 本地归档 sqlite
#include "config_loader.h"

// ✅ deviceNameFromLink() 删除：命名统一由 ParserMessage.device_name 提供

static void mergeSchedulerHealthToAggregator(
        AppManager* self,
        const std::string& device_name) {

    if (!self) return;

    const DeviceScheduler& sched = self->scheduler();

    DevicePollCtx ctx{};
    if (!sched.getPollCtx(device_name, ctx)) return;

    self->aggregator().updateHealthFromScheduler(
        device_name,
        ctx.online,
        ctx.last_ok_ms,
        ctx.disconnect_window_ms,
        ctx.last_offline_ms,
        ctx.disconnect_count
    );
}
bool AppManager::loadBmsCanIndexFromSystem_(int& out_index)
{
    SystemConfig sys{};
    std::string err;
    if (!ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, err)) {
        LOG_SYS_W("[APP][BMS] load system.json failed: %s", err.c_str());
        return false;
    }

    for (const auto& l : sys.can_links) {
        if (!l.enable) continue;
        if (l.protocol_type == "bms_table_v1") {
            out_index = l.can_index;
            return true;
        }
    }

    LOG_SYS_W("[APP][BMS] no can link with protocol=bms_table_v1 found");
    return false;
}
bool AppManager::loadHmiRs485IndexFromSystem_(int& out_index)
{
    SystemConfig sys{};
    std::string err;
    if (!ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, err)) {
        LOG_SYS_W("[APP][HMI] load system.json failed: %s", err.c_str());
        return false;
    }

    for (const auto& l : sys.rs485_links) {
        if (l.role == LinkRole::SlaveHmi) {
            out_index = l.link_index;
            return true;
        }
    }

    LOG_SYS_W("[APP][HMI] no rs485 link with role=slave_hmi found");
    return false;
}

AppManager::AppManager() = default;

AppManager::~AppManager() {

}

bool AppManager::init() {
    scheduler_ = std::make_unique<DeviceScheduler>();

    /* =====================================================
     * 1. Parser
     * ===================================================== */
    parser_ = std::make_unique<parser::ProtocolParserThread>();

    parser_->setOnParsed(
        [this](const parser::ParserMessage& msg) {
            this->onParsed(msg);
        }
    );

    // 先从 system.json 解析 BMS 所在 can_index。
    // BMS 不走普通 CAN 同步解析链路，而是：
    //   CanDispatcher -> BmsQueue -> BmsWorker -> Aggregator/Control
    {
        int cfg_bms_can = bms_can_index_;
        if (loadBmsCanIndexFromSystem_(cfg_bms_can)) {
            bms_can_index_ = cfg_bms_can;
            LOG_SYS_I("[APP][BMS] use bms_can_index=%d from system.json", bms_can_index_);
        } else {
            LOG_SYS_W("[APP][BMS] load bms_can_index from system.json failed, fallback=%d",
                      bms_can_index_);
        }

        parser_->setBmsQueue(bms_can_index_, &bms_queue_);
        LOG_SYS_I("[APP][BMS] bind parser BmsQueue can_index=%d q=%p",
                  bms_can_index_, static_cast<void*>(&bms_queue_));
    }

    // Parser → Driver（发送字节）
      parser_->setSendSerial(
        [this](dev::LinkType t, int idx, const std::vector<uint8_t>& bytes) {
            if (driver_manager_) {
                driver_manager_->sendSerial(t, idx, bytes);
            }
        }
    );

    // ✅ Scheduler 注入 Parser：轮询节奏/决策在 Scheduler，执行在 Parser::sendPoll()
    scheduler_->setParser(parser_.get());

    /* =====================================================
     * 2. DriverManager（从 io_map.json 读取 can0/can1/can2 并启动线程）
     * ===================================================== */
    driver_manager_ = std::make_unique<DriverManager>(*this, *parser_);
    driver_manager_->init();

    // ✅ Scheduler → Driver（发送 CAN）
    // scheduler_->setSendCan(
    //     [this](int can_index, const can_frame& fr) {
    //         if (driver_manager_) {
    //             driver_manager_->sendCan(can_index, fr);
    //         }
    //     }
    // );

    /* =====================================================
     * 2.5 ControlLoop（控制面）
     * ===================================================== */
    if (driver_manager_ && parser_) {
        control_ = std::make_unique<control::ControlLoop>(*driver_manager_, *parser_);
    }    // ✅ 本地 IO（DI/AI）→ ControlLoop

    // - di_bits: bit0->DI1, bit1->DI2, ...
    // - ai: 第2批先只承载 ADC 电压，顺序为 [ADC1_V, ADC2_V, ...]
    if (driver_manager_) {
        driver_manager_->setIoSampleCallback(
            [this](uint64_t ts_ms, uint64_t di_bits, const std::vector<double>& ai_voltages) {
                if (!control_) return;

                control::Event e;
                e.type = control::Event::Type::IoSample;
                e.ts_ms = ts_ms;
                e.io_sample.ts_ms = ts_ms;
                e.io_sample.di_bits = di_bits;
                e.io_sample.ai = ai_voltages;

                control_->post(std::move(e));
            }
        );
    }

    // BMS V2B_CMD 改由 control/bms 命令管理器负责，关闭 scheduler 里的旧最小发送器


    /* =====================================================
     * 3. Aggregator + Dispatcher（解耦后的正确结构）
     * ===================================================== */
    aggregator_ = std::make_unique<agg::DataAggregator>();
    snapshot_dispatcher_ = std::make_unique<SnapshotDispatcher>();

    /*
 * 第七批：ControlLoop -> AppManager 的 TX 镜像回写。
 *
 * PCU TX 镜像不是 Parser RX，不走 scheduler_->onDeviceData()；
 * 它只进入 Aggregator，形成 items.PCU_x.data.ctrl，
 * 然后通过 SnapshotDispatcher 进入 FileSink / SQLite / UplinkSink。
 */
    if (control_) {
        control_->setTxMirrorCallback(
            [this](const DeviceData& d) {
                this->onControlTxMirrorDeviceData_(d);
            }
        );
    }

    scheduler_->setOnDeviceData(
        [this](const DeviceData& d) {
            LOGT("[APP] DeviceData name=%s", d.device_name.c_str());
            aggregator_->onDeviceData(d);
        }
    );
    scheduler_->setOnHealthChanged(
        [this](const std::string& device_name) {
            DevicePollCtx ctx{};
            bool ok = scheduler_->getPollCtx(device_name, ctx);

            // LOG_COMM_D("[APP][HEALTH_CHANGED] dev=%s ok=%d online=%d last_ok=%llu off=%llu dc=%u win=%u",
            //            device_name.c_str(),
            //            ok ? 1 : 0,
            //            (ok && ctx.online) ? 1 : 0,
            //            ok ? (unsigned long long)ctx.last_ok_ms : 0ULL,
            //            ok ? (unsigned long long)ctx.last_offline_ms : 0ULL,
            //            ok ? (unsigned)ctx.disconnect_count : 0u,
            //            ok ? (unsigned)ctx.disconnect_window_ms : 0u);

            mergeSchedulerHealthToAggregator(this, device_name);

            auto snap = aggregator_->snapshot();
            snapshot_dispatcher_->dispatch(snap);

            // 健康状态变化也要进入控制面，
            // 这样就不再需要 logic_refresh_thread 每100ms兜底投递 snapshot。
            if (control_) {
                control::Event es;
                es.type = control::Event::Type::Snapshot;
                es.ts_ms = nowMs();
                es.snapshot.ts_ms = es.ts_ms;
                es.snapshot.snap = std::move(snap);
                control_->post(std::move(es));
            }
        }
    );
    /* =====================================================
     * 4. HMI RS485 索引
     * ===================================================== */
    {
        int cfg_hmi_rs485_index = hmi_rs485_index_;
        if (loadHmiRs485IndexFromSystem_(cfg_hmi_rs485_index)) {
            hmi_rs485_index_ = cfg_hmi_rs485_index;
            LOG_SYS_I("[APP][HMI] use hmi_rs485_index=%d from system.json", hmi_rs485_index_);
        } else {
            LOG_SYS_W("[APP][HMI] load hmi_rs485_index from system.json failed, fallback=%d", hmi_rs485_index_);
        }
    }

    /* =====================================================
     * 5. 注册 Snapshot 持久化 / 上行 Sinks
     *    - 不再包含 HMI 显示映射
     * ===================================================== */

    snapshot_dispatcher_->addSink(std::make_unique<FileSink>());
    // tfcard sqlite 全数据
    // { // 注册 SqliteSnapshotSink,0317
    //     SqliteSnapshotSink::Config cfg;
    //     cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
    //     cfg.busy_timeout_ms = 3000;
    //     cfg.only_when_changed = true;
    //     cfg.min_interval_ms = 0;
    //     snapshot_dispatcher_->addSink(std::make_unique<SqliteSnapshotSink>(cfg));
    // }
    // tfcard sqlite 平摊存储 全数据
    {
        SqliteSnapshotFlatSink::Config cfg;
        cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
        cfg.busy_timeout_ms = 3000;
        cfg.write_main_every_snapshot = true;
        snapshot_dispatcher_->addSink(std::make_unique<SqliteSnapshotFlatSink>(cfg));
    }
    snapshot_dispatcher_->addSink(std::make_unique<UplinkSink>());

    // 本地 jsonl BMS
    {
        BmsFileSink::Config cfg;
        cfg.base_dir = "/home/zlg/running_log/bms";
        cfg.latest_interval_ms = 1000;
        cfg.history_interval_ms = 1000;
        cfg.history_include_summary = false;
        cfg.json_indent = 0;
        snapshot_dispatcher_->addBmsSink(std::make_unique<BmsFileSink>(cfg));
    }    // BMS 历史并行写 SQLite
    // tfcard sqlite BMS
    // {
    //     SqliteBmsSink::Config cfg;
    //     cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
    //     cfg.busy_timeout_ms = 3000;
    //     cfg.history_interval_ms = 1000;
    //     snapshot_dispatcher_->addBmsSink(std::make_unique<SqliteBmsSink>(cfg));
    // }
    // tfcard sqlite 平摊存储 BMS
    {
        SqliteBmsFlatSink::Config cfg;
        cfg.db_path = "/mnt/sqlite_tfcard/bms_data.db";
        cfg.busy_timeout_ms = 3000;
        cfg.write_main_every_snapshot = true;
        snapshot_dispatcher_->addBmsSink(std::make_unique<SqliteBmsFlatSink>(cfg));
    }


    /*
     * 6.1 加载 HMI 映射模型
     *
     * 第五批关键变化：
     * - normal_map_logic.jsonl 只在这里加载一次
     * - ControlLoop::loadHmiMapFile() 内部会：
     *     1) HmiMapLoader::loadJsonl()
     *     2) NormalHmiWriter::bindMap()
     *     3) FaultPageManager::buildLayoutFromHmiMap()
     *
     * 因此后面不再调用 loadNormalMapFile()。
     */
    if (control_) {
        std::string err;
        if (!control_->loadHmiMapFile("/home/zlg/userdata/config/normal_map_logic.jsonl", &err)) {
            LOG_SYS_W("[HMI][MAP] load normal_map_logic.jsonl failed: %s", err.c_str());
        } else {
            LOG_SYS_I("[HMI][MAP] normal_map_logic.jsonl loaded");
        }
    }

    /*
     * 6.2 加载 fault_map.jsonl
     *
     * 第五批后：
     * - fault_map.jsonl 只负责 FaultCatalog + FaultRuntimeMapper
     * - 故障页 HMI 地址布局已经由上面的 loadHmiMapFile() 建立
     */
    if (control_) {
        std::string err;
        if (!control_->loadFaultMapFile("/home/zlg/userdata/config/fault_map.jsonl", &err)) {
            LOG_SYS_W("[FAULT] load fault_map.jsonl failed: %s", err.c_str());
        } else {
            LOG_SYS_I("[FAULT] fault_map.jsonl loaded");
        }
    }

    /*
     * 6.3 打开故障数据库，并恢复历史故障
     *
     * 这里放在 loadHmiMapFile() 和 loadFaultMapFile() 之后：
     * - FaultCatalog 已经可用于 code/name/level 等目录信息；
     * - FaultPageManager 已经知道历史页每页行数；
     * - restoreFaultHistory() 后历史页缓存可以按正确 page_size 工作。
     */
    if (control_) {
        fault_db_ = std::make_unique<SqliteFaultSink>(
            SqliteFaultSink::Config{
                "/mnt/sqlite_tfcard/json_data.db",
                3000,
                1000
            }
        );

        if (!fault_db_->open()) {
            LOG_SYS_W("[FAULT][SQLITE] open failed");
        } else {
            LOG_SYS_I("[FAULT][SQLITE] opened");

            control_->bindFaultDb(fault_db_.get());

            std::vector<FaultHistoryDbRecord> rows;
            if (fault_db_->loadRecentHistory(rows)) {
                control_->restoreFaultHistory(rows);
                LOG_SYS_I("[FAULT][SQLITE] restored history rows=%zu", rows.size());
            } else {
                LOG_SYS_W("[FAULT][SQLITE] loadRecentHistory failed");
            }
        }
    }

    /*
     * 6.4 绑定控制面 HMI
     *
     * 注意：
     * - HMI 映射文件不在这里加载；
     * - 这里只拿 HMIProto 指针、设置写入回调、bindHmi；
     * - 普通变量输出和故障页输出都已经通过 ControlLoop 内部共享 HmiMapModel。
     */
    if (parser_) {
        auto* hmi = parser_->getHmiProto(hmi_rs485_index_);
        LOGD("[APP][HMI] getHmiProto(%d)=%p", hmi_rs485_index_, (void*)hmi);

        if (hmi) {
            // ✅ HMI 写入（FC05/06/0F/10）进入 ControlLoop（写入是控制入口）
            hmi->setOnWrite([this, hmi](const HmiWriteEvent& ev) {
                if (!control_) return;

                control::Event e;
                e.type  = control::Event::Type::HmiWrite;
                e.ts_ms = nowMs();

                e.hmi_write.rs485_index = hmi_rs485_index_;
                e.hmi_write.slave_addr  = hmi ? hmi->slaveAddr() : 0;
                e.hmi_write.func        = 0;                 // 如需 func，可在 HMIProto 扩展
                e.hmi_write.start_addr  = ev.addr;           // ✅ 屏幕地址
                e.hmi_write.ts_ms       = e.ts_ms;

                if (ev.is_bool) {
                    e.hmi_write.bits = { static_cast<uint8_t>(ev.value_u16 ? 1 : 0) };
                } else {
                    e.hmi_write.regs = { ev.value_u16 };
                }

                control_->post(std::move(e));
            });
        }

        if (control_) {
            control_->bindHmi(hmi);
        }
    }

    /* =====================================================
     * 7. 初始状态机
     * ===================================================== */
    state_ = std::make_unique<StateIdle>();
    state_->onEnter(*this);

    LOG_SYS_I("APP init done");
    return true;
}

void AppManager::start() {
    if (running_) return;
    running_ = true;

    parser_->start();
    if (control_) control_->start();

    // ===== BMS worker：解析/聚合/节流分发（不影响 system/hmi）=====
    {
        proto::bms::BmsWorker::Config cfg;
        cfg.wait_timeout_ms = 10;
        cfg.dispatch_throttle_ms = 200; // 200ms 一次 dispatchBms（可调）

        bms_worker_ = std::make_unique<proto::bms::BmsWorker>(bms_queue_, bms_proto_, cfg);

        // 解析出的 DeviceData：更新 bms_snap_；并进入控制面
        bms_worker_->setOnDeviceData([this](const DeviceData& d) {


            aggregator_->onDeviceData(d);

            if (control_) {
                control::Event e;
                e.type = control::Event::Type::DeviceData;
                e.ts_ms = nowMs();
                e.device_data = d;

                control_->post(std::move(e));
            }
        });

        // tick：节流分发 bms snapshot（落盘/上传）
        bms_worker_->setOnTick([this] {
            snapshot_dispatcher_->dispatchBms(aggregator_->bmsSnapshot());
        });

        bms_worker_->start();
    }

    driver_manager_->start();

    // ✅ 轮询节奏/退避由 Scheduler 内部 pollTick() 控制
    scheduler_->start();

    if (!app_tick_timer_) {
        app_tick_timer_ = std::make_unique<SchedulerTimer>();
        app_tick_timer_->addPeriodic(100, [this] {
            Event e;
            e.type = Event::Type::Tick;
            this->post(e);
        });
    }
    app_tick_timer_->start();

    if (!fault_refresh_running_.exchange(true)) {
        fault_refresh_thread_ = std::thread(&AppManager::faultRefreshThreadMain_, this);
    }

    if (!bms_health_sync_running_.exchange(true)) {
        bms_health_sync_thread_ = std::thread(&AppManager::bmsHealthSyncThreadMain_, this);
    }

    post(Event{.type = Event::Type::CmdStart});

    LOG_SYS_I("APP start");
}

void AppManager::stop() {
    running_ = false;

    if (fault_refresh_running_.exchange(false)) {
        if (fault_refresh_thread_.joinable()) {
            fault_refresh_thread_.join();
        }
    }
    if (bms_health_sync_running_.exchange(false)) {
        if (bms_health_sync_thread_.joinable()) {
            bms_health_sync_thread_.join();
        }
    }

    // ✅ 工业级：先停控制面，避免继续下发命令时底层已关闭
    if (control_) control_->stop();
    if (scheduler_) scheduler_->stop();
    if (app_tick_timer_) {
        app_tick_timer_->stop();
    }
    if (bms_worker_) {
        bms_worker_->stop();
        bms_worker_.reset();
    }
    bms_queue_.clear();

    if (parser_) parser_->stop();
    if (driver_manager_) driver_manager_->stop();

    LOG_SYS_I("APP stop");
}

void AppManager::post(const Event& e) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        q_.push_back(e);
    }
    cv_.notify_one();
}

// 事件泵：从队列中取事件，处理，再将 Tick 事件发送给控制面
// 每次调用处理一个事件，或等待 50ms 直到有新事件
// 事件处理：根据事件类型调用状态机处理函数
// 事件发送：将 Tick 事件发送给控制面，包含当前时间戳和周期（500ms）
void AppManager::pumpOnce() {
    Event e;
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::milliseconds(50),
                     [&]{ return !q_.empty() || !running_; });
        if (!running_ || q_.empty()) return;
        e = std::move(q_.front());
        q_.pop_front();
    }

    if (state_) {
        state_->onEvent(*this, e);
        if (e.type == Event::Type::Tick && control_) {
            control::Event ce;
            ce.type = control::Event::Type::Tick;
            ce.ts_ms = nowMs();
            ce.tick.ts_ms = ce.ts_ms;
            ce.tick.period_ms = 100;   // 这里填你 TimerThread addTimer 的 interval（例如 500ms）
            control_->post(std::move(ce));
        }
    }
}

void AppManager::transitionTo(std::unique_ptr<StateBase> next) {
    if (!next) return;
    if (state_) state_->onExit(*this);
    state_ = std::move(next);
    state_->onEnter(*this);
}

// PCU 配置实例号 -> 程序内部设备名
// pcu_instance=1 -> PCU_0
// pcu_instance=2 -> PCU_1
static std::string pcuRuntimeNameFromInstance_(int pcu_instance)
{
    if (pcu_instance == 1) return "PCU_0";
    if (pcu_instance == 2) return "PCU_1";
    return {};
}

static int pcuInstanceFromDeviceData_(const DeviceData& d)
{
    if (auto it = d.value.find("__pcu.instance"); it != d.value.end()) {
        const int inst = static_cast<int>(it->second);
        if (inst >= 1 && inst <= 2) return inst;
    }

    if (auto it = d.value.find("pcu_instance"); it != d.value.end()) {
        const int inst = static_cast<int>(it->second);
        if (inst >= 1 && inst <= 2) return inst;
    }

    return 0;
}

// 归一化 PCU 设备数据：
//   PCU      + pcu_instance=1 -> PCU_0
//   PCU      + pcu_instance=2 -> PCU_1
//   PCU_CTRL + pcu_instance=1 -> PCU_0_CTRL
//   PCU_CTRL + pcu_instance=2 -> PCU_1_CTRL
//
// 注意：
//   不再使用 link_index==0/1 作为主判断依据。
//   link_index 只是物理 CAN 口，不代表 PCU 实例。
static DeviceData normalizePcuDeviceData_(const parser::ParserMessage& msg)
{
    DeviceData d = msg.device_data;

    if (msg.link_type != dev::LinkType::CAN) {
        return d;
    }

    if (d.device_name != "PCU" && d.device_name != "PCU_CTRL") {
        return d;
    }

    const int inst = pcuInstanceFromDeviceData_(d);
    const std::string base_name = pcuRuntimeNameFromInstance_(inst);

    if (base_name.empty()) {
        LOG_THROTTLE_MS("pcu_normalize_no_instance", 1000, LOG_COMM_W,
                        "[APP][PCU] cannot normalize device=%s can_index=%d: missing __pcu.instance",
                        d.device_name.c_str(),
                        msg.link_index);

        // 不做错误归一化，避免把 can_index=1 误认为 PCU_1。
        // 这里保留原名，让问题暴露在 snapshot/log 中。
        d.value["__can_index"] = msg.link_index;
        return d;
    }

    const std::string new_name =
        (d.device_name == "PCU_CTRL") ? (base_name + "_CTRL") : base_name;

    d.device_name = new_name;

    // 给 FileSink / Snapshot / Logic 留实例元信息
    d.value["__can_index"] = msg.link_index;
    d.value["__pcu.instance"] = inst;
    d.value["__pcu.runtime_index"] = inst - 1;

    d.str["__inst_name"] = new_name;
    d.str["__pcu.instance_name"] = base_name;
    d.str["__pcu.display_name"] = "PCU" + std::to_string(inst);

    return d;
}

void AppManager::onParsed(const parser::ParserMessage& msg) {
    using parser::ParsedType;

    auto linkTypeStr = [&](dev::LinkType t) -> const char* {
        switch (t) {
            case dev::LinkType::CAN:   return "CAN";
            case dev::LinkType::RS485: return "RS485";
            case dev::LinkType::RS232: return "RS232";
            default:                   return "UNKNOWN";
        }
    };

    // ===== HMI 心跳 =====
    if (msg.type == ParsedType::HmiHeartbeat) {
        if (control_) {
            control::Event e;
            e.type = control::Event::Type::HmiHeartbeat;
            e.ts_ms = msg.rx_ts_ms; // 携带底层解析到合法报文的精确时间戳
            control_->post(std::move(e));
        }
        return;
    }

    // ===== 成功数据 =====
    if (msg.type == ParsedType::DeviceData) {
        // BMS 不从 AppManager::onParsed() 这条普通 Parser 链路走。
        // 正确链路是：
        //   CAN -> CanDispatcher::handle() -> BmsQueue -> BmsWorker
        //      -> aggregator_->onDeviceData()
        //      -> control_->post(DeviceData)
        //
        // 如果这里还能看到 BMS，说明某处同步 BMS 解析路径还没有删干净。
        // 为避免重复聚合 / 重复故障 / 重复控制，直接丢弃。
        if (msg.device_data.device_name == "BMS") {
            LOG_THROTTLE_MS("bms_unexpected_onParsed", 1000, LOG_COMM_D,
                            "[APP][BMS] drop unexpected normal Parser BMS DeviceData link=%d idx=%d",
                            static_cast<int>(msg.link_type),
                            msg.link_index);
            return;
        }

        DeviceData normalized = normalizePcuDeviceData_(msg);

        scheduler_->onDeviceData(normalized);
        mergeSchedulerHealthToAggregator(this, normalized.device_name);

        auto snap = aggregator_->snapshot();
        snapshot_dispatcher_->dispatch(snap);

        // ===== 控制面：零散 DeviceData 进入 ControlLoop =====
        if (control_) {
            control::Event e;
            e.type = control::Event::Type::DeviceData;
            e.ts_ms = nowMs();
            e.device_data = normalized;
            control_->post(std::move(e));
        }

        // ===== 控制面：普通设备 snapshot 进入 Logic =====
        if (control_) {
            control::Event es;
            es.type = control::Event::Type::Snapshot;
            es.ts_ms = nowMs();
            es.snapshot.ts_ms = es.ts_ms;
            es.snapshot.snap = std::move(snap);
            control_->post(std::move(es));
        }

        return;
    }

    // ===== 其他解析错误 =====
    LOG_THROTTLE_MS("app_parse_error", 500, LOG_COMM_D,
        "PARSE_FAIL %s#%d text=%s",
        linkTypeStr(msg.link_type),
        msg.link_index,
        msg.error_text.c_str());
}

void AppManager::onControlTxMirrorDeviceData_(const DeviceData& d)
{
    if (!aggregator_) {
        return;
    }

    /*
     * PCU TX mirror 只更新 Aggregator 内存快照。
     *
     * 重要：
     *   这里不再立即 snapshot_dispatcher_->dispatch(snap)；
     *   这里不再 control_->post(Snapshot)；
     *
     * 原因：
     *   PCU TX mirror 频率较高，若每次都触发完整快照分发，
     *   会带动 FileSink / SQLite / UplinkSink / ControlLoop Snapshot，
     *   进而挤占 HMI 按键事件和普通数据反馈。
     *
     * 语义：
     *   TX mirror 只是“本机尝试发送过什么”的记录；
     *   不能证明 PCU 在线；
     *   不应驱动 HMI 逻辑刷新；
     *   不应进入 Scheduler health；
     *   不应作为 ControlLoop DeviceData。
     */
    aggregator_->onDeviceData(d);

    const std::string log_key =
        "pcu_tx_mirror_app_" + d.device_name;

    // LOG_THROTTLE_MS(log_key.c_str(), 1000, LOG_COMM_D,
    //                 "[APP][PCU_TX_MIRROR] dev=%s msg=%s",
    //                 d.device_name.c_str(),
    //                 d.str.count("__pcu.msg") ? d.str.at("__pcu.msg").c_str() : "");
}

bool AppManager::sendRs485(int index, const std::vector<uint8_t>& bytes) {
    if (!driver_manager_) return false;
    return driver_manager_->sendSerial(dev::LinkType::RS485, index, bytes);
}


void AppManager::faultRefreshThreadMain_()
{
    LOG_SYS_I("[APP][FAULT_REFRESH] fault refresh thread start interval=100ms");

    while (fault_refresh_running_.load()) {
        try {
            if (control_) {
                const uint64_t ts = nowMs();
                control_->refreshFaultPagesOnly(ts);
            }
        } catch (const std::exception& e) {
            LOG_COMM_D("[APP][FAULT_REFRESH] exception: %s", e.what());
        } catch (...) {
            LOG_COMM_D("[APP][FAULT_REFRESH] unknown exception");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_SYS_I("[APP][FAULT_REFRESH] fault refresh thread exit");
}

// ===== BMS runtime health 低频回写线程 =====
void AppManager::bmsHealthSyncThreadMain_()
{
    LOG_SYS_I("[APP][BMS_HEALTH_SYNC] thread start interval=500ms");

    uint64_t last_dispatch_ms = 0;

    while (bms_health_sync_running_.load()) {
        try {
            if (control_ && aggregator_) {
                auto updates = control_->latestBmsRuntimeHealth();

                if (!updates.empty()) {
                    const bool changed = aggregator_->updateBmsRuntimeHealth(updates);

                    // health 变化时低频分发 BMS 专用快照。
                    // 不在 ControlLoop 线程里 dispatch，避免拖住 HMI / fault page。
                    const uint64_t now = nowMs();
                    if (changed && snapshot_dispatcher_ &&
                        (last_dispatch_ms == 0 || now - last_dispatch_ms >= 1000))
                    {
                        last_dispatch_ms = now;
                        snapshot_dispatcher_->dispatchBms(aggregator_->bmsSnapshot());
                    }
                }
            }
        } catch (const std::exception& e) {
            LOG_COMM_D("[APP][BMS_HEALTH_SYNC] exception: %s", e.what());
        } catch (...) {
            LOG_COMM_D("[APP][BMS_HEALTH_SYNC] unknown exception");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    LOG_SYS_I("[APP][BMS_HEALTH_SYNC] thread exit");
}