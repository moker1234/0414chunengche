// app/app_manager.cpp

#include <thread>
#include <atomic>

#include "app_manager.h"

#include "driver_manager.h"          // ✅ 必须在这里 include
#include "state_machine/state_idle.h"
#include "logger.h"
#include <chrono>
#include <cstring>

#include "display_sink.h"
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
void AppManager::postSnapshotToControl_(
        uint64_t ts_ms)
{
    if (!control_) return;

    control::Event es;
    es.type = control::Event::Type::Snapshot;
    es.ts_ms = ts_ms;
    es.snapshot.ts_ms = ts_ms;
    es.snapshot.snap = aggregator().snapshot();
    control_->post(std::move(es));
}

AppManager::AppManager() = default;

AppManager::~AppManager() {
    // delete driver_mgr_;
}

bool AppManager::init() {
    scheduler_ = std::make_unique<DeviceScheduler>();

    /* =====================================================
     * 1. Parser
     * ===================================================== */
    parser_ = std::make_unique<parser::ProtocolParserThread>();

    // Parser → App
    parser_->setOnParsed(
        [this](const parser::ParserMessage& msg) {
            this->onParsed(msg);
        }
    );
    parser_->setBmsQueue(bms_can_index_, &bms_queue_);

    // Parser → Driver（发送字节）
    parser_->setSendSerial(
        [this](dev::LinkType t, int idx, const std::vector<uint8_t>& bytes) {
            if (driver_mgr_) {
                driver_mgr_->sendSerial(t, idx, bytes);
            }
        }
    );

    // ✅ Scheduler 注入 Parser：轮询节奏/决策在 Scheduler，执行在 Parser::sendPoll()
    scheduler_->setParser(parser_.get());

    /* =====================================================
     * 2. DriverManager（从 io_map.json 读取 can0/can1/can2 并启动线程）
     * ===================================================== */
    driver_mgr_ = std::make_unique<DriverManager>(*this, *parser_);
    driver_mgr_->init();

    // ✅ Scheduler → Driver（发送 CAN）
    scheduler_->setSendCan(
        [this](int can_index, const can_frame& fr) {
            if (driver_mgr_) {
                driver_mgr_->sendCan(can_index, fr);
            }
        }
    );

    /* =====================================================
     * 2.5 ControlLoop（控制面）
     * ===================================================== */
    if (driver_mgr_ && parser_) {
        control_ = std::make_unique<control::ControlLoop>(*driver_mgr_, *parser_);
    }
    // BMS V2B_CMD 改由 control/bms 命令管理器负责，关闭 scheduler 里的旧最小发送器
    if (scheduler_) {
        scheduler_->setBmsTxEnabled(false);
    }

    j1939_mgr_ = std::make_unique<j1939::J1939Manager>();
    // 让 DriverManager 的 CAN RX 能喂给 j1939_mgr_
    driver_mgr_->setJ1939Manager(j1939_mgr_.get());
    // 绑定各 CAN 通道的 TX：把 OpenSAE 发送出来的帧交给 DriverManager::sendCan
    for (int can_index = 0; can_index < 3; ++can_index) {
        j1939_mgr_->bindChannel(
            can_index,
            [this, can_index](const can_frame& fr) {
                if (!driver_mgr_) return;
                driver_mgr_->sendCan(can_index, fr);
            }
        );
    }

    /* =====================================================
     * 3. Aggregator + Dispatcher（解耦后的正确结构）
     * ===================================================== */
    aggregator_ = std::make_unique<agg::DataAggregator>();
    snapshot_dispatcher_ = std::make_unique<SnapshotDispatcher>();
    snapshot_dispatcher_->setHmiMappingEnabled(false);

    scheduler_->setOnDeviceData(
        [this](const DeviceData& d) {
            LOGT("[APP] DeviceData name=%s", d.device_name.c_str());
            aggregator_->onDeviceData(d);
        }
    );
    scheduler_->setOnHealthChanged(  // 20260409 检查online
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
            snapshot_dispatcher_->dispatch(aggregator_->snapshot());

            // 如果你已经补了这句，也顺便打
            // postSnapshotToControl_(this, nowMs());
        }
    );
    /* =====================================================
     * 4. Display RS485 Session
     * ===================================================== */
    display_session_ = std::make_unique<DisplayRs485Session>(3);

    display_session_->setSendSerial(
        [this](dev::LinkType t, int idx, const std::vector<uint8_t>& bytes) {
            if (driver_mgr_) {
                driver_mgr_->sendSerial(t, idx, bytes);
            }
        }
    );

    /* =====================================================
     * 5. 注册 Snapshot Sinks + HMI 映射绑定
     * ===================================================== */
    snapshot_dispatcher_->addSink(std::make_unique<DisplaySink>(*display_session_));
    snapshot_dispatcher_->addSink(std::make_unique<FileSink>());
    // { // 注册 SqliteSnapshotSink,0317
    //     SqliteSnapshotSink::Config cfg;
    //     cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
    //     cfg.busy_timeout_ms = 3000;
    //     cfg.only_when_changed = true;
    //     cfg.min_interval_ms = 0;
    //     snapshot_dispatcher_->addSink(std::make_unique<SqliteSnapshotSink>(cfg));
    // }
    {
        SqliteSnapshotFlatSink::Config cfg;
        cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
        cfg.busy_timeout_ms = 3000;
        cfg.write_main_every_snapshot = true;
        snapshot_dispatcher_->addSink(std::make_unique<SqliteSnapshotFlatSink>(cfg));
    }
    snapshot_dispatcher_->addSink(std::make_unique<UplinkSink>());

    // BMS 独立落盘（大数据）
    {
        BmsFileSink::Config cfg;
        cfg.base_dir = "/home/zlg/running_log/bms";
        cfg.latest_interval_ms = 1000;
        cfg.history_interval_ms = 1000;
        cfg.history_include_summary = false;
        cfg.json_indent = 0;
        snapshot_dispatcher_->addBmsSink(std::make_unique<BmsFileSink>(cfg));
    }    // BMS 历史并行写 SQLite
    // {
    //     SqliteBmsSink::Config cfg;
    //     cfg.db_path = "/mnt/sqlite_tfcard/json_data.db";
    //     cfg.busy_timeout_ms = 3000;
    //     cfg.history_interval_ms = 1000;
    //     snapshot_dispatcher_->addBmsSink(std::make_unique<SqliteBmsSink>(cfg));
    // }
    // BMS SQLite 平摊存储
    {
        SqliteBmsFlatSink::Config cfg;
        cfg.db_path = "/mnt/sqlite_tfcard/bms_data.db";
        cfg.busy_timeout_ms = 3000;
        cfg.write_main_every_snapshot = true;
        snapshot_dispatcher_->addBmsSink(std::make_unique<SqliteBmsFlatSink>(cfg));
    }

    // 1) 先加载 mapping 文件
    {
        std::string err;
        if (!snapshot_dispatcher_->loadHmiMapFile("/home/zlg/userdata/config/display_map.jsonl", &err)) {
            LOG_SYS_W("[HMI] load display_map.jsonl failed: %s", err.c_str());
        } else {
            LOG_SYS_I("[HMI] display_map.jsonl loaded");
        }
    }

    //  加载 fault_map.jsonl（由控制面管理故障页）
    if (control_) {
        std::string err;
        if (!control_->loadFaultMapFile("/home/zlg/userdata/config/fault_map.jsonl", &err)) {
            LOG_SYS_W("[FAULT] load fault_map.jsonl failed: %s", err.c_str());
        } else {
            LOG_SYS_I("[FAULT] fault_map.jsonl loaded");
        }
    }
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
    // 2) 再绑定 hmi（serial_index=3 对应 ttyRS485-4）
    if (parser_) {
        auto* hmi = parser_->getHmiProto(3);
        LOGD("[APP][HMI] getHmiProto(3)=%p", (void*)hmi);

        if (hmi) {
            hmi->setCompatMode(true); // 兼容空实现也没问题
        }

        if (hmi) {
            // ✅ HMI 写入（FC05/06/0F/10）进入 ControlLoop（写入是控制入口）
            hmi->setOnWrite([this, hmi](const HmiWriteEvent& ev) {
                if (!control_) return;

                control::Event e;
                e.type  = control::Event::Type::HmiWrite;
                e.ts_ms = nowMs();

                e.hmi_write.rs485_index = 3;                 // HMI 在 RS485#3（ttyRS485-4）
                e.hmi_write.slave_addr  = hmi ? hmi->slaveAddr() : 0;
                e.hmi_write.func        = 0;                 // 如需 func，可在 HMIProto 扩展
                e.hmi_write.start_addr  = ev.addr;           // ✅ 屏幕地址
                e.hmi_write.ts_ms       = e.ts_ms;

                if (ev.is_bool) {
                    e.hmi_write.bits = { (uint8_t)(ev.value_u16 ? 1 : 0) };
                } else {
                    e.hmi_write.regs = { ev.value_u16 };
                }

                control_->post(std::move(e));
            });
        }
        // 渐进式迁移：加载普通变量旁路映射（jsonl）
        if (control_) {
            std::string err;
            if (!control_->loadNormalMapFile("/home/zlg/userdata/config/normal_map_logic.jsonl", &err)) {
                LOG_SYS_W("[NORMAL] load normal_map_logic.jsonl failed: %s", err.c_str());
            } else {
                LOG_SYS_I("[NORMAL] normal_map_logic.jsonl loaded");
            }
        }

        if (control_) {
            control_->bindHmi(hmi);
        }

        // 故障页 int_read 地址段由控制面独占：
        // - 当前页：0x411B ~ 0x4136
        // - 历史页：0x4141 ~ 0x418F
        //
        // 说明：
        // 1) fault_hmi_layout.jsonl 虽按更大页面能力设计，
        //    但本项目一阶段程序侧只按前5行故障页工作。
        // 2) FaultCenter::flushToHmi() 是这些地址的唯一写入入口。
        snapshot_dispatcher_->blockHmiIntReadRange(0x411B, 0x4136);
        snapshot_dispatcher_->blockHmiIntReadRange(0x4141, 0x418F);
        LOG_SYS_I("[HMI] block int_read ranges 0x411B..0x4136 and 0x4141..0x418F (owned by logic fault pages, stage1 uses first 5 rows)");
        snapshot_dispatcher_->bindHmi(hmi);
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

    driver_mgr_->start();

    // ✅ 轮询节奏/退避由 Scheduler 内部 pollTick() 控制
    scheduler_->start();

    // if (!app_tick_timer_) {
    //     app_tick_timer_ = std::make_unique<SchedulerTimer>();
    //     app_tick_timer_->addPeriodic(500, [this] {
    //         Event e;
    //         e.type = Event::Type::Tick;
    //         this->post(e);
    //     });
    // }
    // app_tick_timer_->start();
    if (j1939_mgr_) j1939_mgr_->start();

    if (!logic_refresh_running_.exchange(true)) {
        logic_refresh_thread_ = std::thread(&AppManager::logicRefreshThreadMain_, this);
    }

    if (!fault_refresh_running_.exchange(true)) {
        fault_refresh_thread_ = std::thread(&AppManager::faultRefreshThreadMain_, this);
    }

    post(Event{.type = Event::Type::Boot});
    post(Event{.type = Event::Type::CmdStart});

    LOG_SYS_I("APP start");
}

void AppManager::stop() {
    running_ = false;
    logic_refresh_running_.store(false);
    if (logic_refresh_thread_.joinable()) {
        logic_refresh_thread_.join();
    }
    if (fault_refresh_running_.exchange(false)) {
        if (fault_refresh_thread_.joinable()) {
            fault_refresh_thread_.join();
        }
    }
    // ✅ 工业级：先停控制面，避免继续下发命令时底层已关闭
    if (control_) control_->stop();
    if (scheduler_)  scheduler_->stop();
    // if (app_tick_timer_) {
    //     app_tick_timer_->stop();
    // }
    if (j1939_mgr_)  j1939_mgr_->stop();
    if (bms_worker_) {
        bms_worker_->stop();
        bms_worker_.reset();
    }
    bms_queue_.clear();

    if (parser_)     parser_->stop();
    if (driver_mgr_) driver_mgr_->stop();



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
            ce.tick.period_ms = 500;   // 这里填你 TimerThread addTimer 的 interval（例如 500ms）
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

// 归一化 PCU 设备名：PCU -> PCU_0/PCU_1；PCU_CTRL -> PCU_0_CTRL/PCU_1_CTRL
static std::string normalizePcuDeviceName_(const std::string& device_name, dev::LinkType link_type,int link_index)
{
    if (link_type != dev::LinkType::CAN) return device_name;

    if (device_name == "PCU") {
        if (link_index == 0) return "PCU_0";
        if (link_index == 1) return "PCU_1";
    }

    if (device_name == "PCU_CTRL") {
        if (link_index == 0) return "PCU_0_CTRL";
        if (link_index == 1) return "PCU_1_CTRL";
    }

    return device_name;
}
// 归一化 PCU 设备数据：PCU -> PCU_0/PCU_1；PCU_CTRL -> PCU_0_CTRL/PCU_1_CTRL
static DeviceData normalizePcuDeviceData_(const parser::ParserMessage& msg)
{
    DeviceData d = msg.device_data;

    const std::string new_name =
        normalizePcuDeviceName_(d.device_name, msg.link_type, msg.link_index);

    if (new_name != d.device_name) {
        d.device_name = new_name;

        // 给 filesink / snapshot 留一点实例元信息，后面排查方便
        d.value["__can_index"] = msg.link_index;
        d.str["__inst_name"] = new_name;
    }

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

    // ===== 成功数据 =====
    if (msg.type == ParsedType::DeviceData) {
        // ✅ BMS 不从这里走：BmsWorker 专用线程已经解析并喂给 aggregator/control，
        // 这里丢弃可避免刷屏和重复处理。
        if (msg.device_data.device_name == "BMS") {
            // printf("BMS into Appmanager::onParse\n"); // 如需可打开

            if (msg.type == parser::ParsedType::DeviceData &&   // 20260410
    msg.device_data.device_name == "BMS") {
                LOG_THROTTLE_MS("trace_bms_onParsed", 200, LOG_COMM_D,
                    "[TRACE][BMS][onParsed] link=%d idx=%d rx_ts=%llu dev=%s",
                    (int)msg.link_type,
                    msg.link_index,
                    (unsigned long long)msg.rx_ts_ms,
                    msg.device_data.device_name.c_str());
    }
            return;
        }


        // 0207 - AirConditioner 的 T1 打印（你原样保留）
        if (msg.device_data.device_name == "AirConditioner") {
            // const uint64_t now = nowMs();
            // const uint32_t ts  = msg.device_data.timestamp;

            const auto& num = msg.device_data.num;
            const auto& val = msg.device_data.value;
            const auto& st  = msg.device_data.status;

            auto get_num = [&](const char* key, double defv = 0.0) -> double {
                auto it = num.find(key);
                return (it == num.end()) ? defv : it->second;
            };
            auto get_i32 = [&](const char* key, int32_t defv = 0) -> int32_t {
                auto it = val.find(key);
                return (it == val.end()) ? defv : it->second;
            };
            auto get_u32 = [&](const char* key, uint32_t defv = 0u) -> uint32_t {
                auto it = st.find(key);
                return (it == st.end()) ? defv : it->second;
            };

            (void)get_i32; (void)get_u32;

            const int32_t version = (int32_t)(get_num("version", 0.0) + 0.5);

            auto get_run01 = [&](const char* key) -> uint32_t {
                auto it = st.find(key);
                if (it != st.end()) return (it->second != 0u);
                auto it2 = val.find(key);
                if (it2 != val.end()) return (it2->second != 0);
                return 0u;
            };

            const uint32_t run_overall   = get_run01("run.overall");
            const uint32_t run_inner_fan = get_run01("run.inner_fan");
            const uint32_t run_outer_fan = get_run01("run.outer_fan");
            const uint32_t run_comp      = get_run01("run.compressor");
            const uint32_t run_heater    = get_run01("run.heater");
            const uint32_t run_em_fan    = get_run01("run.em_fan");

            auto round_x10 = [&](const char* key) -> int32_t {
                double v = get_num(key, 0.0);
                return (int32_t)(v * 10.0 + (v >= 0 ? 0.5 : -0.5));
            };

            const int32_t t_coil_x10     = round_x10("temp.coil_c");
            const int32_t t_outdoor_x10  = round_x10("temp.outdoor_c");
            const int32_t t_cond_x10     = round_x10("temp.condense_c");
            const int32_t t_indoor_x10   = round_x10("temp.indoor_c");
            const int32_t hum_pct        = (int32_t)(get_num("humidity_percent", 0.0) + 0.5);
            const int32_t t_exhaust_x10  = round_x10("temp.exhaust_c");

            const double cur_a = get_num("current_a", 0.0);
            const int32_t current_x10    = (int32_t)(cur_a * 10.0 + (cur_a >= 0 ? 0.5 : -0.5));
            const int32_t ac_v           = (int32_t)(get_num("ac_voltage_v", 0.0) + 0.5);
            const int32_t dc_v           = (int32_t)(get_num("dc_voltage_v", 0.0) + 0.5);

            struct Snap {
                int32_t  version;
                uint32_t ro, rif, rof, rc, rh, ref;
                int32_t  tc, to, tcond, tind, hum, tex;
                int32_t  ia, acv, dcv;
            };

            static Snap last = {
                -1,
                0xFFFFFFFFu,0xFFFFFFFFu,0xFFFFFFFFu,0xFFFFFFFFu,0xFFFFFFFFu,0xFFFFFFFFu,
                0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF,
                0x7FFFFFFF,0x7FFFFFFF,0x7FFFFFFF
            };

            const Snap cur = {
                version,
                run_overall, run_inner_fan, run_outer_fan, run_comp, run_heater, run_em_fan,
                t_coil_x10, t_outdoor_x10, t_cond_x10, t_indoor_x10, hum_pct, t_exhaust_x10,
                current_x10, ac_v, dc_v
            };

            if (std::memcmp(&cur, &last, sizeof(Snap)) != 0) {
                // LOGD("[AC_LAT][T1_RX] now=%llu dev_ts=%u ver=%d "
                //      "run{o=%u in=%u out=%u comp=%u heat=%u em=%u} "
                //      "t{x10 coil=%d out=%d cond=%d in=%d exh=%d} hum=%d "
                //      "elec{I_x10=%d acV=%d dcV=%d}",
                //      (unsigned long long)now, ts, cur.version,
                //      cur.ro, cur.rif, cur.rof, cur.rc, cur.rh, cur.ref,
                //      cur.tc, cur.to, cur.tcond, cur.tind, cur.tex, cur.hum,
                //      cur.ia, cur.acv, cur.dcv);
                last = cur;
            }
        }
        DeviceData normalized = normalizePcuDeviceData_(msg);


        scheduler_->onDeviceData(normalized);                          // 通知状态机
        mergeSchedulerHealthToAggregator(this, normalized.device_name); //msg.device_data.device_name); // 合并状态机状态
        snapshot_dispatcher_->dispatch(aggregator_->snapshot());            // 通知 HMI 和其他 sink

        // ===== 控制面：零散 DeviceData 进入 ControlLoop（不影响通信线程）=====
        if (control_) {
            control::Event e;
            e.type = control::Event::Type::DeviceData;
            e.ts_ms = nowMs();
            e.device_data = normalized;//msg.device_data;
            control_->post(std::move(e));
        }

        //  新链路B：把 snapshot 送到 logic（渐进式迁移普通变量表）
        if (control_) {
            control::Event es;
            es.type = control::Event::Type::Snapshot;
            es.ts_ms = nowMs();
            es.snapshot.ts_ms = es.ts_ms;
            es.snapshot.snap = aggregator_->snapshot();   // copy（后续可优化成 shared_ptr）
            control_->post(std::move(es));
        }
        return;
    }



    // ===== 其他解析错误 =====
    // 注意：这里可能来自 CAN / RS485 / RS232，必须正确打印 link_type
    LOG_THROTTLE_MS("app_parse_error", 500, LOG_COMM_D,
        "PARSE_FAIL %s#%d text=%s",
        linkTypeStr(msg.link_type),
        msg.link_index,
        msg.error_text.c_str());
}

bool AppManager::sendRs485(int index, const std::vector<uint8_t>& bytes) {
    if (!driver_mgr_) return false;
    return driver_mgr_->sendSerial(dev::LinkType::RS485, index, bytes);
}

void AppManager::logicRefreshThreadMain_()
{
    LOG_SYS_I("[APP][REFRESH] latest snapshot refresh thread start interval=100ms");

    while (logic_refresh_running_.load()) {
        try {
            if (control_ && aggregator_) {
                const uint64_t ts = nowMs();
                auto snap = aggregator_->snapshot();

                control::Event es;
                es.type = control::Event::Type::Snapshot;
                es.ts_ms = ts;
                es.snapshot.ts_ms = ts;
                es.snapshot.snap = std::move(snap);

                control_->post(std::move(es));
            }

        } catch (const std::exception& e) {
            LOG_COMM_D("[APP][REFRESH] exception: %s", e.what());
        } catch (...) {
            LOG_COMM_D("[APP][REFRESH] unknown exception");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_SYS_I("[APP][REFRESH] latest snapshot refresh thread exit");
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