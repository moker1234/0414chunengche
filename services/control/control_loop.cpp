// services/control/control_loop.cpp

#include <memory>
#include "control_loop.h"

#include <set>

#include "../utils/logger/logger.h"
#include "../protocol/rs485/hmi/hmi_proto.h"
#include "../snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"
#include "../normal/hmi_map_loader.h"
namespace control {

    ControlLoop::ControlLoop(::DriverManager& drv, parser::ProtocolParserThread& parser)
        : dispatcher_(drv, parser)
    {
        engine_.init(drv);

        std::string maintain_err;
        if (!maintain_.loadConfig("/home/zlg/userdata/config/maintain.json", &maintain_err)) {
            LOG_SYS_W("[MAINTAIN][CFG] init failed: %s", maintain_err.c_str());
        }
        std::string sys_err;
        if (!maintain_.loadSystemConfig("/home/zlg/userdata/config/system.json", &sys_err)) {
            LOG_SYS_W("[MAINTAIN][SYS] init failed: %s", sys_err.c_str());
        }

    }
ControlLoop::~ControlLoop()
{
    stop();
}

void ControlLoop::bindHmi(HMIProto* hmi)
{
    ctx_.hmi = hmi;
    maintain_.bindHmi(hmi);
}
    void ControlLoop::setTxMirrorCallback(std::function<void(const DeviceData&)> cb)
{
    tx_mirror_cb_ = std::move(cb);
}

    bool ControlLoop::isHighPriorityEvent_(const Event& e)
{
    switch (e.type) {
    case Event::Type::HmiWrite:
    case Event::Type::HmiHeartbeat:
    case Event::Type::Stop:
        return true;

    default:
        return false;
    }
}

    void ControlLoop::pushEvent_(const Event& e)
{
    {
        std::lock_guard<std::mutex> lk(q_mtx_);

        if (isHighPriorityEvent_(e)) {
            q_high_.push_back(e);
        } else {
            q_normal_.push_back(e);
        }
    }

    q_cv_.notify_one();
}

    void ControlLoop::pushEvent_(Event&& e)
{
    {
        std::lock_guard<std::mutex> lk(q_mtx_);

        if (isHighPriorityEvent_(e)) {
            q_high_.push_back(std::move(e));
        } else {
            q_normal_.push_back(std::move(e));
        }
    }

    q_cv_.notify_one();
}

    bool ControlLoop::popEvent_(Event& out)
{
    std::unique_lock<std::mutex> lk(q_mtx_);

    q_cv_.wait(lk, [this] {
        return !q_high_.empty() || !q_normal_.empty();
    });

    if (!q_high_.empty()) {
        out = std::move(q_high_.front());
        q_high_.pop_front();
        return true;
    }

    if (!q_normal_.empty()) {
        out = std::move(q_normal_.front());
        q_normal_.pop_front();
        return true;
    }

    return false;
}

    std::vector<agg::BmsRuntimeHealthUpdate>
ControlLoop::latestBmsRuntimeHealth() const
{
    std::lock_guard<std::mutex> lk(bms_health_mtx_);
    return latest_bms_health_;
}

    void ControlLoop::refreshCachedBmsRuntimeHealth_()
{
    std::vector<agg::BmsRuntimeHealthUpdate> out;
    out.reserve(ctx_.bms_cache.items.size());

    auto parse_idx_from_name = [](const std::string& name) -> uint32_t {
        constexpr const char* kPrefix = "BMS_";
        if (name.rfind(kPrefix, 0) != 0) return 0;

        try {
            const int v = std::stoi(name.substr(4));
            if (v >= 1 && v <= 4) {
                return static_cast<uint32_t>(v);
            }
        } catch (...) {
        }

        return 0;
    };

    for (const auto& kv : ctx_.bms_cache.items) {
        const auto& x = kv.second;

        agg::BmsRuntimeHealthUpdate u;
        u.instance_name = !x.instance_name.empty() ? x.instance_name : kv.first;
        u.bms_index = x.bms_index != 0 ? x.bms_index : parse_idx_from_name(u.instance_name);

        if (u.bms_index < 1 || u.bms_index > 4 || u.instance_name.empty()) {
            continue;
        }

        u.online = x.online;
        u.last_ok_ms = x.last_ok_ms;
        u.disconnect_window_ms = x.disconnect_window_ms;
        u.last_offline_ms = x.last_offline_ms;
        u.disconnect_count = x.disconnect_count;

        out.push_back(std::move(u));
    }

    {
        std::lock_guard<std::mutex> lk(bms_health_mtx_);
        latest_bms_health_ = std::move(out);
    }
}

bool ControlLoop::loadHmiMapFile(const std::string& path, std::string* err)
{
    auto model = std::make_shared<normal::HmiMapModel>();

    normal::HmiMapLoader loader;
    std::string load_err;
    if (!loader.loadJsonl(path, *model, &load_err)) {
        ctx_.hmi_map_loaded = false;
        ctx_.hmi_map.reset();

        if (err) {
            *err = "load HmiMapModel failed: " + load_err;
        }

        LOG_SYS_W("[HMI][MAP] load failed path=%s err=%s",
                  path.c_str(),
                  load_err.c_str());
        return false;
    }

    std::string normal_err;
    if (!ctx_.normal_writer.bindMap(model, &normal_err)) {
        ctx_.hmi_map_loaded = false;
        ctx_.hmi_map.reset();

        if (err) {
            *err = "bind NormalHmiWriter failed: " + normal_err;
        }

        LOG_SYS_W("[HMI][MAP] bind normal writer failed path=%s err=%s",
                  path.c_str(),
                  normal_err.c_str());
        return false;
    }

    ctx_.fault_pages.bindCenter(&ctx_.fault_center);

    std::string fault_layout_err;
    if (!ctx_.fault_pages.buildLayoutFromHmiMap(*model, &fault_layout_err)) {
        ctx_.hmi_map_loaded = false;
        ctx_.hmi_map.reset();

        if (err) {
            *err = "build fault layout from HmiMapModel failed: " + fault_layout_err;
        }

        LOG_SYS_W("[HMI][MAP] build fault layout failed path=%s err=%s",
                  path.c_str(),
                  fault_layout_err.c_str());
        return false;
    }

    /*
     * 维护模块第一批接入点：
     * - 复用同一份 HmiMapModel；
     * - 只识别 path -> addr/words/type；
     * - 不写 HMI 表，不处理写入，不改变业务。
     *
     * 注意：
     * - 缺某个维护 path 时，MaintainService 只打 warning；
     * - 只有 model 为空等结构性错误才返回 false。
     */
    std::string maintain_map_err;
    if (!maintain_.bindHmiMap(model, &maintain_map_err)) {
        ctx_.hmi_map_loaded = false;
        ctx_.hmi_map.reset();

        if (err) {
            *err = "bind MaintainService failed: " + maintain_map_err;
        }

        LOG_SYS_W("[MAINTAIN][MAP] bind failed path=%s err=%s",
                  path.c_str(),
                  maintain_map_err.c_str());
        return false;
    }

    ctx_.hmi_map = model;
    ctx_.hmi_map_loaded = true;

    /*
     * 如果当前 AppManager 仍按旧顺序：
     *   loadFaultMapFile()
     *   bindFaultDb()
     *   restoreFaultHistory()
     *   loadNormalMapFile()
     *
     * 那么 fault_history_cache 可能已经按旧默认页大小初始化过。
     * 这里在 HMI map 加载后，重新同步历史页 page_size。
     */
    if (ctx_.fault_history_cache) {
        ctx_.fault_history_cache->setPageSize(ctx_.fault_pages.historyPageRows());
        ctx_.fault_history_cache->setHotCacheRows(
            static_cast<uint16_t>(ctx_.fault_pages.historyPageRows() * 2u)
        );
        ctx_.fault_history_cache->setWindowPages(3);

        const bool ok_meta = ctx_.fault_history_cache->refreshMeta();
        const bool ok_hot  = ok_meta && ctx_.fault_history_cache->refreshHotCache();
        const bool ok_win  = ok_hot && ctx_.fault_history_cache->refreshWindow(ctx_.history_page_no);

        LOG_SYS_I("[HMI][MAP] resync history cache page_size=%u meta=%d hot=%d win=%d",
                  (unsigned)ctx_.fault_pages.historyPageRows(),
                  ok_meta ? 1 : 0,
                  ok_hot ? 1 : 0,
                  ok_win ? 1 : 0);
    }

    LOG_SYS_I("[HMI][MAP] loaded path=%s blocks=%zu items=%zu normal=%zu fault_num=%zu fault_btn=%zu rw=%zu "
              "cur_rows=%u his_rows=%u",
              path.c_str(),
              model->blocks.size(),
              model->items.size(),
              model->normal_items.size(),
              model->fault_num_items.size(),
              model->fault_button_items.size(),
              model->rw_items.size(),
              (unsigned)ctx_.fault_pages.currentPageRows(),
              (unsigned)ctx_.fault_pages.historyPageRows());

    return true;
}


bool ControlLoop::loadFaultMapFile(const std::string& path, std::string* err)
{
    // 1) 加载故障目录（FaultCatalog 用）
    if (!ctx_.fault_catalog.loadJsonl(path, err)) {
        ctx_.fault_map_loaded = false;
        return false;
    }

    ctx_.fault_center.bindCatalog(&ctx_.fault_catalog);
    ctx_.fault_pages.bindCenter(&ctx_.fault_center);

    /*
     * 第四批关键变化：
     *
     * loadFaultMapFile() 不再加载：
     *   - fault_hmi_layout.jsonl
     *   - normal_map_logic.jsonl
     *
     * 故障页 HMI 布局由 loadHmiMapFile(normal_map_logic.jsonl) 建立。
     *
     * 注意：
     * 当前 AppManager 还没进入第五批时，调用顺序可能仍是：
     *   loadFaultMapFile()
     *   bindFaultDb()
     *   loadNormalMapFile()
     *
     * 所以这里不因为 hmi_map 未加载而失败，只打 warning。
     */
    if (!ctx_.hmi_map_loaded || !ctx_.fault_pages.layoutLoaded()) {
        LOG_SYS_W("[FAULT][MAP] fault catalog/runtime will load before HMI map layout; "
                  "fault page output waits for loadHmiMapFile()");
    }

    // 2) 加载 runtime 规则（FaultRuntimeMapper 用）
    if (!engine_.loadFaultRuntimeMapFile(path, err)) {
        ctx_.fault_map_loaded = false;
        return false;
    }

    ctx_.fault_map_loaded = true;

    // 3) 打印 runtime 规则统计
    {
        const auto& mapper = engine_.faultRuntimeMapper();
        const auto& stats = mapper.loadStats();

        size_t cnt_bms = 0;
        size_t cnt_pcu = 0;
        size_t cnt_ups = 0;
        size_t cnt_smoke = 0;
        size_t cnt_gas = 0;
        size_t cnt_air = 0;
        size_t cnt_logic = 0;
        size_t cnt_other = 0;

        for (const auto& r : mapper.rules()) {
            if (r.source_norm == "bms") ++cnt_bms;
            else if (r.source_norm == "pcu") ++cnt_pcu;
            else if (r.source_norm == "ups") ++cnt_ups;
            else if (r.source_norm == "smoke") ++cnt_smoke;
            else if (r.source_norm == "gas") ++cnt_gas;
            else if (r.source_norm == "air") ++cnt_air;
            else if (r.source_norm == "logic") ++cnt_logic;
            else ++cnt_other;
        }

        LOG_SYS_I(
            "[FAULT][MAP] loaded path=%s catalog_ok=1 runtime_rules=%zu "
            "source_dist{bms=%zu pcu=%zu ups=%zu smoke=%zu gas=%zu air=%zu logic=%zu other=%zu} "
            "stats{total=%zu accepted=%zu skip_empty=%zu skip_unsupported=%zu} "
            "hmi_map_loaded=%d fault_layout_loaded=%d",
            path.c_str(),
            mapper.rules().size(),
            cnt_bms, cnt_pcu, cnt_ups, cnt_smoke, cnt_gas, cnt_air, cnt_logic, cnt_other,
            stats.total_items,
            stats.accepted_rules,
            stats.skipped_no_source_or_signal,
            stats.skipped_unsupported_source,
            ctx_.hmi_map_loaded ? 1 : 0,
            ctx_.fault_pages.layoutLoaded() ? 1 : 0
        );
    }

    return true;
}

    void ControlLoop::bindFaultDb(SqliteFaultSink* db)
{
    // 旧链：FaultCenter 仍需要知道 SQLite，用于 begin/clear 历史记录
    ctx_.fault_center.bindFaultDb(db);

    // 新链：历史故障页缓存层必须真正创建并绑定 DB，
    // 否则 logic_hmi_output.cpp 里的 applyHistoryCacheToHmi_()
    // 会因为 ctx_.fault_history_cache == nullptr 而把历史页整段清 0。
    if (!ctx_.fault_history_cache) {
        ctx_.fault_history_cache = std::make_unique<control::FaultHistoryCache>();

        // 历史故障分页行数跟随 fault_hmi_layout.jsonl 加载后的布局，
        // 不再使用 fault_addr_layout.h 的 FAULTS_PER_PAGE。
        ctx_.fault_history_cache->setPageSize(ctx_.fault_pages.historyPageRows());

        // 首页热点缓存保持 2 页左右的数据量。
        ctx_.fault_history_cache->setHotCacheRows(
            static_cast<uint16_t>(ctx_.fault_pages.historyPageRows() * 2u)
        );

        ctx_.fault_history_cache->setWindowPages(3);  // 前一页 + 当前页 + 后一页
        ctx_.history_page_no = 1;
    }

    const bool ok_bind = ctx_.fault_history_cache->bindDb(db);
    const bool ok_meta = ok_bind && ctx_.fault_history_cache->refreshMeta();
    const bool ok_hot  = ok_meta && ctx_.fault_history_cache->refreshHotCache();
    const bool ok_win  = ok_hot  && ctx_.fault_history_cache->refreshWindow(ctx_.history_page_no);
    // 20260418
    // LOG_SYS_I("[FAULT][SQLITE][CACHE_BIND] bind=%d meta=%d hot=%d win=%d total_rows=%u total_pages=%u window=%u~%u page=%u",
    //           ok_bind ? 1 : 0,
    //           ok_meta ? 1 : 0,
    //           ok_hot ? 1 : 0,
    //           ok_win ? 1 : 0,
    //           (unsigned)(ctx_.fault_history_cache ? ctx_.fault_history_cache->totalRows() : 0),
    //           (unsigned)(ctx_.fault_history_cache ? ctx_.fault_history_cache->totalPages() : 0),
    //           (unsigned)(ctx_.fault_history_cache ? ctx_.fault_history_cache->windowStartPage() : 0),
    //           (unsigned)(ctx_.fault_history_cache ? ctx_.fault_history_cache->windowEndPage() : 0),
    //           (unsigned)ctx_.history_page_no);
}

    void ControlLoop::restoreFaultHistory(const std::vector<FaultHistoryDbRecord>& rows)
{
    // 旧链恢复：FaultCenter 仍要保留 begin/clear/当前页所需的历史状态
    ctx_.fault_center.restoreHistoryFromDb(rows);

    // 新链恢复：历史页缓存层以 SQLite 为真源重新拉取一遍
    // 注意：这里不直接把 rows 灌给 cache，而是统一走 cache -> sqlite 查询接口，
    // 避免 cache 和 SQLite 真源分叉。
    if (ctx_.fault_history_cache) {
        const bool ok_meta = ctx_.fault_history_cache->refreshMeta();
        const bool ok_hot  = ok_meta && ctx_.fault_history_cache->refreshHotCache();
        const bool ok_win  = ok_hot  && ctx_.fault_history_cache->refreshWindow(ctx_.history_page_no);

        LOG_SYS_I("[FAULT][SQLITE][CACHE_RESTORE] rows=%zu meta=%d hot=%d win=%d total_rows=%u total_pages=%u window=%u~%u page=%u",
                  rows.size(),
                  ok_meta ? 1 : 0,
                  ok_hot ? 1 : 0,
                  ok_win ? 1 : 0,
                  (unsigned)ctx_.fault_history_cache->totalRows(),
                  (unsigned)ctx_.fault_history_cache->totalPages(),
                  (unsigned)ctx_.fault_history_cache->windowStartPage(),
                  (unsigned)ctx_.fault_history_cache->windowEndPage(),
                  (unsigned)ctx_.history_page_no);
    } else {
        LOG_SYS_W("[FAULT][SQLITE][CACHE_RESTORE] fault_history_cache is null");
    }

    // 这里不要再直接走旧的 ctx_.fault_center.flushToHmi(*ctx_.hmi)。
    // 因为第十二批之后 FaultCenter::flushToHmi() 已经只负责当前故障页，
    // 历史故障页必须走 applyFaultHmi_() 里的 cache 覆盖链。
    //
    // 初始化阶段 HMI 往往还没 bind，后续 fault_refresh_thread 每 100ms 会统一刷新；
    // 若此时 HMI 已经可用，也可以主动触发一次新链刷新。
    if (ctx_.hmi && ctx_.fault_map_loaded) {
        engine_.refreshFaultPagesOnly(nowMs(), ctx_);
    }
}

    void ControlLoop::refreshFaultPagesOnly(uint64_t ts_ms)
{
    std::lock_guard<std::mutex> lk(logic_mtx_);
    engine_.refreshFaultPagesOnly(ts_ms, ctx_);
}
void ControlLoop::start()
{
    if (running_.exchange(true)) return;
    th_ = std::thread(&ControlLoop::threadMain_, this);
    LOG_SYS_I("[CTRL] ControlLoop started");
}

void ControlLoop::stop()
{
    if (!running_.exchange(false)) return;

    // 终止时清掉 snapshot latest buffer，避免 stop 后还拿到旧快照
    {
        std::lock_guard<std::mutex> lk(snapshot_mtx_);
        has_latest_snapshot_ = false;
        snapshot_marker_enqueued_ = false;
    }

    pushEvent_(Event::makeStop());

    if (th_.joinable()) th_.join();
    LOG_SYS_I("[CTRL] ControlLoop stopped");
}

    void ControlLoop::post(const Event& e)
{
    if (e.type == Event::Type::Snapshot) {
        postSnapshotLatest_(e);
        return;
    }

    pushEvent_(e);
}

    void ControlLoop::post(Event&& e)
{
    if (e.type == Event::Type::Snapshot) {
        postSnapshotLatest_(std::move(e));
        return;
    }

    pushEvent_(std::move(e));
}

void ControlLoop::postSnapshotLatest_(const Event& e)
{
    bool need_enqueue_marker = false;
    {
        std::lock_guard<std::mutex> lk(snapshot_mtx_);
        latest_snapshot_event_ = e;
        has_latest_snapshot_ = true;

        if (!snapshot_marker_enqueued_) {
            snapshot_marker_enqueued_ = true;
            need_enqueue_marker = true;
        }
    }

    if (need_enqueue_marker) {
        Event marker;
        marker.type = Event::Type::Snapshot;
        marker.ts_ms = e.ts_ms; // 仅占位；真实 payload 在线程里替换
        pushEvent_(std::move(marker));
    }
}

void ControlLoop::postSnapshotLatest_(Event&& e)
{
    bool need_enqueue_marker = false;
    uint64_t ts = e.ts_ms;

    {
        std::lock_guard<std::mutex> lk(snapshot_mtx_);
        latest_snapshot_event_ = std::move(e);
        has_latest_snapshot_ = true;

        if (!snapshot_marker_enqueued_) {
            snapshot_marker_enqueued_ = true;
            need_enqueue_marker = true;
        }
    }

    if (need_enqueue_marker) {
        Event marker;
        marker.type = Event::Type::Snapshot;
        marker.ts_ms = ts; // 仅占位；真实 payload 在线程里替换
        pushEvent_(std::move(marker));
    }
}

bool ControlLoop::takeLatestSnapshot_(Event& out)
{
    std::lock_guard<std::mutex> lk(snapshot_mtx_);

    if (!has_latest_snapshot_) {
        snapshot_marker_enqueued_ = false;
        return false;
    }

    out = std::move(latest_snapshot_event_);
    has_latest_snapshot_ = false;
    snapshot_marker_enqueued_ = false;
    return true;
}

    void ControlLoop::threadMain_()
{
    while (running_) {
        Event e;
        if (!popEvent_(e)) {
            continue;
        }
        if (e.type == Event::Type::Stop) break;

        if (e.type == Event::Type::Snapshot) {
            if (!takeLatestSnapshot_(e)) {
                continue;
            }
        }

        if (e.ts_ms == 0) e.ts_ms = nowMs();

        std::vector<Command> cmds;
        cmds.reserve(8);

        bool consumed_by_maintain = false;
        // if (e.type == Event::Type::HmiWrite) {
        //     consumed_by_maintain = maintain_.onHmiWrite(e.hmi_write, cmds);
        // }

        if (!consumed_by_maintain) {
            engine_.onEvent(e, ctx_, cmds);
        }

        // 只在 ControlLoop 内部复制一份轻量 BMS health 缓存。
        // 这里不调用 Aggregator，不调用 SnapshotDispatcher，不做落盘。
        switch (e.type) {
        case Event::Type::DeviceData:
        case Event::Type::Snapshot:
        case Event::Type::Tick:
            refreshCachedBmsRuntimeHealth_();
            break;
        default:
            break;
        }

        if (!cmds.empty()) {
            dispatcher_.dispatch(cmds);
            if (tx_mirror_cb_) {
                std::set<std::string> emitted_keys;

                auto get_i32 = [](const DeviceData& d, const char* key) -> int32_t {
                    auto it = d.value.find(key);
                    if (it == d.value.end()) return 0;
                    return static_cast<int32_t>(it->second);
                };

                for (const auto& c : cmds) {
                    if (c.type != Command::Type::SendCan) {
                        continue;
                    }

                    if (!c.can.has_tx_mirror) {
                        continue;
                    }

                    const auto& m = c.can.tx_mirror;

                    const std::string key =
                        m.device_name + "|" +
                        std::to_string(get_i32(m, "__can_index")) + "|" +
                        std::to_string(get_i32(m, "ctrl_heartbeat"));

                    if (!emitted_keys.insert(key).second) {
                        continue;
                    }

                    tx_mirror_cb_(m);
                }
            }
        }
    }
}
} // namespace control