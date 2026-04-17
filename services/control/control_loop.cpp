// services/control/control_loop.cpp
#include "control_loop.h"

#include "../utils/logger/logger.h"
#include "../protocol/rs485/hmi/hmi_proto.h"
#include "../snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h"

namespace control {

ControlLoop::ControlLoop(::DriverManager& drv, parser::ProtocolParserThread& parser)
    : dispatcher_(drv, parser)
{
    engine_.init(drv);
}

ControlLoop::~ControlLoop()
{
    stop();
}

void ControlLoop::bindHmi(HMIProto* hmi)
{
    ctx_.hmi = hmi;
}

bool ControlLoop::loadNormalMapFile(const std::string& path, std::string* err)
{
    return ctx_.normal_writer.loadMapFile(path, err);
}

bool ControlLoop::loadFaultMapFile(const std::string& path, std::string* err)
{
    // 1) 先加载故障目录（FaultCatalog 用）
    if (!ctx_.fault_catalog.loadJsonl(path, err)) {
        ctx_.fault_map_loaded = false;
        return false;
    }

    ctx_.fault_center.bindCatalog(&ctx_.fault_catalog);
    ctx_.fault_pages.bindCenter(&ctx_.fault_center);

    // 2) 再加载 runtime 规则（FaultRuntimeMapper 用）
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
            "stats{total=%zu accepted=%zu skip_empty=%zu skip_unsupported=%zu}",
            path.c_str(),
            mapper.rules().size(),
            cnt_bms, cnt_pcu, cnt_ups, cnt_smoke, cnt_gas, cnt_air, cnt_logic, cnt_other,
            stats.total_items,
            stats.accepted_rules,
            stats.skipped_no_source_or_signal,
            stats.skipped_unsupported_source
        );
    }

    return true;
}

void ControlLoop::bindFaultDb(SqliteFaultSink* db)
{
    ctx_.fault_center.bindFaultDb(db);
}

void ControlLoop::restoreFaultHistory(const std::vector<FaultHistoryDbRecord>& rows)
{
    ctx_.fault_center.restoreHistoryFromDb(rows);

    if (ctx_.hmi && ctx_.fault_map_loaded) {
        ctx_.fault_center.flushToHmi(*ctx_.hmi);
    }
}

    void ControlLoop::refreshFromLatestSnapshot(const agg::SystemSnapshot& snap, uint64_t ts_ms)
{
    Event e;
    e.type = Event::Type::Snapshot;
    e.ts_ms = ts_ms;
    e.snapshot.ts_ms = ts_ms;
    e.snapshot.snap = snap;

    post(std::move(e));
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

    q_.push(Event::makeStop());

    if (th_.joinable()) th_.join();
    LOG_SYS_I("[CTRL] ControlLoop stopped");
}

    void ControlLoop::post(const Event& e)
{
    if (e.type == Event::Type::Snapshot) {
        postSnapshotLatest_(e);
        return;
    }

    q_.push(e);
}

    void ControlLoop::post(Event&& e)
{
    if (e.type == Event::Type::Snapshot) {
        postSnapshotLatest_(std::move(e));
        return;
    }
    q_.push(std::move(e));
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
        q_.push(std::move(marker));
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
        q_.push(std::move(marker));
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
        Event e = q_.pop();
        if (e.type == Event::Type::Stop) break;

        if (e.type == Event::Type::Snapshot) {
            if (!takeLatestSnapshot_(e)) {
                continue;
            }
        }

        if (e.ts_ms == 0) e.ts_ms = nowMs();

        std::vector<Command> cmds;
        cmds.reserve(8);

        engine_.onEvent(e, ctx_, cmds);

        if (!cmds.empty()) {
            dispatcher_.dispatch(cmds);
        }
    }
}

} // namespace control