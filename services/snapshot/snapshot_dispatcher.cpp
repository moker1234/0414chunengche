#include "snapshot_dispatcher.h"

void SnapshotDispatcher::addSink(std::unique_ptr<SnapshotSink> sink)
{
    std::lock_guard<std::mutex> lock(dispatch_mtx_);
    sinks_.push_back(std::move(sink));
}

void SnapshotDispatcher::dispatch(const agg::SystemSnapshot& snap)
{
    std::lock_guard<std::mutex> lock(dispatch_mtx_);

    for (auto& s : sinks_) {
        if (s) {
            s->onSnapshot(snap);
        }
    }
}

void SnapshotDispatcher::addBmsSink(std::unique_ptr<BmsSnapshotSink> sink)
{
    std::lock_guard<std::mutex> lock(bms_dispatch_mtx_);
    bms_sinks_.push_back(std::move(sink));
}

void SnapshotDispatcher::dispatchBms(const snapshot::BmsSnapshot& snap)
{
    std::lock_guard<std::mutex> lock(bms_dispatch_mtx_);

    for (auto& s : bms_sinks_) {
        if (s) {
            s->onBmsSnapshot(snap);
        }
    }
}