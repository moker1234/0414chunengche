//
// Created by lxy on 2026/1/24.
//

// services/snapshot/snapshot_dispatcher.cpp
#include "snapshot_dispatcher.h"

#include "logger.h"

void SnapshotDispatcher::addSink(std::unique_ptr<SnapshotSink> sink) {
    sinks_.push_back(std::move(sink));
}

void SnapshotDispatcher::dispatch(const agg::SystemSnapshot& snap) { // 分发系统快照

    std::lock_guard<std::mutex> lock(dispatch_mtx_);
    if (hmi_mapping_enabled_) {
        hmi_mapper_.onSnapshot(snap);
    }
    for (auto& s : sinks_) {
        s->onSnapshot(snap);
    }
}

// bms 相关
void SnapshotDispatcher::addBmsSink(std::unique_ptr<BmsSnapshotSink> sink) {
    bms_sinks_.push_back(std::move(sink));
}
void SnapshotDispatcher::dispatchBms(const snapshot::BmsSnapshot& snap) {
    for (auto& s : bms_sinks_) {
        s->onBmsSnapshot(snap);
    }
}


// services/snapshot/snapshot_dispatcher.cpp

void SnapshotDispatcher::blockHmiIntReadRange(uint16_t start, uint16_t end)
{
    hmi_mapper_.addBlockedIntReadRange(start, end);
}

void SnapshotDispatcher::blockHmiBoolReadRange(uint16_t start, uint16_t end)
{
    hmi_mapper_.addBlockedBoolReadRange(start, end);
}

void SnapshotDispatcher::clearHmiBlockedRanges()
{
    hmi_mapper_.clearBlockedRanges();
}
