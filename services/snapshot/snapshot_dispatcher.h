#pragma once

#include "snapshot_sink.h"

#include <memory>
#include <mutex>
#include <vector>

#include "bms_filesink/bms_snapshot_sink.h"

namespace snapshot {
    struct BmsSnapshot;
}

class SnapshotDispatcher {
public:
    void addSink(std::unique_ptr<SnapshotSink> sink);

    // 由 Aggregator 调用
    void dispatch(const agg::SystemSnapshot& snap);

    // BMS 相关
    void addBmsSink(std::unique_ptr<BmsSnapshotSink> sink);

    // 分发 BMS 大数据快照（与 SystemSnapshot 并行）
    void dispatchBms(const snapshot::BmsSnapshot& snap);

private:
    std::vector<std::unique_ptr<SnapshotSink>> sinks_;
    std::vector<std::unique_ptr<BmsSnapshotSink>> bms_sinks_;

    std::mutex dispatch_mtx_;
    std::mutex bms_dispatch_mtx_;
};