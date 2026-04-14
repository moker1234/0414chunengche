//
// Created by lxy on 2026/2/14.
//

#ifndef ENERGYSTORAGE_BMS_SNAPSHOT_SINK_H
#define ENERGYSTORAGE_BMS_SNAPSHOT_SINK_H

#pragma once

#include <memory>

namespace snapshot {
    struct BmsSnapshot;
}

/**
 * BMS 专用 Sink 接口：
 * - 与现有 SnapshotSink（SystemSnapshot）并行
 * - 专门用于处理 BmsSnapshot（大数据）
 */
class BmsSnapshotSink {
public:
    virtual ~BmsSnapshotSink() = default;

    /** 收到一份 BMS 快照（通常是最新全量） */
    virtual void onBmsSnapshot(const snapshot::BmsSnapshot& snap) = 0;
};

using BmsSnapshotSinkPtr = std::shared_ptr<BmsSnapshotSink>;


#endif //ENERGYSTORAGE_BMS_SNAPSHOT_SINK_H
