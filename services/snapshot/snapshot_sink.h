//
// Created by lxy on 2026/1/24.
//

#ifndef ENERGYSTORAGE_SNAPSHOT_SINK_H
#define ENERGYSTORAGE_SNAPSHOT_SINK_H
// services/snapshot/snapshot_sink.h
#pragma once
#include "../aggregator/system_snapshot.h"

class SnapshotSink {
public:
    virtual ~SnapshotSink() = default;

    // snapshot 发生变化时调用
    virtual void onSnapshot(const agg::SystemSnapshot& snap) = 0;
};

#endif //ENERGYSTORAGE_SNAPSHOT_SINK_H