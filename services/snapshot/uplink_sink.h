//
// Created by lxy on 2026/1/24.
//

#ifndef ENERGYSTORAGE_UPLINK_SINK_H
#define ENERGYSTORAGE_UPLINK_SINK_H
#pragma once

#include "snapshot_sink.h"

/**
 * UplinkSink
 *  - 以太网上传（占位）
 *  - 当前版本不做任何事
 */
class UplinkSink : public SnapshotSink {
public:
    void onSnapshot(const agg::SystemSnapshot& snap) override;
};

#endif //ENERGYSTORAGE_UPLINK_SINK_H