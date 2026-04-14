//
// Created by lxy on 2026/1/24.
//

#ifndef ENERGYSTORAGE_DISPLAY_SINK_H
#define ENERGYSTORAGE_DISPLAY_SINK_H
#pragma once

#include "snapshot_sink.h"
#include "../output/display_rs485_session.h"

/**
 * DisplaySink
 *  - Snapshot → 显示屏
 *  - 实际发送由 DisplayRs485Session 完成
 */
class DisplaySink : public SnapshotSink {
public:
    explicit DisplaySink(DisplayRs485Session& session);

    void onSnapshot(const agg::SystemSnapshot& snap) override;

private:
    DisplayRs485Session& session_;
};

#endif //ENERGYSTORAGE_DISPLAY_SINK_H