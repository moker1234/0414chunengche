//
// Created by lxy on 2026/2/14.
//

#ifndef ENERGYSTORAGE_BMS_FILE_SINK_H
#define ENERGYSTORAGE_BMS_FILE_SINK_H

#pragma once

#include <cstdint>
#include <string>

#include "bms_snapshot_sink.h"

namespace snapshot {
    struct BmsSnapshot;
}

/**
 * BMS 专用 FileSink（大数据）：
 * - 独立目录：默认 /home/zlg/running_log/bms
 * - latest.json：节流写入（默认 1000ms）
 * - history.jsonl：节流追加（默认 1000ms），每行一个 JSON
 *
 * 第一批目标：
 * - latest.json 直接输出 BmsSnapshot::toJson()
 * - history.jsonl 默认输出：
 *   {
 *     "ts_ms": ...,
 *     "items": { ... }
 *   }
 */
class BmsFileSink final : public BmsSnapshotSink {
public:
    struct Config {
        std::string base_dir{"/home/zlg/running_log/bms"};

        uint32_t latest_interval_ms{1000};
        uint32_t history_interval_ms{1000};

        // true=history 写全量；false=history 写 ts_ms + items（更省空间）
        bool history_include_summary{false};

        int json_indent{0};

        uint64_t history_roll_bytes{50ull * 1024 * 1024}; // 50MB
    };

    explicit BmsFileSink(Config cfg);

    void onBmsSnapshot(const snapshot::BmsSnapshot& snap) override;

private:
    static uint64_t nowMs();

    bool ensureDirs();
    void writeLatest(const snapshot::BmsSnapshot& snap);
    void appendHistory(const snapshot::BmsSnapshot& snap);

    void rollHistoryIfNeeded();

private:
    Config cfg_;

    uint64_t last_latest_ms_{0};
    uint64_t last_hist_ms_{0};

    std::string latest_path_;
    std::string history_path_;
};

#endif // ENERGYSTORAGE_BMS_FILE_SINK_H