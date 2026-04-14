#pragma once

#include "snapshot_sink.h"
#include <map>
#include <string>
#include <chrono>
#include <mutex>

class FileSink : public SnapshotSink {
public:
    explicit FileSink(std::string base_dir = "/home/zlg/running_log",
                      uint64_t min_interval_ms = 100);

    void onSnapshot(const agg::SystemSnapshot& snap) override;

private:
    // ===== 变化检测 =====
    bool hasChanged_(const agg::SystemSnapshot& snap);
    bool deviceChanged_(const std::string& name,
                        const SnapshotItem& item);
    bool healthChanged_(const std::string& name,
                        const SnapshotItem& item);

    // ===== 空调专用 =====
    static std::string airconKeyJson_(const DeviceData& d);
    static bool airconStable_(const DeviceData& d);

    // ===== 最新快照 =====
    bool timeToFlushLatest_() const;
    void writeLatestAtomic_(const std::string& content);

    // ===== 历史快照 =====
    void appendHistory_(const agg::SystemSnapshot& snap);

    // ===== 工具 =====
    static std::string todayDir_();
    static std::string hourFile_();
    static void ensureDir_(const std::string& path);

private:
    std::string base_dir_;
    std::string latest_path_;
    std::string latest_tmp_path_;
    std::string history_dir_;

    uint64_t min_interval_ms_{100};
    std::chrono::steady_clock::time_point last_latest_flush_{};

    bool has_pending_{false};
    agg::SystemSnapshot pending_snap_{};

    // 变化检测 cache（按设备）
    std::map<std::string, std::string> last_device_json_cache_;
    std::map<std::string, std::string> last_device_health_cache_;


    std::mutex mtx_;
};
