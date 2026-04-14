#include "file_sink.h"

#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <ctime>

#include "../../utils/logger/logger.h"
#include "../aggregator/system_snapshot.h"

using Clock = std::chrono::steady_clock;

/* ================= health key ================= */

static std::string healthKeyJson_(const SnapshotItem& item) {
    nlohmann::json j;
    j["state"]                  = (int)item.health;
    j["online"]                 = item.online;
    j["last_ok_ms"]             = item.last_ok_ms;
    j["disconnect_window_ms"]   = item.disconnect_window_ms;
    j["last_offline_ms"]        = item.last_offline_ms;
    j["disconnect_count"]       = item.disconnect_count;
    return j.dump();
}

/* ================= 工具函数 ================= */

void FileSink::ensureDir_(const std::string& path) {
    ::mkdir(path.c_str(), 0755);
}

std::string FileSink::todayDir_() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    char buf[32];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02d",
                  tm.tm_year + 1900,
                  tm.tm_mon + 1,
                  tm.tm_mday);
    return buf;
}

std::string FileSink::hourFile_() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    char buf[32];
    std::snprintf(buf, sizeof(buf),
                  "hour-%02d.jsonl",
                  tm.tm_hour);
    return buf;
}

/* ================= FileSink ================= */

FileSink::FileSink(std::string base_dir,
                   uint64_t min_interval_ms)
    : base_dir_(std::move(base_dir)),
      min_interval_ms_(min_interval_ms) {

    latest_path_     = base_dir_ + "/latest.json";
    latest_tmp_path_ = latest_path_ + ".tmp";
    history_dir_     = base_dir_ + "/history";

    ensureDir_(base_dir_);
    ensureDir_(history_dir_);

    last_latest_flush_ =
        Clock::now() - std::chrono::milliseconds(min_interval_ms_);
}

/* ================= 变化检测 ================= */

bool FileSink::hasChanged_(const agg::SystemSnapshot& snap) {
    bool changed = false;

    for (const auto& [name, item] : snap.items) {
        if (deviceChanged_(name, item) ||
            healthChanged_(name, item)) {
            changed = true;
        }
    }
    return changed;
}

bool FileSink::deviceChanged_(const std::string& name,
                              const SnapshotItem& item) {
    std::string cur;

    /* ---------- AirConditioner：仍用精简 key ---------- */
    if (name == "AirConditioner") {
        cur = airconKeyJson_(item.data);
    }
    /* ---------- UPS：使用分组后的数据 ---------- */
    else if (name == "UPS") {
        nlohmann::json j;
        for (const auto& [cmd, grp] : item.ups_groups) {
            j[cmd] = {
                {"num", grp.num},
                {"value", grp.value},
                {"status", grp.status},
                {"ts_ms", grp.ts_ms}
            };
        }
        cur = j.dump();
    }
    /* ---------- 其他设备：使用 device data ---------- */
    else {
        nlohmann::json j;
        j["num"]    = item.data.num;
        j["status"] = item.data.status;
        cur = j.dump();
    }

    auto it = last_device_json_cache_.find(name);
    if (it == last_device_json_cache_.end() || it->second != cur) {
        last_device_json_cache_[name] = cur;
        return true;
    }
    return false;
}

bool FileSink::healthChanged_(const std::string& name,
                              const SnapshotItem& item) {
    std::string cur = healthKeyJson_(item);

    auto it = last_device_health_cache_.find(name);
    if (it == last_device_health_cache_.end() || it->second != cur) {
        last_device_health_cache_[name] = cur;
        return true;
    }
    return false;
}

/* ================= 空调专用 ================= */

std::string FileSink::airconKeyJson_(const DeviceData& d) {
    nlohmann::json j;
    j["run.overall"]   = d.num.count("run.overall") ? d.num.at("run.overall") : 0;
    j["temp.indoor_c"] = d.num.count("temp.indoor_c") ? d.num.at("temp.indoor_c") : 0;
    j["alarm.any"]     = d.status.count("alarm.any") ? d.status.at("alarm.any") : 0;
    j["remote.power"]  = d.num.count("remote.power") ? d.num.at("remote.power") : 0;
    return j.dump();
}

bool FileSink::airconStable_(const DeviceData& d) {
    return d.num.count("temp.indoor_c") &&
           d.num.count("run.overall") &&
           d.status.count("alarm.any");
}

/* ================= 最新快照 ================= */

bool FileSink::timeToFlushLatest_() const {
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - last_latest_flush_).count();
    return elapsed >= (long long)min_interval_ms_;
}

void FileSink::writeLatestAtomic_(const std::string& content) {
    {
        std::ofstream ofs(latest_tmp_path_, std::ios::trunc);
        if (!ofs.is_open()) return;
        ofs << content;
        ofs.flush();
    }

    int fd = ::open(latest_tmp_path_.c_str(), O_RDONLY);
    if (fd >= 0) {
        ::fsync(fd);
        ::close(fd);
    }

    ::rename(latest_tmp_path_.c_str(), latest_path_.c_str());
    last_latest_flush_ = Clock::now();
}

/* ================= 历史快照 ================= */

void FileSink::appendHistory_(const agg::SystemSnapshot& snap) {
    const std::string day_dir =
        history_dir_ + "/" + todayDir_();
    ensureDir_(day_dir);

    const std::string file =
        day_dir + "/" + hourFile_();

    std::ofstream ofs(file, std::ios::out | std::ios::app);
    if (!ofs.is_open()) return;

    ofs << snap.toJson().dump() << "\n";
}

/* ================= onSnapshot ================= */

void FileSink::onSnapshot(const agg::SystemSnapshot& snap) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!hasChanged_(snap)) return;

    bool allow_history = true;
    auto it = snap.items.find("AirConditioner");
    if (it != snap.items.end()) {
        allow_history = airconStable_(it->second.data);
    }

    if (allow_history) {
        appendHistory_(snap);
    }

    pending_snap_ = snap;
    has_pending_ = true;

    if (!timeToFlushLatest_()) return;

    if (has_pending_) {
        writeLatestAtomic_(pending_snap_.toJson().dump(2));
        has_pending_ = false;
    }
}
