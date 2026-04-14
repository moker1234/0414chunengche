//
// Created by lxy on 2026/2/14.
//

#include "bms_file_sink.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <filesystem>

#include "logger.h"
#include "../../aggregator/bms/bms_snapshot.h"

namespace fs = std::filesystem;

BmsFileSink::BmsFileSink(Config cfg)
    : cfg_(std::move(cfg))
{
    latest_path_  = cfg_.base_dir + "/latest.json";
    history_path_ = cfg_.base_dir + "/history.jsonl";
    ensureDirs();
}

uint64_t BmsFileSink::nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool BmsFileSink::ensureDirs()
{
    try {
        fs::create_directories(cfg_.base_dir);
        return true;
    } catch (const std::exception& e) {
        LOGERR("[BMS_FILE] create dir failed: %s err=%s", cfg_.base_dir.c_str(), e.what());
        return false;
    }
}

void BmsFileSink::rollHistoryIfNeeded()
{
    if (cfg_.history_roll_bytes == 0) return;

    try {
        if (!fs::exists(history_path_)) return;
        const auto sz = fs::file_size(history_path_);
        if (sz < cfg_.history_roll_bytes) return;

        auto t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                      tm.tm_hour, tm.tm_min, tm.tm_sec);

        const std::string rolled = cfg_.base_dir + "/history_" + buf + ".jsonl";
        fs::rename(history_path_, rolled);
        LOGI("[BMS_FILE] history rolled: %s -> %s", history_path_.c_str(), rolled.c_str());
    } catch (const std::exception& e) {
        LOGERR("[BMS_FILE] roll history failed: %s", e.what());
    }
}

void BmsFileSink::writeLatest(const snapshot::BmsSnapshot& snap)
{
    try {
        const std::string tmp = latest_path_ + ".tmp";
        std::ofstream ofs(tmp, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) {
            LOGERR("[BMS_FILE] open latest tmp failed: %s", tmp.c_str());
            return;
        }

        ofs << snap.toJson().dump(cfg_.json_indent) << "\n";
        ofs.close();

        fs::rename(tmp, latest_path_);
    } catch (const std::exception& e) {
        LOGERR("[BMS_FILE] write latest failed: %s", e.what());
    }
}

void BmsFileSink::appendHistory(const snapshot::BmsSnapshot& snap)
{
    if (cfg_.history_interval_ms == 0) return;

    try {
        rollHistoryIfNeeded();

        std::ofstream ofs(history_path_, std::ios::out | std::ios::app);
        if (!ofs.is_open()) {
            LOGERR("[BMS_FILE] open history failed: %s", history_path_.c_str());
            return;
        }

        nlohmann::json line;
        if (cfg_.history_include_summary) {
            line = snap.toJson();
        } else {
            line["ts_ms"] = snap.ts_ms;
            line["items"] = snap.toJsonItemsOnly();
        }

        ofs << line.dump(0) << "\n";
        ofs.close();
    } catch (const std::exception& e) {
        LOGERR("[BMS_FILE] append history failed: %s", e.what());
    }
}

void BmsFileSink::onBmsSnapshot(const snapshot::BmsSnapshot& snap)
{
    (void)ensureDirs();

    const uint64_t now = nowMs();

    if (last_latest_ms_ == 0 || (now - last_latest_ms_) >= cfg_.latest_interval_ms) {
        writeLatest(snap);
        last_latest_ms_ = now;
    }

    if (cfg_.history_interval_ms > 0) {
        if (last_hist_ms_ == 0 || (now - last_hist_ms_) >= cfg_.history_interval_ms) {
            appendHistory(snap);
            last_hist_ms_ = now;
        }
    }
}