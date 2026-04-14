//
// Created by lxy on 2026/2/14.
//

#include "bms_worker.h"

#include <chrono>
#include <linux/can.h>   // can_frame, CAN_EFF_FLAG
#include <cstring>

#include "bms_proto.h"   // BmsProto
#include "logger.h"      // LOGI/LOGD/LOGERR

// 你的项目里 DeviceData 在 services/device/device_base.h
// #include "device_base.h"
#include "protocol_base.h"

namespace proto::bms {

BmsWorker::BmsWorker(BmsQueue& q, BmsProto& proto, Config cfg)
    : q_(q), proto_(proto), cfg_(cfg)
{
}

BmsWorker::~BmsWorker()
{
    stop();
}

uint64_t BmsWorker::nowMs_()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void BmsWorker::fillCanFrame_(const BmsFrame& in, can_frame& out)
{
    std::memset(&out, 0, sizeof(out));
    // SocketCAN 扩展帧需要带 CAN_EFF_FLAG
    out.can_id  = (in.id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
    out.can_dlc = in.dlc;
    for (int i = 0; i < 8; ++i) out.data[i] = in.data[(size_t)i];
}

void BmsWorker::start()
{
    if (running_.exchange(true)) return;
    th_ = std::thread([this] { threadMain_(); });
    LOGI("[BMS_WORKER] start");
}

void BmsWorker::stop()
{
    if (!running_.exchange(false)) return;
    // 唤醒 wait
    q_.push(BmsFrame{});
    if (th_.joinable()) th_.join();
    LOGI("[BMS_WORKER] stop");
}

void BmsWorker::threadMain_()
{
    last_tick_ms_ = 0;

    while (running_.load()) {
        // 等待数据或超时
        (void)q_.waitForData(cfg_.wait_timeout_ms);

        // 批量取走最新帧
        auto frames = q_.popAll();
        // LOG_COMM_D("[BMS][WORKER] popAll size=%zu", frames.size());
        if (!running_.load()) break;

        // 限制单轮处理量（可选）
        if (cfg_.max_frames_per_round > 0 && frames.size() > cfg_.max_frames_per_round) {
            frames.resize(cfg_.max_frames_per_round);
        }

        // 解析
        for (const auto& f : frames) {
           //  LOG_COMM_D("[BMS][WORKER] raw id29=0x%08X dlc=%d",
           // (unsigned)f.id29, (int)f.dlc);
            // stop() 注入的空帧可能在这里出现，过滤掉
            if (f.id29 == 0 && f.dlc == 8) {
                continue;
            }

            can_frame fr{};
            fillCanFrame_(f, fr);

            DeviceData d;
            const bool ok = proto_.parse(fr, d);
            // LOG_COMM_D("[BMS][WORKER][PARSE] id=0x%08X ok=%d dev=%s",
            //            (unsigned)(fr.can_id & CAN_EFF_MASK),
            //            ok ? 1 : 0,
            //            ok ? d.device_name.c_str() : "NA");
            if (ok) {
                if (on_device_data_) on_device_data_(d);
            }
        }

        // 可选 tick：用于 worker 内部节流触发 dispatchBms/落盘
        if (on_tick_ && cfg_.dispatch_throttle_ms > 0) {
            const uint64_t now = nowMs_();
            if (last_tick_ms_ == 0 || (now - last_tick_ms_) >= cfg_.dispatch_throttle_ms) {
                last_tick_ms_ = now;
                on_tick_();
            }
        }
    }
}

} // namespace proto::bms
