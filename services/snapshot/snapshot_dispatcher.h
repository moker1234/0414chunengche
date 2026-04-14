//
// Created by lxy on 2026/1/24.
//

#ifndef ENERGYSTORAGE_SNAPSHOT_DISPATCHER_H
#define ENERGYSTORAGE_SNAPSHOT_DISPATCHER_H


// services/snapshot/snapshot_dispatcher.h
#pragma once
#include "snapshot_sink.h"
#include <vector>
#include <memory>
#include <mutex>

#include "logger.h"
#include "display/hmi_snapshot_mapper.h"
#include "hmi/hmi_proto.h"

#include "bms_filesink/bms_snapshot_sink.h"
namespace snapshot { struct BmsSnapshot; }

class SnapshotDispatcher {
public:
    void addSink(std::unique_ptr<SnapshotSink> sink);

    // 由 Aggregator 调用
    void dispatch(const agg::SystemSnapshot& snap);

    // bms 相关
    void addBmsSink(std::unique_ptr<BmsSnapshotSink> sink);
    // 分发 BMS 大数据快照（与 SystemSnapshot 并行）
    void dispatchBms(const snapshot::BmsSnapshot& snap);
    // bms 相关


    void bindHmi(HMIProto* hmi) {
        LOGD("[DISP] bindHmi hmi=%p", (void*)hmi);
        hmi_mapper_.bind(hmi);
    }
    // 加载映射文件（建议 AppManager init 时调用一次）
    bool loadHmiMapFile(const std::string& path, std::string* err = nullptr) {
        return hmi_mapper_.loadMapFile(path, err);
    }


    void blockHmiIntReadRange(uint16_t start, uint16_t end);
    void blockHmiBoolReadRange(uint16_t start, uint16_t end);
    void clearHmiBlockedRanges();

    // 旧链路屏蔽
    // 旧链路屏蔽，默认关闭
    void setHmiMappingEnabled(bool en) { hmi_mapping_enabled_ = en; }
    bool hmiMappingEnabled() const { return hmi_mapping_enabled_; }
private:
    std::vector<std::unique_ptr<SnapshotSink>> sinks_;
    std::vector<std::unique_ptr<BmsSnapshotSink>> bms_sinks_; // bms 相关的 sink

    HmiSnapshotMapper hmi_mapper_;

    bool hmi_mapping_enabled_{false};// 旧链路屏蔽，默认关闭

    std::mutex dispatch_mtx_;
};



#endif //ENERGYSTORAGE_SNAPSHOT_DISPATCHER_H