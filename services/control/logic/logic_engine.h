#ifndef ENERGYSTORAGE_LOGIC_ENGINE_H
#define ENERGYSTORAGE_LOGIC_ENGINE_H

#pragma once

#include <string>
#include <vector>

#include "control_events.h"
#include "control_commands.h"
#include "logic_context.h"

#include "bms/bms_command_manager.h"
#include "bms/bms_logic_adapter.h"
#include "bms/bms_fault_mapper.h"
#include "bms/bms_fault_evaluator.h"
#include "./out/model_path_exporter.h"
#include "fault/fault_runtime_mapper.h"
#include "fault/fault_logic_evaluator.h"
class DriverManager;

namespace control {

    class LogicEngine {
    public:
        void init(::DriverManager& drv);
        void onEvent(const Event& e, LogicContext& ctx, std::vector<Command>& out_cmds);

        // 加载 runtime fault 规则（第一版消费 fault_map.jsonl 中的 BMS source/signal/instance）
        bool loadFaultRuntimeMapFile(const std::string& path, std::string* err = nullptr);

        const FaultRuntimeMapper& faultRuntimeMapper() const { return fault_runtime_mapper_; }
        void refreshFromSnapshotOnly(const agg::SystemSnapshot& snap,
                             uint64_t ts_ms,
                             LogicContext& ctx);
        void refreshFaultPagesOnly(uint64_t ts_ms, LogicContext& ctx);
    private:
        // ===================== 事件域拆分 =====================
        void onDeviceData_(const DeviceData& d,
                           uint64_t ts,
                           LogicContext& ctx,
                           std::vector<Command>& out_cmds);

        void onSnapshot_(const SnapshotEvent& s,
                         LogicContext& ctx,
                         std::vector<Command>& out_cmds);

        void onHmiWrite_(const HmiWriteEvent& w,
                         LogicContext& ctx,
                         std::vector<Command>& out_cmds);

        void handleHmiCoilWrite_(const HmiWriteEvent& w,
                         LogicContext& ctx,
                         std::vector<Command>& out_cmds);

        void onHmiButtonClick_(uint16_t addr,
                               LogicContext& ctx,
                               std::vector<Command>& out_cmds);

        void onIoSample_(const IoSampleEvent& s,
                         LogicContext& ctx,
                         std::vector<Command>& out_cmds);

        void onTick_(const TickEvent& t,
                     LogicContext& ctx,
                     std::vector<Command>& out_cmds);

        void onLinkHealth_(const LinkHealthEvent& lh,
                           LogicContext& ctx,
                           std::vector<Command>& out_cmds);

        // ===================== 视图构建 / 输出 =====================
        void rebuildLogicView_(LogicContext& ctx);
        void applyFaultPages_(LogicContext& ctx, uint64_t now_ms);

        // 新增：HMI 下行输出统一收口
        void applyNormalHmi_(LogicContext& ctx);
        void applyFaultHmi_(LogicContext& ctx);
        void applyHmiOutputs_(LogicContext& ctx);

        // ===================== 在线判定 =====================
        static bool isSnapshotItemOnline_(const agg::SystemSnapshot& snap,
                                          const std::string& device_name,
                                          uint64_t now_ms,
                                          uint32_t timeout_ms);

        static bool isBmsInstanceOnline_(const bms::BmsLogicCache& cache,
                                         uint32_t instance_index,
                                         uint64_t now_ms,
                                         uint32_t timeout_ms);

        // ===================== PCU 在线状态 =====================
        void updatePcuOnlineState_(const DeviceData& d, uint64_t ts, LogicContext& ctx);

        // 当前优先按 cabinet_id 分路：3 -> pcu1, 4 -> pcu2
        static bool tryResolvePcuInstance_(const DeviceData& d, uint32_t& out_instance);

        static bool tryGetPcuCabinetId_(const DeviceData& d, uint32_t& out_cabinet_id);
        static bool tryGetPcuHeartbeat_(const DeviceData& d, uint32_t& out_heartbeat);

        // ===================== 第三批：runtime aging =====================
        void updatePcuRuntimeHealth_(LogicContext& ctx, uint64_t now_ms);
        void updateBmsRuntimeHealth_(LogicContext& ctx, uint64_t now_ms);


        static void updateOnePcuRuntimeHealth_(PcuOnlineState& st,
                                               uint64_t now_ms,
                                               uint32_t rx_timeout_ms,
                                               uint32_t hb_stale_timeout_ms);

        static bool calcFreshByTimeout_(uint64_t now_ms,
                                        uint64_t last_ms,
                                        uint32_t timeout_ms);

        static double calcAgeMs_(uint64_t now_ms,
                                 uint64_t last_ms);

        static const char* pcuOfflineReasonText_(int code);
        static const char* bmsOfflineReasonText_(int code);

    private:
        // ===================== BMS 子域 =====================
        bms::BmsLogicAdapter    bms_adapter_;
        bms::BmsFaultEvaluator  bms_fault_evaluator_;
        bms::BmsFaultMapper     bms_fault_mapper_;
        bms::BmsCommandManager  bms_cmd_mgr_;
        bool bms_cmd_mgr_inited_{false};

        // ===================== 通用故障确认层 / runtime 映射 =====================
        fault::FaultLogicEvaluator fault_logic_evaluator_;
        FaultRuntimeMapper         fault_runtime_mapper_;

        // ===================== 调试导出 =====================
        ModelPathExporter model_exporter_;
        bool out2json{true};
    };

} // namespace control

#endif // ENERGYSTORAGE_LOGIC_ENGINE_H