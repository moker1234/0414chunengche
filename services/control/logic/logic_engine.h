#ifndef ENERGYSTORAGE_LOGIC_ENGINE_H
#define ENERGYSTORAGE_LOGIC_ENGINE_H

#pragma once

#include <array>
#include <cstdint>
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

#include "business/business_engine.h"
#include "business/business_engine.h"
class DriverManager;

namespace control {

    class LogicEngine {
    public:
        void init(::DriverManager& drv);
        void onEvent(const Event& e, LogicContext& ctx, std::vector<Command>& out_cmds);

        // 加载 runtime fault 规则（第一版消费 fault_map.jsonl 中的 BMS source/signal/instance）
        bool loadFaultRuntimeMapFile(const std::string& path, std::string* err = nullptr);

        const FaultRuntimeMapper& faultRuntimeMapper() const { return fault_runtime_mapper_; }

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
                               const std::string& path,
                               const std::string& name,
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

        // ===== PCU TX：长期方案，控制面统一发送 =====
        struct PcuTxRuntimeCfg {
            bool valid{false};

            // system.json
            int can_index{-1};
            uint8_t pcu_instance{0}; // 1=PCU1, 2=PCU2

            bool tx_enable{false};
            bool send_ctrl{true};
            bool send_status{true};

            uint32_t period_ms{200};
            uint64_t next_due_ms{0};

            uint32_t id_emu_ctrl{0x1801A0E0};
            uint32_t id_emu_status{0x1802A0E0};

            uint8_t heartbeat{0};

            // 配置默认值，作为业务真源缺失时的兜底
            uint8_t ctrl_enable_default{1};
            uint8_t plug_default{0};
            uint8_t estop_default{0};
            uint8_t batt1_estop_default{0};
            uint8_t batt2_estop_default{0};

            // PCU 协议状态帧 Byte4/5，当前项目先固定 2
            uint8_t batt1_branches{2};
            uint8_t batt2_branches{2};
        };

        void loadPcuTxRuntimeCfg_();
        void emitPcuPeriodicCommands_(LogicContext& ctx,
                                      uint64_t now_ms,
                                      std::vector<Command>& out_cmds);


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
        // 新增：HMI 弱在线观测
        void feedHmiAlive(uint64_t now_ms, LogicContext& ctx);

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
        uint32_t bms_cmd_period_ms_{100};
        bool bms_cmd_tx_enabled_{true};

        // ===================== BMS 子域 =====================
        bms::BmsLogicAdapter    bms_adapter_;
        bms::BmsFaultEvaluator  bms_fault_evaluator_;
        bms::BmsFaultMapper     bms_fault_mapper_;
        bms::BmsCommandManager  bms_cmd_mgr_;
        bool bms_cmd_mgr_inited_{false};

        std::array<PcuTxRuntimeCfg, 2> pcu_tx_cfg_{};
        bool pcu_tx_cfg_loaded_{false};

        // ===================== 通用故障确认层 / runtime 映射 =====================
        fault::FaultLogicEvaluator fault_logic_evaluator_;
        FaultRuntimeMapper         fault_runtime_mapper_;

        // ===================== 调试导出 =====================
        ModelPathExporter model_exporter_;
        bool out2json{true};

        // ===================== 业务逻辑 =====================
        business::BusinessEngine business_engine_;
    };

} // namespace control

#endif // ENERGYSTORAGE_LOGIC_ENGINE_H