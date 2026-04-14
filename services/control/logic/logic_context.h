#ifndef ENERGYSTORAGE_LOGIC_CONTEXT_H
#define ENERGYSTORAGE_LOGIC_CONTEXT_H

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "../protocol/protocol_base.h"
#include "../aggregator/system_snapshot.h"
#include "../normal/normal_hmi_writer.h"

#include "bms/bms_logic_types.h"
#include "../fault/fault_catalog.h"
#include "../fault/fault_center.h"
#include "../fault/fault_page_manager.h"
#include "../fault/fault_condition_engine.h"

class HMIProto;

namespace control {

    // ===== PCU 在线状态 / 运行态健康 =====
    struct PcuOnlineState {
        // -------- 原始观测 --------
        bool seen_once{false};

        uint64_t last_rx_ms{0};          // 最近一次收到该 PCU 状态帧
        uint64_t last_hb_change_ms{0};   // 最近一次心跳变化时间

        uint32_t last_heartbeat{0};      // 最近一次心跳值
        bool has_last_heartbeat{false};  // 是否已有上一次心跳可比较

        uint32_t cabinet_id{0};          // 附加信息，不再作为主分路依据

        // -------- 运行态附加位 --------
        bool estop{false};               // PCU 状态帧中的急停位（Byte1）

        // -------- 心跳诊断 --------
        uint32_t hb_repeat_count{0};     // 连续重复计数
        uint32_t hb_jump_err_count{0};   // 跳变异常计数
        int32_t  last_hb_delta{0};       // 最近一次心跳差值（便于诊断）

        // -------- runtime 判定结果 --------
        bool rx_alive{false};            // 收包仍新鲜
        bool hb_alive{false};            // 心跳仍在推进
        bool online{false};              // 最终在线 = rx_alive && hb_alive

        double last_rx_age_ms{0.0};         // 距离最近一次收包的年龄
        double last_hb_change_age_ms{0.0};  // 距离最近一次心跳变化的年龄

        uint64_t last_online_change_ms{0};

        // 0=None 1=NoData 2=RxTimeout 3=HeartbeatStale
        uint32_t offline_reason_code{0};

        uint64_t last_offline_ms{0};
        uint32_t disconnect_count{0};
        std::string offline_reason_text;
    };
       // ===== HMI 按键状态机缓存 =====
    struct HmiButtonState {
        bool is_down{false};          // 当前是否处于按下态
        bool armed{false};            // 是否已经收到按下，等待松开确认
        uint64_t press_ts_ms{0};      // 最近一次按下时间
        uint64_t release_ts_ms{0};    // 最近一次松开时间
    };

    // ===== UPS runtime / fault 真源 =====
    struct UpsFaultState {
        bool seen_once{false};
        uint64_t last_rx_ms{0};

        bool q1_seen{false};
        bool q6_seen{false};
        bool wa_seen{false};

        bool online{false};
        bool alarm_any{false};
        bool fault_any{false};

        int work_mode{0};
        int battery_low{0};
        int bypass_active{0};
        int ups_fault_code{0};

        // ---- Q1 / WA 基础状态位 ----
        bool mains_abnormal{false};      // bit7
        bool battery_low_state{false};   // bit6
        bool bypass_mode{false};         // bit5
        bool ups_fault_state{false};     // bit4
        bool backup_mode{false};         // bit3
        bool self_test_active{false};    // bit2

        // ---- Q6 warning bits（先按协议低32位展开）----
        uint32_t warning_bits{0};

        bool internal_warning{false};        // bit0
        bool epo_active{false};              // bit1
        bool module_unlock{false};           // bit2
        bool line_loss{false};               // bit3
        bool ipn_loss{false};                // bit4
        bool line_phase_err{false};          // bit5
        bool site_fail{false};               // bit6
        bool bypass_loss{false};             // bit7
        bool bypass_phase_err{false};        // bit8
        bool bat_open{false};                // bit9
        bool bat_low_warning{false};         // bit10
        bool over_chg_warning{false};        // bit11
        bool bat_reverse{false};             // bit12
        bool overload_warning{false};        // bit13
        bool overload_fail{false};           // bit14
        bool fan_lock_warning{false};        // bit15
        bool maintain_on{false};             // bit16
        bool chg_fail{false};                // bit17
        bool error_location{false};          // bit18
        bool turn_on_abnormal{false};        // bit19
        bool redundant_loss{false};          // bit20
        bool module_hotswap_active{false};   // bit21
        bool battery_inform{false};          // bit22
        bool inspection_inform{false};       // bit23
        bool guarantee_inform{false};        // bit24
        bool temp_low_warning{false};        // bit25
        bool temp_high_warning{false};       // bit26
        bool bat_overtemp{false};            // bit27
        bool fan_maint_inform{false};        // bit28
        bool bus_cap_maint_inform{false};    // bit29
        bool sys_over_capacity_warning{false}; // bit30
        bool high_external_warning{false};   // bit31

        // ---- Q6 fault code ----
        bool bus_soft_timeout{false};
        bool bus_over{false};
        bool bus_under{false};
        bool bus_unbalance{false};
        bool bus_short{false};
        bool inv_soft_timeout{false};
        bool inv_volt_high{false};
        bool inv_volt_low{false};
        bool op_volt_short{false};
        bool over_load_fault{false};
        bool over_temperature{false};
        bool comm_line_loss{false};
        bool power_fault{false};
        bool ups_all_fault{false};
        bool battery_abnormal{false};
        bool battery_over_charge_fault{false};
        bool epo_fault{false};
    };

    // ===== Smoke runtime / fault 真源 =====
    struct SmokeFaultState {
        bool seen_once{false};
        uint64_t last_rx_ms{0};

        bool online{false};
        bool alarm_any{false};
        bool fault_any{false};

        double smoke_percent{0.0};
        double temp_c{0.0};

        bool smoke_alarm{false};
        bool temp_alarm{false};

        bool smoke_sensor_fault{false};
        bool smoke_pollution_fault{false};
        bool temp_sensor_fault{false};
    };

    // ===== Gas runtime / fault 真源 =====
    struct GasFaultState {
        bool seen_once{false};
        uint64_t last_rx_ms{0};

        bool online{false};
        bool alarm_any{false};
        bool fault_any{false};

        uint32_t status_code{0};
        double gas_value{0.0};

        bool sensor_fault{false};
        bool low_alarm{false};
        bool high_alarm{false};
    };

    // ===== AirConditioner runtime / fault 真源 =====
    struct AirFaultState {
        bool seen_once{false};
        uint64_t last_rx_ms{0};

        bool online{false};
        bool alarm_any{false};
        bool fault_any{false};

        int run_state{0};
        int power_state{0};
        double indoor_temp_c{0.0};
        double humidity_percent{0.0};

        bool high_temp_alarm{false};
        bool low_temp_alarm{false};
        bool high_humidity_alarm{false};
        bool low_humidity_alarm{false};
        bool coil_freeze_protect{false};
        bool exhaust_high_temp_alarm{false};

        bool coil_temp_sensor_fault{false};
        bool outdoor_temp_sensor_fault{false};
        bool condenser_temp_sensor_fault{false};
        bool indoor_temp_sensor_fault{false};
        bool exhaust_temp_sensor_fault{false};
        bool humidity_sensor_fault{false};

        bool internal_fan_fault{false};
        bool external_fan_fault{false};
        bool compressor_fault{false};
        bool heater_fault{false};
        bool emergency_fan_fault{false};

        bool high_pressure_alarm{false};
        bool low_pressure_alarm{false};
        bool water_alarm{false};
        bool smoke_alarm{false};
        bool gating_alarm{false};

        bool high_pressure_lock{false};
        bool low_pressure_lock{false};
        bool exhaust_lock{false};

        bool ac_over_voltage_alarm{false};
        bool ac_under_voltage_alarm{false};
        bool ac_power_loss{false};
        bool lose_phase_alarm{false};
        bool freq_fault{false};
        bool anti_phase_alarm{false};
        bool dc_over_voltage_alarm{false};
        bool dc_under_voltage_alarm{false};
    };

    // ===== Logic 聚合故障真源 =====
    struct LogicFaultState {
        uint64_t last_eval_ts_ms{0};

        bool any_fault{false};

        bool pcu_any_offline{false};
        bool bms_any_offline{false};
        bool ups_offline{false};
        bool smoke_offline{false};
        bool gas_offline{false};
        bool air_offline{false};

        bool env_any_alarm{false};
        bool system_estop{false};

        // ---- 第六批新增：system / comm 类预留真源 ----
        // 说明：
        // 1) 当前原始代码里还没有稳定来源，这里先把真源位正式挂进 LogicContext
        // 2) 后续批次再分别在 HMI / remote / IO / 外设链路里补实际赋值来源
        bool hmi_comm_fault{false};
        bool remote_comm_fault{false};
        bool sdcard_fault{false};
    };

    // ===== BMS 已确认故障真源（第二批先做通用容器）=====
    struct BmsConfirmedFaultState {
        // key 例子：
        // "BMS_1.umax_ge_3p9"
        // "BMS_2.soc_le_20"
        // "BMS_4.hv_delta_abnormal"
        std::unordered_map<std::string, bool> signals;
    };

    // ===== 通用/VCU/logic 已确认故障真源（第二批先做通用容器）=====
    struct ConfirmedFaultState {
        // key 例子：
        // "logic.pcu1_offline"
        // "logic.ups_fault"
        // "logic.env_any_alarm"
        std::unordered_map<std::string, bool> signals;
    };

    struct LogicContext {
        // ===== 最近事件时间 =====
        uint64_t last_event_ts{0};

        // ===== 最新设备数据缓存（按 device_name）=====
        std::unordered_map<std::string, DeviceData> latest_device;

        // ===== 最新系统快照（Aggregator -> logic）=====
        agg::SystemSnapshot latest_snapshot{};

        // ===== 逻辑显示视图（最终供 HMI 显示）=====
        nlohmann::json logic_view = nlohmann::json::object();

        // ===== BMS control 专属缓存 =====
        bms::BmsLogicCache bms_cache;

        // ===== PCU 在线状态 =====
        PcuOnlineState pcu0_state;
        PcuOnlineState pcu1_state;

        // ===== 故障页目录与运行态 =====
        FaultCatalog fault_catalog;
        FaultCenter fault_center;
        FaultPageManager fault_pages;
        bool fault_map_loaded{false};

        // ===== 非 BMS 设备 / logic 故障真源缓存 =====
        UpsFaultState   ups_faults;
        SmokeFaultState smoke_faults;
        GasFaultState   gas_faults;
        AirFaultState   air_faults;
        LogicFaultState logic_faults;

        // ===== 统一故障条件确认引擎（第二批新增）=====
        fault::FaultConditionEngine fault_cond_engine;

        // ===== 已确认故障真源缓存（供后续 evaluator / mapper 使用）=====
        BmsConfirmedFaultState bms_confirmed_faults;
        ConfirmedFaultState    confirmed_faults;

        // ===== IO 缓存 =====
        uint64_t last_io_ts{0};
        uint64_t di_bits{0};
        std::vector<double> ai;

        // ===== HMI 控制缓存（可用于“按钮沿触发/锁存”）=====
        uint64_t last_hmi_write_ts{0};
        uint16_t last_hmi_addr{0};
        uint16_t last_hmi_value{0};

        // 第八批：HMI 弱在线观测
        bool hmi_seen_once{false};
        uint32_t hmi_comm_timeout_ms{10000};

        // 第九批：remote 预留接口
        //
        // 说明：
        // 1) 当前项目里还没有 remote 的具体程序与稳定 device_name；
        // 2) 这里先把“remote 最近一次数据到达”的观测字段挂进 context；
        // 3) 后续 remote 程序只要把 DeviceData.device_name 设成约定名字，
        //    就能通过 logic_device_data.cpp 自动接上。
        bool remote_seen_once{false};
        uint64_t last_remote_rx_ts{0};
        uint32_t remote_comm_timeout_ms{10000};

        // 按地址缓存 HMI coil / reg 当前值
        std::unordered_map<uint16_t, uint16_t> hmi_coil_state;
        std::unordered_map<uint16_t, uint16_t> hmi_reg_state;

        // 按地址缓存“按下->松开才生效”的按键状态机
        std::unordered_map<uint16_t, HmiButtonState> hmi_button_states;



        // ===== 系统模式（示例）=====
        enum class Mode : uint8_t {
            Auto = 0,
            Manual = 1,
        };
        Mode mode{Mode::Auto};

        bool e_stop_latched{false};

        // ===== HMI（写 AddressTable）=====
        HMIProto* hmi{nullptr};

        // ===== 普通变量输出 =====
        normal::NormalHmiWriter normal_writer;
    };

} // namespace control

#endif // ENERGYSTORAGE_LOGIC_CONTEXT_H