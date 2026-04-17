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
        // bool mains_abnormal{false};      // bit7
        bool battery_low_state{false};   // bit6
        bool bypass_mode{false};         // bit5
        bool ups_fault_state{false};     // bit4
        bool backup_mode{false};         // bit3
        bool self_test_active{false};    // bit2

        // ---- Q6 warning bits（先按协议低32位展开）----
        uint32_t warning_bits{0};

// ------------------ 替换以下内容 ------------------
        // === 32 个 Warning Bits (警告位) ===
        bool internal_warning = false;
        bool epo_active = false;
        bool module_unlock = false;
        bool mains_abnormal = false;
        bool neutral_lost = false;
        bool mains_phase_error = false;
        bool ln_reverse = false;
        bool bypass_abnormal = false;
        bool bypass_phase_error = false;
        bool battery_not_connected = false;
        bool battery_low_warning = false;
        bool battery_overcharge = false;
        bool battery_reverse = false;
        bool overload_warning = false;
        bool overload_alarm = false;
        bool fan_fault = false;
        bool bypass_cover_open = false;
        bool charger_fault = false;
        bool position_error = false;
        bool boot_condition_not_met = false;
        bool redundancy_lost = false;
        bool module_loose = false;
        bool battery_maint_due = false;
        bool inspection_maint_due = false;
        bool warranty_maint_due = false;
        bool temp_low_warning = false;
        bool temp_high_warning = false;
        bool battery_overtemp = false;
        bool fan_maint_due = false;
        bool bus_cap_maint_due = false;
        bool system_overload = false;
        bool reserved_warning = false;

        // === 60 个 Fault Codes (严重故障) ===
        bool bus_overvoltage_fault = false;
        bool bus_undervoltage_fault = false;
        bool bus_imbalance_fault = false;
        bool bus_short_circuit = false;
        bool inv_softstart_timeout = false;
        bool inv_overvoltage_fault = false;
        bool inv_undervoltage_fault = false;
        bool output_short_circuit = false;
        bool r_inv_short_circuit = false;
        bool s_inv_short_circuit = false;
        bool t_inv_short_circuit = false;
        bool rs_short_circuit = false;
        bool st_short_circuit = false;
        bool tr_short_circuit = false;
        bool reverse_power_fault = false;
        bool r_reverse_power_fault = false;
        bool s_reverse_power_fault = false;
        bool t_reverse_power_fault = false;
        bool total_reverse_power_fault = false;
        bool current_imbalance_fault = false;
        bool overload_fault = false;
        bool overtemp_fault = false;
        bool inv_relay_fail_close = false;
        bool inv_relay_stuck = false;
        bool mains_scr_fault = false;
        bool battery_scr_fault = false;
        bool bypass_scr_fault = false;
        bool rectifier_fault = false;
        bool input_overcurrent_fault = false;
        bool wiring_error = false;
        bool comm_cable_disconnected = false;
        bool host_cable_fault = false;
        bool can_comm_fault = false;
        bool sync_signal_fault = false;
        bool power_supply_fault = false;
        bool all_fan_fault = false;
        bool dsp_error = false;
        bool charger_softstart_timeout = false;
        bool all_module_fault = false;
        bool mains_ntc_open_fault = false;
        bool mains_fuse_open_fault = false;
        bool output_imbalance_fault = false;
        bool input_mismatch_fault = false;
        bool eeprom_data_lost = false;
        bool mains_support_failed = false;
        bool power_failed = false;
        bool system_overload_fault = false;
        bool ads7869_error = false;
        bool bypass_mode_no_op = false;
        bool op_breaker_off_parallel = false;
        bool r_bus_fuse_fault = false;
        bool s_bus_fuse_fault = false;
        bool t_bus_fuse_fault = false;
        bool ntc_fault = false;
        bool parallel_cable_fault = false;
        bool battery_fault = false;
        bool frequent_overcurrent_fault = false;
        bool battery_overcharge_fault = false;
        bool battery_overcharge_persist = false;
        bool epo_critical_fault = false;
        // --------------------------------------------------
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
        uint64_t last_hmi_comm_ts{0};
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