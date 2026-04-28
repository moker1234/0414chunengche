#ifndef ENERGYSTORAGE_LOGIC_CONTEXT_H
#define ENERGYSTORAGE_LOGIC_CONTEXT_H

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

#include "../protocol/protocol_base.h"
#include "../aggregator/system_snapshot.h"
#include "../normal/normal_hmi_writer.h"

#include "bms/bms_logic_types.h"
#include "../fault/fault_catalog.h"
#include "../fault/fault_center.h"
#include "../fault/fault_page_manager.h"
#include "../fault/fault_condition_engine.h"
#include "../fault/fault_history_cache.h"

#include "../../normal/hmi_map_model.h"

class HMIProto;

namespace control {

    // ===== PCU 在线状态 / 运行态健康 =====
    struct PcuOnlineState {
        // -------- 原始观测 --------
        bool seen_once{false};

        // 最近一次收到该 PCU 状态帧 0x1801EFA0
        uint64_t last_rx_ms{0};

        // 最近一次 heartbeat 发生变化的时间
        uint64_t last_hb_change_ms{0};

        // PCU 状态帧 Byte0：心跳
        uint32_t last_heartbeat{0};
        bool has_last_heartbeat{false};

        // PCU 状态帧 Byte2：PCU状态
        // 0=待机，1=充电，2=故障，其他值只做诊断
        uint32_t pcu_state{0};
        bool pcu_state_valid{false};

        // PCU 状态帧 Byte3：柜编号
        // 只作为附加信息，不再作为 PCU1/PCU2 主分路依据
        uint32_t cabinet_id{0};

        // PCU 状态帧 Byte1：故障急停位
        bool estop{false};

        // -------- 心跳诊断 --------
        uint32_t hb_repeat_count{0};
        uint32_t hb_jump_err_count{0};
        int32_t  last_hb_delta{0};

        // -------- runtime 判定结果 --------
        bool rx_alive{false};   // 最近是否收到 PCU 状态帧
        bool hb_alive{false};   // 心跳是否仍在推进
        bool online{false};     // 最终在线 = rx_alive && hb_alive

        double last_rx_age_ms{0.0};
        double last_hb_change_age_ms{0.0};

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
    uint32_t q1_status_bits{0};
    uint32_t wa_status_bits{0};

    bool mains_abnormal{false};       // Q1/WA bit7 或 Q6 warning bit3
    bool battery_low_state{false};    // Q1/WA bit6
    bool bypass_mode{false};          // Q1/WA bit5
    bool ups_fault_state{false};      // Q1/WA bit4
    bool backup_mode{false};          // Q1/WA bit3
    bool self_test_active{false};     // Q1/WA bit2

    // ---- Q6 warning bits ----
    uint32_t warning_bits{0};

    bool internal_warning{false};       // bit0
    bool epo_active{false};             // bit1
    bool module_unlock{false};          // bit2
    bool neutral_lost{false};           // bit4
    bool mains_phase_error{false};      // bit5
    bool ln_reverse{false};             // bit6
    bool bypass_abnormal{false};        // bit7
    bool bypass_phase_error{false};     // bit8
    bool battery_not_connected{false};  // bit9
    bool battery_low_warning{false};    // bit10
    bool battery_overcharge{false};     // bit11
    bool battery_reverse{false};        // bit12
    bool overload_warning{false};       // bit13
    bool overload_alarm{false};         // bit14
    bool fan_fault{false};              // bit15
    bool bypass_cover_open{false};      // bit16
    bool charger_fault{false};          // bit17
    bool position_error{false};         // bit18
    bool boot_condition_not_met{false}; // bit19
    bool redundancy_lost{false};        // bit20
    bool module_loose{false};           // bit21
    bool battery_maint_due{false};      // bit22
    bool inspection_maint_due{false};   // bit23
    bool warranty_maint_due{false};     // bit24
    bool temp_low_warning{false};       // bit25
    bool temp_high_warning{false};      // bit26
    bool battery_overtemp{false};       // bit27
    bool fan_maint_due{false};          // bit28
    bool bus_cap_maint_due{false};      // bit29
    bool system_overload{false};        // bit30
    bool reserved_warning{false};       // bit31

    // ---- Q6 fault containers ----
    uint32_t fault_bits{0};
    int fault_code_1{0};
    int fault_code_2{0};
    int fault_code_3{0};
    int fault_code_4{0};

    // ---- UPS Fault Table：0 是无故障，1~60 对应协议表 ----
    bool bus_softstart_timeout{false};     // 1
    bool bus_overvoltage_fault{false};     // 2
    bool bus_undervoltage_fault{false};    // 3
    bool bus_imbalance_fault{false};       // 4
    bool bus_short_circuit{false};         // 5

    bool inv_softstart_timeout{false};     // 6
    bool inv_overvoltage_fault{false};     // 7
    bool inv_undervoltage_fault{false};    // 8
    bool output_short_circuit{false};      // 9
    bool r_inv_short_circuit{false};       // 10
    bool s_inv_short_circuit{false};       // 11
    bool t_inv_short_circuit{false};       // 12
    bool rs_short_circuit{false};          // 13
    bool st_short_circuit{false};          // 14
    bool tr_short_circuit{false};          // 15

    bool reverse_power_fault{false};       // 16
    bool r_reverse_power_fault{false};     // 17
    bool s_reverse_power_fault{false};     // 18
    bool t_reverse_power_fault{false};     // 19
    bool total_reverse_power_fault{false}; // 20
    bool current_imbalance_fault{false};   // 21
    bool overload_fault{false};            // 22
    bool overtemp_fault{false};            // 23

    bool inv_relay_fail_close{false};      // 24
    bool inv_relay_stuck{false};           // 25
    bool mains_scr_fault{false};           // 26
    bool battery_scr_fault{false};         // 27
    bool bypass_scr_fault{false};          // 28
    bool rectifier_fault{false};           // 29
    bool input_overcurrent_fault{false};   // 30
    bool wiring_error{false};              // 31

    bool comm_cable_disconnected{false};   // 32
    bool host_cable_fault{false};          // 33
    bool can_comm_fault{false};            // 34
    bool sync_signal_fault{false};         // 35
    bool power_supply_fault{false};        // 36
    bool all_fan_fault{false};             // 37
    bool dsp_error{false};                 // 38
    bool charger_softstart_timeout{false}; // 39
    bool all_module_fault{false};          // 40

    bool mains_ntc_open_fault{false};      // 41
    bool mains_fuse_open_fault{false};     // 42
    bool output_imbalance_fault{false};    // 43
    bool input_mismatch_fault{false};      // 44
    bool eeprom_data_lost{false};          // 45
    bool mains_support_failed{false};      // 46
    bool power_failed{false};              // 47
    bool system_overload_fault{false};     // 48
    bool ads7869_error{false};             // 49
    bool bypass_mode_no_op{false};         // 50
    bool op_breaker_off_parallel{false};   // 51

    bool r_bus_fuse_fault{false};          // 52
    bool s_bus_fuse_fault{false};          // 53
    bool t_bus_fuse_fault{false};          // 54
    bool ntc_fault{false};                 // 55
    bool parallel_cable_fault{false};      // 56
    bool battery_fault{false};             // 57
    bool frequent_overcurrent_fault{false}; // 58
    bool battery_overcharge_fault{false};  // 59
    bool epo_critical_fault{false};        // 60

    // 旧表里这个名字没有独立协议码，保留字段但永远不由协议置位。
    bool battery_overcharge_persist{false};
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

        // ===== HMI 映射模型 =====
        //
        // normal_map_logic.jsonl 统一解析后的模型。
        // 后续普通 HMI 输出、故障页布局、HMI 写入反查都应尽量从这里取。
        std::shared_ptr<const normal::HmiMapModel> hmi_map;
        bool hmi_map_loaded{false};

        // ===== 故障页目录与运行态 =====
        FaultCatalog fault_catalog;
        FaultCenter fault_center;
        FaultPageManager fault_pages;
        bool fault_map_loaded{false};

        // ===== 历史故障缓存层 =====
        // 说明：
        // 1) SQLite 作为历史故障真源
        // 2) fault_history_cache 只做热点缓存 + 窗口缓存
        // 3) 当前页/是否处于历史页视图，也先挂在 LogicContext
        std::unique_ptr<control::FaultHistoryCache> fault_history_cache;
        uint16_t history_page_no{1};
        bool history_view_active{false};
        uint64_t history_cache_version{0};

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

        // HMI 最近一次有效按钮动作。
        // 注意：这里保存的是 normal_map_logic.jsonl 中的 path，
        // 而不是 0x1001 / 0x1005 这种硬编码地址。
        uint64_t last_hmi_button_ts{0};
        uint16_t last_hmi_button_addr{0};
        std::string last_hmi_button_path;
        std::string last_hmi_button_name;

        // 业务上下电按钮请求。
        // 0x1001 / 0x1002 只在 jsonl 中映射到这两个 path，
        // logic_hmi_input 不再直接认地址。
        bool hmi_hv_on_requested{false};
        bool hmi_hv_off_requested{false};
        uint64_t hmi_hv_on_request_ts{0};
        uint64_t hmi_hv_off_request_ts{0};


        // HMI 弱在线观测
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

        /*
         * HMI 参数写入缓存。
         *
         * 来源：
         *   normal_map_logic.jsonl 的 int_rw 项：
         *     path = hmi.param.xxx
         *
         * 作用：
         *   1. HMI 写寄存器不再只按地址缓存；
         *   2. 控制层/业务层后续可以按稳定 path 读取参数；
         *   3. 新增参数点位时，优先只改 jsonl，不改地址判断。
         */
        std::unordered_map<std::string, int32_t> hmi_param_values;
        std::unordered_map<std::string, uint16_t> hmi_param_addr;
        std::unordered_map<std::string, std::string> hmi_param_name;
        std::unordered_map<std::string, uint64_t> hmi_param_ts;

        uint64_t last_hmi_param_ts{0};
        uint16_t last_hmi_param_addr{0};
        uint16_t last_hmi_param_value{0};
        std::string last_hmi_param_path;
        std::string last_hmi_param_name;

        // 常用参数的显式镜像，方便业务层直接读，不必每次查 map。
        int32_t hmi_param_maintain_password{0};
        int32_t hmi_param_box_num{0};
        int32_t hmi_param_ups_shutdown_time{0};
        int32_t hmi_param_aircon_set_temp{0};
        int32_t hmi_param_aircon_set_humidity{0};
        int32_t hmi_param_password_old{0};
        int32_t hmi_param_password_new_1{0};
        int32_t hmi_param_password_new_2{0};

        /*
         * HMI 功能按钮请求锁存。
         *
         * hv_on / hv_off 已经在前面有专用字段；
         * 这里补充 fault_reset / UPS关机 / 空调设置 / 密码设置 / 箱号设置等。
         */
        bool hmi_fault_reset_requested{false};
        uint64_t hmi_fault_reset_request_ts{0};

        bool hmi_fault_home_ack_requested{false};
        uint64_t hmi_fault_home_ack_request_ts{0};

        bool hmi_fault_ack_all_requested{false};
        uint64_t hmi_fault_ack_all_request_ts{0};

        bool hmi_maintain_password_enter_requested{false};
        uint64_t hmi_maintain_password_enter_request_ts{0};

        bool hmi_box_num_set_requested{false};
        uint64_t hmi_box_num_set_request_ts{0};

        bool hmi_ups_shutdown_requested{false};
        uint64_t hmi_ups_shutdown_request_ts{0};

        bool hmi_aircon_set_requested{false};
        uint64_t hmi_aircon_set_request_ts{0};

        bool hmi_password_set_requested{false};
        uint64_t hmi_password_set_request_ts{0};
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