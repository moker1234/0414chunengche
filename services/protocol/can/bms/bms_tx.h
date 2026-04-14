//
// Created by lxy on 2026/2/13.
//

#ifndef ENERGYSTORAGE_BMS_TX_H
#define ENERGYSTORAGE_BMS_TX_H

#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <vector>

// #include "driver_manager.h"
#include "proto_bms_table_runtime.h"
#include "bms_command_types.h"

class DriverManager;
namespace proto::bms {

/**
 * BMS 发送器：
 * 1. 兼容旧接口：init()/tickSend()
 * 2. 新接口：buildFrame()/buildFrameForInstance()
 *
 * 推荐新用法：
 *   - control/bms/bms_command_manager 组出 V2bCmdFields
 *   - 调用 buildFrameForInstance() 得到 can_frame
 *   - 再封装成 control::Command::SendCan 下发
 */
class BmsTx {
public:
    BmsTx() = default;

    // 兼容旧逻辑：允许直接持有 DriverManager 并发送
    bool init(DriverManager& drv_mgr, int can_index = 2);

    // ===== 旧业务接口：保留兼容 =====
    void setHvOnOff(uint32_t v) { hv_onoff_ = v; }
    void setCmdEnable(uint32_t v) { cmd_enable_ = v; }
    void setOtherU8(const char* sig_name, uint32_t v);

    bool tickSend();

    uint32_t v2bCmdId29() const { return v2b_cmd_id29_; }

    // ===== 新接口：只组帧，不发送 =====
    bool buildFrame(const V2bCmdFields& cmd, can_frame& out) const;
    bool buildFrameForInstance(const V2bCmdFields& cmd, can_frame& out) const;

private:
    static void packBitsLsb(uint8_t data[8], int start_lsb, int len, uint64_t value);

    bool bindSignals_();
    static uint32_t rewriteRuntimeId_(uint32_t proto_id29, uint32_t instance_index);

private:
    DriverManager* drv_{nullptr};
    int can_index_{2};

    // V2B_CMD 协议模板 ID（不带实例）
    uint32_t v2b_cmd_id29_{0x1802F3EF};
    const proto_bms_gen::MessageDef* v2b_msg_{nullptr};

    // 常用 signals（如果表里没有则允许为空）
    const proto_bms_gen::SignalDef* sig_life_{nullptr};
    const proto_bms_gen::SignalDef* sig_hv_{nullptr};
    const proto_bms_gen::SignalDef* sig_enable_{nullptr};

    const proto_bms_gen::SignalDef* sig_aux1_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux2_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux3_{nullptr};
    const proto_bms_gen::SignalDef* sig_speed_{nullptr};

    const proto_bms_gen::SignalDef* sig_main_pos_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_pos_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_neg_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_neg_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_prechg_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_prechg_flt_{nullptr};

    // ===== 旧状态（兼容 tickSend）=====
    uint8_t life_{0};
    uint32_t hv_onoff_{0};
    uint32_t cmd_enable_{1};

    struct U8Override {
        std::string sig_name;
        uint8_t value{0};
    };
    std::vector<U8Override> u8_overrides_;
};

} // namespace proto::bms

#endif // ENERGYSTORAGE_BMS_TX_H