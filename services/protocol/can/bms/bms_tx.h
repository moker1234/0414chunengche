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

#include "proto_bms_table_runtime.h"
#include "bms_command_types.h"

class DriverManager;

namespace proto::bms {

class BmsTx {
public:
    BmsTx() = default;

    // v2b_cmd_id29 推荐传协议模板 ID：0x1802F3EF。
    // 如果误传 canhub 运行态 ID：0x1802F101，也会通过 BmsFrameRouter 归一化。
    bool init(DriverManager& drv_mgr,
              int default_can_index = 2,
              uint32_t v2b_cmd_id29 = 0x1802F3EF);

    // ===== 旧接口，保留兼容 =====
    void setHvOnOff(uint32_t v) { hv_onoff_ = v; }
    void setCmdEnable(uint32_t v) { cmd_enable_ = v; } // 旧字段，当前协议不再落位
    void setOtherU8(const char* sig_name, uint32_t v);

    bool tickSend();

    uint32_t v2bCmdId29() const { return v2b_cmd_id29_; }

    // ===== 新接口：只组帧，不发送 =====
    bool buildFrame(const V2bCmdFields& cmd, can_frame& out) const;
    bool buildFrameForInstance(const V2bCmdFields& cmd, can_frame& out) const;

    void setV2bCmdId29(uint32_t id29)
    {
        v2b_cmd_id29_ = id29;
        v2b_msg_ = nullptr;
        sig_life_ = nullptr;
    }

private:
    static void packBitsLsb(uint8_t data[8],
                            int start_lsb,
                            int len,
                            uint64_t value);

    bool bindSignals_();

    // 只保留一个薄封装，实际 canhub ID 映射在 BmsFrameRouter。
    static uint32_t rewriteRuntimeId_(uint32_t proto_id29,
                                      uint32_t instance_index);

private:
    DriverManager* drv_{nullptr};
    int can_index_{2};

    // 保存配置输入 ID。
    // 推荐为 0x1802F3EF，也兼容 0x1802F101。
    uint32_t v2b_cmd_id29_{0x1802F3EF};

    // 绑定 signal 时使用 generated 表消息定义。
    const proto_bms_gen::MessageDef* v2b_msg_{nullptr};

    const proto_bms_gen::SignalDef* sig_life_{nullptr};
    const proto_bms_gen::SignalDef* sig_hv_{nullptr};

    const proto_bms_gen::SignalDef* sig_aux1_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux2_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux3_{nullptr};
    const proto_bms_gen::SignalDef* sig_speed_{nullptr};

    const proto_bms_gen::SignalDef* sig_main_pos_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_pos_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_neg_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_main_neg_flt_{nullptr};

    const proto_bms_gen::SignalDef* sig_chrg_pos_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_chrg_pos_flt_{nullptr};

    const proto_bms_gen::SignalDef* sig_heat_pos_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_heat_pos_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_heat_neg_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_heat_neg_flt_{nullptr};

    const proto_bms_gen::SignalDef* sig_aux1_relay_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux1_relay_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux2_relay_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux2_relay_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux3_relay_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux3_relay_flt_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux4_relay_st_{nullptr};
    const proto_bms_gen::SignalDef* sig_aux4_relay_flt_{nullptr};

    const proto_bms_gen::SignalDef* sig_reserved1_{nullptr};
    const proto_bms_gen::SignalDef* sig_reserved2_{nullptr};

    // ===== 旧状态，兼容 tickSend =====
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