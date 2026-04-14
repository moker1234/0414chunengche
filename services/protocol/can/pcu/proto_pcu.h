//
// Created by lxy on 2026/2/11.
//

#ifndef ENERGYSTORAGE_PROTO_PCU_H
#define ENERGYSTORAGE_PROTO_PCU_H

#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>

#include "../can_protocol_base.h"

namespace proto::pcu {

    // 组 EMU→PCU 控制帧（第一个协议）
    void buildEmuCtrl(can_frame& fr,
                      uint32_t id29,
                      uint8_t heartbeat,
                      uint8_t plug_state,
                      uint8_t estop,
                      uint8_t batt1_estop,
                      uint8_t batt2_estop,
                      uint8_t sys_enable);

    // 组 EMU→PCU 状态帧（第一个协议）
    // batt*_kw_x10: 单位 0.1kW/bit
    void buildEmuStatus(can_frame& fr,
                        uint32_t id29,
                        uint16_t batt1_kw_x10,
                        uint16_t batt2_kw_x10,
                        uint8_t batt1_branches,
                        uint8_t batt2_branches);

    // 解析 PCU→EMU 状态反馈（0x1801EFA0）
    bool parsePcuStatus(uint32_t expect_id29,
                        const can_frame& fr,
                        DeviceData& out);

    // 协议对象：当前只解析 PCU→EMU
    class ProtoPCU : public proto::CanProtocolBase {
    public:
        ProtoPCU(uint32_t id_pcu_status, uint32_t id_emu_ctrl, uint32_t id_emu_status)
            : id_pcu_status_(id_pcu_status),
              id_emu_ctrl_(id_emu_ctrl),
              id_emu_status_(id_emu_status) {}

        std::string name() const override { return "emu_pcu_v1"; }

        bool parse(const can_frame& fr, DeviceData& out) override {
            return parsePcuStatus(id_pcu_status_, fr, out);
        }

    private:
        uint32_t id_pcu_status_{0};
        uint32_t id_emu_ctrl_{0};
        uint32_t id_emu_status_{0};
    };

} // namespace proto::pcu

#endif // ENERGYSTORAGE_PROTO_PCU_H
