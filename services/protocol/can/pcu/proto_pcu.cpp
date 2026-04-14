//
// Created by lxy on 2026/2/11.
//

#include "./proto_pcu.h"
#include "logger.h"

namespace proto::pcu {

static inline void set_u16_le(uint8_t* p, uint16_t v) {
    p[0] = uint8_t(v & 0xFF);
    p[1] = uint8_t((v >> 8) & 0xFF);
}

void buildEmuCtrl(can_frame& fr,
                  uint32_t id29,
                  uint8_t heartbeat,
                  uint8_t plug_state,
                  uint8_t estop,
                  uint8_t batt1_estop,
                  uint8_t batt2_estop,
                  uint8_t sys_enable)
{
    fr.can_id  = (id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
    fr.can_dlc = 8;

    fr.data[0] = heartbeat;
    fr.data[1] = plug_state;
    fr.data[2] = estop;
    fr.data[3] = batt1_estop;
    fr.data[4] = batt2_estop;
    fr.data[5] = 0;
    fr.data[6] = 0;
    fr.data[7] = sys_enable;
}

void buildEmuStatus(can_frame& fr,
                    uint32_t id29,
                    uint16_t batt1_kw_x10,
                    uint16_t batt2_kw_x10,
                    uint8_t batt1_branches,
                    uint8_t batt2_branches)
{
    fr.can_id  = (id29 & CAN_EFF_MASK) | CAN_EFF_FLAG;
    fr.can_dlc = 8;

    set_u16_le(&fr.data[0], batt1_kw_x10);
    set_u16_le(&fr.data[2], batt2_kw_x10);
    fr.data[4] = batt1_branches;
    fr.data[5] = batt2_branches;
    fr.data[6] = 0;
    fr.data[7] = 0;
}

bool parsePcuStatus(uint32_t expect_id29,
                    const can_frame& fr,
                    DeviceData& out)
{
    // printf("parsePcuStatus expect_id29=0x%08x can_id=0x%08x can_dlc=%d\n",
    //        expect_id29, fr.can_id, fr.can_dlc);
    const uint32_t id29 = (fr.can_id & CAN_EFF_MASK);

    if ((fr.can_id & CAN_EFF_FLAG) == 0) {
        // 只接收扩展帧
        return false;
    }
    if (id29 != (expect_id29 & CAN_EFF_MASK)) return false;
    if (fr.can_dlc < 8) return false;

    // 协议：PCU→EMU 0x1801EFA0
    // Byte0 心跳
    // Byte1 故障急停
    // Byte2 PCU状态 0/1/2
    // Byte3 移动充电柜编号
    out.device_name = "PCU";

    out.value["heartbeat"] = fr.data[0];
    out.status["estop"] = fr.data[1] ? 1u : 0u;
    out.value["pcu_state"] = fr.data[2];
    out.value["cabinet_id"] = fr.data[3];

    return true;
}

} // namespace proto::pcu
