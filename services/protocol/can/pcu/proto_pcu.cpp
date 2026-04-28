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
    const uint32_t id29 = (fr.can_id & CAN_EFF_MASK);

    if ((fr.can_id & CAN_EFF_FLAG) == 0) {
        // PCU 协议使用 CAN 2.0B 29 位扩展帧
        return false;
    }

    if (id29 != (expect_id29 & CAN_EFF_MASK)) {
        return false;
    }

    if (fr.can_dlc < 8) {
        LOG_THROTTLE_MS("pcu_rx_short_frame", 1000, LOG_COMM_W,
                        "[PCU][RX][DROP] short frame id=0x%08X dlc=%u expect_dlc=8",
                        static_cast<unsigned>(id29),
                        static_cast<unsigned>(fr.can_dlc));
        return false;
    }

    auto frameHex = [](const can_frame& f) -> std::string {
        char buf[4];
        std::string s;
        s.reserve(8 * 3);

        const uint8_t n = (f.can_dlc > 8) ? 8 : f.can_dlc;
        for (uint8_t i = 0; i < n; ++i) {
            std::snprintf(buf, sizeof(buf), "%02X", f.data[i]);
            if (!s.empty()) s.push_back(' ');
            s.append(buf);
        }

        return s;
    };

    auto stateText = [](uint8_t st) -> const char* {
        switch (st) {
        case 0x00: return "standby";
        case 0x01: return "charging";
        case 0x02: return "fault";
        default:   return "unknown";
        }
    };

    /*
     * 协议：PCU → EMU 状态反馈 0x1801EFA0
     *
     * Byte0: 心跳数据，0~255，每帧加一
     * Byte1: 故障急停，0x00 无故障，0x01 故障紧急下电
     * Byte2: PCU 状态，0x00 待机，0x01 充电，0x02 故障
     * Byte3: 移动充电柜编号
     * Byte4~7: 保留
     */
    const uint8_t heartbeat  = fr.data[0];
    const uint8_t estop_raw  = fr.data[1];
    const uint8_t pcu_state  = fr.data[2];
    const uint8_t cabinet_id = fr.data[3];

    out.device_name = "PCU";

    // ===== 原有主字段：保持不变，避免影响 Aggregator / Logic / SQLite =====
    out.value["heartbeat"]  = heartbeat;
    out.status["estop"]     = estop_raw ? 1u : 0u;
    out.value["pcu_state"]  = pcu_state;
    out.value["cabinet_id"] = cabinet_id;

    // ===== 新增协议元信息：用于诊断、落盘、后续链路排查 =====
    out.value["__pcu.id"] = static_cast<int32_t>(id29);
    out.value["__pcu.heartbeat"] = heartbeat;
    out.value["__pcu.estop_raw"] = estop_raw;
    out.value["__pcu.pcu_state_raw"] = pcu_state;
    out.value["__pcu.cabinet_id"] = cabinet_id;

    out.str["__pcu.msg"] = "PCU_STATUS";
    out.str["__pcu.raw_hex"] = frameHex(fr);
    out.str["pcu_state_text"] = stateText(pcu_state);

    // ===== 轻诊断：不拒绝报文，只报警 =====
    if (estop_raw != 0x00 && estop_raw != 0x01) {
        LOG_THROTTLE_MS("pcu_rx_bad_estop", 1000, LOG_COMM_W,
                        "[PCU][RX][WARN] bad estop_raw=%u id=0x%08X raw=%s",
                        static_cast<unsigned>(estop_raw),
                        static_cast<unsigned>(id29),
                        out.str["__pcu.raw_hex"].c_str());
    }

    if (pcu_state > 0x02) {
        LOG_THROTTLE_MS("pcu_rx_bad_state", 1000, LOG_COMM_W,
                        "[PCU][RX][WARN] bad pcu_state=%u id=0x%08X raw=%s",
                        static_cast<unsigned>(pcu_state),
                        static_cast<unsigned>(id29),
                        out.str["__pcu.raw_hex"].c_str());
    }

    if (fr.data[4] != 0 || fr.data[5] != 0 || fr.data[6] != 0 || fr.data[7] != 0) {
        LOG_THROTTLE_MS("pcu_rx_reserved_nonzero", 1000, LOG_COMM_W,
                        "[PCU][RX][WARN] reserved bytes nonzero b4=%u b5=%u b6=%u b7=%u id=0x%08X raw=%s",
                        static_cast<unsigned>(fr.data[4]),
                        static_cast<unsigned>(fr.data[5]),
                        static_cast<unsigned>(fr.data[6]),
                        static_cast<unsigned>(fr.data[7]),
                        static_cast<unsigned>(id29),
                        out.str["__pcu.raw_hex"].c_str());
    }

    return true;
}

} // namespace proto::pcu
