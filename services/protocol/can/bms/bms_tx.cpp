//
// Created by lxy on 2026/2/13.
//

#include "bms_tx.h"

#include <cstring>

#include "logger.h"
#include "driver_manager.h"

namespace proto::bms {

bool BmsTx::bindSignals_()
{
    v2b_msg_ = ProtoBmsTableRuntime::findMessage(v2b_cmd_id29_);
    if (!v2b_msg_) {
        LOGERR("[BMS][TX] V2B_CMD msg not found in table: id=0x%08X", v2b_cmd_id29_);
        return false;
    }

    sig_life_   = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_LifeSignal");
    sig_hv_     = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HV_OnOff");
    sig_enable_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_SystemEnable");

    sig_aux1_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux1OnOff");
    sig_aux2_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux2OnOff");
    sig_aux3_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux3OnOff");
    sig_speed_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_VehicleSpeed");

    sig_main_pos_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainPosRelayST");
    sig_main_pos_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainPosRelayFlt");
    sig_main_neg_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainNegRelayST");
    sig_main_neg_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainNegRelayFlt");
    sig_prechg_st_    = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_PreChargeRelayST");
    sig_prechg_flt_   = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_PreChargeRelayFlt");

    if (!sig_life_) {
        LOGERR("[BMS][TX] signal not found: V2B_CMD_LifeSignal");
        return false;
    }

    LOGI("[BMS][TX] bind ok. id=0x%08X proto_ver=%s",
         v2b_cmd_id29_, proto_bms_gen::kProtoVersion);
    return true;
}

bool BmsTx::init(DriverManager& drv_mgr, int can_index)
{
    drv_ = &drv_mgr;
    can_index_ = can_index;
    return bindSignals_();
}

void BmsTx::setOtherU8(const char* sig_name, uint32_t v)
{
    if (!sig_name) return;
    for (auto& it : u8_overrides_) {
        if (it.sig_name == sig_name) {
            it.value = static_cast<uint8_t>(v & 0xFFu);
            return;
        }
    }
    U8Override o;
    o.sig_name = sig_name;
    o.value = static_cast<uint8_t>(v & 0xFFu);
    u8_overrides_.push_back(o);
}

void BmsTx::packBitsLsb(uint8_t data[8], int start_lsb, int len, uint64_t value)
{
    if (!data || start_lsb < 0 || len <= 0 || start_lsb >= 64) return;
    if (start_lsb + len > 64) len = 64 - start_lsb;

    uint64_t u = 0;
    for (int i = 0; i < 8; ++i) {
        u |= (static_cast<uint64_t>(data[i]) << (i * 8));
    }

    const uint64_t mask = (len == 64) ? ~0ULL : ((1ULL << len) - 1ULL);
    u &= ~(mask << start_lsb);
    u |= ((value & mask) << start_lsb);

    for (int i = 0; i < 8; ++i) {
        data[i] = static_cast<uint8_t>((u >> (i * 8)) & 0xFFu);
    }
}

// uint32_t BmsTx::rewriteRuntimeId_(uint32_t proto_id29, uint32_t instance_index)
// {
//     if (instance_index < 1 || instance_index > 4) {
//         return proto_id29;
//     }
//
//     const uint32_t msg_code = proto_id29 & 0xFFu;
//     const uint32_t new_low12 = ((instance_index & 0xFu) << 8) | msg_code;
//     return (proto_id29 & ~0xFFFu) | new_low12;
// }
    static uint32_t msgCodeFromProtoId_(uint32_t proto_id29)
{
    switch (proto_id29) {
    case 0x1802F3EFu: return 0x01; // V2B_CMD

    case 0x1881EFF3u: return 0x02; // B2V_Fault1
    case 0x1882EFF3u: return 0x03; // B2V_Fault2
    case 0x1883EFF3u: return 0x04; // B2V_ST1
    case 0x1884EFF3u: return 0x05; // B2V_ST2
    case 0x1885EFF3u: return 0x06; // B2V_ST3
    case 0x1886EFF3u: return 0x07; // B2V_ST4
    case 0x1887EFF3u: return 0x08; // B2V_ST5
    case 0x1888EFF3u: return 0x09; // B2V_ST6
    case 0x1889EFF3u: return 0x0A; // B2V_ST7

    case 0x18C3EFF3u: return 0x0B; // B2V_ElecEnergy
    case 0x18C4EFF3u: return 0x0C; // B2V_CurrentLimit

    case 0x18FF45F4u: return 0x0D; // B2TM_Info
    case 0x18FFC13Au: return 0x0E; // TM2B_Info
    case 0x18FFF7F6u: return 0x0F; // Fire2B_Info

    default: return 0u;
    }
}

    uint32_t BmsTx::rewriteRuntimeId_(uint32_t proto_id29, uint32_t instance_index)
{
    if (instance_index < 1 || instance_index > 4) {
        return proto_id29;
    }

    const uint32_t msg_code = msgCodeFromProtoId_(proto_id29);
    if (msg_code == 0u) {
        return proto_id29;
    }

    // canhub 规则：
    // 后三位 = [实例号][报文号]
    // 例如：
    //   V2B_CMD        0x1802F3EF -> 0x1802F101
    //   B2V_ST1        0x1883EFF3 -> 0x1883E104
    //   B2V_CurrentLimit 0x18C4EFF3 -> 0x18C4E10C
    const uint32_t new_low12 = ((instance_index & 0xFu) << 8) | (msg_code & 0xFFu);
    return (proto_id29 & ~0xFFFu) | new_low12;
}

bool BmsTx::buildFrame(const V2bCmdFields& cmd, can_frame& out) const
{
    // 默认按模板 ID 组帧，不做实例改写
    return buildFrameForInstance(cmd, out);
}

bool BmsTx::buildFrameForInstance(const V2bCmdFields& cmd, can_frame& out) const
{
    if (!cmd.valid) return false;
    if (!v2b_msg_ || !sig_life_) return false;

    std::memset(&out, 0, sizeof(out));
    out.can_id  = CAN_EFF_FLAG | rewriteRuntimeId_(v2b_cmd_id29_, cmd.instance_index);
    out.can_dlc = 8;

    // 必需字段：LifeSignal
    packBitsLsb(out.data, sig_life_->startbit_lsb, sig_life_->length, cmd.life_signal);

    if (sig_hv_) {
        packBitsLsb(out.data, sig_hv_->startbit_lsb, sig_hv_->length, cmd.hv_onoff);
    }
    if (sig_enable_) {
        packBitsLsb(out.data, sig_enable_->startbit_lsb, sig_enable_->length, cmd.system_enable);
    }

    if (sig_aux1_) packBitsLsb(out.data, sig_aux1_->startbit_lsb, sig_aux1_->length, cmd.aux1_onoff);
    if (sig_aux2_) packBitsLsb(out.data, sig_aux2_->startbit_lsb, sig_aux2_->length, cmd.aux2_onoff);
    if (sig_aux3_) packBitsLsb(out.data, sig_aux3_->startbit_lsb, sig_aux3_->length, cmd.aux3_onoff);
    if (sig_speed_) packBitsLsb(out.data, sig_speed_->startbit_lsb, sig_speed_->length, cmd.vehicle_speed);

    if (sig_main_pos_st_)  packBitsLsb(out.data, sig_main_pos_st_->startbit_lsb,  sig_main_pos_st_->length,  cmd.main_pos_relay_st);
    if (sig_main_pos_flt_) packBitsLsb(out.data, sig_main_pos_flt_->startbit_lsb, sig_main_pos_flt_->length, cmd.main_pos_relay_flt);
    if (sig_main_neg_st_)  packBitsLsb(out.data, sig_main_neg_st_->startbit_lsb,  sig_main_neg_st_->length,  cmd.main_neg_relay_st);
    if (sig_main_neg_flt_) packBitsLsb(out.data, sig_main_neg_flt_->startbit_lsb, sig_main_neg_flt_->length, cmd.main_neg_relay_flt);
    if (sig_prechg_st_)    packBitsLsb(out.data, sig_prechg_st_->startbit_lsb,    sig_prechg_st_->length,    cmd.prechg_relay_st);
    if (sig_prechg_flt_)   packBitsLsb(out.data, sig_prechg_flt_->startbit_lsb,   sig_prechg_flt_->length,   cmd.prechg_relay_flt);

    // 兼容旧覆盖表：只支持 <=8 bit
    for (const auto& o : u8_overrides_) {
        const auto* s = ProtoBmsTableRuntime::findSignal(v2b_msg_, o.sig_name.c_str());
        if (!s) continue;
        if (s->length > 8) continue;
        packBitsLsb(out.data, s->startbit_lsb, s->length, o.value);
    }

    return true;
}

bool BmsTx::tickSend()
{
    if (!drv_) return false;
    if (!v2b_msg_ || !sig_life_) {
        if (!bindSignals_()) return false;
    }

    V2bCmdFields cmd;
    cmd.instance_index = 1;          // 兼容旧模式：默认只发实例1
    cmd.can_index = can_index_;
    cmd.life_signal = life_;
    cmd.hv_onoff = hv_onoff_;
    cmd.system_enable = cmd_enable_;
    cmd.source = "legacy_tick";

    can_frame fr{};
    if (!buildFrameForInstance(cmd, fr)) {
        LOGD("[BMS][TX] buildFrameForInstance failed");
        return false;
    }

    const bool ok = drv_->sendCan(can_index_, fr);
    if (!ok) {
        LOGD("[BMS][TX] sendCan failed can=%d id=0x%08X", can_index_, (fr.can_id & CAN_EFF_MASK));
        return false;
    }

    life_ = static_cast<uint8_t>(life_ + 1);
    return true;
}

} // namespace proto::bms