//
// Created by lxy on 2026/2/13.
//

#include "bms_tx.h"

#include <cstring>

#include "logger.h"
#include "driver_manager.h"
#include "bms_thread/bms_frame_router.h"

namespace proto::bms {

bool BmsTx::bindSignals_()
{
    v2b_msg_ = nullptr;

    sig_life_ = nullptr;
    sig_hv_ = nullptr;

    sig_aux1_ = nullptr;
    sig_aux2_ = nullptr;
    sig_aux3_ = nullptr;
    sig_speed_ = nullptr;

    sig_main_pos_st_ = nullptr;
    sig_main_pos_flt_ = nullptr;
    sig_main_neg_st_ = nullptr;
    sig_main_neg_flt_ = nullptr;

    sig_chrg_pos_st_ = nullptr;
    sig_chrg_pos_flt_ = nullptr;

    sig_heat_pos_st_ = nullptr;
    sig_heat_pos_flt_ = nullptr;
    sig_heat_neg_st_ = nullptr;
    sig_heat_neg_flt_ = nullptr;

    sig_aux1_relay_st_ = nullptr;
    sig_aux1_relay_flt_ = nullptr;
    sig_aux2_relay_st_ = nullptr;
    sig_aux2_relay_flt_ = nullptr;
    sig_aux3_relay_st_ = nullptr;
    sig_aux3_relay_flt_ = nullptr;
    sig_aux4_relay_st_ = nullptr;
    sig_aux4_relay_flt_ = nullptr;

    sig_reserved1_ = nullptr;
    sig_reserved2_ = nullptr;

    // 统一由 BmsFrameRouter 处理：
    // - 如果配置写 0x1802F3EF，返回 0x1802F3EF
    // - 如果配置误写 0x1802F101，也能归一化回 0x1802F3EF
    const uint32_t bind_id29 = BmsFrameRouter::normalizeToProtoId(v2b_cmd_id29_);

    v2b_msg_ = ProtoBmsTableRuntime::findMessage(bind_id29);
    if (!v2b_msg_) {
        LOGERR("[BMS][TX] V2B_CMD msg not found in table: cfg_id=0x%08X bind_id=0x%08X",
               v2b_cmd_id29_, bind_id29);
        return false;
    }

    sig_life_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_LifeSignal");
    sig_hv_   = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HV_OnOff");

    sig_aux1_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux1OnOff");
    sig_aux2_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux2OnOff");
    sig_aux3_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux3OnOff");
    sig_speed_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_VehicleSpeed");

    sig_main_pos_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainPosRelayST");
    sig_main_pos_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainPosRelayFlt");
    sig_main_neg_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainNegRelayST");
    sig_main_neg_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_MainNegRelayFlt");

    sig_chrg_pos_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_ChrgPosRelayST");
    sig_chrg_pos_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_ChrgPosRelayFlt");

    sig_heat_pos_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HeatPosRelayST");
    sig_heat_pos_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HeatPosRelayFlt");
    sig_heat_neg_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HeatNegRelayST");
    sig_heat_neg_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_HeatNegRelayFlt");

    sig_aux1_relay_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux1RelayST");
    sig_aux1_relay_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux1RelayFlt");
    sig_aux2_relay_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux2RelayST");
    sig_aux2_relay_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux2RelayFlt");
    sig_aux3_relay_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux3RelayST");
    sig_aux3_relay_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux3RelayFlt");
    sig_aux4_relay_st_  = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux4RelayST");
    sig_aux4_relay_flt_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Aux4RelayFlt");

    sig_reserved1_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Reserved1");
    sig_reserved2_ = ProtoBmsTableRuntime::findSignal(v2b_msg_, "V2B_CMD_Reserved2");

    if (!sig_life_) {
        LOGERR("[BMS][TX] signal not found: V2B_CMD_LifeSignal");
        return false;
    }

    LOGI("[BMS][TX] bind ok. cfg_id=0x%08X bind_id=0x%08X proto_ver=%s",
         v2b_cmd_id29_, bind_id29, proto_bms_gen::kProtoVersion);

    return true;
}

bool BmsTx::init(DriverManager& drv_mgr,
                 int default_can_index,
                 uint32_t v2b_cmd_id29)
{
    drv_ = &drv_mgr;
    can_index_ = default_can_index;
    v2b_cmd_id29_ = v2b_cmd_id29;

    return bindSignals_();
}

void BmsTx::setOtherU8(const char* sig_name,
                       uint32_t v)
{
    if (!sig_name) {
        return;
    }

    for (auto& it : u8_overrides_) {
        if (it.sig_name == sig_name) {
            it.value = static_cast<uint8_t>(v & 0xFFu);
            return;
        }
    }

    U8Override o;
    o.sig_name = sig_name;
    o.value = static_cast<uint8_t>(v & 0xFFu);
    u8_overrides_.push_back(std::move(o));
}

void BmsTx::packBitsLsb(uint8_t data[8],
                        int start_lsb,
                        int len,
                        uint64_t value)
{
    if (!data || start_lsb < 0 || len <= 0 || start_lsb >= 64) {
        return;
    }

    if (start_lsb + len > 64) {
        len = 64 - start_lsb;
    }

    uint64_t u = 0;
    for (int i = 0; i < 8; ++i) {
        u |= (static_cast<uint64_t>(data[i]) << (i * 8));
    }

    const uint64_t mask =
        (len == 64) ? ~0ULL : ((1ULL << len) - 1ULL);

    u &= ~(mask << start_lsb);
    u |= ((value & mask) << start_lsb);

    for (int i = 0; i < 8; ++i) {
        data[i] = static_cast<uint8_t>((u >> (i * 8)) & 0xFFu);
    }
}

uint32_t BmsTx::rewriteRuntimeId_(uint32_t proto_id29,
                                  uint32_t instance_index)
{
    return BmsFrameRouter::rewriteRuntimeId(proto_id29, instance_index);
}

bool BmsTx::buildFrame(const V2bCmdFields& cmd,
                       can_frame& out) const
{
    return buildFrameForInstance(cmd, out);
}

bool BmsTx::buildFrameForInstance(const V2bCmdFields& cmd,
                                  can_frame& out) const
{
    if (!cmd.valid) {
        return false;
    }

    if (!v2b_msg_ || !sig_life_) {
        return false;
    }

    std::memset(&out, 0, sizeof(out));

    out.can_id = CAN_EFF_FLAG | rewriteRuntimeId_(v2b_cmd_id29_, cmd.instance_index);
    out.can_dlc = 8;

    packBitsLsb(out.data, sig_life_->startbit_lsb, sig_life_->length, cmd.life_signal);

    if (sig_hv_) {
        packBitsLsb(out.data, sig_hv_->startbit_lsb, sig_hv_->length, cmd.hv_onoff);
    }

    if (sig_aux1_) {
        packBitsLsb(out.data, sig_aux1_->startbit_lsb, sig_aux1_->length, cmd.aux1_onoff);
    }

    if (sig_aux2_) {
        packBitsLsb(out.data, sig_aux2_->startbit_lsb, sig_aux2_->length, cmd.aux2_onoff);
    }

    if (sig_aux3_) {
        packBitsLsb(out.data, sig_aux3_->startbit_lsb, sig_aux3_->length, cmd.aux3_onoff);
    }

    if (sig_speed_) {
        packBitsLsb(out.data, sig_speed_->startbit_lsb, sig_speed_->length, cmd.vehicle_speed);
    }

    if (sig_main_pos_st_) {
        packBitsLsb(out.data, sig_main_pos_st_->startbit_lsb, sig_main_pos_st_->length,
                    cmd.main_pos_relay_st);
    }

    if (sig_main_pos_flt_) {
        packBitsLsb(out.data, sig_main_pos_flt_->startbit_lsb, sig_main_pos_flt_->length,
                    cmd.main_pos_relay_flt);
    }

    if (sig_main_neg_st_) {
        packBitsLsb(out.data, sig_main_neg_st_->startbit_lsb, sig_main_neg_st_->length,
                    cmd.main_neg_relay_st);
    }

    if (sig_main_neg_flt_) {
        packBitsLsb(out.data, sig_main_neg_flt_->startbit_lsb, sig_main_neg_flt_->length,
                    cmd.main_neg_relay_flt);
    }

    if (sig_chrg_pos_st_) {
        packBitsLsb(out.data, sig_chrg_pos_st_->startbit_lsb, sig_chrg_pos_st_->length,
                    cmd.chrg_pos_relay_st);
    }

    if (sig_chrg_pos_flt_) {
        packBitsLsb(out.data, sig_chrg_pos_flt_->startbit_lsb, sig_chrg_pos_flt_->length,
                    cmd.chrg_pos_relay_flt);
    }

    if (sig_heat_pos_st_) {
        packBitsLsb(out.data, sig_heat_pos_st_->startbit_lsb, sig_heat_pos_st_->length,
                    cmd.heat_pos_relay_st);
    }

    if (sig_heat_pos_flt_) {
        packBitsLsb(out.data, sig_heat_pos_flt_->startbit_lsb, sig_heat_pos_flt_->length,
                    cmd.heat_pos_relay_flt);
    }

    if (sig_heat_neg_st_) {
        packBitsLsb(out.data, sig_heat_neg_st_->startbit_lsb, sig_heat_neg_st_->length,
                    cmd.heat_neg_relay_st);
    }

    if (sig_heat_neg_flt_) {
        packBitsLsb(out.data, sig_heat_neg_flt_->startbit_lsb, sig_heat_neg_flt_->length,
                    cmd.heat_neg_relay_flt);
    }

    if (sig_aux1_relay_st_) {
        packBitsLsb(out.data, sig_aux1_relay_st_->startbit_lsb, sig_aux1_relay_st_->length,
                    cmd.aux1_relay_st);
    }

    if (sig_aux1_relay_flt_) {
        packBitsLsb(out.data, sig_aux1_relay_flt_->startbit_lsb, sig_aux1_relay_flt_->length,
                    cmd.aux1_relay_flt);
    }

    if (sig_aux2_relay_st_) {
        packBitsLsb(out.data, sig_aux2_relay_st_->startbit_lsb, sig_aux2_relay_st_->length,
                    cmd.aux2_relay_st);
    }

    if (sig_aux2_relay_flt_) {
        packBitsLsb(out.data, sig_aux2_relay_flt_->startbit_lsb, sig_aux2_relay_flt_->length,
                    cmd.aux2_relay_flt);
    }

    if (sig_aux3_relay_st_) {
        packBitsLsb(out.data, sig_aux3_relay_st_->startbit_lsb, sig_aux3_relay_st_->length,
                    cmd.aux3_relay_st);
    }

    if (sig_aux3_relay_flt_) {
        packBitsLsb(out.data, sig_aux3_relay_flt_->startbit_lsb, sig_aux3_relay_flt_->length,
                    cmd.aux3_relay_flt);
    }

    if (sig_aux4_relay_st_) {
        packBitsLsb(out.data, sig_aux4_relay_st_->startbit_lsb, sig_aux4_relay_st_->length,
                    cmd.aux4_relay_st);
    }

    if (sig_aux4_relay_flt_) {
        packBitsLsb(out.data, sig_aux4_relay_flt_->startbit_lsb, sig_aux4_relay_flt_->length,
                    cmd.aux4_relay_flt);
    }

    if (sig_reserved1_) {
        packBitsLsb(out.data, sig_reserved1_->startbit_lsb, sig_reserved1_->length,
                    cmd.reserved1);
    }

    if (sig_reserved2_) {
        packBitsLsb(out.data, sig_reserved2_->startbit_lsb, sig_reserved2_->length,
                    cmd.reserved2);
    }

    for (const auto& o : u8_overrides_) {
        const auto* s = ProtoBmsTableRuntime::findSignal(v2b_msg_, o.sig_name.c_str());
        if (!s) {
            continue;
        }

        if (s->length > 8) {
            continue;
        }

        packBitsLsb(out.data, s->startbit_lsb, s->length, o.value);
    }

    return true;
}

bool BmsTx::tickSend()
{
    if (!drv_) {
        return false;
    }

    if (!v2b_msg_ || !sig_life_) {
        if (!bindSignals_()) {
            return false;
        }
    }

    V2bCmdFields cmd;
    cmd.valid = true;
    cmd.instance_index = 1;
    cmd.can_index = can_index_;
    cmd.life_signal = life_;
    cmd.hv_onoff = hv_onoff_;
    cmd.source = "legacy_tick";

    // 旧接口里的 cmd_enable_ 保留变量，不再落位到当前 V2B_CMD 协议
    (void)cmd_enable_;

    can_frame fr{};
    if (!buildFrameForInstance(cmd, fr)) {
        LOGD("[BMS][TX] buildFrameForInstance failed");
        return false;
    }

    const bool ok = drv_->sendCan(can_index_, fr);
    if (!ok) {
        LOGD("[BMS][TX] sendCan failed can=%d id=0x%08X",
             can_index_, static_cast<unsigned>(fr.can_id & CAN_EFF_MASK));
        return false;
    }

    life_ = static_cast<uint8_t>(life_ + 1);
    return true;
}

} // namespace proto::bms