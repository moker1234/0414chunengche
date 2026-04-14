//
// Created by lxy on 2026/3/11.
//

#include "bms_command_manager.h"
#include <cstdio>
#include <cstring>

namespace control::bms {
    const char* BmsCommandManager::reasonText_(BmsCmdReason r)
    {
        switch (r) {
        case BmsCmdReason::Init:            return "Init";
        case BmsCmdReason::NoData:          return "NoData";
        case BmsCmdReason::Offline:         return "Offline";
        case BmsCmdReason::RqHvPowerOff:    return "RqHvPowerOff";
        case BmsCmdReason::FaultLevelBlock: return "FaultLevelBlock";
        case BmsCmdReason::FireFaultBlock:  return "FireFaultBlock";
        case BmsCmdReason::TmsFaultBlock:   return "TmsFaultBlock";
        case BmsCmdReason::AllowClose:      return "AllowClose";
        case BmsCmdReason::FallbackPowerOff:return "FallbackPowerOff";
        default:                            return "Unknown";
        }
    }

    const char* BmsCommandManager::hvText_(uint32_t hv_onoff)
    {
        switch (hv_onoff) {
        case 0: return "Reserved";
        case 1: return "PowerOn";
        case 2: return "PowerOff";
        case 3: return "Invalid";
        default:return "Unknown";
        }
    }

    std::string BmsCommandManager::canIdHex_(uint32_t can_id)
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%08X", (can_id & CAN_EFF_MASK));
        return std::string(buf);
    }

    std::string BmsCommandManager::frameHex_(const can_frame& fr)
    {
        std::string s;
        s.reserve(fr.can_dlc * 3);

        char buf[4];
        for (uint8_t i = 0; i < fr.can_dlc; ++i) {
            std::snprintf(buf, sizeof(buf), "%02X", fr.data[i]);
            if (!s.empty()) s.push_back(' ');
            s.append(buf);
        }
        return s;
    }

bool BmsCommandManager::init(DriverManager& drv_mgr, int default_can_index)
{
    default_can_index_ = default_can_index;
    return tx_.init(drv_mgr, default_can_index_);
}

std::string BmsCommandManager::makeName_(uint32_t instance_index)
{
    return "BMS_" + std::to_string(instance_index);
}

BmsCommandState& BmsCommandManager::ensureState_(uint32_t instance_index)
{
    auto& st = states_[instance_index];
    if (st.desired.instance_index == 0) {
        st.desired.instance_index = instance_index;
        st.desired.can_index = default_can_index_;
        st.desired.hv_onoff = 2;       // 默认 PowerOff，更保守
        st.desired.system_enable = 1;
        st.desired.source = "init";

        st.last_sent = st.desired;
        st.reason = BmsCmdReason::Init;
    }
    return st;
}

void BmsCommandManager::rebuildDesiredFromCache(const BmsLogicCache& cache, uint64_t ts_ms)
{
    for (uint32_t idx = 1; idx <= 4; ++idx) {
        auto& st = ensureState_(idx);

        st.desired.instance_index = idx;
        st.desired.can_index = default_can_index_;
        st.desired.request_ts_ms = ts_ms;
        st.desired.source = "logic";

        auto it = cache.items.find(makeName_(idx));
        if (it == cache.items.end()) {
            st.desired.hv_onoff = 2;
            st.desired.system_enable = 1;
            st.desired.vehicle_speed = 0;
            st.reason = BmsCmdReason::NoData;
            st.last_build_ts_ms = ts_ms;
            continue;
        }

        const auto& x = it->second;

        if (!x.online) {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::Offline;
        }
        else if (x.rq_hv_power_off) {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::RqHvPowerOff;
        }
        else if (x.fire_fault_level >= 2) {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::FireFaultBlock;
        }
        else if (x.fault_level >= 2) {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::FaultLevelBlock;
        }
        else if (x.tms_fault_level >= 2) {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::TmsFaultBlock;
        }
        else if (x.hv_allow_close) {
            st.desired.hv_onoff = 1;
            st.reason = BmsCmdReason::AllowClose;
        }
        else {
            st.desired.hv_onoff = 2;
            st.reason = BmsCmdReason::FallbackPowerOff;
        }

        st.desired.system_enable = 1;
        st.desired.vehicle_speed = 0;

        // 当前阶段附件回路先清零，后续 HMI / 策略再接
        st.desired.aux1_onoff = 0;
        st.desired.aux2_onoff = 0;
        st.desired.aux3_onoff = 0;

        // 继电器状态先按“未知/保守”清零
        st.desired.main_pos_relay_st = 0;
        st.desired.main_pos_relay_flt = 0;
        st.desired.main_neg_relay_st = 0;
        st.desired.main_neg_relay_flt = 0;
        st.desired.prechg_relay_st = 0;
        st.desired.prechg_relay_flt = 0;

        st.last_build_ts_ms = ts_ms;
    }
}

void BmsCommandManager::emitPeriodicCommands(uint64_t ts_ms,
                                             std::vector<control::Command>& out_cmds,
                                             uint32_t period_ms)
{
    for (uint32_t idx = 1; idx <= 4; ++idx) {
        auto& st = ensureState_(idx);

        if (st.last_send_ts_ms != 0 && (ts_ms - st.last_send_ts_ms) < period_ms) {
            continue;
        }

        auto cmd_fields = st.desired;
        cmd_fields.life_signal = st.last_life;
        cmd_fields.request_ts_ms = ts_ms;

        can_frame fr{};
        if (!tx_.buildFrameForInstance(cmd_fields, fr)) {
            continue;
        }

        control::Command c;
        c.type = control::Command::Type::SendCan;
        c.can.can_index = cmd_fields.can_index;
        c.can.frame = fr;
        out_cmds.push_back(c);

        st.last_sent = cmd_fields;
        st.has_last_sent = true;
        st.last_send_ts_ms = ts_ms;
        st.last_frame = fr;
        st.has_last_frame = true;
        st.last_life = static_cast<uint8_t>(st.last_life + 1);
    }
}

    std::map<uint32_t, BmsCommandView>
BmsCommandManager::buildCommandView(uint64_t now_ms, uint32_t alive_timeout_ms) const
{
    std::map<uint32_t, BmsCommandView> out;

    for (const auto& kv : states_) {
        const uint32_t idx = kv.first;
        const auto& st = kv.second;

        BmsCommandView v;
        v.instance_index = idx;
        v.can_index = st.desired.can_index;
        v.hv_onoff = static_cast<int>(st.desired.hv_onoff);
        v.system_enable = static_cast<int>(st.desired.system_enable);
        v.life_signal = static_cast<int>(st.last_life);
        v.last_build_ts_ms = st.last_build_ts_ms;
        v.last_send_ts_ms = st.last_send_ts_ms;
        v.has_last_sent = st.has_last_sent;
        v.valid = st.desired.valid;
        v.source = st.desired.source;

        v.reason_code = static_cast<int>(st.reason);
        v.reason_text = reasonText_(st.reason);
        v.hv_text = hvText_(st.desired.hv_onoff);

        if (!st.has_last_sent) {
            v.tx_state_text = "Idle";
        } else if (v.tx_alive) {
            v.tx_state_text = "Active";
        } else {
            v.tx_state_text = "Stale";
        }

        if (st.last_send_ts_ms != 0 && now_ms >= st.last_send_ts_ms) {
            v.last_send_age_ms = static_cast<double>(now_ms - st.last_send_ts_ms);
        } else {
            v.last_send_age_ms = 0.0;
        }

        if (st.has_last_frame) {
            v.dlc = static_cast<int>(st.last_frame.can_dlc);
            v.can_id_hex = canIdHex_(st.last_frame.can_id);
            v.frame_hex = frameHex_(st.last_frame);
        } else {
            v.dlc = 0;
            v.can_id_hex.clear();
            v.frame_hex.clear();
        }

        if (st.last_send_ts_ms != 0 && now_ms >= st.last_send_ts_ms) {
            const uint64_t age = now_ms - st.last_send_ts_ms;
            v.tx_alive = (age <= alive_timeout_ms);
        } else {
            v.tx_alive = false;
        }

        out[idx] = std::move(v);
    }

    return out;
}

} // namespace control::bms