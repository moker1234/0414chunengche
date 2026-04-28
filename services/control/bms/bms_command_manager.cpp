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
        case BmsCmdReason::Init:               return "Init";
        case BmsCmdReason::NoData:             return "NoData";
        case BmsCmdReason::Offline:            return "Offline";

        case BmsCmdReason::RqHvPowerOff:       return "RqHvPowerOff";
        case BmsCmdReason::FaultLevelBlock:    return "FaultLevelBlock";
        case BmsCmdReason::FireFaultBlock:     return "FireFaultBlock";
        case BmsCmdReason::TmsFaultBlock:      return "TmsFaultBlock";

        case BmsCmdReason::AllowClose:         return "AllowClose";
        case BmsCmdReason::FallbackPowerOff:   return "FallbackPowerOff";

        case BmsCmdReason::RuntimeStale:       return "RuntimeStale";
        case BmsCmdReason::St2Stale:           return "St2Stale";
        case BmsCmdReason::Fault1Stale:        return "Fault1Stale";
        case BmsCmdReason::Fault2Stale:        return "Fault2Stale";
        case BmsCmdReason::CurrentLimitStale:  return "CurrentLimitStale";
        case BmsCmdReason::NotReady:           return "NotReady";

        default:                               return "Unknown";
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

    bool BmsCommandManager::init(DriverManager& drv_mgr,
                                 int default_can_index,
                                 uint32_t v2b_cmd_id29)
    {
        default_can_index_ = default_can_index;
        v2b_cmd_id29_ = v2b_cmd_id29;

        tx_.setV2bCmdId29(v2b_cmd_id29_);
        return tx_.init(drv_mgr, default_can_index_);
    }

std::string BmsCommandManager::makeName_(uint32_t instance_index)
{
    return "BMS_" + std::to_string(instance_index);
}

    BmsCmdReason BmsCommandManager::safetyBlockReason_(const BmsPerInstanceCache& x)
    {
        // 1) 实例级在线是第一前提
        if (!x.seen_once) {
            return BmsCmdReason::NoData;
        }

        if (!x.online) {
            return BmsCmdReason::Offline;
        }

        // 2) 关键运行态组不能陈旧
        // runtime_fault_stale 是 Logic Tick 根据关键组 aging 统一算出的真源。
        if (x.runtime_fault_stale) {
            return BmsCmdReason::RuntimeStale;
        }

        // 3) 关键报文必须已经出现且处于 fresh 状态
        // ST2：fault_level / fault_code / rq_hv_power_off / pack 电压电流 SOC 等核心控制量
        if (x.last_st2_ms == 0 || !x.st2_online) {
            return BmsCmdReason::St2Stale;
        }

        // Fault1/Fault2：BMS 关键故障位来源
        if (x.last_fault1_ms == 0 || !x.fault1_online) {
            return BmsCmdReason::Fault1Stale;
        }

        if (x.last_fault2_ms == 0 || !x.fault2_online) {
            return BmsCmdReason::Fault2Stale;
        }

        // CurrentLimit：当前项目里参与充放电控制，作为上电前置条件。
        // 如果后续确认 CurrentLimit 不参与上电安全，可把这一段降级为告警而不是阻断。
        if (x.last_current_limit_ms == 0 || !x.current_limit_online) {
            return BmsCmdReason::CurrentLimitStale;
        }

        // 4) 显式下高压 / 故障等级阻断
        if (x.rq_hv_power_off) {
            return BmsCmdReason::RqHvPowerOff;
        }

        if (x.fire_fault_level >= 2) {
            return BmsCmdReason::FireFaultBlock;
        }

        if (x.fault_level >= 2) {
            return BmsCmdReason::FaultLevelBlock;
        }

        if (x.tms_fault_level >= 2) {
            return BmsCmdReason::TmsFaultBlock;
        }

        // 5) 协议/业务故障层面的阻断
        if (!x.hv_allow_close) {
            return BmsCmdReason::FallbackPowerOff;
        }

        return BmsCmdReason::AllowClose;
    }

    BmsCommandState& BmsCommandManager::ensureState_(uint32_t instance_index)
    {
        auto& st = states_[instance_index];
        if (st.desired.instance_index == 0) {
            st.desired.instance_index = instance_index;
            st.desired.can_index = default_can_index_;
            st.desired.hv_onoff = 2;
            st.desired.system_enable = 0;
            st.desired.reserved1 = 0;
            st.desired.reserved2 = 0;
            st.desired.source = "init";

            st.last_sent = st.desired;
            st.reason = BmsCmdReason::Init;
        }
        return st;
    }

    /*
     * 获取或创建指定实例的 desired 命令
     * @param instance_index BMS 实例索引
     * @param source 命令来源
     * @return 指向 desired 命令的指针
     * @note 调用者负责维护命令的 valid 标志位
     */
    proto::bms::V2bCmdFields* BmsCommandManager::mutableDesired(uint32_t instance_index,
                                                            const char* source)
    {
        auto& st = ensureState_(instance_index);

        st.desired.instance_index = instance_index;
        st.desired.can_index = default_can_index_;
        st.desired.valid = true;

        if (source && *source) {
            st.desired.source = source;
        }

        return &st.desired;
    }

    const proto::bms::V2bCmdFields* BmsCommandManager::desired(uint32_t instance_index) const
    {
        auto it = states_.find(instance_index);
        if (it == states_.end()) return nullptr;
        return &it->second.desired;
    }

void BmsCommandManager::rebuildDesiredFromCache(const BmsLogicCache& cache, uint64_t ts_ms)
{
    auto resetPassiveFields = [](proto::bms::V2bCmdFields& d)
    {
        d.vehicle_speed = 0;

        d.aux1_onoff = 0;
        d.aux2_onoff = 0;
        d.aux3_onoff = 0;

        d.main_pos_relay_st = 0;
        d.main_pos_relay_flt = 0;
        d.main_neg_relay_st = 0;
        d.main_neg_relay_flt = 0;

        d.chrg_pos_relay_st = 0;
        d.chrg_pos_relay_flt = 0;

        d.heat_pos_relay_st = 0;
        d.heat_pos_relay_flt = 0;
        d.heat_neg_relay_st = 0;
        d.heat_neg_relay_flt = 0;

        d.aux1_relay_st = 0;
        d.aux1_relay_flt = 0;
        d.aux2_relay_st = 0;
        d.aux2_relay_flt = 0;
        d.aux3_relay_st = 0;
        d.aux3_relay_flt = 0;
        d.aux4_relay_st = 0;
        d.aux4_relay_flt = 0;

        d.reserved1 = 0;
        d.reserved2 = 0;

        d.prechg_relay_st = 0;
        d.prechg_relay_flt = 0;
        d.system_enable = 0;
    };

    for (uint32_t idx = 1; idx <= 4; ++idx) {
        auto& st = ensureState_(idx);

        st.desired.instance_index = idx;
        st.desired.can_index = default_can_index_;
        st.desired.request_ts_ms = ts_ms;
        st.desired.source = "logic";
        st.desired.valid = true;

        resetPassiveFields(st.desired);

        auto it = cache.items.find(makeName_(idx));
        if (it == cache.items.end()) {
            st.desired.hv_onoff = 2; // PowerOff
            st.reason = BmsCmdReason::NoData;
            st.last_build_ts_ms = ts_ms;
            continue;
        }

        const auto& x = it->second;
        const BmsCmdReason reason = safetyBlockReason_(x);

        if (reason == BmsCmdReason::AllowClose) {
            // 这里保持原有行为：满足安全条件时，logic 默认允许上高压。
            // 后续 BusinessEngine 可在本周期把 desired 改成 PowerOff。
            st.desired.hv_onoff = 1; // PowerOn
        } else {
            st.desired.hv_onoff = 2; // PowerOff
        }

        st.reason = reason;
        st.last_build_ts_ms = ts_ms;
    }
}
    void BmsCommandManager::enforceSafetyFromCache(const BmsLogicCache& cache, uint64_t ts_ms)
    {
        for (uint32_t idx = 1; idx <= 4; ++idx) {
            auto& st = ensureState_(idx);

            st.desired.instance_index = idx;
            st.desired.can_index = default_can_index_;
            st.desired.request_ts_ms = ts_ms;
            st.desired.valid = true;

            auto it = cache.items.find(makeName_(idx));
            if (it == cache.items.end()) {
                st.desired.hv_onoff = 2; // PowerOff
                st.reason = BmsCmdReason::NoData;
                st.last_build_ts_ms = ts_ms;
                continue;
            }

            const auto& x = it->second;
            const BmsCmdReason reason = safetyBlockReason_(x);

            if (reason != BmsCmdReason::AllowClose) {
                // 安全条件不满足时，无论业务层刚才写了什么，都强制 PowerOff。
                st.desired.hv_onoff = 2;
                st.reason = reason;
            } else {
                // 安全条件满足时，保留业务层的本周期选择。
                // 如果业务层没有动过，通常会保持 rebuildDesiredFromCache() 给出的 PowerOn。
                if (st.desired.hv_onoff == 1) {
                    st.reason = BmsCmdReason::AllowClose;
                } else {
                    st.reason = BmsCmdReason::FallbackPowerOff;
                }
            }

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

            if (st.last_send_ts_ms != 0 && now_ms >= st.last_send_ts_ms) {
                const uint64_t age = now_ms - st.last_send_ts_ms;
                v.last_send_age_ms = static_cast<double>(age);
                v.tx_alive = (age <= alive_timeout_ms);
            } else {
                v.last_send_age_ms = 0.0;
                v.tx_alive = false;
            }

            if (!st.has_last_sent) {
                v.tx_state_text = "Idle";
            } else if (v.tx_alive) {
                v.tx_state_text = "Active";
            } else {
                v.tx_state_text = "Stale";
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

            out[idx] = std::move(v);
        }

        return out;
    }

} // namespace control::bms