#include "business_engine.h"

#include <cmath>

#include "../logic/logic_context.h"
#include "../bms/bms_command_manager.h"
#include "../../aggregator/system_snapshot.h"

namespace control::business
{
    namespace
    {
        static std::string makeBmsName_(uint32_t idx)
        {
            return "BMS_" + std::to_string(idx);
        }
    } // namespace

    bool BusinessEngine::testDi(const LogicContext& ctx, int channel_id)
    {
        if (channel_id < 1 || channel_id > 64) return false;
        const int bit = channel_id - 1;
        return (((ctx.di_bits >> bit) & 0x1ULL) != 0ULL);
    }

    double BusinessEngine::readAiVoltage(const LogicContext& ctx, int adc_channel)
    {
        if (adc_channel < 1) return std::nan("");
        const std::size_t idx = static_cast<std::size_t>(adc_channel - 1);
        if (idx >= ctx.ai.size()) return std::nan("");
        return ctx.ai[idx];
    }

    bms::BmsPerInstanceCache* BusinessEngine::getBms(LogicContext& ctx, uint32_t instance_index)
    {
        const std::string key = makeBmsName_(instance_index);
        auto it = ctx.bms_cache.items.find(key);
        if (it == ctx.bms_cache.items.end()) return nullptr;
        return &it->second;
    }

    const bms::BmsPerInstanceCache* BusinessEngine::getBms(const LogicContext& ctx, uint32_t instance_index)
    {
        const std::string key = makeBmsName_(instance_index);
        auto it = ctx.bms_cache.items.find(key);
        if (it == ctx.bms_cache.items.end()) return nullptr;
        return &it->second;
    }

    PcuOnlineState* BusinessEngine::getPcu(LogicContext& ctx, uint32_t instance_index)
    {
        if (instance_index == 0) return &ctx.pcu0_state;
        if (instance_index == 1) return &ctx.pcu1_state;
        return nullptr;
    }

    const PcuOnlineState* BusinessEngine::getPcu(const LogicContext& ctx, uint32_t instance_index)
    {
        if (instance_index == 0) return &ctx.pcu0_state;
        if (instance_index == 1) return &ctx.pcu1_state;
        return nullptr;
    }

    const SnapshotItem* BusinessEngine::findSnapshotItem(const LogicContext& ctx, const std::string& device_name)
    {
        auto it = ctx.latest_snapshot.items.find(device_name);
        if (it == ctx.latest_snapshot.items.end()) return nullptr;
        return &it->second;
    }

    bool BusinessEngine::isSnapshotOnline(const LogicContext& ctx, const std::string& device_name)
    {
        const auto* item = findSnapshotItem(ctx, device_name);
        return item ? item->online : false;
    }

    void BusinessEngine::emitWriteDo(std::vector<Command>& out_cmds,
                                     int channel_id,
                                     bool on)
    {
        Command c;
        c.type = Command::Type::WriteDo;
        c.write_do.channel_id = channel_id;
        c.write_do.value = on;
        out_cmds.push_back(std::move(c));
    }

    void BusinessEngine::evaluate(LogicContext& ctx,
                                  uint64_t ts_ms,
                                  bms::BmsCommandManager& bms_mgr,
                                  std::vector<Command>& out_cmds)
    {
        (void)ts_ms;

        // =========================================================
        // 这里给你放“示例骨架”，你后续主要改这里
        // =========================================================

        // ---------- 1) 读取本地 IO ----------
        const bool estop_pressed = testDi(ctx, 1); // DI1：急停

        const bool gun_di_13 = testDi(ctx, 13); // DI13
        const bool gun_di_14 = testDi(ctx, 14); // DI14
        const bool gun_di_15 = testDi(ctx, 15); // DI15
        const bool gun_di_16 = testDi(ctx, 16); // DI16
        const bool gun_di_17 = testDi(ctx, 17); // DI17
        const bool gun_di_18 = testDi(ctx, 18); // DI18
        const double gun_adc1_v = readAiVoltage(ctx, 1); // ADC1
        const double gun_adc2_v = readAiVoltage(ctx, 2); // ADC2
        const auto* bms1 = getBms(ctx, 1);
        const auto* bms2 = getBms(ctx, 2);
        const auto* bms3 = getBms(ctx, 3);
        const auto* bms4 = getBms(ctx, 4);
        const bool bms1_exists = (bms1 != nullptr);
        const bool bms2_exists = (bms2 != nullptr);
        const bool bms3_exists = (bms3 != nullptr);
        const bool bms4_exists = (bms4 != nullptr);


        const bool gun_1_state = gun_di_13;
        const bool gun_2_state = gun_di_14;
        const bool gun_3_state = gun_di_15;
        const bool gun_4_state = gun_di_16;
        const bool gun_5_state = gun_di_17;
        const bool gun_6_state = gun_di_18;
        const bool gun_7_state = gun_adc1_v > 8.0;
        const bool gun_8_state = gun_adc2_v > 8.0;
        const bool gun_state_all_ok = gun_1_state && gun_2_state && gun_3_state && gun_4_state && gun_5_state && gun_6_state &&
            gun_7_state && gun_8_state;
        const bool gun_state_all_off = !gun_1_state && !gun_2_state && !gun_3_state && !gun_4_state && !gun_5_state && !gun_6_state &&
            !gun_7_state && !gun_8_state;

        const bool bms1_state_ok = bms1_exists ? (bms1->st1_bms_hv_status != 0) : false;
        const bool bms2_state_ok = bms2_exists ? (bms2->st1_bms_hv_status != 0) : false;
        const bool bms3_state_ok = bms3_exists ? (bms3->st1_bms_hv_status != 0) : false;
        const bool bms4_state_ok = bms4_exists ? (bms4->st1_bms_hv_status != 0) : false;
        const bool bms_state_ok = bms1_state_ok != 0 && bms2_state_ok != 0 && bms3_state_ok != 0 && bms4_state_ok != 0;

        const bool bms1_chrgState = bms1_exists ? (bms1->st1_chrg_mode != 0) : false;
        const bool bms2_chrgState = bms2_exists ? (bms2->st1_chrg_mode != 0) : false;
        const bool bms3_chrgState = bms3_exists ? (bms3->st1_chrg_mode != 0) : false;
        const bool bms4_chrgState = bms4_exists ? (bms4->st1_chrg_mode != 0) : false;
        const bool bms_chrgState_noChrg = bms1_chrgState == 0 && bms2_chrgState == 0 && bms3_chrgState == 0 &&
            bms4_chrgState == 0;

        const bool smoke_warning = ctx.smoke_faults.smoke_alarm;
        const bool gas_hign_warning = ctx.gas_faults.high_alarm;

        const bool pcu0_estop = getPcu(ctx, 0)->estop;
        const bool pcu1_estop = getPcu(ctx, 1)->estop;
        const bool pcu_estop = pcu0_estop || pcu1_estop;

        static bool out_hv_on = false;
        static bool out_hv_off = false;


        // 2.2.3 PCU故障急停
        if (pcu_estop)
            out_hv_off = true;

        //2.2 动力电池
        // 2.2.1 放电控制
        // 2.2.1.1 高压上电
        if (true) // hmi上按下上电按键
        {
            if (gun_state_all_ok && !estop_pressed && bms_state_ok && bms_chrgState_noChrg)
            {
                out_hv_on = true;
            }
        }

        //2.3.1.3 故障下电
        if (estop_pressed || smoke_warning || gas_hign_warning)
        {
            out_hv_off = true;
        }


        static bool final_hv_state = false;
        if (out_hv_off)
        {
            final_hv_state = false;
        }
        else if (out_hv_on)
        {
            final_hv_state = true;
        }
        else
        {
            final_hv_state = false;
        }

        for (uint32_t idx = 1; idx <= 4; ++idx)
        {
            auto* cmd = bms_mgr.mutableDesired(idx, "business");
            if (!cmd) continue;
            cmd->hv_onoff = final_hv_state+1; // PowerOn
        }
    }
} // namespace control::business
