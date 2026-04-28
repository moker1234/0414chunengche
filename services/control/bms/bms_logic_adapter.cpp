//
// Created by lxy on 2026/3/10.
//

#include "bms_logic_adapter.h"

#include <cctype>
#include <string>

#include "logger.h"
#include "./bms_logic_adapter_oneFactor.h"

namespace control::bms
{
    bool BmsLogicAdapter::isBmsDevice_(const DeviceData& d)
    {
        return d.device_name == "BMS";
    }

    uint32_t BmsLogicAdapter::parseInstanceIndex_(const DeviceData& d)
    {
        auto itv = d.value.find("__bms.instance_index");
        if (itv != d.value.end())
        {
            const int32_t v = itv->second;
            if (v >= 1 && v <= 4) return static_cast<uint32_t>(v);
        }

        auto its = d.str.find("__bms.instance");
        if (its != d.str.end())
        {
            const std::string& s = its->second; // BMS_1
            if (s.rfind("BMS_", 0) == 0 && s.size() > 4)
            {
                const char c = s[4];
                if (c >= '1' && c <= '4') return static_cast<uint32_t>(c - '0');
            }
        }

        auto itid = d.value.find("__bms.id");
        if (itid != d.value.end())
        {
            const uint32_t id29 = static_cast<uint32_t>(itid->second);
            const uint32_t low12 = (id29 & 0xFFFu);
            const uint32_t idx = (low12 >> 8) & 0xFu;
            if (idx >= 1 && idx <= 4) return idx;
        }

        return 0;
    }

    std::string BmsLogicAdapter::parseInstanceName_(const DeviceData& d, uint32_t idx)
    {
        auto its = d.str.find("__bms.instance");
        if (its != d.str.end() && !its->second.empty())
        {
            return its->second;
        }

        if (idx >= 1 && idx <= 4)
        {
            return "BMS_" + std::to_string(idx);
        }
        return "BMS_0";
    }

    const char* BmsLogicAdapter::parseMsgName_(const DeviceData& d)
    {
        auto its = d.str.find("__bms.msg");
        if (its != d.str.end() && !its->second.empty())
        {
            return its->second.c_str();
        }
        return "UNKNOWN";
    }

    std::string BmsLogicAdapter::parseRawHex_(const DeviceData& d)
    {
        auto its = d.str.find("__bms.raw_hex");
        return (its != d.str.end()) ? its->second : std::string{};
    }

    bool BmsLogicAdapter::readNum_(const DeviceData& d,
                                   std::initializer_list<const char*> keys,
                                   double& out)
    {
        for (const char* k : keys)
        {
            auto it = d.num.find(k);
            if (it != d.num.end())
            {
                out = it->second;
                return true;
            }
        }
        return false;
    }

    bool BmsLogicAdapter::readInt_(const DeviceData& d,
                                   std::initializer_list<const char*> keys,
                                   int32_t& out)
    {
        for (const char* k : keys)
        {
            auto itv = d.value.find(k);
            if (itv != d.value.end())
            {
                out = itv->second;
                return true;
            }
            auto its = d.status.find(k);
            if (its != d.status.end())
            {
                out = static_cast<int32_t>(its->second);
                return true;
            }
            auto itn = d.num.find(k);
            if (itn != d.num.end())
            {
                out = static_cast<int32_t>(itn->second);
                return true;
            }
        }
        return false;
    }

    bool BmsLogicAdapter::readBool_(const DeviceData& d,
                                    std::initializer_list<const char*> keys,
                                    bool& out)
    {
        int32_t v = 0;
        if (!readInt_(d, keys, v)) return false;
        out = (v != 0);
        return true;
    }

    static bool isSocInvalid_(double v)
    {
        // 协议原始 invalid 常见是 255，物理量按 0.4 缩放后常见会变成 102
        // 这里兼容两种情况：
        // 1) parser 还没做正规化，直接透出 255
        // 2) parser 已经做 factor，透出 102.0
        return (v >= 254.5 && v <= 255.5) || (v >= 101.5 && v <= 102.5);
    }

    static bool isSohInvalid_(double v)
    {
        return (v >= 254.5 && v <= 255.5) || (v >= 101.5 && v <= 102.5);
    }

    static bool isPackCurrentInvalid_(double v)
    {
        // 原始 invalid 常见 65535，若 factor/offset 后仍异常大，也视为 invalid
        return (v > 60000.0 || v < -60000.0);
    }

    static bool isPackVoltageInvalid_(double v)
    {
        // 电池包电压不应是极端异常值
        return (v > 60000.0 || v < -1.0);
    }

    static bool isTempInvalid_(double v)
    {
        // 工程上先做运行态约束，避免明显非法值进入 logic_view
        return (v < -100.0 || v > 300.0);
    }

    static bool isCurrentLimitInvalid_(double v)
    {
        return (v < -1.0 || v > 5000.0);
    }

    static bool isInsResInvalid_(double v)
    {
        // 协议表 max 为 60000 kohm；运行态超过这个范围或为负值都视为无效
        return (v < 0.0 || v > 60000.0);
    }

    void BmsLogicAdapter::updateFromFault1_(const DeviceData& d, BmsPerInstanceCache& x)
{
    int32_t iv = 0;
    bool bv = false;

    auto pick_level = [&](std::initializer_list<const char*> keys, int32_t& out) -> bool
    {
        return readInt_(d, keys, out);
    };

    auto pick_bool = [&](std::initializer_list<const char*> keys, bool& out) -> bool
    {
        return readBool_(d, keys, out);
    };

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_DelTemp",
                       "B2V_Fult1_DelTemp",
                   }, iv))
    {
        x.f1_del_temp = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_OverTemp",
                       "B2V_Fult1_OverTemp",
                   }, iv))
    {
        x.f1_over_temp = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_OverUcell",
                       "B2V_Fult1_OverUcell",
                   }, iv))
    {
        x.f1_over_ucell = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_LowUcell",
                       "B2V_Fult1_LowUcell",
                   }, iv))
    {
        x.f1_low_ucell = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_PackOverHVolt",
                       "B2V_Fult1_PackOverHVolt",
                   }, iv))
    {
        x.f1_pack_over_hvolt = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_PackLowHVolt",
                       "B2V_Fult1_PackLowHVolt",
                   }, iv))
    {
        x.f1_pack_low_hvolt = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_LowSOC",
                       "B2V_Fult1_LowSOC",
                   }, iv))
    {
        x.f1_low_soc = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_LowInsRes",
                       "B2V_Fult1_LowInsRes",
                   }, iv))
    {
        x.f1_low_ins_res = iv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_UcellUniformity",
                      "B2V_Fult1_UcellUniformity",
                  }, bv))
    {
        x.f1_ucell_uniformity = bv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_OverChg",
                      "B2V_Fult1_OverChg",
                  }, bv))
    {
        x.f1_over_chg = bv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_OverSOC",
                      "B2V_Fult1_OverSOC",
                  }, bv))
    {
        x.f1_over_soc = bv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_SOCChangeFast",
                      "B2V_Fult1_SOCChangeFast",
                  }, bv))
    {
        x.f1_soc_change_fast = bv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_BatSysNotMatch",
                      "B2V_Fult1_BatSysNotMatch",
                  }, bv))
    {
        x.f1_bat_sys_not_match = bv;
    }

    if (pick_bool({
                      "bms.B2V_Fault1.B2V_Fult1_HVILFault",
                      "B2V_Fult1_HVILFault",
                  }, bv))
    {
        x.f1_hvil_fault = bv;
    }

    if (pick_level({
                       "bms.B2V_Fault1.B2V_Fult1_FaultNum_32960",
                       "B2V_Fult1_FaultNum_32960",
                   }, iv))
    {
        x.f1_fault_num = iv;
    }
}

   void BmsLogicAdapter::updateFromFault2_(const DeviceData& d, BmsPerInstanceCache& x)
{
    auto pick_bool = [&](std::initializer_list<const char*> keys) -> bool
    {
        bool bv = false;
        return readBool_(d, keys, bv) ? bv : false;
    };

    auto pick_level = [&](std::initializer_list<const char*> keys, int32_t& out) -> bool
    {
        return readInt_(d, keys, out);
    };

    int32_t iv = 0;

    x.f2_cell_over_dischrg = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_CellOverDischrg",
        "B2V_Fult2_CellOverDischrg",
    });

    x.f2_cell_low_temp = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_CellLowTemp",
        "B2V_Fult2_CellLowTemp",
    });

    x.f2_pack_fire_warning = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PackFireWarning",
        "B2V_Fult2_PackFireWarning",
    });

    if (pick_level({
                       "bms.B2V_Fault2.B2V_Fult2_OverDischrgCurr",
                       "B2V_Fult2_OverDischrgCurr",
                   }, iv))
    {
        x.f2_over_dischrg_curr_level = iv;
    }

    if (pick_level({
                       "bms.B2V_Fault2.B2V_Fult2_OverChrgCurrInTheDrive",
                       "B2V_Fult2_OverChrgCurrInTheDrive",
                   }, iv))
    {
        x.f2_over_chrg_curr_in_drive_level = iv;
    }

    x.f2_curr_sensor_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_CurrSensorErr",
        "B2V_Fult2_CurrSensorErr",
    });

    x.f2_power_supply_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PowerSupplyErr",
        "B2V_Fult2_PowerSupplyErr",
    });

    x.f2_soc_differ_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_SOCDifferErr",
        "B2V_Fult2_SOCDifferErr",
    });

    x.f2_chrg_connect_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ChrgConnectErr",
        "B2V_Fult2_ChrgConnectErr",
    });

    x.f2_over_dischrg_curr_when_in_chrg = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_OverDischrgCurrWhenInChrg",
        "B2V_Fult2_OverDischrgCurrWhenInChrg",
    });

    x.f2_over_chrg_curr_in_the_chrg = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_OverChrgCurrInTheChrg",
        "B2V_Fult2_OverChrgCurrInTheChrg",
    });

    x.f2_chrg_ntc_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ChrgNTCErr",
        "B2V_Fult2_ChrgNTCErr",
    });

    if (pick_level({
                       "bms.B2V_Fault2.B2V_Fult2_ChrgNTCTempOver",
                       "B2V_Fult2_ChrgNTCTempOver",
                   }, iv))
    {
        x.f2_chrg_ntc_temp_over_level = iv;
    }

    x.f2_tms_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_TMSErr",
        "B2V_Fult2_TMSErr",
    });

    x.f2_pack_self_protect = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PackSelfProtect",
        "B2V_Fult2_PackSelfProtect",
    });

    x.f2_main_loop_prechg_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_MainLoopPrechrgErr",
        "B2V_Fult2_MainLoopPrechrgErr",
    });

    x.f2_aux_loop_prechg_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_AuxLoopPrechrgErr",
        "B2V_Fult2_AuxLoopPrechrgErr",
    });

    x.f2_chrg_ins_low_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ChrgInsLowErr",
        "B2V_Fult2_ChrgInsLowErr",
    });

    x.f2_acan_lost = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ACANLost",
        "B2V_Fult2_ACANLost",
    });

    x.f2_inner_comm_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_InnerCommunicationErr",
        "B2V_Fult2_InnerCommunicationErr",
    });

    x.f2_dcdc_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCDCErr",
        "B2V_Fult2_DCDCErr",
    });

    x.f2_branch_break_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_BranchBreakErr",
        "B2V_Fult2_BranchBreakErr",
    });

    x.f2_heat_relay_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_HeatRelayOpenErr",
        "B2V_Fult2_HeatRelayOpenErr",
    });

    x.f2_heat_relay_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_HeatRelayWeldErr",
        "B2V_Fult2_HeatRelayWeldErr",
    });

    x.f2_main_pos_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_MainPosRelayOpenErr",
        "B2V_Fult2_MainPosRelayOpenErr",
    });

    x.f2_main_pos_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_MainPosRelayWeldErr",
        "B2V_Fult2_MainPosRelayWeldErr",
    });

    x.f2_main_neg_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_MainNegRelayOpenErr",
        "B2V_Fult2_MainNegRelayOpenErr",
    });

    x.f2_main_neg_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_MainNegRelayWeldErr",
        "B2V_Fult2_MainNegRelayWeldErr",
    });

    x.f2_dc_chrg_pos1_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgPos1RelayOpenErr",
        "B2V_Fult2_DCChrgPos1RelayOpenErr",
    });

    x.f2_dc_chrg_pos1_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgPos1RelayWeldErr",
        "B2V_Fult2_DCChrgPos1RelayWeldErr",
    });

    x.f2_dc_chrg_neg1_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgNeg1RelayOpenErr",
        "B2V_Fult2_DCChrgNeg1RelayOpenErr",
    });

    x.f2_dc_chrg_neg1_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgNeg1RelayWeldErr",
        "B2V_Fult2_DCChrgNeg1RelayWeldErr",
    });

    x.f2_dc_chrg_pos2_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgPos2RelayOpenErr",
        "B2V_Fult2_DCChrgPos2RelayOpenErr",
    });

    x.f2_dc_chrg_pos2_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgPos2RelayWeldErr",
        "B2V_Fult2_DCChrgPos2RelayWeldErr",
    });

    x.f2_dc_chrg_neg2_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgNeg2RelayOpenErr",
        "B2V_Fult2_DCChrgNeg2RelayOpenErr",
    });

    x.f2_dc_chrg_neg2_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_DCChrgNeg2RelayWeldErr",
        "B2V_Fult2_DCChrgNeg2RelayWeldErr",
    });

    x.f2_ac_chrg_pos_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ACChrgPosRelayOpenErr",
        "B2V_Fult2_ACChrgPosRelayOpenErr",
    });

    x.f2_ac_chrg_pos_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ACChrgPosRelayWeldErr",
        "B2V_Fult2_ACChrgPosRelayWeldErr",
    });

    x.f2_ac_chrg_neg_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ACChrgNegRelayOpenErr",
        "B2V_Fult2_ACChrgNegRelayOpenErr",
    });

    x.f2_ac_chrg_neg_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_ACChrgNegRelayWeldErr",
        "B2V_Fult2_ACChrgNegRelayWeldErr",
    });

    x.f2_panto_chrg_pos_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PantoChrgPosRelayOpenErr",
        "B2V_Fult2_PantoChrgPosRelayOpenErr",
    });

    x.f2_panto_chrg_pos_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PantoChrgPosRelayWeldErr",
        "B2V_Fult2_PantoChrgPosRelayWeldErr",
    });

    x.f2_panto_chrg_neg_open_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PantoChrgNegRelayOpenErr",
        "B2V_Fult2_PantoChrgNegRelayOpenErr",
    });

    x.f2_panto_chrg_neg_weld_err = pick_bool({
        "bms.B2V_Fault2.B2V_Fult2_PantoChrgNegRelayWeldErr",
        "B2V_Fult2_PantoChrgNegRelayWeldErr",
    });
}

    void BmsLogicAdapter::updateFromSt1_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        int32_t iv = 0;

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_MainPosRelaySt",
                         "B2V_ST1_MainPosRelaySt",
                     }, iv))
        {
            x.st1_main_pos_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_MainNegRelaySt",
                         "B2V_ST1_MainNegRelaySt",
                     }, iv))
        {
            x.st1_main_neg_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_PrechrgRelaySt",
                         "B2V_ST1_PrechrgRelaySt"
                     }, iv))
        {
            x.st1_prechrg_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_HeatPosRelaySt",
                         "B2V_ST1_HeatPosRelaySt"
                     }, iv))
        {
            x.st1_heat_pos_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_HeatNegRelaySt",
                         "B2V_ST1_HeatNegRelaySt",
                     }, iv))
        {
            x.st1_heat_neg_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_BMS_HVStatus",
                         "B2V_ST1_BMS_HVStatus",
                     }, iv))
        {
            x.st1_bms_hv_status = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_BMS_BalanceStatus",
                         "B2V_ST1_BMS_BalanceStatus",
                     }, iv))
        {
            x.st1_balance_status = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_DCChrgConnectSt",
                         "B2V_ST1_DCChrgConnectSt",
                     }, iv))
        {
            x.st1_dc_chrg_connect_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_PantoChrgConnectSt",
                         "B2V_ST1_PantoChrgConnectSt",
                     }, iv))
        {
            x.st1_panto_chrg_connect_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_ACChrgConnectSt",
                         "B2V_ST1_ACChrgConnectSt",
                     }, iv))
        {
            x.st1_ac_chrg_connect_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_ChrgMode",
                         "B2V_ST1_ChrgMode",
                     }, iv))
        {
            x.st1_chrg_mode = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_ChrgStatus",
                         "B2V_ST1_ChrgStatus",
                     }, iv))
        {
            x.st1_chrg_status = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_HeatingStatus",
                         "B2V_ST1_HeatingStatus",
                     }, iv))
        {
            x.st1_heating_status = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_CoolingStatus",
                         "B2V_ST1_CoolingStatus",
                     }, iv))
        {
            x.st1_cooling_status = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_RechrgCycels",
                         "B2V_ST1_RechrgCycels",
                     }, iv))
        {
            x.st1_rechrg_cycles = iv;
        }
        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_DCChrgPos1RelaySt",
                         "B2V_ST1_DCChrgPos1RelaySt",
                     }, iv))
        {
            x.st1_dc_chrg_pos1_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_DCChrgNeg1RelaySt",
                         "B2V_ST1_DCChrgNeg1RelaySt",
                     }, iv))
        {
            x.st1_dc_chrg_neg1_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_DCChrgPos2RelaySt",
                         "B2V_ST1_DCChrgPos2RelaySt",
                     }, iv))
        {
            x.st1_dc_chrg_pos2_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_DCChrgNeg2RelaySt",
                         "B2V_ST1_DCChrgNeg2RelaySt",
                     }, iv))
        {
            x.st1_dc_chrg_neg2_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_PantoChrgPosRelaySt",
                         "B2V_ST1_PantoChrgPosRelaySt",
                     }, iv))
        {
            x.st1_panto_chrg_pos_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_PantoChrgNegRelaySt",
                         "B2V_ST1_PantoChrgNegRelaySt",
                     }, iv))
        {
            x.st1_panto_chrg_neg_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_ACChrgPosRelaySt",
                         "B2V_ST1_ACChrgPosRelaySt",
                     }, iv))
        {
            x.st1_ac_chrg_pos_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_ACChrgNegRelaySt",
                         "B2V_ST1_ACChrgNegRelaySt",
                     }, iv))
        {
            x.st1_ac_chrg_neg_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_Aux1RelaySt",
                         "B2V_ST1_Aux1RelaySt",
                     }, iv))
        {
            x.st1_aux1_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_Aux2RelaySt",
                         "B2V_ST1_Aux2RelaySt",
                     }, iv))
        {
            x.st1_aux2_relay_st = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST1.B2V_ST1_Aux3RelaySt",
                         "B2V_ST1_Aux3RelaySt",
                     }, iv))
        {
            x.st1_aux3_relay_st = iv;
        }
    }

    void BmsLogicAdapter::updateFromSt2_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;
        int32_t iv = 0;
        bool bv = false;

        if (readNum_(d, {
                         "bms.B2V_ST2.B2V_ST2_SOC",
                         "B2V_ST2_SOC",
                     }, v))
        {
            if (!isSocInvalid_(v))
            {
                x.soc = v;
                x.soc_valid = true;
            }
            else
            {
                x.soc_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST2.B2V_ST2_SOH",
                         "B2V_ST2_SOH",
                     }, v))
        {
            if (!isSohInvalid_(v))
            {
                x.soh = v;
                x.soh_valid = true;
            }
            else
            {
                x.soh_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST2.B2V_ST2_PackInsideVolt",
                         "B2V_ST2_PackInsideVolt",
                     }, v))
        {
            if (!isPackVoltageInvalid_(v))
            {
                x.pack_v = v;
                x.pack_v_valid = true;
            }
            else
            {
                x.pack_v_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST2.B2V_ST2_PackCurrent",
                         "B2V_ST2_PackCurrent",
                     }, v))
        {
            if (!isPackCurrentInvalid_(v))
            {
                x.pack_i = v;
                x.pack_i_valid = true;
            }
            else
            {
                x.pack_i = 0.0;
                x.pack_i_valid = false;
            }
        }

        if (readInt_(d, {
                         "bms.B2V_ST2.B2V_ST2_FaultLevel",
                         "B2V_ST2_FaultLevel",
                     }, iv))
        {
            x.fault_level = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST2.B2V_ST2_FaultCode",
                         "B2V_ST2_FaultCode",
                     }, iv))
        {
            x.fault_code = iv;
        }

        if (readBool_(d, {
                          "bms.B2V_ST2.B2V_ST2_RqHVPoerOff",
                          "B2V_ST2_RqHVPoerOff",
                      }, bv))
        {
            x.rq_hv_power_off = bv;
        }

        x.alarm_any = (x.fault_level > 0) || x.rq_hv_power_off;
    }

    void BmsLogicAdapter::updateFromSt3_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;
        bool bv = false;

        if (readNum_(d, {
                         "bms.B2V_ST3.B2V_ST3_PosInsRes",
                         "B2V_ST3_PosInsRes",
                     }, v))
        {
            if (!isInsResInvalid_(v))
            {
                x.st3_ins_pos_res = v;
                x.st3_ins_pos_valid = true;
            }
            else
            {
                x.st3_ins_pos_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST3.B2V_ST3_NegInsRes",
                         "B2V_ST3_NegInsRes",
                     }, v))
        {
            if (!isInsResInvalid_(v))
            {
                x.st3_ins_neg_res = v;
                x.st3_ins_neg_valid = true;
            }
            else
            {
                x.st3_ins_neg_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST3.B2V_ST3_SysInsRes",
                         "B2V_ST3_SysInsRes",
                     }, v))
        {
            if (!isInsResInvalid_(v))
            {
                x.st3_ins_sys_res = v;
                x.st3_ins_sys_valid = true;
            }
            else
            {
                x.st3_ins_sys_valid = false;
            }
        }

        if (readBool_(d, {
                          "bms.B2V_ST3.B2V_ST3_InsDetectorSt",
                          "B2V_ST3_InsDetectorSt",
                      }, bv))
        {
            x.st3_ins_detector_on = bv;
            x.st3_ins_detector_valid = true;
        }
    }

    void BmsLogicAdapter::updateFromSt4_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;

        if (readNum_(d, {
                         "bms.B2V_ST4.B2V_ST4_Max_Temp",
                         "B2V_ST4_Max_Temp",
                     }, v))
        {
            if (!isTempInvalid_(v))
            {
                x.st4_temp_max = v;
                x.st4_temp_max_valid = true;
            }
            else
            {
                x.st4_temp_max_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST4.B2V_ST4_Min_Temp",
                         "B2V_ST4_Min_Temp",
                     }, v))
        {
            if (!isTempInvalid_(v))
            {
                x.st4_temp_min = v;
                x.st4_temp_min_valid = true;
            }
            else
            {
                x.st4_temp_min_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ST4.B2V_ST4_Avg_Temp",
                         "B2V_ST4_Avg_Temp",
                     }, v))
        {
            if (!isTempInvalid_(v))
            {
                x.st4_temp_avg = v;
                x.st4_temp_avg_valid = true;
            }
            else
            {
                x.st4_temp_avg_valid = false;
            }
        }

        int32_t iv = 0;

        // MaxTemp_Position
        if (readInt_(d, {
                         "bms.B2V_ST4.B2V_ST4_MaxTemp_Position",
                         "B2V_ST4_MaxTemp_Position",
                     }, iv))
        {
            x.st4_max_temp_pos = iv;
            x.st4_max_temp_pos_valid = true;
        }
        else
        {
            x.st4_max_temp_pos = 0;
            x.st4_max_temp_pos_valid = false;
        }

        // MinTemp_Position
        if (readInt_(d, {
                         "bms.B2V_ST4.B2V_ST4_MinTemp_Position",
                         "B2V_ST4_MinTemp_Position",
                     }, iv))
        {
            x.st4_min_temp_pos = iv;
            x.st4_min_temp_pos_valid = true;
        }
        else
        {
            x.st4_min_temp_pos = 0;
            x.st4_min_temp_pos_valid = false;
        }
    }

    void BmsLogicAdapter::updateFromSt5_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;

        if (readNum_(d, {
                         "bms.B2V_ST5.B2V_ST5_Max_Ucell",
                         "B2V_ST5_Max_Ucell",
                     }, v))
        {
            x.st5_max_ucell = v;
            x.st5_max_ucell_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST5.B2V_ST5_Min_Ucell",
                         "B2V_ST5_Min_Ucell",
                     }, v))
        {
            x.st5_min_ucell = v;
            x.st5_min_ucell_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST5.B2V_ST5_Avg_Ucell",
                         "B2V_ST5_Avg_Ucell",
                     }, v))
        {
            x.st5_avg_ucell = v;
            x.st5_avg_ucell_valid = true;
        }
    }

    void BmsLogicAdapter::updateFromSt6_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        int32_t iv = 0;

        if (readInt_(d, {
                         "bms.B2V_ST6.B2V_ST6_MaxUcell_CSCNo",
                         "B2V_ST6_MaxUcell_CSCNo",
                     }, iv))
        {
            x.st6_max_ucell_csc_no = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST6.B2V_ST6_MaxUcell_Position",
                         "B2V_ST6_MaxUcell_Position",
                     }, iv))
        {
            x.st6_max_ucell_position = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST6.B2V_ST6_MinUcell_CSCNo",
                         "B2V_ST6_MinUcell_CSCNo",
                     }, iv))
        {
            x.st6_min_ucell_csc_no = iv;
        }

        if (readInt_(d, {
                         "bms.B2V_ST6.B2V_ST6_MinUcell_Position",
                         "B2V_ST6_MinUcell_Position",
                     }, iv))
        {
            x.st6_min_ucell_position = iv;
        }
    }

    void BmsLogicAdapter::updateFromSt7_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_Gun1DCPosTemp",
                         "B2V_ST7_Gun1DCPosTemp",
                     }, v))
        {
            x.st7_gun1_dc_pos_temp = v;
            x.st7_gun1_dc_pos_temp_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_Gun1DCNegTemp",
                         "B2V_ST7_Gun1DCNegTemp",
                     }, v))
        {
            x.st7_gun1_dc_neg_temp = v;
            x.st7_gun1_dc_neg_temp_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_Gun2DCPosTemp",
                         "B2V_ST7_Gun2DCPosTemp",
                     }, v))
        {
            x.st7_gun2_dc_pos_temp = v;
            x.st7_gun2_dc_pos_temp_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_Gun2DCNegTemp",
                         "B2V_ST7_Gun2DCNegTemp",
                     }, v))
        {
            x.st7_gun2_dc_neg_temp = v;
            x.st7_gun2_dc_neg_temp_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_GunACPosTemp",
                         "B2V_ST7_GunACPosTemp",
                     }, v))
        {
            x.st7_gun_ac_pos_temp = v;
            x.st7_gun_ac_pos_temp_valid = true;
        }

        if (readNum_(d, {
                         "bms.B2V_ST7.B2V_ST7_GunACNegTemp",
                         "B2V_ST7_GunACNegTemp",
                     }, v))
        {
            x.st7_gun_ac_neg_temp = v;
            x.st7_gun_ac_neg_temp_valid = true;
        }
    }

    void BmsLogicAdapter::updateFromElecEnergy_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;

        if (readNum_(d, {
                         "bms.B2V_ElecEnergy.B2V_TotChgEnergy",
                         "B2V_TotChgEnergy",
                     }, v))
        {
            if (v >= 0.0 && v <= 20000000.0)
            {
                x.EE_tot_chg_energy = v;
                x.EE_tot_chg_energy_valid = true;
            }
            else
            {
                x.EE_tot_chg_energy_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ElecEnergy.B2V_TotDischgEnergy",
                         "B2V_TotDischgEnergy",
                     }, v))
        {
            if (v >= 0.0 && v <= 20000000.0)
            {
                x.EE_tot_dischg_energy = v;
                x.EE_tot_dischg_energy_valid = true;
            }
            else
            {
                x.EE_tot_dischg_energy_valid = false;
            }
        }

        if (readNum_(d, {
                         "bms.B2V_ElecEnergy.B2V_SingleChgEnergy",
                         "B2V_SingleChgEnergy",
                     }, v))
        {
            if (v >= 0.0 && v <= 6553.5)
            {
                x.EE_single_chg_energy = v;
                x.EE_single_chg_energy_valid = true;
            }
            else
            {
                x.EE_single_chg_energy_valid = false;
            }
        }
    }

    void BmsLogicAdapter::updateFromCurrentLimit_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        double v = 0.0;

        // 1) 脉冲放电电流
        if (readNum_(d, {
                         "bms.B2V_CurrentLimit.B2V_AvailInpulseDischrgCurr",
                         "B2V_AvailInpulseDischrgCurr",
                     }, v))
        {
            if (!isCurrentLimitInvalid_(v))
            {
                x.CL_pulse_discharge_limit_a = v;
                x.CL_pulse_discharge_limit_valid = true;
            }
            else
            {
                x.CL_pulse_discharge_limit_valid = false;
            }
        }

        // 2) 脉冲充电电流
        if (readNum_(d, {
                         "bms.B2V_CurrentLimit.B2V_AvailInpulseChrgCurr",
                         "B2V_AvailInpulseChrgCurr",
                     }, v))
        {
            if (!isCurrentLimitInvalid_(v))
            {
                x.CL_pulse_charge_limit_a = v;
                x.CL_pulse_charge_limit_valid = true;
            }
            else
            {
                x.CL_pulse_charge_limit_valid = false;
            }
        }

        // 3) 持续充电电流
        if (readNum_(d, {
                         "bms.B2V_CurrentLimit.B2V_AvailFollowChrgCurr",
                         "B2V_AvailFollowChrgCurr",

                     }, v))
        {
            if (!isCurrentLimitInvalid_(v))
            {
                x.CL_follow_charge_limit_a = v;
                x.CL_follow_charge_limit_valid = true;

                // 兼容旧字段
                x.charge_limit_a = v;
                x.charge_limit_valid = true;
            }
            else
            {
                x.CL_follow_charge_limit_valid = false;
                x.charge_limit_valid = false;
            }
        }

        // 4) 持续放电电流
        if (readNum_(d, {
                         "bms.B2V_CurrentLimit.B2V_AvailFollowDishrgCurr",
                         "B2V_AvailFollowDishrgCurr",
                     }, v))
        {
            if (!isCurrentLimitInvalid_(v))
            {
                x.CL_follow_discharge_limit_a = v;
                x.CL_follow_discharge_limit_valid = true;

                // 兼容旧字段
                x.discharge_limit_a = v;
                x.discharge_limit_valid = true;
            }
            else
            {
                x.CL_follow_discharge_limit_valid = false;
                x.discharge_limit_valid = false;
            }
        }
    }

    void BmsLogicAdapter::updateFromTm2b_(const DeviceData& d, BmsPerInstanceCache& x)
    {
        int32_t iv = 0;

        auto pick_int = [&](std::initializer_list<const char*> keys, int32_t& out) -> bool
        {
            return readInt_(d, keys, out);
        };

        // 1) 工作状态
        if (pick_int({
                         "bms.TM2B_Info.T2B_WorkStatus_A",
                         "bms.TM2B_Info.TM2B_Info_WorkState",
                         "T2B_WorkStatus_A",
                         "TM2B_Info_WorkState",
                     }, iv))
        {
            x.tms_work_state = iv;
        }

        // 2) 故障码：这次最关键，必须真正接入
        if (pick_int({
                         "bms.TM2B_Info.TM2B_FaultCode_A",
                         "TM2B_FaultCode_A",
                     }, iv))
        {
            x.tms_fault_code = iv;

            // 关键 TMS 子故障按 code 锁存时间，供 evaluator 做 confirmed 投影
            if (iv == 0x03) {
                x.tms_panel_aircon_seen_ms = x.last_rx_ms;
            } else if (iv == 0x1C) {
                x.tms_ats_water_pump_seen_ms = x.last_rx_ms;
            }
        }

        // 3) 故障等级
        if (pick_int({
                         "bms.TM2B_Info.TM2B_FaultLevel_A",
                         "bms.TM2B_Info.TM2B_Info_FaultLevel",
                         "TM2B_FaultLevel_A",
                         "TM2B_Info_FaultLevel",
                     }, iv))
        {
            x.tms_fault_level = iv;
        }
    }
void BmsLogicAdapter::updateFromFire2b_(const DeviceData& d, BmsPerInstanceCache& x)
{
    int32_t iv = 0;
    double dv = 0.0;
    bool bv = false;

    if (readInt_(d, {
                     "bms.Fire2B_state.Fire_valueNumber",
                     "Fire_valueNumber",
                 }, iv))
    {
        x.fire_value_number = iv;
    }

    if (readInt_(d, {
                     "bms.Fire2B_state.Fire_valueAlarmLevel",
                     "Fire_valueAlarmLevel",
                 }, iv))
    {
        x.fire_value_alarm_level = iv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_byStartSts",
                      "Fire_byStartSts",
                  }, bv))
    {
        x.fire_by_start_sts = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_SnerrrorSts",
                      "Fire_SnerrrorSts",
                  }, bv))
    {
        x.fire_sensor_error_sts = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_HwerrrorSts",
                      "Fire_HwerrrorSts",
                  }, bv))
    {
        x.fire_hw_error_sts = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_byTempSenor1Err",
                      "Fire_byTempSenor1Err",
                  }, bv))
    {
        x.fire_temp_sensor1_err = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_byTempSenor2Err",
                      "Fire_byTempSenor2Err",
                  }, bv))
    {
        x.fire_temp_sensor2_err = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_byAirSenorErr",
                      "Fire_byAirSenorErr",
                  }, bv))
    {
        x.fire_air_sensor_err = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_bySAMSenorErr",
                      "Fire_bySAMSenorErr",
                  }, bv))
    {
        x.fire_sam_sensor_err = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_Tmp_alarm",
                      "Fire_Tmp_alarm",
                  }, bv))
    {
        x.fire_temp_alarm = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_Gus_alarm",
                      "Fire_Gus_alarm",
                  }, bv))
    {
        x.fire_gas_alarm = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_Fog_alarm",
                      "Fire_Fog_alarm",
                  }, bv))
    {
        x.fire_fog_alarm = bv;
    }

    if (readBool_(d, {
                      "bms.Fire2B_state.Fire_Leak_alarm",
                      "Fire_Leak_alarm",
                  }, bv))
    {
        x.fire_leak_alarm = bv;
    }

    if (readNum_(d, {
                     "bms.Fire2B_state.Fire_Max_Tmp",
                     "Fire_Max_Tmp",
                 }, dv))
    {
        x.fire_max_temp = dv;
        x.fire_max_temp_valid = true;
    }

    if (readNum_(d, {
                     "bms.Fire2B_state.Fire_Max_Gus",
                     "Fire_Max_Gus",
                 }, dv))
    {
        x.fire_max_gas = dv;
        x.fire_max_gas_valid = true;
    }

    if (readNum_(d, {
                     "bms.Fire2B_state.Fire_Max_Fog",
                     "Fire_Max_Fog",
                 }, dv))
    {
        x.fire_max_fog = dv;
        x.fire_max_fog_valid = true;
    }


}

    static int calcFaultSummaryCode_(const BmsPerInstanceCache& x)
    {
        // 0 = None
        // 1 = AlarmOnly
        // 2 = RqHvPowerOff
        // 3 = BmsFaultBlock
        // 4 = FireFaultBlock
        // 5 = TmsFaultBlock
        // 6 = OfflineOrStale

        const bool fire_any =
            (x.fire_value_alarm_level > 0) ||
            x.fire_by_start_sts ||
            x.fire_sensor_error_sts ||
            x.fire_hw_error_sts;

        const bool tms_any =
            (x.tms_fault_level > 0) ||
            (x.tms_fault_code != 0);

        if (fire_any) return 4;
        if (x.fault_level >= 2) return 3;
        if (tms_any) return 5;
        if (x.rq_hv_power_off) return 2;
        if (x.alarm_any) return 1;
        return 0;
    }

    static const char* calcFaultSummaryText_(int code)
    {
        switch (code)
        {
        case 0: return "None";
        case 1: return "AlarmOnly";
        case 2: return "RqHvPowerOff";
        case 3: return "BmsFaultBlock";
        case 4: return "FireFaultBlock";
        case 5: return "TmsFaultBlock";
        case 6: return "OfflineOrStale";
        default: return "Unknown";
        }
    }

    static const char* calcFaultSourceText_(const BmsPerInstanceCache& x)
    {
        const bool fire_any =
            (x.fire_value_alarm_level > 0) ||
            x.fire_by_start_sts ||
            x.fire_sensor_error_sts ||
            x.fire_hw_error_sts;

        const bool tms_any =
            (x.tms_fault_level > 0) ||
            (x.tms_fault_code != 0);

        if (fire_any) return "Fire2B";
        if (x.fault_level >= 2) return "B2V_ST2";
        if (tms_any) return "TM2B";
        if (x.rq_hv_power_off) return "B2V_ST2";
        if (x.alarm_any) return "BMS";
        return "None";
    }

    static void rebuildUnifiedRawFaults_(BmsPerInstanceCache& x)
    {
        // ------------------------------------------------------------
        // 第二批：把旧 F1/F2/TMS/Fire 原始位，统一折叠成“业务故障真源”
        //
        // 原则：
        // 1) 这里只做“真源整理”，不做持续确认，不做 HMI 落码
        // 2) 命名以故障表业务语义为主，而不是沿用 F1/F2 原始信号名
        // 3) 某些业务项当前没有完整 parser 真源时，先用“最接近、最保守”的现有字段映射
        // ------------------------------------------------------------

        // 电流传感器故障：第二批补上 F2 CurrSensorErr 原始位后，直接映射
        x.raw_current_sensor_fault = x.f2_curr_sensor_err;

        // 低压供电异常：对应 F2 PowerSupplyErr
        x.raw_low_voltage_supply_alarm = x.f2_power_supply_err;

        // SOC 跳变：当前最接近原始位是 F1 SOCChangeFast
        x.raw_soc_jump_alarm = x.f1_soc_change_fast;

        // Fire：按 workbook 指定的 4 个关键字段直接折叠
        x.raw_fire_alarm =
            (x.fire_value_alarm_level > 0) ||
            x.fire_by_start_sts ||
            x.fire_sensor_error_sts ||
            x.fire_hw_error_sts;

        // TMS：以 TM2B_FaultCode_A 为主真源，FaultLevel_A / F2_TMSErr 为兼容补充
        x.raw_tms_unit_fault =
            (x.tms_fault_code != 0) ||
            (x.tms_fault_level > 0) ||
            x.f2_tms_err;

        // 包自保护
        x.raw_battery_self_protect_fault = x.f2_pack_self_protect;

        // 预充故障：主回路/辅件回路预充故障统一折叠
        x.raw_precharge_fault =
            x.f2_main_loop_prechg_err ||
            x.f2_aux_loop_prechg_err;

        // 充电绝缘低
        x.raw_charge_insulation_low_alarm = x.f2_chrg_ins_low_err;

        // 通讯类
        x.raw_acan_comm_fault = x.f2_acan_lost;
        x.raw_internal_comm_fault = x.f2_inner_comm_err;
        x.raw_branch_circuit_open_fault = x.f2_branch_break_err;
        x.raw_hvil_alarm = x.f1_hvil_fault;

        // 可充电储能系统不匹配
        x.raw_storage_mismatch_alarm = x.f1_bat_sys_not_match;

        // 充电接口 / 电流 / 回路类
        x.raw_charge_gun_connection_abnormal = x.f2_chrg_connect_err;
        x.raw_charge_discharge_current_overflow = x.f2_over_dischrg_curr_when_in_chrg;
        x.raw_charge_current_overflow_alarm = x.f2_over_chrg_curr_in_the_chrg;
        x.raw_charge_connector_ntc_fault = x.f2_chrg_ntc_err;

        // 第八批：按温度等级值拆分 connector overtemp 两级
        // 约定：
        //  level == 1 -> lvl1
        //  level >= 2 -> lvl2
        x.raw_charge_connector_overtemp_lvl1 =
            (x.f2_chrg_ntc_temp_over_level == 1);

        x.raw_charge_connector_overtemp_lvl2 =
            (x.f2_chrg_ntc_temp_over_level >= 2);

        // 内侧高压回路断路故障：当前程序没有独立 parser 位，
        // 先保守映射为“支路断路 + 主回路关键接触器开路类”的聚合真源。
        x.raw_internal_hv_circuit_open_fault =
            x.f2_branch_break_err ||
            x.f2_main_pos_open_err ||
            x.f2_main_neg_open_err;

        // 接触器/回路 open-weld 统一业务真源
        x.raw_heat_relay_open_fault = x.f2_heat_relay_open_err;
        x.raw_heat_relay_weld_fault = x.f2_heat_relay_weld_err;

        x.raw_main_pos_relay_open_fault = x.f2_main_pos_open_err;
        x.raw_main_pos_relay_weld_fault = x.f2_main_pos_weld_err;
        x.raw_main_neg_relay_open_fault = x.f2_main_neg_open_err;
        x.raw_main_neg_relay_weld_fault = x.f2_main_neg_weld_err;

        x.raw_dc_chrg_pos1_relay_open_fault = x.f2_dc_chrg_pos1_open_err;
        x.raw_dc_chrg_pos1_relay_weld_fault = x.f2_dc_chrg_pos1_weld_err;
        x.raw_dc_chrg_neg1_relay_open_fault = x.f2_dc_chrg_neg1_open_err;
        x.raw_dc_chrg_neg1_relay_weld_fault = x.f2_dc_chrg_neg1_weld_err;

        x.raw_dc_chrg_pos2_relay_open_fault = x.f2_dc_chrg_pos2_open_err;
        x.raw_dc_chrg_pos2_relay_weld_fault = x.f2_dc_chrg_pos2_weld_err;
        x.raw_dc_chrg_neg2_relay_open_fault = x.f2_dc_chrg_neg2_open_err;
        x.raw_dc_chrg_neg2_relay_weld_fault = x.f2_dc_chrg_neg2_weld_err;

        x.raw_ac_chrg_pos_relay_open_fault = x.f2_ac_chrg_pos_open_err;
        x.raw_ac_chrg_pos_relay_weld_fault = x.f2_ac_chrg_pos_weld_err;
        x.raw_ac_chrg_neg_relay_open_fault = x.f2_ac_chrg_neg_open_err;
        x.raw_ac_chrg_neg_relay_weld_fault = x.f2_ac_chrg_neg_weld_err;

        x.raw_panto_chrg_pos_relay_open_fault = x.f2_panto_chrg_pos_open_err;
        x.raw_panto_chrg_pos_relay_weld_fault = x.f2_panto_chrg_pos_weld_err;
        x.raw_panto_chrg_neg_relay_open_fault = x.f2_panto_chrg_neg_open_err;
        x.raw_panto_chrg_neg_relay_weld_fault = x.f2_panto_chrg_neg_weld_err;
    }

    void BmsLogicAdapter::onDeviceData(const DeviceData& d, uint64_t ts_ms, BmsLogicCache& cache) const
    {
        if (!isBmsDevice_(d)) return;

        const uint32_t idx = parseInstanceIndex_(d);
        const std::string instance_name = parseInstanceName_(d, idx);
        const char* msg_name = parseMsgName_(d);

        auto& x = cache.ensure(instance_name, idx);

        x.seen_once = true;
        x.last_rx_ms = ts_ms;
        x.last_ok_ms = ts_ms;
        x.last_msg_name = msg_name ? msg_name : "UNKNOWN";

        if (x.instance_name.empty())
        {
            x.instance_name = instance_name;
        }
        if (x.bms_index == 0 && idx >= 1 && idx <= 4)
        {
            x.bms_index = idx;
        }
        // 注意：
        // 这里不再直接把 x.online = true。
        // online / offline / disconnect_count / offline_reason
        // 统一留给第三批 Tick aging 计算。

        const std::string raw_hex = parseRawHex_(d);
        if (msg_name && *msg_name && !raw_hex.empty())
        {
            x.last_raw_hex_by_group[msg_name] = raw_hex;
        }
        if (msg_name && *msg_name)
        {
            x.last_group_rx_ms[msg_name] = ts_ms;

            if (std::string(msg_name) == "B2V_ST1") x.st1_online = true;
            else if (std::string(msg_name) == "B2V_ST2") x.st2_online = true;
            else if (std::string(msg_name) == "B2V_ST3") x.st3_online = true;
            else if (std::string(msg_name) == "B2V_ST4") x.st4_online = true;
            else if (std::string(msg_name) == "B2V_ST5") x.st5_online = true;
            else if (std::string(msg_name) == "B2V_ST6") x.st6_online = true;
            else if (std::string(msg_name) == "B2V_ST7") x.st7_online = true;
            else if (std::string(msg_name) == "B2V_ElecEnergy") x.elec_energy_online = true;
            else if (std::string(msg_name) == "B2V_CurrentLimit") x.current_limit_online = true;
            else if (std::string(msg_name) == "TM2B_Info") x.tm2b_online = true;
            else if (std::string(msg_name) == "Fire2B_Info") x.fire2b_online = true;
            else if (std::string(msg_name) == "B2V_Fault1") x.fault1_online = true;
            else if (std::string(msg_name) == "B2V_Fault2") x.fault2_online = true;
        }
        const std::string msg = msg_name ? msg_name : "UNKNOWN";
        // LOG_COMM_D("[BMS][ADAPTER] inst=%d msg=%s ts=%llu",
        //           x.bms_index, msg.c_str(), (unsigned long long)ts_ms);
        if (msg == "B2V_ST1")
        {
            updateFromSt1_(d, x);
            x.last_st1_ms = ts_ms;
        }
        else if (msg == "B2V_ST2")
        {
            updateFromSt2_(d, x);
            x.last_st2_ms = ts_ms;
        }
        else if (msg == "B2V_ST3")
        {
            updateFromSt3_(d, x);
            x.last_st3_ms = ts_ms;
        }
        else if (msg == "B2V_ST4")
        {
            updateFromSt4_(d, x);
            x.last_st4_ms = ts_ms;
        }
        else if (msg == "B2V_ST5")
        {
            updateFromSt5_(d, x);
            x.last_st5_ms = ts_ms;
        }
        else if (msg == "B2V_ST6")
        {
            updateFromSt6_(d, x);
            x.last_st6_ms = ts_ms;
        }
        else if (msg == "B2V_ST7")
        {
            updateFromSt7_(d, x);
            x.last_st7_ms = ts_ms;
        }
        else if (msg == "B2V_ElecEnergy")
        {
            updateFromElecEnergy_(d, x);
            x.last_elec_energy_ms = ts_ms;
        }
        else if (msg == "B2V_CurrentLimit")
        {
            updateFromCurrentLimit_(d, x);
            x.last_current_limit_ms = ts_ms;
        }
        else if (msg == "TM2B_Info")
        {
            updateFromTm2b_(d, x);
            x.last_tm2b_ms = ts_ms;
        }
        else if (msg == "Fire2B_Info")
        {
            updateFromFire2b_(d, x);
            x.last_fire2b_ms = ts_ms;
        }
        else if (msg == "B2V_Fault1")
        {
            updateFromFault1_(d, x);
            x.last_fault1_ms = ts_ms;
        }
        else if (msg == "B2V_Fault2")
        {
            updateFromFault2_(d, x);
            x.last_fault2_ms = ts_ms;
        }

        // 这里只判断“协议/故障层面的高压阻断”。
        // 注意：hv_allow_close 不再表示“可以直接上高压”。
        // 最终是否允许 V2B_CMD_HV_OnOff=PowerOn，需要在 BmsCommandManager 中同时检查：
        // - 实例在线
        // - ST2/Fault1/Fault2/CurrentLimit 已出现且 fresh
        // - runtime_fault_stale == false
        // - rq_hv_power_off / fault_level / fire_fault_level / tms_fault_level 等未阻断
        x.hv_should_open =
            x.rq_hv_power_off ||
            (x.fault_level >= 2) ||
            (x.fire_fault_level >= 2) ||
            x.f1_hvil_fault ||
            x.f1_over_chg ||
            (x.f1_low_ins_res >= 3) ||
            x.f2_pack_self_protect ||
            x.f2_main_loop_prechg_err ||
            x.f2_aux_loop_prechg_err ||
            x.f2_chrg_ins_low_err;

        // 协议故障层面的“无阻断”标志。
        // 发送层还会做 runtime 新鲜度钳制。
        x.hv_allow_close = !x.hv_should_open;

        // 第二批：统一整理业务故障真源
        rebuildUnifiedRawFaults_(x);

        // 收到该实例报文后，至少说明当前不是“从未收到数据”的状态
        if (x.offline_reason_code == 1)
        {
            x.offline_reason_code = 0;
            x.offline_reason_text = "None";
        }
    }

    nlohmann::json BmsLogicAdapter::buildLogicView(const BmsLogicCache& cache) const
    {
        nlohmann::json j = nlohmann::json::object();
        // const uint64_t now_ms = 0; // 这里先占位，不在 adapter 内部算“当前时刻”, 因为 buildLogicView() 当前没有 now_ms 参数，改到 logic_engine.cpp 里算更稳

        int online_count = 0;
        bool any_alarm = false;
        bool any_fault = false;
        bool any_fault_block = false;
        int block_hv_count = 0;

        bool any_ins = false;
        bool any_ins_block = false;
        int ins_count_low = 0;
        int ins_valid_count = 0;

        bool any_f1 = false;
        bool any_f1_block = false;

        bool any_f2 = false;
        bool any_f2_hv_block = false;
        bool any_f2_contact = false;
        bool any_f2_comm = false;

        for (int idx = 1; idx <= 4; ++idx)
        {
            const std::string name = "BMS_" + std::to_string(idx);
            auto it = cache.items.find(name);

            nlohmann::json one = nlohmann::json::object();
            one["online"] = 0;
            one["bms_index"] = idx;
            one["last_msg_name"] = "";
            one["soc"] = 0.0;
            one["soh"] = 0.0;
            one["pack_v"] = 0.0;
            one["pack_i"] = 0.0;
            one["st4_temp_max"] = 0.0;
            one["st4_temp_min"] = 0.0;
            one["st4_temp_avg"] = 0.0;

            one["CL_pulse_discharge_limit_a"] = 0.0;
            one["CL_pulse_charge_limit_a"] = 0.0;
            one["CL_follow_charge_limit_a"] = 0.0;
            one["CL_follow_discharge_limit_a"] = 0.0;
            one["CL_pulse_discharge_limit_valid"] = 0;
            one["CL_pulse_charge_limit_valid"] = 0;
            one["CL_follow_charge_limit_valid"] = 0;
            one["CL_follow_discharge_limit_valid"] = 0;
            one["charge_limit_a"] = 0.0;
            one["discharge_limit_a"] = 0.0;
            one["alarm_any"] = 0;
            one["fault_level"] = 0;
            one["fault_code"] = 0;

            one["st1_online"] = 0;
            one["st1_age_ms"] = 0.0;

            one["st1_main_pos_relay_st"] = 0;
            one["st1_main_neg_relay_st"] = 0;
            one["st1_prechrg_relay_st"] = 0;
            one["st1_heat_pos_relay_st"] = 0;
            one["st1_heat_neg_relay_st"] = 0;
            one["st1_bms_hv_status"] = 0;
            one["st1_balance_status"] = 0;
            one["st1_dc_chrg_connect_st"] = 0;
            one["st1_panto_chrg_connect_st"] = 0;
            one["st1_ac_chrg_connect_st"] = 0;
            one["st1_chrg_mode"] = 0;
            one["st1_chrg_status"] = 0;
            one["st1_heating_status"] = 0;
            one["st1_cooling_status"] = 0;
            one["st1_rechrg_cycles"] = 0;

            one["st1_dc_chrg_pos1_relay_st"] = 0;
            one["st1_dc_chrg_neg1_relay_st"] = 0;
            one["st1_dc_chrg_pos2_relay_st"] = 0;
            one["st1_dc_chrg_neg2_relay_st"] = 0;
            one["st1_panto_chrg_pos_relay_st"] = 0;
            one["st1_panto_chrg_neg_relay_st"] = 0;
            one["st1_ac_chrg_pos_relay_st"] = 0;
            one["st1_ac_chrg_neg_relay_st"] = 0;
            one["st1_aux1_relay_st"] = 0;
            one["st1_aux2_relay_st"] = 0;
            one["st1_aux3_relay_st"] = 0;

            one["rq_hv_power_off"] = 0;

            // ===== TMS / Fire：给 HMI 映射预留稳定字段 =====
            one["tms_work_state"] = 0;
            one["tms_fault_code"] = 0;
            one["tms_fault_level"] = 0;
            one["tms_panel_aircon_seen_ms"] = 0.0;
            one["tms_ats_water_pump_seen_ms"] = 0.0;

            one["fire_value_number"] = 0;
            one["fire_value_alarm_level"] = 0;
            one["fire_by_start_sts"] = 0;
            one["fire_sensor_error_sts"] = 0;
            one["fire_hw_error_sts"] = 0;

            one["fire_temp_sensor1_err"] = 0;
            one["fire_temp_sensor2_err"] = 0;
            one["fire_air_sensor_err"] = 0;
            one["fire_sam_sensor_err"] = 0;

            one["fire_temp_alarm"] = 0;
            one["fire_gas_alarm"] = 0;
            one["fire_fog_alarm"] = 0;
            one["fire_leak_alarm"] = 0;

            one["fire_max_temp"] = 0.0;
            one["fire_max_gas"] = 0.0;
            one["fire_max_fog"] = 0.0;

            one["fire_fault_level"] = 0;
            one["fire_fault_code"] = 0;

            one["hv_allow_close"] = 1;
            one["hv_should_open"] = 0;

            one["fault_any"] = 0;
            one["fault_block_hv"] = 0;
            one["fault_summary_code"] = 0;
            one["fault_summary_text"] = "None";
            one["fault_source_text"] = "None";

            one["f1_any"] = 0;
            one["f1_block_hv"] = 0;
            one["f1_summary_text"] = "None";

            one["f1_del_temp"] = 0;
            one["f1_over_temp"] = 0;
            one["f1_over_ucell"] = 0;
            one["f1_low_ucell"] = 0;
            one["f1_low_ins_res"] = 0;
            one["f1_ucell_uniformity"] = 0;
            one["f1_over_chg"] = 0;
            one["f1_over_soc"] = 0;
            one["f1_soc_change_fast"] = 0;
            one["f1_bat_sys_not_match"] = 0;
            one["f1_hvil_fault"] = 0;
            one["f1_fault_num"] = 0;

            one["f2_any"] = 0;
            one["f2_hv_block_any"] = 0;
            one["f2_contact_err_any"] = 0;
            one["f2_comm_err_any"] = 0;
            one["f2_summary_text"] = "None";
            one["f2_tms_err"] = 0;
            one["f2_pack_self_protect"] = 0;
            one["f2_main_loop_prechg_err"] = 0;
            one["f2_aux_loop_prechg_err"] = 0;
            one["f2_chrg_ins_low_err"] = 0;
            one["f2_acan_lost"] = 0;
            one["f2_inner_comm_err"] = 0;
            one["f2_dcdc_err"] = 0;
            one["f2_branch_break_err"] = 0;
            one["f2_heat_relay_open_err"] = 0;
            one["f2_heat_relay_weld_err"] = 0;
            one["f2_main_pos_open_err"] = 0;
            one["f2_main_pos_weld_err"] = 0;
            one["f2_main_neg_open_err"] = 0;
            one["f2_main_neg_weld_err"] = 0;

            // ===== ST2 alias =====
            one["st2_soc"] = 0.0;
            one["st2_soh"] = 0.0;
            one["st2_pack_v"] = 0.0;
            one["st2_pack_i"] = 0.0;
            one["st2_fault_level"] = 0;
            one["st2_fault_code"] = 0;
            one["st2_rq_hv_power_off"] = 0;

            // ===== ST3 alias =====
            one["st3_ins_pos_res"] = 0.0;
            one["st3_ins_neg_res"] = 0.0;
            one["st3_ins_sys_res"] = 0.0;
            one["st3_ins_detector_on"] = 0;

            // 绝缘语义摘要别名，方便 HMI 直接引用
            one["ins_pos_res"] = 0.0;
            one["ins_neg_res"] = 0.0;
            one["ins_sys_res"] = 0.0;
            one["ins_detector_on"] = 0;
            one["ins_valid"] = 0;
            one["ins_low_any"] = 0;
            one["ins_summary_code"] = 0;
            one["ins_summary_text"] = "None";

            // ===== ST4 alias =====
            one["st4_temp_max"] = 0.0;
            one["st4_temp_min"] = 0.0;
            one["st4_temp_avg"] = 0.0;
            one["st4_max_temp_pos"] = 0;
            one["st4_min_temp_pos"] = 0;

            // ===== ST5 =====
            one["st5_max_ucell"] = 0.0;
            one["st5_min_ucell"] = 0.0;
            one["st5_avg_ucell"] = 0.0;

            // ===== ST6 =====
            one["st6_max_ucell_csc_no"] = 0;
            one["st6_max_ucell_position"] = 0;
            one["st6_min_ucell_csc_no"] = 0;
            one["st6_min_ucell_position"] = 0;

            // ===== ST7 =====
            one["st7_gun1_dc_pos_temp"] = 0.0;
            one["st7_gun1_dc_neg_temp"] = 0.0;
            one["st7_gun2_dc_pos_temp"] = 0.0;
            one["st7_gun2_dc_neg_temp"] = 0.0;
            one["st7_gun_ac_pos_temp"] = 0.0;
            one["st7_gun_ac_neg_temp"] = 0.0;

            // ===== ElecEnergy =====
            one["EE_tot_chg_energy"] = 0.0;
            one["EE_tot_dischg_energy"] = 0.0;
            one["EE_single_chg_energy"] = 0.0;
            one["EE_tot_chg_energy_valid"] = 0;
            one["EE_tot_dischg_energy_valid"] = 0;
            one["EE_single_chg_energy_valid"] = 0;

            if (it != cache.items.end())
            {
                const auto& x = it->second;

                one["online"] = x.online ? 1 : 0;
                one["bms_index"] = x.bms_index;
                one["last_msg_name"] = x.last_msg_name;
                one["soc"] = x.soc_valid ? x.soc : 0.0;
                one["soh"] = x.soh_valid ? x.soh : 0.0;
                one["pack_v"] = x.pack_v_valid ? x.pack_v : 0.0;
                one["pack_i"] = x.pack_i_valid ? x.pack_i : 0.0;
                one["st4_temp_max"] = x.st4_temp_max_valid ? x.st4_temp_max : 0.0;
                one["st4_temp_min"] = x.st4_temp_min_valid ? x.st4_temp_min : 0.0;
                one["st4_temp_avg"] = x.st4_temp_avg_valid ? x.st4_temp_avg : 0.0;

                one["alarm_any"] = x.alarm_any ? 1 : 0;
                one["fault_level"] = x.fault_level;
                one["fault_code"] = x.fault_code;

                one["st1_main_pos_relay_st"] = x.st1_main_pos_relay_st;
                one["st1_main_neg_relay_st"] = x.st1_main_neg_relay_st;
                one["st1_prechrg_relay_st"] = x.st1_prechrg_relay_st;
                one["st1_heat_pos_relay_st"] = x.st1_heat_pos_relay_st;
                one["st1_heat_neg_relay_st"] = x.st1_heat_neg_relay_st;
                one["st1_bms_hv_status"] = x.st1_bms_hv_status;
                one["st1_balance_status"] = x.st1_balance_status;
                one["st1_dc_chrg_connect_st"] = x.st1_dc_chrg_connect_st;
                one["st1_panto_chrg_connect_st"] = x.st1_panto_chrg_connect_st;
                one["st1_ac_chrg_connect_st"] = x.st1_ac_chrg_connect_st;
                one["st1_chrg_mode"] = x.st1_chrg_mode;
                one["st1_chrg_status"] = x.st1_chrg_status;
                one["st1_heating_status"] = x.st1_heating_status;
                one["st1_cooling_status"] = x.st1_cooling_status;
                one["st1_rechrg_cycles"] = x.st1_rechrg_cycles;

                one["st1_dc_chrg_pos1_relay_st"] = x.st1_dc_chrg_pos1_relay_st;
                one["st1_dc_chrg_neg1_relay_st"] = x.st1_dc_chrg_neg1_relay_st;
                one["st1_dc_chrg_pos2_relay_st"] = x.st1_dc_chrg_pos2_relay_st;
                one["st1_dc_chrg_neg2_relay_st"] = x.st1_dc_chrg_neg2_relay_st;
                one["st1_panto_chrg_pos_relay_st"] = x.st1_panto_chrg_pos_relay_st;
                one["st1_panto_chrg_neg_relay_st"] = x.st1_panto_chrg_neg_relay_st;
                one["st1_ac_chrg_pos_relay_st"] = x.st1_ac_chrg_pos_relay_st;
                one["st1_ac_chrg_neg_relay_st"] = x.st1_ac_chrg_neg_relay_st;
                one["st1_aux1_relay_st"] = x.st1_aux1_relay_st;
                one["st1_aux2_relay_st"] = x.st1_aux2_relay_st;
                one["st1_aux3_relay_st"] = x.st1_aux3_relay_st;

                // ===== ST2 alias =====
                one["st2_soc"] = x.soc_valid ? x.soc : 0.0;
                one["st2_soh"] = x.soh_valid ? x.soh : 0.0;
                one["st2_pack_v"] = x.pack_v_valid ? x.pack_v : 0.0;
                one["st2_pack_i"] = x.pack_i_valid ? x.pack_i : 0.0;
                one["st2_fault_level"] = x.fault_level;
                one["st2_fault_code"] = x.fault_code;
                one["st2_rq_hv_power_off"] = x.rq_hv_power_off ? 1 : 0;

                // ===== ST3 alias =====
                one["st3_ins_pos_res"] = x.st3_ins_pos_valid ? x.st3_ins_pos_res : 0.0;
                one["st3_ins_neg_res"] = x.st3_ins_neg_valid ? x.st3_ins_neg_res : 0.0;
                one["st3_ins_sys_res"] = x.st3_ins_sys_valid ? x.st3_ins_sys_res : 0.0;
                one["st3_ins_detector_on"] = x.st3_ins_detector_valid ? (x.st3_ins_detector_on ? 1 : 0) : 0;

                // ===== ST4 alias =====
                one["st4_temp_max"] = x.st4_temp_max_valid ? x.st4_temp_max : 0.0;
                one["st4_temp_min"] = x.st4_temp_min_valid ? x.st4_temp_min : 0.0;
                one["st4_temp_avg"] = x.st4_temp_avg_valid ? x.st4_temp_avg : 0.0;
                one["st4_max_temp_pos"] = x.st4_max_temp_pos_valid ? static_cast<double>(x.st4_max_temp_pos) : 0.0;
                one["st4_min_temp_pos"] = x.st4_min_temp_pos_valid ? static_cast<double>(x.st4_min_temp_pos) : 0.0;

                // ===== ST5 =====
                one["st5_max_ucell"] = x.st5_max_ucell_valid ? x.st5_max_ucell : 0.0;
                one["st5_min_ucell"] = x.st5_min_ucell_valid ? x.st5_min_ucell : 0.0;
                one["st5_avg_ucell"] = x.st5_avg_ucell_valid ? x.st5_avg_ucell : 0.0;

                // ===== ST6 =====
                one["st6_max_ucell_csc_no"] = x.st6_max_ucell_csc_no;
                one["st6_max_ucell_position"] = x.st6_max_ucell_position;
                one["st6_min_ucell_csc_no"] = x.st6_min_ucell_csc_no;
                one["st6_min_ucell_position"] = x.st6_min_ucell_position;

                // ===== ST7 =====
                one["st7_gun1_dc_pos_temp"] = x.st7_gun1_dc_pos_temp_valid ? x.st7_gun1_dc_pos_temp : 0.0;
                one["st7_gun1_dc_neg_temp"] = x.st7_gun1_dc_neg_temp_valid ? x.st7_gun1_dc_neg_temp : 0.0;
                one["st7_gun2_dc_pos_temp"] = x.st7_gun2_dc_pos_temp_valid ? x.st7_gun2_dc_pos_temp : 0.0;
                one["st7_gun2_dc_neg_temp"] = x.st7_gun2_dc_neg_temp_valid ? x.st7_gun2_dc_neg_temp : 0.0;
                one["st7_gun_ac_pos_temp"] = x.st7_gun_ac_pos_temp_valid ? x.st7_gun_ac_pos_temp : 0.0;
                one["st7_gun_ac_neg_temp"] = x.st7_gun_ac_neg_temp_valid ? x.st7_gun_ac_neg_temp : 0.0;

                // ===== ElecEnergy =====
                // 这里只输出工程值，HMI 整数化交给 normal_map_logic.jsonl 的“缩放”。
                one["EE_tot_chg_energy"] =  x.EE_tot_chg_energy_valid ? x.EE_tot_chg_energy : 0.0;
                one["EE_tot_dischg_energy"] = x.EE_tot_dischg_energy_valid ? x.EE_tot_dischg_energy : 0.0;
                one["EE_single_chg_energy"] = x.EE_single_chg_energy_valid ? x.EE_single_chg_energy : 0.0;
                one["EE_tot_chg_energy_valid"] = x.EE_tot_chg_energy_valid ? 1 : 0;
                one["EE_tot_dischg_energy_valid"] = x.EE_tot_dischg_energy_valid ? 1 : 0;
                one["EE_single_chg_energy_valid"] = x.EE_single_chg_energy_valid ? 1 : 0;

                // ===== B2V_CurrentLimit 完整4变量 =====
                // 这里只输出工程值 A。
                one["CL_pulse_discharge_limit_a"] =  x.CL_pulse_discharge_limit_valid ? x.CL_pulse_discharge_limit_a : 0.0;
                one["CL_pulse_charge_limit_a"] =  x.CL_pulse_charge_limit_valid ? x.CL_pulse_charge_limit_a : 0.0;
                one["CL_follow_charge_limit_a"] = x.CL_follow_charge_limit_valid ? x.CL_follow_charge_limit_a : 0.0;
                one["CL_follow_discharge_limit_a"] =    x.CL_follow_discharge_limit_valid ? x.CL_follow_discharge_limit_a : 0.0;

                one["CL_pulse_discharge_limit_valid"] = x.CL_pulse_discharge_limit_valid ? 1 : 0;
                one["CL_pulse_charge_limit_valid"] = x.CL_pulse_charge_limit_valid ? 1 : 0;
                one["CL_follow_charge_limit_valid"] = x.CL_follow_charge_limit_valid ? 1 : 0;
                one["CL_follow_discharge_limit_valid"] = x.CL_follow_discharge_limit_valid ? 1 : 0;
                one["charge_limit_a"] = x.charge_limit_valid ? x.charge_limit_a : 0.0;
                one["discharge_limit_a"] = x.discharge_limit_valid ? x.discharge_limit_a : 0.0;
                int ins_valid = (x.st3_ins_pos_valid && x.st3_ins_neg_valid && x.st3_ins_sys_valid) ? 1 : 0;

                const bool ins_low_any =
                    (x.st3_ins_pos_valid && x.st3_ins_pos_res > 0.0 && x.st3_ins_pos_res < 100.0) ||
                    (x.st3_ins_neg_valid && x.st3_ins_neg_res > 0.0 && x.st3_ins_neg_res < 100.0) ||
                    (x.st3_ins_sys_valid && x.st3_ins_sys_res > 0.0 && x.st3_ins_sys_res < 100.0);

                int ins_summary_code = 0;
                const char* ins_summary_text = "None";

                // 建议编码：
                // 0 = None
                // 1 = Invalid
                // 2 = DetectorOff
                // 3 = LowInsulation
                if (!ins_valid)
                {
                    ins_summary_code = 1;
                    ins_summary_text = "Invalid";
                }
                else if (x.st3_ins_detector_valid && !x.st3_ins_detector_on)
                {
                    ins_summary_code = 2;
                    ins_summary_text = "DetectorOff";
                }
                else if (ins_low_any)
                {
                    ins_summary_code = 3;
                    ins_summary_text = "LowInsulation";
                }

                one["ins_pos_res"] = one["st3_ins_pos_res"];
                one["ins_neg_res"] = one["st3_ins_neg_res"];
                one["ins_sys_res"] = one["st3_ins_sys_res"];
                one["ins_detector_on"] = one["st3_ins_detector_on"];

                one["ins_valid"] = ins_valid;
                one["ins_low_any"] = ins_low_any ? 1 : 0;
                one["ins_summary_code"] = ins_summary_code;
                one["ins_summary_text"] = ins_summary_text;

                one["rq_hv_power_off"] = x.rq_hv_power_off ? 1 : 0;

                // ===== TMS =====
                one["tms_work_state"] = x.tms_work_state;
                one["tms_fault_code"] = x.tms_fault_code;
                one["tms_fault_level"] = x.tms_fault_level;
                one["tms_panel_aircon_seen_ms"] = static_cast<double>(x.tms_panel_aircon_seen_ms);
                one["tms_ats_water_pump_seen_ms"] = static_cast<double>(x.tms_ats_water_pump_seen_ms);

                // ===== Fire =====
                one["fire_value_number"] = x.fire_value_number;
                one["fire_value_alarm_level"] = x.fire_value_alarm_level;

                one["fire_by_start_sts"] = x.fire_by_start_sts ? 1 : 0;
                one["fire_sensor_error_sts"] = x.fire_sensor_error_sts ? 1 : 0;
                one["fire_hw_error_sts"] = x.fire_hw_error_sts ? 1 : 0;

                one["fire_temp_sensor1_err"] = x.fire_temp_sensor1_err ? 1 : 0;
                one["fire_temp_sensor2_err"] = x.fire_temp_sensor2_err ? 1 : 0;
                one["fire_air_sensor_err"] = x.fire_air_sensor_err ? 1 : 0;
                one["fire_sam_sensor_err"] = x.fire_sam_sensor_err ? 1 : 0;

                one["fire_temp_alarm"] = x.fire_temp_alarm ? 1 : 0;
                one["fire_gas_alarm"] = x.fire_gas_alarm ? 1 : 0;
                one["fire_fog_alarm"] = x.fire_fog_alarm ? 1 : 0;
                one["fire_leak_alarm"] = x.fire_leak_alarm ? 1 : 0;

                one["fire_max_temp"] = x.fire_max_temp_valid ? x.fire_max_temp : 0.0;
                one["fire_max_gas"] = x.fire_max_gas_valid ? x.fire_max_gas : 0.0;
                one["fire_max_fog"] = x.fire_max_fog_valid ? x.fire_max_fog : 0.0;

                one["fire_fault_level"] = x.fire_fault_level;
                one["fire_fault_code"] = x.fire_fault_code;

                one["hv_allow_close"] = x.hv_allow_close ? 1 : 0;
                one["hv_should_open"] = x.hv_should_open ? 1 : 0;

                const int summary_code = calcFaultSummaryCode_(x);
                const int fault_any = (summary_code != 0) ? 1 : 0;
                const int fault_block_hv = x.hv_should_open ? 1 : 0;

                const int f2_any =
                    x.f2_tms_err ||
                    x.f2_pack_self_protect ||
                    x.f2_main_loop_prechg_err ||
                    x.f2_aux_loop_prechg_err ||
                    x.f2_chrg_ins_low_err ||
                    x.f2_acan_lost ||
                    x.f2_inner_comm_err ||
                    x.f2_dcdc_err ||
                    x.f2_branch_break_err ||
                    x.f2_heat_relay_open_err ||
                    x.f2_heat_relay_weld_err ||
                    x.f2_main_pos_open_err ||
                    x.f2_main_pos_weld_err ||
                    x.f2_main_neg_open_err ||
                    x.f2_main_neg_weld_err;

                const int f2_contact_err_any =
                    x.f2_heat_relay_open_err ||
                    x.f2_heat_relay_weld_err ||
                    x.f2_main_pos_open_err ||
                    x.f2_main_pos_weld_err ||
                    x.f2_main_neg_open_err ||
                    x.f2_main_neg_weld_err;

                const int f2_comm_err_any =
                    x.f2_acan_lost ||
                    x.f2_inner_comm_err ||
                    x.f2_dcdc_err;

                const int f2_hv_block_any =
                    x.f2_pack_self_protect ||
                    x.f2_main_loop_prechg_err ||
                    x.f2_aux_loop_prechg_err ||
                    x.f2_chrg_ins_low_err ||
                    f2_contact_err_any;

                const char* f2_summary_text = "None";
                if (x.f2_pack_self_protect) f2_summary_text = "PackSelfProtect";
                else if (x.f2_main_loop_prechg_err) f2_summary_text = "MainLoopPrechrgErr";
                else if (x.f2_aux_loop_prechg_err) f2_summary_text = "AuxLoopPrechrgErr";
                else if (x.f2_chrg_ins_low_err) f2_summary_text = "ChrgInsLowErr";
                else if (x.f2_tms_err) f2_summary_text = "TMSErr";
                else if (f2_contact_err_any) f2_summary_text = "ContactorErr";
                else if (f2_comm_err_any) f2_summary_text = "CommOrPowerErr";
                else if (x.f2_branch_break_err) f2_summary_text = "BranchBreakErr";

                one["f2_any"] = f2_any;
                one["f2_hv_block_any"] = f2_hv_block_any;
                one["f2_contact_err_any"] = f2_contact_err_any;
                one["f2_comm_err_any"] = f2_comm_err_any;
                one["f2_summary_text"] = f2_summary_text;

                one["f2_tms_err"] = x.f2_tms_err ? 1 : 0;
                one["f2_pack_self_protect"] = x.f2_pack_self_protect ? 1 : 0;
                one["f2_main_loop_prechg_err"] = x.f2_main_loop_prechg_err ? 1 : 0;
                one["f2_aux_loop_prechg_err"] = x.f2_aux_loop_prechg_err ? 1 : 0;
                one["f2_chrg_ins_low_err"] = x.f2_chrg_ins_low_err ? 1 : 0;
                one["f2_acan_lost"] = x.f2_acan_lost ? 1 : 0;
                one["f2_inner_comm_err"] = x.f2_inner_comm_err ? 1 : 0;
                one["f2_dcdc_err"] = x.f2_dcdc_err ? 1 : 0;
                one["f2_branch_break_err"] = x.f2_branch_break_err ? 1 : 0;
                one["f2_heat_relay_open_err"] = x.f2_heat_relay_open_err ? 1 : 0;
                one["f2_heat_relay_weld_err"] = x.f2_heat_relay_weld_err ? 1 : 0;
                one["f2_main_pos_open_err"] = x.f2_main_pos_open_err ? 1 : 0;
                one["f2_main_pos_weld_err"] = x.f2_main_pos_weld_err ? 1 : 0;
                one["f2_main_neg_open_err"] = x.f2_main_neg_open_err ? 1 : 0;
                one["f2_main_neg_weld_err"] = x.f2_main_neg_weld_err ? 1 : 0;

                // ===== 第二批：统一业务故障真源（联调用）=====  20260413
                one["raw_fire_alarm"] = x.raw_fire_alarm ? 1 : 0;
                one["raw_current_sensor_fault"] = x.raw_current_sensor_fault ? 1 : 0;
                one["raw_soc_jump_alarm"] = x.raw_soc_jump_alarm ? 1 : 0;
                one["raw_tms_unit_fault"] = x.raw_tms_unit_fault ? 1 : 0;
                one["raw_battery_self_protect_fault"] = x.raw_battery_self_protect_fault ? 1 : 0;
                one["raw_precharge_fault"] = x.raw_precharge_fault ? 1 : 0;
                one["raw_charge_insulation_low_alarm"] = x.raw_charge_insulation_low_alarm ? 1 : 0;
                one["raw_acan_comm_fault"] = x.raw_acan_comm_fault ? 1 : 0;
                one["raw_internal_comm_fault"] = x.raw_internal_comm_fault ? 1 : 0;
                one["raw_branch_circuit_open_fault"] = x.raw_branch_circuit_open_fault ? 1 : 0;
                one["raw_hvil_alarm"] = x.raw_hvil_alarm ? 1 : 0;

                const int f1_any =
                    (x.f1_del_temp != 0) ||
                    (x.f1_over_temp != 0) ||
                    (x.f1_over_ucell != 0) ||
                    (x.f1_low_ucell != 0) ||
                    (x.f1_low_ins_res != 0) ||
                    x.f1_ucell_uniformity ||
                    x.f1_over_chg ||
                    x.f1_over_soc ||
                    x.f1_soc_change_fast ||
                    x.f1_bat_sys_not_match ||
                    x.f1_hvil_fault;

                const int f1_block_hv =
                    (x.f1_over_temp >= 3) ||
                    (x.f1_over_ucell >= 3) ||
                    (x.f1_low_ucell >= 3) ||
                    (x.f1_low_ins_res >= 3) ||
                    x.f1_over_chg ||
                    x.f1_hvil_fault;

                const char* f1_summary_text = "None";
                if (x.f1_hvil_fault) f1_summary_text = "HVILFault";
                else if (x.f1_low_ins_res >= 3) f1_summary_text = "LowInsRes";
                else if (x.f1_over_ucell >= 3) f1_summary_text = "OverUcell";
                else if (x.f1_low_ucell >= 3) f1_summary_text = "LowUcell";
                else if (x.f1_over_temp >= 3) f1_summary_text = "OverTemp";
                else if (x.f1_over_chg) f1_summary_text = "OverChg";
                else if (x.f1_over_soc) f1_summary_text = "OverSOC";
                else if (x.f1_ucell_uniformity) f1_summary_text = "UcellUniformity";
                else if (x.f1_soc_change_fast) f1_summary_text = "SOCChangeFast";
                else if (x.f1_bat_sys_not_match) f1_summary_text = "BatSysNotMatch";
                else if (x.f1_del_temp != 0) f1_summary_text = "DelTemp";

                one["f1_any"] = f1_any;
                one["f1_block_hv"] = f1_block_hv;
                one["f1_summary_text"] = f1_summary_text;

                one["f1_del_temp"] = x.f1_del_temp;
                one["f1_over_temp"] = x.f1_over_temp;
                one["f1_over_ucell"] = x.f1_over_ucell;
                one["f1_low_ucell"] = x.f1_low_ucell;
                one["f1_low_ins_res"] = x.f1_low_ins_res;

                one["f1_ucell_uniformity"] = x.f1_ucell_uniformity ? 1 : 0;
                one["f1_over_chg"] = x.f1_over_chg ? 1 : 0;
                one["f1_over_soc"] = x.f1_over_soc ? 1 : 0;
                one["f1_soc_change_fast"] = x.f1_soc_change_fast ? 1 : 0;
                one["f1_bat_sys_not_match"] = x.f1_bat_sys_not_match ? 1 : 0;
                one["f1_hvil_fault"] = x.f1_hvil_fault ? 1 : 0;
                one["f1_fault_num"] = x.f1_fault_num;
                one["fault_any"] = fault_any;
                one["fault_block_hv"] = fault_block_hv;
                one["fault_summary_code"] = summary_code;
                one["fault_summary_text"] = calcFaultSummaryText_(summary_code);
                one["fault_source_text"] = calcFaultSourceText_(x);
                if (f1_any) any_f1 = true;
                if (f1_block_hv) any_f1_block = true;

                if (fault_any) any_fault = true;
                if (fault_block_hv)
                {
                    any_fault_block = true;
                    block_hv_count++;
                }
                if (f2_any) any_f2 = true;
                if (f2_hv_block_any) any_f2_hv_block = true;
                if (f2_contact_err_any) any_f2_contact = true;
                if (f2_comm_err_any) any_f2_comm = true;

                if (x.online) online_count++;
                if (x.alarm_any) any_alarm = true;
                if (x.fault_level > 0 || x.fire_fault_level > 0 || x.tms_fault_level > 0)
                {
                    any_fault = true;
                }
                if (ins_valid) ins_valid_count++;
                if (ins_low_any)
                {
                    any_ins = true;
                    any_ins_block = true;
                    ins_count_low++;
                }
            }

            j["bms_" + std::to_string(idx)] = one;

            // 额外输出扁平键，方便现有 normal_map_logic.jsonl 直接接
            j["bms_" + std::to_string(idx) + "_online"] = one["online"];
            j["bms_" + std::to_string(idx) + "_soc"] = one["soc"];
            j["bms_" + std::to_string(idx) + "_soh"] = one["soh"];
            j["bms_" + std::to_string(idx) + "_pack_v"] = one["pack_v"];
            j["bms_" + std::to_string(idx) + "_pack_i"] = one["pack_i"];
            j["bms_" + std::to_string(idx) + "_temp_max"] = one["st4_temp_max"];
            j["bms_" + std::to_string(idx) + "_temp_min"] = one["st4_temp_min"];
            j["bms_" + std::to_string(idx) + "_temp_avg"] = one["st4_temp_avg"];
            // B2V_CurrentLimit
            j["bms_" + std::to_string(idx) + "_CL_pulse_discharge_limit_a"] = one["CL_pulse_discharge_limit_a"];
            j["bms_" + std::to_string(idx) + "_CL_pulse_charge_limit_a"] = one["CL_pulse_charge_limit_a"];
            j["bms_" + std::to_string(idx) + "_CL_follow_charge_limit_a"] = one["CL_follow_charge_limit_a"];
            j["bms_" + std::to_string(idx) + "_CL_follow_discharge_limit_a"] = one["CL_follow_discharge_limit_a"];
            j["bms_" + std::to_string(idx) + "_CL_pulse_discharge_limit_valid"] = one["CL_pulse_discharge_limit_valid"];
            j["bms_" + std::to_string(idx) + "_CL_pulse_charge_limit_valid"] = one["CL_pulse_charge_limit_valid"];
            j["bms_" + std::to_string(idx) + "_CL_follow_charge_limit_valid"] = one["CL_follow_charge_limit_valid"];
            j["bms_" + std::to_string(idx) + "_CL_follow_discharge_limit_valid"] = one[
                "CL_follow_discharge_limit_valid"];
            j["bms_" + std::to_string(idx) + "_charge_limit_a"] = one["charge_limit_a"];
            j["bms_" + std::to_string(idx) + "_discharge_limit_a"] = one["discharge_limit_a"];
            j["bms_" + std::to_string(idx) + "_alarm_any"] = one["alarm_any"];
            j["bms_" + std::to_string(idx) + "_fault_level"] = one["fault_level"];
            j["bms_" + std::to_string(idx) + "_fault_code"] = one["fault_code"];
            j["bms_" + std::to_string(idx) + "_rq_hv_power_off"] = one["rq_hv_power_off"];
            j["bms_" + std::to_string(idx) + "_tms_work_state"] = one["tms_work_state"];
            j["bms_" + std::to_string(idx) + "_tms_fault_code"] = one["tms_fault_code"];
            j["bms_" + std::to_string(idx) + "_tms_fault_level"] = one["tms_fault_level"];
            j["bms_" + std::to_string(idx) + "_tms_panel_aircon_seen_ms"] = one["tms_panel_aircon_seen_ms"];
            j["bms_" + std::to_string(idx) + "_tms_ats_water_pump_seen_ms"] = one["tms_ats_water_pump_seen_ms"];

            j["bms_" + std::to_string(idx) + "_fire_value_number"] = one["fire_value_number"];
            j["bms_" + std::to_string(idx) + "_fire_value_alarm_level"] = one["fire_value_alarm_level"];
            j["bms_" + std::to_string(idx) + "_fire_by_start_sts"] = one["fire_by_start_sts"];
            j["bms_" + std::to_string(idx) + "_fire_sensor_error_sts"] = one["fire_sensor_error_sts"];
            j["bms_" + std::to_string(idx) + "_fire_hw_error_sts"] = one["fire_hw_error_sts"];

            j["bms_" + std::to_string(idx) + "_fire_temp_sensor1_err"] = one["fire_temp_sensor1_err"];
            j["bms_" + std::to_string(idx) + "_fire_temp_sensor2_err"] = one["fire_temp_sensor2_err"];
            j["bms_" + std::to_string(idx) + "_fire_air_sensor_err"] = one["fire_air_sensor_err"];
            j["bms_" + std::to_string(idx) + "_fire_sam_sensor_err"] = one["fire_sam_sensor_err"];

            j["bms_" + std::to_string(idx) + "_fire_temp_alarm"] = one["fire_temp_alarm"];
            j["bms_" + std::to_string(idx) + "_fire_gas_alarm"] = one["fire_gas_alarm"];
            j["bms_" + std::to_string(idx) + "_fire_fog_alarm"] = one["fire_fog_alarm"];
            j["bms_" + std::to_string(idx) + "_fire_leak_alarm"] = one["fire_leak_alarm"];

            j["bms_" + std::to_string(idx) + "_fire_max_temp"] = one["fire_max_temp"];
            j["bms_" + std::to_string(idx) + "_fire_max_gas"] = one["fire_max_gas"];
            j["bms_" + std::to_string(idx) + "_fire_max_fog"] = one["fire_max_fog"];

            j["bms_" + std::to_string(idx) + "_fire_fault_level"] = one["fire_fault_level"];
            j["bms_" + std::to_string(idx) + "_fire_fault_code"] = one["fire_fault_code"];

            j["bms_" + std::to_string(idx) + "_hv_allow_close"] = one["hv_allow_close"];
            j["bms_" + std::to_string(idx) + "_hv_should_open"] = one["hv_should_open"];

            j["bms_" + std::to_string(idx) + "_st1_online"] = one["st1_online"];
            j["bms_" + std::to_string(idx) + "_st1_age_ms"] = one["st1_age_ms"];
            j["bms_" + std::to_string(idx) + "_st1_main_pos_relay_st"] = one["st1_main_pos_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_main_neg_relay_st"] = one["st1_main_neg_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_prechrg_relay_st"] = one["st1_prechrg_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_heat_pos_relay_st"] = one["st1_heat_pos_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_heat_neg_relay_st"] = one["st1_heat_neg_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_bms_hv_status"] = one["st1_bms_hv_status"];
            j["bms_" + std::to_string(idx) + "_st1_balance_status"] = one["st1_balance_status"];
            j["bms_" + std::to_string(idx) + "_st1_dc_chrg_connect_st"] = one["st1_dc_chrg_connect_st"];
            j["bms_" + std::to_string(idx) + "_st1_panto_chrg_connect_st"] = one["st1_panto_chrg_connect_st"];
            j["bms_" + std::to_string(idx) + "_st1_ac_chrg_connect_st"] = one["st1_ac_chrg_connect_st"];
            j["bms_" + std::to_string(idx) + "_st1_chrg_mode"] = one["st1_chrg_mode"];
            j["bms_" + std::to_string(idx) + "_st1_chrg_status"] = one["st1_chrg_status"];
            j["bms_" + std::to_string(idx) + "_st1_heating_status"] = one["st1_heating_status"];
            j["bms_" + std::to_string(idx) + "_st1_cooling_status"] = one["st1_cooling_status"];
            j["bms_" + std::to_string(idx) + "_st1_rechrg_cycles"] = one["st1_rechrg_cycles"];

            j["bms_" + std::to_string(idx) + "_st1_dc_chrg_pos1_relay_st"] = one["st1_dc_chrg_pos1_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_dc_chrg_neg1_relay_st"] = one["st1_dc_chrg_neg1_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_dc_chrg_pos2_relay_st"] = one["st1_dc_chrg_pos2_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_dc_chrg_neg2_relay_st"] = one["st1_dc_chrg_neg2_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_panto_chrg_pos_relay_st"] = one["st1_panto_chrg_pos_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_panto_chrg_neg_relay_st"] = one["st1_panto_chrg_neg_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_ac_chrg_pos_relay_st"] = one["st1_ac_chrg_pos_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_ac_chrg_neg_relay_st"] = one["st1_ac_chrg_neg_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_aux1_relay_st"] = one["st1_aux1_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_aux2_relay_st"] = one["st1_aux2_relay_st"];
            j["bms_" + std::to_string(idx) + "_st1_aux3_relay_st"] = one["st1_aux3_relay_st"];
            // ===== ST2 flat alias =====
            j["bms_" + std::to_string(idx) + "_st2_soc"] = one["st2_soc"];
            j["bms_" + std::to_string(idx) + "_st2_soh"] = one["st2_soh"];
            j["bms_" + std::to_string(idx) + "_st2_pack_v"] = one["st2_pack_v"];
            j["bms_" + std::to_string(idx) + "_st2_pack_i"] = one["st2_pack_i"];
            j["bms_" + std::to_string(idx) + "_st2_fault_level"] = one["st2_fault_level"];
            j["bms_" + std::to_string(idx) + "_st2_fault_code"] = one["st2_fault_code"];
            j["bms_" + std::to_string(idx) + "_st2_rq_hv_power_off"] = one["st2_rq_hv_power_off"];
            // ===== ST3 flat alias =====
            // ===== ST3 flat alias =====
            j["bms_" + std::to_string(idx) + "_st3_ins_pos_res"] = one["st3_ins_pos_res"];
            j["bms_" + std::to_string(idx) + "_st3_ins_neg_res"] = one["st3_ins_neg_res"];
            j["bms_" + std::to_string(idx) + "_st3_ins_sys_res"] = one["st3_ins_sys_res"];
            j["bms_" + std::to_string(idx) + "_st3_ins_detector_on"] = one["st3_ins_detector_on"];

            // 绝缘语义摘要别名
            j["bms_" + std::to_string(idx) + "_ins_pos_res"] = one["ins_pos_res"];
            j["bms_" + std::to_string(idx) + "_ins_neg_res"] = one["ins_neg_res"];
            j["bms_" + std::to_string(idx) + "_ins_sys_res"] = one["ins_sys_res"];
            j["bms_" + std::to_string(idx) + "_ins_detector_on"] = one["ins_detector_on"];
            j["bms_" + std::to_string(idx) + "_ins_valid"] = one["ins_valid"];
            j["bms_" + std::to_string(idx) + "_ins_low_any"] = one["ins_low_any"];
            j["bms_" + std::to_string(idx) + "_ins_summary_code"] = one["ins_summary_code"];
            j["bms_" + std::to_string(idx) + "_ins_summary_text"] = one["ins_summary_text"];
            // ===== ST4 flat alias =====
            j["bms_" + std::to_string(idx) + "_st4_temp_max"] = one["st4_temp_max"];
            j["bms_" + std::to_string(idx) + "_st4_temp_min"] = one["st4_temp_min"];
            j["bms_" + std::to_string(idx) + "_st4_temp_avg"] = one["st4_temp_avg"];
            j["bms_" + std::to_string(idx) + "_st4_max_temp_pos"] = one["st4_max_temp_pos"];
            j["bms_" + std::to_string(idx) + "_st4_min_temp_pos"] = one["st4_min_temp_pos"];
            // ===== ST5 =====
            j["bms_" + std::to_string(idx) + "_st5_max_ucell"] = one["st5_max_ucell"];
            j["bms_" + std::to_string(idx) + "_st5_min_ucell"] = one["st5_min_ucell"];
            j["bms_" + std::to_string(idx) + "_st5_avg_ucell"] = one["st5_avg_ucell"];
            // ===== ST6 =====
            j["bms_" + std::to_string(idx) + "_st6_max_ucell_csc_no"] = one["st6_max_ucell_csc_no"];
            j["bms_" + std::to_string(idx) + "_st6_max_ucell_position"] = one["st6_max_ucell_position"];
            j["bms_" + std::to_string(idx) + "_st6_min_ucell_csc_no"] = one["st6_min_ucell_csc_no"];
            j["bms_" + std::to_string(idx) + "_st6_min_ucell_position"] = one["st6_min_ucell_position"];
            // ===== ST7 =====
            j["bms_" + std::to_string(idx) + "_st7_gun1_dc_pos_temp"] = one["st7_gun1_dc_pos_temp"];
            j["bms_" + std::to_string(idx) + "_st7_gun1_dc_neg_temp"] = one["st7_gun1_dc_neg_temp"];
            j["bms_" + std::to_string(idx) + "_st7_gun2_dc_pos_temp"] = one["st7_gun2_dc_pos_temp"];
            j["bms_" + std::to_string(idx) + "_st7_gun2_dc_neg_temp"] = one["st7_gun2_dc_neg_temp"];
            j["bms_" + std::to_string(idx) + "_st7_gun_ac_pos_temp"] = one["st7_gun_ac_pos_temp"];
            j["bms_" + std::to_string(idx) + "_st7_gun_ac_neg_temp"] = one["st7_gun_ac_neg_temp"];
            // ===== ElecEnergy =====
            j["bms_" + std::to_string(idx) + "_EE_tot_chg_energy"] = one["EE_tot_chg_energy"];
            j["bms_" + std::to_string(idx) + "_EE_tot_dischg_energy"] = one["EE_tot_dischg_energy"];
            j["bms_" + std::to_string(idx) + "_EE_single_chg_energy"] = one["EE_single_chg_energy"];
            j["bms_" + std::to_string(idx) + "_EE_tot_chg_energy_valid"] = one["EE_tot_chg_energy_valid"];
            j["bms_" + std::to_string(idx) + "_EE_tot_dischg_energy_valid"] = one["EE_tot_dischg_energy_valid"];
            j["bms_" + std::to_string(idx) + "_EE_single_chg_energy_valid"] = one["EE_single_chg_energy_valid"];

            // 20260413
            j["bms_" + std::to_string(idx) + "_raw_fire_alarm"] = one["raw_fire_alarm"];
            j["bms_" + std::to_string(idx) + "_raw_current_sensor_fault"] = one["raw_current_sensor_fault"];
            j["bms_" + std::to_string(idx) + "_raw_soc_jump_alarm"] = one["raw_soc_jump_alarm"];
            j["bms_" + std::to_string(idx) + "_raw_tms_unit_fault"] = one["raw_tms_unit_fault"];
            j["bms_" + std::to_string(idx) + "_raw_battery_self_protect_fault"] = one["raw_battery_self_protect_fault"];
            j["bms_" + std::to_string(idx) + "_raw_precharge_fault"] = one["raw_precharge_fault"];
            j["bms_" + std::to_string(idx) + "_raw_charge_insulation_low_alarm"] = one[
                "raw_charge_insulation_low_alarm"];
            j["bms_" + std::to_string(idx) + "_raw_acan_comm_fault"] = one["raw_acan_comm_fault"];
            j["bms_" + std::to_string(idx) + "_raw_internal_comm_fault"] = one["raw_internal_comm_fault"];
            j["bms_" + std::to_string(idx) + "_raw_branch_circuit_open_fault"] = one["raw_branch_circuit_open_fault"];
            j["bms_" + std::to_string(idx) + "_raw_hvil_alarm"] = one["raw_hvil_alarm"];
        }
        j["bms_count_online"] = online_count;
        j["bms_any_alarm"] = any_alarm ? 1 : 0;
        j["bms_any_fault"] = any_fault ? 1 : 0;

        j["bms_fault_any"] = any_fault ? 1 : 0;
        j["bms_fault_block_any"] = any_fault_block ? 1 : 0;
        j["bms_fault_count_block_hv"] = block_hv_count;

        j["bms_ins_any"] = any_ins ? 1 : 0;
        j["bms_ins_block_any"] = any_ins_block ? 1 : 0;
        j["bms_ins_count_low"] = ins_count_low;
        j["bms_ins_valid_count"] = ins_valid_count;

        j["bms_f2_any"] = any_f2 ? 1 : 0;
        j["bms_f2_hv_block_any"] = any_f2_hv_block ? 1 : 0;
        j["bms_f2_contact_err_any"] = any_f2_contact ? 1 : 0;
        j["bms_f2_comm_err_any"] = any_f2_comm ? 1 : 0;

        j["bms_f1_any"] = any_f1 ? 1 : 0;
        j["bms_f1_block_hv_any"] = any_f1_block ? 1 : 0;


        return j;
    }
} // namespace control::bms
