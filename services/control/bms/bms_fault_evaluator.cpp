//
// Created by lxy on 2026/4/12.
//


#include "bms_fault_evaluator.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "logger.h"
#include "../logic/logic_context.h"
#include "../fault/fault_condition_engine.h"

namespace control::bms {
    namespace {

static const char* const kImplementedBmsSignals[] = {
    // ===== 第一批：阈值型 =====
    "cell_overvoltage_lvl1",
    "cell_overvoltage_lvl2",
    "cell_overvoltage_lvl3",

    "cell_undervoltage_lvl1",
    "cell_undervoltage_lvl2",
    "cell_undervoltage_lvl3",

    "cell_overdischarge_fault",

    "cell_overtemp_lvl1",
    "cell_overtemp_lvl2",
    "cell_overtemp_lvl3",
    "cell_lowtemp_alarm",

    "temp_diff_over_lvl1",
    "temp_diff_over_lvl2",
    "temp_diff_over_lvl3",

    "soc_low_lvl1",
    "soc_low_lvl2",
    "soc_low_lvl3",
    "soc_high_alarm",

    // ===== 第三批：统一业务真源类 =====
    "fire_alarm",
    "current_sensor_fault",
    "low_voltage_supply_alarm",
    "soc_jump_alarm",
    "tms_unit_fault",
    "battery_self_protect_fault",
    "precharge_fault",
    "charge_insulation_low_alarm",
    "acan_comm_fault",
    "internal_comm_fault",
    "branch_circuit_open_fault",
    "hvil_alarm",
    "storage_mismatch_alarm",

    // ===== 第四批：接触器/回路 open-weld =====
    "heat_relay_open_fault",
    "heat_relay_weld_fault",

    "main_pos_relay_open_fault",
    "main_pos_relay_weld_fault",
    "main_neg_relay_open_fault",
    "main_neg_relay_weld_fault",

    "dc_chrg_pos1_relay_open_fault",
    "dc_chrg_pos1_relay_weld_fault",
    "dc_chrg_neg1_relay_open_fault",
    "dc_chrg_neg1_relay_weld_fault",

    "dc_chrg_pos2_relay_open_fault",
    "dc_chrg_pos2_relay_weld_fault",
    "dc_chrg_neg2_relay_open_fault",
    "dc_chrg_neg2_relay_weld_fault",

    "ac_chrg_pos_relay_open_fault",
    "ac_chrg_pos_relay_weld_fault",
    "ac_chrg_neg_relay_open_fault",
    "ac_chrg_neg_relay_weld_fault",

    "panto_chrg_pos_relay_open_fault",
    "panto_chrg_pos_relay_weld_fault",
    "panto_chrg_neg_relay_open_fault",
    "panto_chrg_neg_relay_weld_fault",

    // ===== 第五批：0x203B~0x2044 =====
    "charge_gun_connection_abnormal",
    "charge_discharge_current_overflow",
    "charge_current_overflow_alarm",
    "charge_connector_ntc_fault",
    "charge_connector_overtemp_lvl1",
    "charge_connector_overtemp_lvl2",
    "internal_hv_circuit_open_fault",

    // ===== 第十/十一批 =====
    "cell_consistency_poor_alarm",
    "driving_insulation_low_lvl1",
    "driving_insulation_low_lvl2",
    "driving_insulation_low_lvl3",
};

static constexpr size_t kImplementedBmsSignalCount =
    sizeof(kImplementedBmsSignals) / sizeof(kImplementedBmsSignals[0]);

} // namespace

std::string BmsFaultEvaluator::makeInstName_(uint32_t inst)
{
    return "BMS_" + std::to_string(inst);
}

std::string BmsFaultEvaluator::makeSignalKey_(uint32_t inst, const char* signal)
{
    return makeInstName_(inst) + "." + (signal ? signal : "unknown");
}

    void BmsFaultEvaluator::clearInstSignals_(uint32_t inst,
                                              control::LogicContext& ctx)
    {
        for (size_t i = 0; i < kImplementedBmsSignalCount; ++i) {
            setConfirmed_(inst, kImplementedBmsSignals[i], false, ctx);
        }

        LOG_THROTTLE_MS(("bms_eval_clear_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
            "[FAULT][BMS][CLEAR] inst=%u cleared implemented signals=%zu",
            (unsigned)inst,
            kImplementedBmsSignalCount);
    }

    void BmsFaultEvaluator::setConfirmed_(uint32_t inst,
                                          const char* signal,
                                          bool on,
                                          control::LogicContext& ctx)
    {
        const std::string key = makeSignalKey_(inst, signal);

        auto it = ctx.bms_confirmed_faults.signals.find(key);
        const bool existed = (it != ctx.bms_confirmed_faults.signals.end());
        const bool old = existed ? it->second : false;

        ctx.bms_confirmed_faults.signals[key] = on;

        if (!existed || old != on) {
            LOGINFO("[FAULT][BMS][CONFIRMED] key=%s old=%d new=%d",
                    key.c_str(),
                    old ? 1 : 0,
                    on ? 1 : 0);
        }
    }

bool BmsFaultEvaluator::hasValidInstance_(const BmsPerInstanceCache* x)
{
    return x != nullptr && x->seen_once;
}

bool BmsFaultEvaluator::rawSocLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->soc_valid && x->soc <= limit;
}

bool BmsFaultEvaluator::rawSocGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->soc_valid && x->soc >= limit;
}

bool BmsFaultEvaluator::rawSocGt_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->soc_valid && x->soc > limit;
}

bool BmsFaultEvaluator::rawSocLt_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->soc_valid && x->soc < limit;
}

bool BmsFaultEvaluator::rawPackVGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->pack_v_valid && x->pack_v >= limit;
}

bool BmsFaultEvaluator::rawPackVLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->pack_v_valid && x->pack_v <= limit;
}

bool BmsFaultEvaluator::rawPackIGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->pack_i_valid && x->pack_i >= limit;
}

bool BmsFaultEvaluator::rawPackILe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->pack_i_valid && x->pack_i <= limit;
}

bool BmsFaultEvaluator::rawUmaxGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st5_max_ucell_valid && x->st5_max_ucell >= limit;
}

bool BmsFaultEvaluator::rawUmaxLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st5_max_ucell_valid && x->st5_max_ucell <= limit;
}

    bool BmsFaultEvaluator::rawUminGt_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st5_min_ucell_valid && x->st5_min_ucell > limit;
}

    bool BmsFaultEvaluator::rawUminGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st5_min_ucell_valid && x->st5_min_ucell >= limit;
}

    bool BmsFaultEvaluator::rawUminLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st5_min_ucell_valid && x->st5_min_ucell <= limit;
}

    bool BmsFaultEvaluator::rawTmaxGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_max_valid && x->st4_temp_max >= limit;
}

    bool BmsFaultEvaluator::rawTmaxLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_max_valid && x->st4_temp_max <= limit;
}

    bool BmsFaultEvaluator::rawTminGe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_min_valid && x->st4_temp_min >= limit;
}

    bool BmsFaultEvaluator::rawTminGt_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_min_valid && x->st4_temp_min > limit;
}

    bool BmsFaultEvaluator::rawTminLe_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_min_valid && x->st4_temp_min <= limit;
}

    bool BmsFaultEvaluator::rawTminLt_(const BmsPerInstanceCache* x, double limit)
{
    return x && x->st4_temp_min_valid && x->st4_temp_min < limit;
}

    bool BmsFaultEvaluator::rawTempDeltaGe_(const BmsPerInstanceCache* x, double limit)
{
    if (!x) return false;
    if (!x->st4_temp_max_valid || !x->st4_temp_min_valid) return false;
    return (x->st4_temp_max - x->st4_temp_min) >= limit;
}

    bool BmsFaultEvaluator::rawTempDeltaLe_(const BmsPerInstanceCache* x, double limit)
{
    if (!x) return false;
    if (!x->st4_temp_max_valid || !x->st4_temp_min_valid) return false;
    return (x->st4_temp_max - x->st4_temp_min) <= limit;
}

    bool BmsFaultEvaluator::rawFireAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_fire_alarm;
}

    bool BmsFaultEvaluator::rawCurrentSensorFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_current_sensor_fault;
}

    bool BmsFaultEvaluator::rawSocJumpAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_soc_jump_alarm;
}

    bool BmsFaultEvaluator::rawTmsUnitFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_tms_unit_fault;
}

    bool BmsFaultEvaluator::rawBatterySelfProtectFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_battery_self_protect_fault;
}

    bool BmsFaultEvaluator::rawPrechargeFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_precharge_fault;
}

    bool BmsFaultEvaluator::rawChargeInsulationLowAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_insulation_low_alarm;
}

    bool BmsFaultEvaluator::rawAcanCommFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_acan_comm_fault;
}

    bool BmsFaultEvaluator::rawInternalCommFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_internal_comm_fault;
}

    bool BmsFaultEvaluator::rawBranchCircuitOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_branch_circuit_open_fault;
}

    bool BmsFaultEvaluator::rawHvilAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_hvil_alarm;
}
    bool BmsFaultEvaluator::rawLowVoltageSupplyAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_low_voltage_supply_alarm;
}

    bool BmsFaultEvaluator::rawStorageMismatchAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_storage_mismatch_alarm;
}

    bool BmsFaultEvaluator::rawChargeGunConnectionAbnormal_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_gun_connection_abnormal;
}

    bool BmsFaultEvaluator::rawChargeDischargeCurrentOverflow_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_discharge_current_overflow;
}

    bool BmsFaultEvaluator::rawChargeCurrentOverflowAlarm_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_current_overflow_alarm;
}

    bool BmsFaultEvaluator::rawChargeConnectorNtcFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_connector_ntc_fault;
}

    bool BmsFaultEvaluator::rawChargeConnectorOvertempLvl1_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_connector_overtemp_lvl1;
}

    bool BmsFaultEvaluator::rawChargeConnectorOvertempLvl2_(const BmsPerInstanceCache* x)
{
    return x && x->raw_charge_connector_overtemp_lvl2;
}

    bool BmsFaultEvaluator::rawInternalHvCircuitOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_internal_hv_circuit_open_fault;
}

    bool BmsFaultEvaluator::anyChargeConnectorPresent_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // ST1:
    // DCChrgConnectSt: 0=Not connected, 1=single gun, 2=double gun
    // Panto/AC connect: 0=Not connected, 1=Connected
    return
        (x->st1_dc_chrg_connect_st == 1 || x->st1_dc_chrg_connect_st == 2) ||
        (x->st1_panto_chrg_connect_st == 1) ||
        (x->st1_ac_chrg_connect_st == 1);
}

    bool BmsFaultEvaluator::chargeSessionActive_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // ST1:
    // ChrgMode:   0=Notcharging, 1=DC charging, 3=Other charging
    // ChrgStatus: 0=not charging, 1=Charging, 3=Charge error
    //
    // 这里取一个更稳的“充电相关状态门控”：
    // - 正在充电
    // - 或者充电模式已进入
    // - 或者插枪已连接
    return
        (x->st1_chrg_status == 1 || x->st1_chrg_status == 3) ||
        (x->st1_chrg_mode == 1 || x->st1_chrg_mode == 3) ||
        anyChargeConnectorPresent_(x);
}

    bool BmsFaultEvaluator::hvLoopExpectedClosed_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // 当前项目里没有完整的“高压流程状态机”对外暴露，
    // 这一版先采用更稳的工程门控：
    //
    // 1) BMS 已进入非空闲高压状态，说明主回路应已建立
    // 2) 或者处于充电相关状态，也说明内侧高压回路不应处于断路
    //
    // 注意：
    // st1_bms_hv_status 的枚举语义当前未在程序里完整展开，
    // 因此这里不硬编码“某个值=HV_ON”，而只把 0 视为明显非建立状态。
    // 后续若你拿到精确枚举定义，可再细化。
    return
        (x->st1_bms_hv_status != 0) ||
        chargeSessionActive_(x);
}

    bool BmsFaultEvaluator::mainHvRelaysNotClosed_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // 主正/主负/预充继电器状态当前都已从 ST1 解析进 cache。
    //
    // 这一版先做保守判定：
    // - 0 视为未闭合/未知
    // - 非 0 视为已进入某种有效状态
    //
    // 若后续你确认了各状态位的精确枚举（比如 1=断开, 2=闭合 ...），
    // 再把这里改成“只认闭合态”为 true。
    const bool main_pos_closed = (x->st1_main_pos_relay_st != 0);
    const bool main_neg_closed = (x->st1_main_neg_relay_st != 0);

    // 预充继电器不作为必须条件，只作为辅助观察量；
    // 因为很多稳定导通态下预充本来就可能断开。
    return !(main_pos_closed && main_neg_closed);
}
    bool BmsFaultEvaluator::prechargePhaseExpected_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // 工程保守门控：
    // 1) 若已经进入高压相关状态，则预充相关故障有业务意义
    // 2) 若处于充电相关上下文，也允许看预充故障
    return hvLoopExpectedClosed_(x) || chargeSessionActive_(x);
}

    bool BmsFaultEvaluator::mainContactorsExpectedClosed_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // 主正/主负接触器“应闭合”的上下文，先和高压回路预期建立保持一致。
    return hvLoopExpectedClosed_(x);
}

    bool BmsFaultEvaluator::insulationMeasurementUsable_(const BmsPerInstanceCache* x)
{
    if (!x) return false;

    // 旧链里已经明确区分：
    // - Invalid
    // - DetectorOff
    // - LowInsulation
    // 说明绝缘类故障必须先过“测量可用”门槛。
    return x->st3_ins_sys_valid &&
           x->st3_ins_detector_valid &&
           x->st3_ins_detector_on;
}

    bool BmsFaultEvaluator::clearDrivingInsulationLow_(const BmsPerInstanceCache* x)
{
    // 只要测量不可用，或系统绝缘已经回到正常区间，都允许恢复
    if (!x) return true;
    if (!insulationMeasurementUsable_(x)) return true;
    return (x->st3_ins_sys_res > 100.0);
}

    bool BmsFaultEvaluator::rawDrivingInsulationLowLvl1_(const BmsPerInstanceCache* x)
{
    if (!insulationMeasurementUsable_(x)) return false;
    return (x->st3_ins_sys_res > 0.0 && x->st3_ins_sys_res <= 100.0);
}

    bool BmsFaultEvaluator::rawDrivingInsulationLowLvl2_(const BmsPerInstanceCache* x)
{
    if (!insulationMeasurementUsable_(x)) return false;
    return (x->st3_ins_sys_res > 0.0 && x->st3_ins_sys_res <= 50.0);
}

    bool BmsFaultEvaluator::rawDrivingInsulationLowLvl3_(const BmsPerInstanceCache* x)
{
    if (!insulationMeasurementUsable_(x)) return false;
    return (x->st3_ins_sys_res > 0.0 && x->st3_ins_sys_res <= 10.0);
}

    bool BmsFaultEvaluator::rawCellConsistencyPoorAlarm_(const BmsPerInstanceCache* x)
{
    if (!x) return false;
    if (!x->st5_max_ucell_valid || !x->st5_min_ucell_valid) return false;

    // 表：Umax - Umin >= 300mV，持续5s
    return (x->st5_max_ucell - x->st5_min_ucell) >= 0.300;
}

    bool BmsFaultEvaluator::clearCellConsistencyPoorAlarm_(const BmsPerInstanceCache* x)
{
    if (!x) return false;
    if (!x->st5_max_ucell_valid || !x->st5_min_ucell_valid) return false;

    // 表：Umax - Umin < 250mV，持续200ms
    return (x->st5_max_ucell - x->st5_min_ucell) < 0.250;
}

    bool BmsFaultEvaluator::rawHeatRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_heat_relay_open_fault;
}

bool BmsFaultEvaluator::rawHeatRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_heat_relay_weld_fault;
}

bool BmsFaultEvaluator::rawMainPosRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_main_pos_relay_open_fault;
}

bool BmsFaultEvaluator::rawMainPosRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_main_pos_relay_weld_fault;
}

bool BmsFaultEvaluator::rawMainNegRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_main_neg_relay_open_fault;
}

bool BmsFaultEvaluator::rawMainNegRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_main_neg_relay_weld_fault;
}

bool BmsFaultEvaluator::rawDcChrgPos1RelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_pos1_relay_open_fault;
}

bool BmsFaultEvaluator::rawDcChrgPos1RelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_pos1_relay_weld_fault;
}

bool BmsFaultEvaluator::rawDcChrgNeg1RelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_neg1_relay_open_fault;
}

bool BmsFaultEvaluator::rawDcChrgNeg1RelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_neg1_relay_weld_fault;
}

bool BmsFaultEvaluator::rawDcChrgPos2RelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_pos2_relay_open_fault;
}

bool BmsFaultEvaluator::rawDcChrgPos2RelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_pos2_relay_weld_fault;
}

bool BmsFaultEvaluator::rawDcChrgNeg2RelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_neg2_relay_open_fault;
}

bool BmsFaultEvaluator::rawDcChrgNeg2RelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_dc_chrg_neg2_relay_weld_fault;
}

bool BmsFaultEvaluator::rawAcChrgPosRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_ac_chrg_pos_relay_open_fault;
}

bool BmsFaultEvaluator::rawAcChrgPosRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_ac_chrg_pos_relay_weld_fault;
}

bool BmsFaultEvaluator::rawAcChrgNegRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_ac_chrg_neg_relay_open_fault;
}

bool BmsFaultEvaluator::rawAcChrgNegRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_ac_chrg_neg_relay_weld_fault;
}

bool BmsFaultEvaluator::rawPantoChrgPosRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_panto_chrg_pos_relay_open_fault;
}

bool BmsFaultEvaluator::rawPantoChrgPosRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_panto_chrg_pos_relay_weld_fault;
}

bool BmsFaultEvaluator::rawPantoChrgNegRelayOpenFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_panto_chrg_neg_relay_open_fault;
}

bool BmsFaultEvaluator::rawPantoChrgNegRelayWeldFault_(const BmsPerInstanceCache* x)
{
    return x && x->raw_panto_chrg_neg_relay_weld_fault;
}

void BmsFaultEvaluator::evaluateAll(const BmsLogicCache& cache,
                                    control::LogicContext& ctx,
                                    uint64_t now_ms) const
{
    for (uint32_t inst = 1; inst <= 4; ++inst) {
        const std::string name = makeInstName_(inst);

        auto it = cache.items.find(name);
        const BmsPerInstanceCache* x =
            (it == cache.items.end()) ? nullptr : &it->second;

        evaluateOne_(inst, x, ctx, now_ms);
    }
        LOG_THROTTLE_MS("bms_eval_impl_count", 2000, LOGINFO,
    "[FAULT][BMS][EVAL_AUDIT] implemented confirmed signals=%zu",
    kImplementedBmsSignalCount);
}

void BmsFaultEvaluator::evaluateOne_(uint32_t inst,
                                     const BmsPerInstanceCache* x,
                                     control::LogicContext& ctx,
                                     uint64_t now_ms) const
{
    if (!hasValidInstance_(x)) {
        clearInstSignals_(inst, ctx);

        LOG_THROTTLE_MS(("bms_eval_invalid_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
            "[FAULT][BMS][EVAL] inst=%u invalid instance -> cleared all confirmed signals",
            (unsigned)inst);

        return;
    }
    // ============================================================
    // 第一批：只接入“现有 cache 就能稳定计算”的阈值型故障
    //
    // 已接：
    // - 单体过压 3 级
    // - 单体欠压 3 级
    // - 电芯过放故障
    // - 电芯高温 3 级
    // - 电芯低温报警
    // - 温差过大 3 级
    // - SOC 低 3 级
    // - SOC 高报警
    //
    // 暂不接：
    // - 总压过压/欠压（当前缺可靠 N 真源）
    // - 电流溢出类（表中还是 TBD）
    // - fire / soc_jump / current_sensor / tms_unit 等位故障
    // ============================================================

    auto update_cond = [&](const char* signal,
                           bool assert_now,
                           bool clear_now,
                           uint32_t assert_ms,
                           uint32_t clear_ms)
    {
        const auto key = makeSignalKey_(inst, signal);
        ctx.fault_cond_engine.updateConditionWithClear(
            key,
            assert_now,
            clear_now,
            now_ms,
            assert_ms,
            clear_ms
        );
        setConfirmed_(inst, signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    };

    // ---------- 单体过压 3 级 ----------
    update_cond("cell_overvoltage_lvl1",
                rawUmaxGe_(x, 3.80),
                rawUmaxLe_(x, 3.65),
                5000, 200);

    update_cond("cell_overvoltage_lvl2",
                rawUmaxGe_(x, 3.85),
                rawUmaxLe_(x, 3.65),
                5000, 200);

    update_cond("cell_overvoltage_lvl3",
                rawUmaxGe_(x, 3.90),
                rawUmaxLe_(x, 3.70),
                5000, 200);

    // ---------- 单体欠压 3 级（按 Tmin 分段） ----------
    {
        bool assert_now = false;
        bool clear_now  = false;

        if (x && x->st4_temp_min_valid && x->st5_min_ucell_valid) {
            if (x->st4_temp_min >= 0.0) {
                assert_now = (x->st5_min_ucell <= 2.55);
                clear_now  = (x->st5_min_ucell > 2.85);
            } else {
                assert_now = (x->st5_min_ucell <= 2.40);
                clear_now  = (x->st5_min_ucell > 2.40);
            }
        }

        update_cond("cell_undervoltage_lvl1", assert_now, clear_now, 5000, 10000);
    }

    {
        bool assert_now = false;
        bool clear_now  = false;

        if (x && x->st4_temp_min_valid && x->st5_min_ucell_valid) {
            if (x->st4_temp_min >= 0.0) {
                assert_now = (x->st5_min_ucell <= 2.20);
                clear_now  = (x->st5_min_ucell > 2.50);
            } else {
                assert_now = (x->st5_min_ucell <= 1.90);
                clear_now  = (x->st5_min_ucell > 2.20);
            }
        }

        update_cond("cell_undervoltage_lvl2", assert_now, clear_now, 5000, 10000);
    }

    {
        bool assert_now = false;
        bool clear_now  = false;

        if (x && x->st4_temp_min_valid && x->st5_min_ucell_valid) {
            if (x->st4_temp_min >= 0.0) {
                assert_now = (x->st5_min_ucell <= 1.90);
                clear_now  = (x->st5_min_ucell > 2.20);
            } else {
                assert_now = (x->st5_min_ucell <= 1.80);
                clear_now  = (x->st5_min_ucell > 2.10);
            }
        }

        update_cond("cell_undervoltage_lvl3", assert_now, clear_now, 5000, 10000);
    }

    // ---------- 电芯过放故障 ----------
    // 恢复条件按表为“诊断工具清除”，程序侧先不自动恢复
    update_cond("cell_overdischarge_fault",
                rawUminLe_(x, 1.0),
                false,
                10000, 0);

    // ---------- 电芯高温 3 级 ----------
    update_cond("cell_overtemp_lvl1",
                rawTmaxGe_(x, 60.0),
                rawTmaxLe_(x, 58.0),
                5000, 200);

    // 注：
    // CSV 里 lvl2 原文是 “Tmax>=60 / clear<63”，逻辑上明显不合理。
    // 这里按更合理的 65/63 实现；若你坚持完全照表，把 65.0 改回 60.0 即可。
    update_cond("cell_overtemp_lvl2",
                rawTmaxGe_(x, 65.0),
                rawTmaxLe_(x, 63.0),
                5000, 200);

    update_cond("cell_overtemp_lvl3",
                rawTmaxGe_(x, 67.0),
                rawTmaxLe_(x, 65.0),
                5000, 200);

    // ---------- 电芯低温报警 ----------
    update_cond("cell_lowtemp_alarm",
                rawTminLe_(x, -35.0),
                rawTminGt_(x, -30.0),
                5000, 200);

    // ---------- 温差过大 3 级 ----------
    update_cond("temp_diff_over_lvl1",
                rawTempDeltaGe_(x, 25.0),
                rawTempDeltaLe_(x, 23.0),
                5000, 200);

    update_cond("temp_diff_over_lvl2",
                rawTempDeltaGe_(x, 30.0),
                rawTempDeltaLe_(x, 28.0),
                5000, 200);

    update_cond("temp_diff_over_lvl3",
                rawTempDeltaGe_(x, 35.0),
                rawTempDeltaLe_(x, 32.0),
                5000, 200);

    // ---------- SOC 低 3 级 ----------
    update_cond("soc_low_lvl1",
                rawSocLe_(x, 20.0),
                rawSocGt_(x, 22.0),
                2500, 2500);

    update_cond("soc_low_lvl2",
                rawSocLe_(x, 12.0),
                rawSocGt_(x, 15.0),
                2500, 2500);

    update_cond("soc_low_lvl3",
                rawSocLe_(x, 8.0),
                rawSocGt_(x, 12.0),
                2500, 2500);

    // ---------- SOC 高报警 ----------
    update_cond("soc_high_alarm",
                rawSocGt_(x, 100.0),
                rawSocLe_(x, 100.0),
                10000, 10000);

    // ============================================================
    // 第三批：统一业务真源类故障
    //
    // 说明：
    // 1) 这一批开始让第二批补出来的 raw_xxx 真源走 evaluator -> confirmed 新链
    // 2) 对于表中“低压下电恢复 / 诊断工具清除”的项，当前程序没有专门的低压下电状态机，
    //    先采用“原始位消失后短延迟恢复”这一版。后续若要严格实现“锁存到下电”，再单独补。
    // 3) current_sensor_fault 当前仍缺可靠真源，这一批只预留，不正式确认。
    // ============================================================

    // 火灾报警：表中码位 0x2014，触发=火灾报警标志位置1，恢复=低压下电恢复
    update_cond("fire_alarm",
                rawFireAlarm_(x),
                !rawFireAlarm_(x),
                300, 300);

    // 电流传感器故障：表中码位 0x201B，触发 1s / 恢复 100ms
    update_cond("current_sensor_fault",
                rawCurrentSensorFault_(x),
                !rawCurrentSensorFault_(x),
                1000, 100);

    // BMS 低压供电异常：表中码位 0x201C，触发 2s / 恢复 1s
    update_cond("low_voltage_supply_alarm",
                rawLowVoltageSupplyAlarm_(x),
                !rawLowVoltageSupplyAlarm_(x),
                2000, 1000);

    // 可充电储能系统不匹配：表中码位 0x203C，恢复条件为低压下电恢复。
    // 当前程序没有单独低压下电状态机，这一版先按原始位消失后短延迟恢复。
    update_cond("storage_mismatch_alarm",
                rawStorageMismatchAlarm_(x),
                !rawStorageMismatchAlarm_(x),
                300, 300);

    // ============================================================
    // 第六批：充电接口故障门控
    //
    // 原则：
    // 1) 这些故障只有在“充电相关状态”下才允许触发
    // 2) 避免未插枪、未充电时原始位短暂抖动导致误报码
    // 3) 对 connector/NTC 类故障额外要求“至少存在插枪/连接状态”
    // ============================================================

    const bool charge_ctx = chargeSessionActive_(x);
    const bool connector_present = anyChargeConnectorPresent_(x);

    // 插枪连接信号异常：表中码位 0x203B
    update_cond("charge_gun_connection_abnormal",
                charge_ctx && rawChargeGunConnectionAbnormal_(x),
                (!charge_ctx) || (!rawChargeGunConnectionAbnormal_(x)),
                300, 300);

    // 充电时放电电流过大：表中码位 0x203D
    update_cond("charge_discharge_current_overflow",
                charge_ctx && rawChargeDischargeCurrentOverflow_(x),
                (!charge_ctx) || (!rawChargeDischargeCurrentOverflow_(x)),
                300, 300);

    // 充电电流超限报警：表中码位 0x203E
    update_cond("charge_current_overflow_alarm",
                charge_ctx && rawChargeCurrentOverflowAlarm_(x),
                (!charge_ctx) || (!rawChargeCurrentOverflowAlarm_(x)),
                300, 300);

    // 充电插座 NTC 故障：表中码位 0x203F
    update_cond("charge_connector_ntc_fault",
                connector_present && rawChargeConnectorNtcFault_(x),
                (!connector_present) || (!rawChargeConnectorNtcFault_(x)),
                300, 300);

    // 充电插座过温一级：表中码位 0x2040
    update_cond("charge_connector_overtemp_lvl1",
                connector_present && rawChargeConnectorOvertempLvl1_(x),
                (!connector_present) || (!rawChargeConnectorOvertempLvl1_(x)),
                300, 300);

    // 充电插座过温二级：表中码位 0x2041
    update_cond("charge_connector_overtemp_lvl2",
                connector_present && rawChargeConnectorOvertempLvl2_(x),
                (!connector_present) || (!rawChargeConnectorOvertempLvl2_(x)),
                300, 300);
    LOG_THROTTLE_MS(("bms_eval_stage8_charge_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
    "[FAULT][BMS][EVAL8] inst=%u conn=%d charge_ctx=%d ntc=%d ot_lvl1=%d ot_lvl2=%d",
    (unsigned)inst,
    connector_present ? 1 : 0,
    charge_ctx ? 1 : 0,
    rawChargeConnectorNtcFault_(x) ? 1 : 0,
    rawChargeConnectorOvertempLvl1_(x) ? 1 : 0,
    rawChargeConnectorOvertempLvl2_(x) ? 1 : 0);

    // 内侧高压回路断路故障：表中码位 0x2044
    //
    // 第七批改进：
    // 不再只用“聚合 raw 位”直接确认，而是增加业务门控：
    //
    // 1) 只有在“高压回路本应建立”的上下文下才允许触发
    // 2) 触发源优先看：
    //    - BranchBreakErr（明确支路断路）
    //    - 主正/主负接触器开路类 fault2 位
    //    - 或者 ST1 显示主回路没有闭合
    //
    // 当前 rawInternalHvCircuitOpenFault_() 本身已经是：
    // branch_break + main_pos_open + main_neg_open 的保守聚合；
    // 这里再叠加 hvLoopExpectedClosed_ / mainHvRelaysNotClosed_，
    // 就能把“非工作态误报码”压下去。
    {
        const bool hv_expected = hvLoopExpectedClosed_(x);
        const bool raw_open_fault = rawInternalHvCircuitOpenFault_(x);
        const bool relay_not_closed = mainHvRelaysNotClosed_(x);

        const bool assert_now =
            hv_expected && (raw_open_fault || relay_not_closed);

        // 表里写的是“低压下电清除”；
        // 当前程序还没有单独的低压下电状态机，这里先维持：
        // 1) 不再处于高压应建立上下文
        // 2) 且原始开路条件消失
        // 作为工程近似恢复条件。
        const bool clear_now =
            (!hv_expected) ||
            (!raw_open_fault && !relay_not_closed);

        update_cond("internal_hv_circuit_open_fault",
                    assert_now,
                    clear_now,
                    5000, 300);
    LOG_THROTTLE_MS(("bms_eval_stage10_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
        "[FAULT][BMS][EVAL10] inst=%u ucell_delta=%.3f consistency_raw=%d",
        (unsigned)inst,
        (x && x->st5_max_ucell_valid && x->st5_min_ucell_valid) ? (x->st5_max_ucell - x->st5_min_ucell) : -1.0,
        rawCellConsistencyPoorAlarm_(x) ? 1 : 0);

    }
    // 行驶绝缘低 1/2/3 级：当前直接基于 ST3 SysInsRes
    //
    // 说明：
    // 1) 这一版先按系统绝缘值 ins_sys_res 做三档分级
    // 2) 若后续你要严格区分“行驶中”和“充电中”，再补运行模式门控
    // 3) 触发/恢复时序先统一按 20s 触发、300ms 恢复，和你前面 charge_insulation_low_alarm 的风格保持一致

    update_cond("driving_insulation_low_lvl1",
                rawDrivingInsulationLowLvl1_(x),
                clearDrivingInsulationLow_(x),
                20000, 300);

    update_cond("driving_insulation_low_lvl2",
                rawDrivingInsulationLowLvl2_(x),
                clearDrivingInsulationLow_(x),
                20000, 300);

    update_cond("driving_insulation_low_lvl3",
                rawDrivingInsulationLowLvl3_(x),
                clearDrivingInsulationLow_(x),
                20000, 300);
    LOG_THROTTLE_MS(("bms_eval_stage11_ins_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
    "[FAULT][BMS][EVAL11] inst=%u usable=%d ins_sys_valid=%d detector_valid=%d detector_on=%d ins_sys=%.1f "
    "lvl={%d,%d,%d}",
    (unsigned)inst,
    insulationMeasurementUsable_(x) ? 1 : 0,
    (x && x->st3_ins_sys_valid) ? 1 : 0,
    (x && x->st3_ins_detector_valid) ? 1 : 0,
    (x && x->st3_ins_detector_on) ? 1 : 0,
    (x && x->st3_ins_sys_valid) ? x->st3_ins_sys_res : -1.0,
    rawDrivingInsulationLowLvl1_(x) ? 1 : 0,
    rawDrivingInsulationLowLvl2_(x) ? 1 : 0,
    rawDrivingInsulationLowLvl3_(x) ? 1 : 0);


    // SOC 跳变报警：表中码位 0x201E，当前原始位来源于 F1 SOCChangeFast
    update_cond("soc_jump_alarm",
                rawSocJumpAlarm_(x),
                !rawSocJumpAlarm_(x),
                300, 300);

    // 热管理机组故障：表中码位 0x2022，恢复条件“无 TMS 故障”
    update_cond("tms_unit_fault",
                rawTmsUnitFault_(x),
                !rawTmsUnitFault_(x),
                300, 300);

    // 电池自保护故障：表中码位 0x2023
    update_cond("battery_self_protect_fault",
                rawBatterySelfProtectFault_(x),
                !rawBatterySelfProtectFault_(x),
                300, 300);

    // 预充故障：表中码位 0x2024
    //
    // 第九批改进：
    // 只有在“预充/高压建立相关上下文”下才允许触发，
    // 避免空闲态原始位抖动直接报码。
    {
        const bool phase_expected = prechargePhaseExpected_(x);
        const bool raw_fault = rawPrechargeFault_(x);

        const bool assert_now = phase_expected && raw_fault;
        const bool clear_now  = (!phase_expected) || (!raw_fault);

        update_cond("precharge_fault",
                    assert_now,
                    clear_now,
                    300, 300);
    }

    // 充电绝缘低：表中码位 0x2028，触发条件本表是 R≤100Ω/V 持续20s
    // 当前程序还没有单独充电绝缘阻值真源，这里先用 raw_charge_insulation_low_alarm 位真源，
    // 仍按表的 20s 触发节奏接入。
    update_cond("charge_insulation_low_alarm",
                rawChargeInsulationLowAlarm_(x),
                !rawChargeInsulationLowAlarm_(x),
                20000, 300);

    // ACAN 通信故障：表中码位 0x2029，触发 5s
    update_cond("acan_comm_fault",
                rawAcanCommFault_(x),
                !rawAcanCommFault_(x),
                5000, 300);

    // 内部通信故障：表中码位 0x202A
    update_cond("internal_comm_fault",
                rawInternalCommFault_(x),
                !rawInternalCommFault_(x),
                300, 300);

    // 支路断路故障：表中码位 0x202B
    update_cond("branch_circuit_open_fault",
                rawBranchCircuitOpenFault_(x),
                !rawBranchCircuitOpenFault_(x),
                300, 300);

    // HVIL 故障：表中码位 0x202C
    // 表里写了“上高压前持续300ms，上高压后持续3s”；当前程序还没有精确区分前/后状态，
    // 这一版先统一按更保守的 3000ms 触发。
    update_cond("hvil_alarm",
                rawHvilAlarm_(x),
                !rawHvilAlarm_(x),
                3000, 300);

        // ============================================================
    // 第四批：接触器/回路 open-weld 故障
    //
    // 这一组本质上都是 F2 位故障，当前先统一按 300ms 触发 / 300ms 恢复接入 confirmed。
    // 如果后续拿到故障表里更精细的持续时间要求，再按单项调整。
    // ============================================================

    update_cond("heat_relay_open_fault",
                rawHeatRelayOpenFault_(x),
                !rawHeatRelayOpenFault_(x),
                300, 300);

    update_cond("heat_relay_weld_fault",
                rawHeatRelayWeldFault_(x),
                !rawHeatRelayWeldFault_(x),
                300, 300);

    {
        const bool main_expected = mainContactorsExpectedClosed_(x);

        const bool raw_main_pos_open = rawMainPosRelayOpenFault_(x);
        const bool raw_main_pos_weld = rawMainPosRelayWeldFault_(x);
        const bool raw_main_neg_open = rawMainNegRelayOpenFault_(x);
        const bool raw_main_neg_weld = rawMainNegRelayWeldFault_(x);

        update_cond("main_pos_relay_open_fault",
                    main_expected && raw_main_pos_open,
                    (!main_expected) || (!raw_main_pos_open),
                    300, 300);

        update_cond("main_pos_relay_weld_fault",
                    main_expected && raw_main_pos_weld,
                    (!main_expected) || (!raw_main_pos_weld),
                    300, 300);

        update_cond("main_neg_relay_open_fault",
                    main_expected && raw_main_neg_open,
                    (!main_expected) || (!raw_main_neg_open),
                    300, 300);

        update_cond("main_neg_relay_weld_fault",
                    main_expected && raw_main_neg_weld,
                    (!main_expected) || (!raw_main_neg_weld),
                    300, 300);
        LOG_THROTTLE_MS(("bms_eval_stage9_hv_" + std::to_string((unsigned)inst)).c_str(), 1000, LOGINFO,
    "[FAULT][BMS][EVAL9] inst=%u hv_expected=%d prechg_phase=%d "
    "raw_prechg=%d raw_mainP_open=%d raw_mainP_weld=%d raw_mainN_open=%d raw_mainN_weld=%d",
    (unsigned)inst,
    mainContactorsExpectedClosed_(x) ? 1 : 0,
    prechargePhaseExpected_(x) ? 1 : 0,
    rawPrechargeFault_(x) ? 1 : 0,
    rawMainPosRelayOpenFault_(x) ? 1 : 0,
    rawMainPosRelayWeldFault_(x) ? 1 : 0,
    rawMainNegRelayOpenFault_(x) ? 1 : 0,
    rawMainNegRelayWeldFault_(x) ? 1 : 0);
    }

    update_cond("dc_chrg_pos1_relay_open_fault",
                rawDcChrgPos1RelayOpenFault_(x),
                !rawDcChrgPos1RelayOpenFault_(x),
                300, 300);

    update_cond("dc_chrg_pos1_relay_weld_fault",
                rawDcChrgPos1RelayWeldFault_(x),
                !rawDcChrgPos1RelayWeldFault_(x),
                300, 300);

    update_cond("dc_chrg_neg1_relay_open_fault",
                rawDcChrgNeg1RelayOpenFault_(x),
                !rawDcChrgNeg1RelayOpenFault_(x),
                300, 300);

    update_cond("dc_chrg_neg1_relay_weld_fault",
                rawDcChrgNeg1RelayWeldFault_(x),
                !rawDcChrgNeg1RelayWeldFault_(x),
                300, 300);

    update_cond("dc_chrg_pos2_relay_open_fault",
                rawDcChrgPos2RelayOpenFault_(x),
                !rawDcChrgPos2RelayOpenFault_(x),
                300, 300);

    update_cond("dc_chrg_pos2_relay_weld_fault",
                rawDcChrgPos2RelayWeldFault_(x),
                !rawDcChrgPos2RelayWeldFault_(x),
                300, 300);

    update_cond("dc_chrg_neg2_relay_open_fault",
                rawDcChrgNeg2RelayOpenFault_(x),
                !rawDcChrgNeg2RelayOpenFault_(x),
                300, 300);

    update_cond("dc_chrg_neg2_relay_weld_fault",
                rawDcChrgNeg2RelayWeldFault_(x),
                !rawDcChrgNeg2RelayWeldFault_(x),
                300, 300);

    update_cond("ac_chrg_pos_relay_open_fault",
                rawAcChrgPosRelayOpenFault_(x),
                !rawAcChrgPosRelayOpenFault_(x),
                300, 300);

    update_cond("ac_chrg_pos_relay_weld_fault",
                rawAcChrgPosRelayWeldFault_(x),
                !rawAcChrgPosRelayWeldFault_(x),
                300, 300);

    update_cond("ac_chrg_neg_relay_open_fault",
                rawAcChrgNegRelayOpenFault_(x),
                !rawAcChrgNegRelayOpenFault_(x),
                300, 300);

    update_cond("ac_chrg_neg_relay_weld_fault",
                rawAcChrgNegRelayWeldFault_(x),
                !rawAcChrgNegRelayWeldFault_(x),
                300, 300);

    update_cond("panto_chrg_pos_relay_open_fault",
                rawPantoChrgPosRelayOpenFault_(x),
                !rawPantoChrgPosRelayOpenFault_(x),
                300, 300);

    update_cond("panto_chrg_pos_relay_weld_fault",
                rawPantoChrgPosRelayWeldFault_(x),
                !rawPantoChrgPosRelayWeldFault_(x),
                300, 300);

    update_cond("panto_chrg_neg_relay_open_fault",
                rawPantoChrgNegRelayOpenFault_(x),
                !rawPantoChrgNegRelayOpenFault_(x),
                300, 300);

    update_cond("panto_chrg_neg_relay_weld_fault",
                rawPantoChrgNegRelayWeldFault_(x),
                !rawPantoChrgNegRelayWeldFault_(x),
                300, 300);
}

} // namespace control::bms
