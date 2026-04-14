//
// Created by lxy on 2026/4/12.
//

#ifndef ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H
#define ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H

#pragma once

#include <cstdint>
#include <string>

#include "bms_logic_types.h"

namespace control {
    struct LogicContext;
}

namespace control::bms {

/**
 * BmsFaultEvaluator
 *
 * 作用：
 * 1. 从 BmsLogicCache / BmsPerInstanceCache 中读取原始运行态与数值
 * 2. 使用 LogicContext::fault_cond_engine 做持续时间确认
 * 3. 将“已确认的 BMS 故障真源”写入 LogicContext::bms_confirmed_faults
 *
 * 当前第三批目标：
 * - 先搭 evaluator 框架
 * - 先定义 key 命名规则和写回接口
 * - 暂不大规模接入具体故障公式
 *
 * 后续批次中，类似以下复杂条件都应继续加到本类，而不是加到 BmsFaultMapper：
 * - Umax >= 3.9V 持续 5s
 * - SOC <= 20% 持续 2.5s
 * - SOC > 100% 持续 10s
 * - 分段温度/电压阈值
 * - 接触器压差持续异常
 */
class BmsFaultEvaluator {
public:
    BmsFaultEvaluator() = default;

    /**
     * 遍历全部 BMS 实例，更新已确认故障真源
     */
    void evaluateAll(const BmsLogicCache& cache,
                     control::LogicContext& ctx,
                     uint64_t now_ms) const;

private:
    void evaluateOne_(uint32_t inst,
                      const BmsPerInstanceCache* x,
                      control::LogicContext& ctx,
                      uint64_t now_ms) const;

    // ---------- key helpers ----------
    static std::string makeInstName_(uint32_t inst);
    static std::string makeSignalKey_(uint32_t inst, const char* signal);

    // ---------- write helpers ----------
    static void clearInstSignals_(uint32_t inst, control::LogicContext& ctx);
    static void setConfirmed_(uint32_t inst,
                              const char* signal,
                              bool on,
                              control::LogicContext& ctx);

    // ---------- raw condition helpers ----------
    static bool hasValidInstance_(const BmsPerInstanceCache* x);

    static bool rawSocLe_(const BmsPerInstanceCache* x, double limit);
    static bool rawSocGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawSocGt_(const BmsPerInstanceCache* x, double limit);
    static bool rawSocLt_(const BmsPerInstanceCache* x, double limit);

    static bool rawPackVGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawPackVLe_(const BmsPerInstanceCache* x, double limit);
    static bool rawPackIGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawPackILe_(const BmsPerInstanceCache* x, double limit);

    // ST5：单体电压
    static bool rawUmaxGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawUmaxLe_(const BmsPerInstanceCache* x, double limit);
    static bool rawUminGt_(const BmsPerInstanceCache* x, double limit);
    static bool rawUminGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawUminLe_(const BmsPerInstanceCache* x, double limit);

    // 温度
    static bool rawTmaxGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawTmaxLe_(const BmsPerInstanceCache* x, double limit);
    static bool rawTminGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawTminGt_(const BmsPerInstanceCache* x, double limit);
    static bool rawTminLe_(const BmsPerInstanceCache* x, double limit);
    static bool rawTminLt_(const BmsPerInstanceCache* x, double limit);

    static bool rawTempDeltaGe_(const BmsPerInstanceCache* x, double limit);
    static bool rawTempDeltaLe_(const BmsPerInstanceCache* x, double limit);

    // 第二批补出的统一业务真源
    static bool rawFireAlarm_(const BmsPerInstanceCache* x);
    static bool rawCurrentSensorFault_(const BmsPerInstanceCache* x);
    static bool rawLowVoltageSupplyAlarm_(const BmsPerInstanceCache* x);
    static bool rawSocJumpAlarm_(const BmsPerInstanceCache* x);
    static bool rawTmsUnitFault_(const BmsPerInstanceCache* x);
    static bool rawBatterySelfProtectFault_(const BmsPerInstanceCache* x);
    static bool rawPrechargeFault_(const BmsPerInstanceCache* x);
    static bool rawChargeInsulationLowAlarm_(const BmsPerInstanceCache* x);
    static bool rawAcanCommFault_(const BmsPerInstanceCache* x);
    static bool rawInternalCommFault_(const BmsPerInstanceCache* x);
    static bool rawBranchCircuitOpenFault_(const BmsPerInstanceCache* x);
    static bool rawHvilAlarm_(const BmsPerInstanceCache* x);
    static bool rawStorageMismatchAlarm_(const BmsPerInstanceCache* x);

    static bool rawChargeGunConnectionAbnormal_(const BmsPerInstanceCache* x);
    static bool rawChargeDischargeCurrentOverflow_(const BmsPerInstanceCache* x);
    static bool rawChargeCurrentOverflowAlarm_(const BmsPerInstanceCache* x);
    static bool rawChargeConnectorNtcFault_(const BmsPerInstanceCache* x);
    static bool rawChargeConnectorOvertempLvl1_(const BmsPerInstanceCache* x);
    static bool rawChargeConnectorOvertempLvl2_(const BmsPerInstanceCache* x);
    static bool rawInternalHvCircuitOpenFault_(const BmsPerInstanceCache* x);

    // 第六批：充电接口故障门控
    static bool anyChargeConnectorPresent_(const BmsPerInstanceCache* x);
    static bool chargeSessionActive_(const BmsPerInstanceCache* x);

    // 第七批：内侧高压回路断路门控
    static bool hvLoopExpectedClosed_(const BmsPerInstanceCache* x);
    static bool mainHvRelaysNotClosed_(const BmsPerInstanceCache* x);

    // 第九批：主回路/预充故障门控
    static bool prechargePhaseExpected_(const BmsPerInstanceCache* x);
    static bool mainContactorsExpectedClosed_(const BmsPerInstanceCache* x);

    static bool rawDrivingInsulationLowLvl1_(const BmsPerInstanceCache* x);
    static bool rawDrivingInsulationLowLvl2_(const BmsPerInstanceCache* x);
    static bool rawDrivingInsulationLowLvl3_(const BmsPerInstanceCache* x);

    // 第十一批：绝缘检测器门控
    static bool insulationMeasurementUsable_(const BmsPerInstanceCache* x);
    static bool clearDrivingInsulationLow_(const BmsPerInstanceCache* x);

    // 第十批：一致性差报警
    static bool rawCellConsistencyPoorAlarm_(const BmsPerInstanceCache* x);
    static bool clearCellConsistencyPoorAlarm_(const BmsPerInstanceCache* x);

    static bool rawHeatRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawHeatRelayWeldFault_(const BmsPerInstanceCache* x);

    static bool rawMainPosRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawMainPosRelayWeldFault_(const BmsPerInstanceCache* x);
    static bool rawMainNegRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawMainNegRelayWeldFault_(const BmsPerInstanceCache* x);

    static bool rawDcChrgPos1RelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgPos1RelayWeldFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgNeg1RelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgNeg1RelayWeldFault_(const BmsPerInstanceCache* x);

    static bool rawDcChrgPos2RelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgPos2RelayWeldFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgNeg2RelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawDcChrgNeg2RelayWeldFault_(const BmsPerInstanceCache* x);

    static bool rawAcChrgPosRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawAcChrgPosRelayWeldFault_(const BmsPerInstanceCache* x);
    static bool rawAcChrgNegRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawAcChrgNegRelayWeldFault_(const BmsPerInstanceCache* x);

    static bool rawPantoChrgPosRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawPantoChrgPosRelayWeldFault_(const BmsPerInstanceCache* x);
    static bool rawPantoChrgNegRelayOpenFault_(const BmsPerInstanceCache* x);
    static bool rawPantoChrgNegRelayWeldFault_(const BmsPerInstanceCache* x);
};

} // namespace control::bms

#endif // ENERGYSTORAGE_BMS_FAULT_EVALUATOR_H