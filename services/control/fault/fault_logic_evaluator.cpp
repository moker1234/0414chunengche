//
// Created by lxy on 2026/4/12.
//

#include "fault_logic_evaluator.h"

#include <string>

#include "../logic/logic_context.h"
#include "fault_condition_engine.h"
#include "logger.h"

namespace control::fault {

namespace {


inline void updateConfirmedWithClear_(control::LogicContext& ctx,
                                      uint64_t now_ms,
                                      const std::string& key,
                                      bool assert_now,
                                      bool clear_now,
                                      uint32_t assert_ms,
                                      uint32_t clear_ms)
{
    ctx.fault_cond_engine.updateConditionWithClear(
        key,
        assert_now,
        clear_now,
        now_ms,
        assert_ms,
        clear_ms
    );
}

} // namespace

std::string FaultLogicEvaluator::makeSignalKey_(const char* signal)
{
    return std::string("logic.") + (signal ? signal : "unknown");
}

void FaultLogicEvaluator::setConfirmed_(const char* signal,
                                        bool on,
                                        control::LogicContext& ctx)
{
    const std::string key = makeSignalKey_(signal);

    bool old = false;
    auto it_old = ctx.confirmed_faults.signals.find(key);
    if (it_old != ctx.confirmed_faults.signals.end()) {
        old = it_old->second;
    }

    ctx.confirmed_faults.signals[key] = on;

    if (old != on) {
        LOGINFO("[FAULT][LOGIC][CONFIRMED] key=%s value=%d",
                key.c_str(),
                on ? 1 : 0);
    }
}

bool FaultLogicEvaluator::rawPcu0Offline_(const control::LogicContext& ctx)
{
    return !ctx.pcu0_state.online;
}

bool FaultLogicEvaluator::rawPcu1Offline_(const control::LogicContext& ctx)
{
    return !ctx.pcu1_state.online;
}
    bool FaultLogicEvaluator::rawPcu0EmergencyStop_(const control::LogicContext& ctx)
{
    /*
     * PCU 急停来自 PCU 状态反馈帧 Byte1。
     *
     * 注意：
     * - PCU 离线时不沿用旧 estop 值；
     * - 只有 online && estop 才认为 PCU 急停有效。
     */
    return ctx.pcu0_state.online && ctx.pcu0_state.estop;
}

    bool FaultLogicEvaluator::rawPcu1EmergencyStop_(const control::LogicContext& ctx)
{
    return ctx.pcu1_state.online && ctx.pcu1_state.estop;
}


bool FaultLogicEvaluator::rawEnvAnyAlarm_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.env_any_alarm;
}

bool FaultLogicEvaluator::rawAnyFault_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.any_fault;
}

    bool FaultLogicEvaluator::rawHmiCommFault_(const control::LogicContext& ctx, uint64_t now_ms)
{
    // 如果系统通过其他路径已经明确置位了故障，则直接返回 true
    // if (ctx.logic_faults.hmi_comm_fault) {
    //     return true;
    // }

    // 如果自从启动以来从未见过任何 HMI 报文（读或写），则保持 false 不误报
    if (!ctx.hmi_seen_once) {
        return false;
    }

    // 此时 last_hmi_comm_ts 已代表“最后通信时间”
    if (ctx.last_hmi_comm_ts == 0) {
        return false;
    }

    // 防御性检查：当前时间小于记录时间（如系统调时）
    if (now_ms < ctx.last_hmi_comm_ts) {
        return false;
    }

    // 判断当前时间与最后一次合法通信的时间差是否超过 5000ms（或配置值）
    bool isfault = (now_ms - ctx.last_hmi_comm_ts) > ctx.hmi_comm_timeout_ms;
    return isfault;
}

    bool FaultLogicEvaluator::rawRemoteCommFault_(const control::LogicContext& ctx, uint64_t now_ms)
{
    // 第九批：remote 弱在线真源（预留接口版）
    // 规则：
    // 1) 如果其他模块已经显式把 logic_faults.remote_comm_fault 置位，则直接认为 raw fault；
    // 2) 如果还从未见过 remote 数据，不主动报码，避免“功能未接入时开机即报码”；
    // 3) 一旦见过 remote 数据，超过 remote_comm_timeout_ms 没再收到，则视为 raw remote_comm_fault。
    if (ctx.logic_faults.remote_comm_fault) {
        return true;
    }

    if (!ctx.remote_seen_once) {
        return false;
    }

    if (ctx.last_remote_rx_ts == 0) {
        return false;
    }

    if (now_ms < ctx.last_remote_rx_ts) {
        return false;
    }

    return (now_ms - ctx.last_remote_rx_ts) > ctx.remote_comm_timeout_ms;
}

    bool FaultLogicEvaluator::rawSdcardFault_(const control::LogicContext& ctx)
{
    return ctx.logic_faults.sdcard_fault;
}

    void FaultLogicEvaluator::evaluatePcu_(control::LogicContext& ctx, uint64_t now_ms) const
{
    auto eval_confirmed = [&](const char* signal,
                              bool raw_active,
                              uint32_t trigger_ms,
                              uint32_t clear_ms)
    {
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx,
            now_ms,
            key,
            raw_active,
            !raw_active,
            trigger_ms,
            clear_ms
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    };

    /*
     * 通信故障：
     * 内部命名：
     *   pcu0_offline -> PCU1通信故障
     *   pcu1_offline -> PCU2通信故障
     *
     * 同时写别名：
     *   pcu1_comm_fault
     *   pcu2_comm_fault
     *
     * 这样 fault_map 中 source=VCU/signal=pcu1_comm_fault 或
     * source=VCU/signal=pcu2_comm_fault 都能命中 confirmed。
     */
    eval_confirmed("pcu0_offline", rawPcu0Offline_(ctx), 3000, 0);
    eval_confirmed("pcu1_offline", rawPcu1Offline_(ctx), 3000, 0);

    eval_confirmed("pcu1_comm_fault", rawPcu0Offline_(ctx), 3000, 0);
    eval_confirmed("pcu2_comm_fault", rawPcu1Offline_(ctx), 3000, 0);

    /*
     * PCU 故障急停：
     * fault_map 中是 source=PCU:
     *   pcu1_emergency_stop
     *   pcu2_emergency_stop
     *
     * 规则：
     *   online && estop 持续 300ms 触发；
     *   恢复即时。
     *
     * 这里不用 3000ms，是因为急停属于强安全信号，不应等待 3 秒。
     */
    eval_confirmed("pcu0_emergency_stop", rawPcu0EmergencyStop_(ctx), 300, 0);
    eval_confirmed("pcu1_emergency_stop", rawPcu1EmergencyStop_(ctx), 300, 0);

    // 面向 fault_map / HMI 的别名
    eval_confirmed("pcu1_emergency_stop", rawPcu0EmergencyStop_(ctx), 300, 0);
    eval_confirmed("pcu2_emergency_stop", rawPcu1EmergencyStop_(ctx), 300, 0);
}

void FaultLogicEvaluator::evaluateLogic_(control::LogicContext& ctx, uint64_t now_ms) const
{
    // ------------------------------------------------------------
    // 第14批：
    // 先在 fault refresh 主链里把 system/logic 原始聚合重新收一遍，
    // 避免某些路径（HMI 写入 / remote 数据到达 / IO 更新）没有先经过 snapshot，
    // 导致 any_fault / env_any_alarm 落后于明细真源。
    // ------------------------------------------------------------

    const bool raw_hmi_fault    = rawHmiCommFault_(ctx, now_ms);
    const bool raw_remote_fault = rawRemoteCommFault_(ctx, now_ms);
    const bool raw_sd_fault     = rawSdcardFault_(ctx);

    // 把弱在线推导结果回写到 raw logic fault 真源
    ctx.logic_faults.hmi_comm_fault    = raw_hmi_fault;
    ctx.logic_faults.remote_comm_fault = raw_remote_fault;
    ctx.logic_faults.sdcard_fault      = raw_sd_fault;

    // 环境类总告警：只保留 UPS / Smoke / Gas / Air 的 alarm 语义
    ctx.logic_faults.env_any_alarm =
        ctx.ups_faults.alarm_any ||
        ctx.smoke_faults.alarm_any ||
        ctx.gas_faults.alarm_any ||
        ctx.air_faults.alarm_any;

    // 总故障：system/comm + offline 聚合 + 设备 fault 聚合
    ctx.logic_faults.any_fault =
        ctx.logic_faults.system_estop ||
        ctx.logic_faults.sdcard_fault ||
        ctx.logic_faults.hmi_comm_fault ||
        ctx.logic_faults.remote_comm_fault ||

        ctx.logic_faults.pcu_any_offline ||
        ctx.logic_faults.bms_any_offline ||
        ctx.logic_faults.ups_offline ||
        ctx.logic_faults.smoke_offline ||
        ctx.logic_faults.gas_offline ||
        ctx.logic_faults.air_offline ||

        ctx.ups_faults.fault_any ||
        ctx.smoke_faults.fault_any ||
        ctx.gas_faults.fault_any ||
        ctx.air_faults.fault_any;

    // ------------------------------------------------------------
    // 聚合项 confirmed
    // ------------------------------------------------------------
    {
        const char* signal = "env_any_alarm";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawEnvAnyAlarm_(ctx),
            !rawEnvAnyAlarm_(ctx),
            2000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    {
        const char* signal = "any_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            rawAnyFault_(ctx),
            !rawAnyFault_(ctx),
            2000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // ------------------------------------------------------------
    // system / comm 类 confirmed
    // ------------------------------------------------------------

    // 定义一个通用的防抖闭包：持续 3000ms 触发，持续 1000ms 恢复
    auto eval_comm_fault = [&](const char* signal, bool raw_offline) {
        const std::string key = makeSignalKey_(signal);
        updateConfirmedWithClear_(ctx, now_ms, key, raw_offline, !raw_offline, 3000, 1000);
        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    };

    // 1. HMI 通信故障 (0x1000)
    eval_comm_fault("hmi_comm_fault", raw_hmi_fault);

    // 2. 远程无线传输装置通信故障 (0x1009)
    eval_comm_fault("remote_comm_fault", raw_remote_fault);

    // 3. 空调通信故障 (0x1007)
    eval_comm_fault("aircon_comm_fault", !ctx.air_faults.online);

    // 4. UPS通信故障 (0x1008)
    eval_comm_fault("ups_comm_fault", !ctx.ups_faults.online);

    // 5. TSS感温感烟传感器通信故障 (0x100A)
    eval_comm_fault("tss_comm_fault", !ctx.smoke_faults.online);

    // 6. CGS可燃气体传感器通信故障 (0x100B)
    eval_comm_fault("cgs_comm_fault", !ctx.gas_faults.online);

    // 7. PCU1通信故障 (0x1005，底层对应 pcu0)
    eval_comm_fault("pcu1_comm_fault", !ctx.pcu0_state.online);

    // 8. PCU2通信故障 (0x1006，底层对应 pcu1)
    eval_comm_fault("pcu2_comm_fault", !ctx.pcu1_state.online);

    // 9~12. BMS1 ~ BMS4 通信故障 (0x1001 ~ 0x1004)
    for (int i = 1; i <= 4; ++i) {
        std::string bms_key = "BMS_" + std::to_string(i);
        std::string sig = "bms" + std::to_string(i) + "_comm_fault";
        bool is_offline = true; // 默认未收到数据即视为离线

        auto it = ctx.bms_cache.items.find(bms_key);
        if (it != ctx.bms_cache.items.end()) {
            is_offline = !it->second.online;
        }

        eval_comm_fault(sig.c_str(), is_offline);
    }


    // --------------------------- 原本的 confirmed ---------------------------
    // HMI 通信故障：持续 3000ms 触发，恢复 1000ms
    // {
    //     const char* signal = "hmi_comm_fault";
    //     const std::string key = makeSignalKey_(signal);
    //
    //     updateConfirmedWithClear_(
    //         ctx, now_ms, key,
    //         raw_hmi_fault,
    //         !raw_hmi_fault,
    //         3000, 1000
    //     );
    //
    //     setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    // }

    // Remote 通信故障：持续 3000ms 触发，恢复 1000ms
    {
        const char* signal = "remote_comm_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            raw_remote_fault,
            !raw_remote_fault,
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }

    // SDCard 故障：持续 3000ms 触发，恢复 1000ms
    {
        const char* signal = "sdcard_fault";
        const std::string key = makeSignalKey_(signal);

        updateConfirmedWithClear_(
            ctx, now_ms, key,
            raw_sd_fault,
            !raw_sd_fault,
            3000, 1000
        );

        setConfirmed_(signal, ctx.fault_cond_engine.getConfirmed(key), ctx);
    }
}

void FaultLogicEvaluator::evaluateAll(control::LogicContext& ctx, uint64_t now_ms) const
{
    evaluatePcu_(ctx, now_ms);
    // evaluateUps_(ctx, now_ms);
    // evaluateSmoke_(ctx, now_ms);
    // evaluateGas_(ctx, now_ms);
    // evaluateAir_(ctx, now_ms);
    evaluateLogic_(ctx, now_ms);
}

} // namespace control::fault