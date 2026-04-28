//
// services/control/logic/logic_io.cpp
//
// IoSample 处理：
// - 缓存 DI/AI 采样
// - 生成 system / IO 类原始真源
// - 第3批：接入急停 / 插枪检测 / 指示灯 DO 占位输出
//

#include "logic_engine.h"

#include <cmath>

namespace control {

namespace {

// ===================== DI 通道定义 =====================
// 约定：di_bits 的 bit0 -> DI1, bit1 -> DI2, ...
static constexpr int kDiChEstop = 1;

static constexpr int kDiChPlug1 = 13;
static constexpr int kDiChPlug2 = 14;
static constexpr int kDiChPlug3 = 15;
static constexpr int kDiChPlug4 = 16;
static constexpr int kDiChPlug5 = 17;
static constexpr int kDiChPlug6 = 18;

// ===================== ADC 通道定义 =====================
// 约定：IoSampleEvent.ai 只承载电压值：
// ai[0] = ADC1_V
// ai[1] = ADC2_V
static constexpr int kAdcChPlug1 = 1;
static constexpr int kAdcChPlug2 = 2;

// 插枪判定阈值：低于 8V 视为插枪
static constexpr double kPlugAdcThresholdV = 8.0;

// ===================== DO 占位定义（后续你可自行改名） =====================
// 第3批先只做占位功能定义：
// DO1: 急停指示灯
// DO2: 插枪总状态灯
// DO3: 离散插枪总状态灯（DI13~18 任意一路）
// DO4: ADC1 插枪灯
// DO5: ADC2 插枪灯
// DO6: IO 活跃灯（收到 IoSample 就亮）
// DO7: 系统故障总灯
// static constexpr int kDoLampEstop      = 1;
// static constexpr int kDoLampPlugAny    = 2;
// static constexpr int kDoLampPlugDiAny  = 3;
// static constexpr int kDoLampAdc1Plug   = 4;
// static constexpr int kDoLampAdc2Plug   = 5;
// static constexpr int kDoLampIoAlive    = 6;
// static constexpr int kDoLampAnyFault   = 7;

static inline bool testDiChannelOn_(uint64_t bits, int channel_id)
{
    if (channel_id < 1 || channel_id > 64) return false;
    const int bit = channel_id - 1;
    return ((bits >> bit) & 0x1ULL) != 0ULL;
}

static inline double readAiVoltage_(const std::vector<double>& ai, int adc_channel)
{
    if (adc_channel < 1) {
        return std::nan("");
    }

    const std::size_t idx = static_cast<std::size_t>(adc_channel - 1);
    if (idx >= ai.size()) {
        return std::nan("");
    }

    return ai[idx];
}

static inline bool isPlugByAdc_(double v)
{
    // 当前批次只消费“电压值”本身，不带 valid 位：
    // - 有效且 < 8V -> 插枪
    // - 无效(NaN/Inf) -> 不判插枪
    return std::isfinite(v) && (v < kPlugAdcThresholdV);
}

static inline void emitWriteDo_(std::vector<Command>& out_cmds,
                                int channel_id,
                                bool on)
{
    Command c;
    c.type = Command::Type::WriteDo;
    c.write_do.channel_id = channel_id;
    c.write_do.value = on;
    out_cmds.push_back(std::move(c));
}

} // namespace

void LogicEngine::onIoSample_(const IoSampleEvent& s,
                              LogicContext& ctx,
                              std::vector<Command>& out_cmds)
{
    // ------------------------------------------------------------
    // 1) 缓存原始 IO 采样
    // ------------------------------------------------------------
    ctx.last_io_ts = s.ts_ms;
    ctx.di_bits = s.di_bits;
    ctx.ai = s.ai;
    ctx.logic_faults.last_eval_ts_ms = s.ts_ms;

    // ------------------------------------------------------------
    // 2) DI 真源
    // ------------------------------------------------------------
    const bool estop_pressed = testDiChannelOn_(s.di_bits, kDiChEstop);

    const bool plug_di_1 = testDiChannelOn_(s.di_bits, kDiChPlug1);
    const bool plug_di_2 = testDiChannelOn_(s.di_bits, kDiChPlug2);
    const bool plug_di_3 = testDiChannelOn_(s.di_bits, kDiChPlug3);
    const bool plug_di_4 = testDiChannelOn_(s.di_bits, kDiChPlug4);
    const bool plug_di_5 = testDiChannelOn_(s.di_bits, kDiChPlug5);
    const bool plug_di_6 = testDiChannelOn_(s.di_bits, kDiChPlug6);

    const bool plug_di_any =
        plug_di_1 || plug_di_2 || plug_di_3 ||
        plug_di_4 || plug_di_5 || plug_di_6;

    // ------------------------------------------------------------
    // 3) ADC 真源
    // ------------------------------------------------------------
    const double adc1_v = readAiVoltage_(s.ai, kAdcChPlug1);
    const double adc2_v = readAiVoltage_(s.ai, kAdcChPlug2);

    const bool plug_adc_1 = isPlugByAdc_(adc1_v);
    const bool plug_adc_2 = isPlugByAdc_(adc2_v);

    const bool plug_any = plug_di_any || plug_adc_1 || plug_adc_2;

    // ------------------------------------------------------------
    // 4) 写入 logic/system 类真源
    // ------------------------------------------------------------
    ctx.logic_faults.system_estop = estop_pressed;

    // 说明：
    // 旧版 logic_io.cpp 曾临时把某个 DI 位当成 SD 卡在位占位真源。
    // 现在 DI1 已正式用于“急停”，这里不再复用 DI 做 sdcard_fault 占位，
    // 保持 sdcard_fault 由其他链路/后续批次维护，避免信号冲突。

    // 环境类总告警：保持与 snapshot 聚合口径一致
    ctx.logic_faults.env_any_alarm =
        ctx.ups_faults.alarm_any ||
        ctx.smoke_faults.alarm_any ||
        ctx.gas_faults.alarm_any ||
        ctx.air_faults.alarm_any;

    // 总故障：这里按 logic_snapshot.cpp 的聚合口径做一次即时收口，
    // 保证 IO 到达后 system_estop 能立刻影响总故障灯。
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
    // 5) 第3批：输出占位指示灯
    //
    // DO1: 急停灯
    // DO2: 插枪总灯
    // DO3: DI 插枪总灯
    // DO4: ADC1 插枪灯
    // DO5: ADC2 插枪灯
    // DO6: IO 活跃灯
    // DO7: 总故障灯
    // ------------------------------------------------------------
    // emitWriteDo_(out_cmds, kDoLampEstop,     estop_pressed);
    // emitWriteDo_(out_cmds, kDoLampPlugAny,   plug_any);
    // emitWriteDo_(out_cmds, kDoLampPlugDiAny, plug_di_any);
    // emitWriteDo_(out_cmds, kDoLampAdc1Plug,  plug_adc_1);
    // emitWriteDo_(out_cmds, kDoLampAdc2Plug,  plug_adc_2);
    // emitWriteDo_(out_cmds, kDoLampIoAlive,   true);
    // emitWriteDo_(out_cmds, kDoLampAnyFault,  ctx.logic_faults.any_fault);

    // 后续批次可继续在这里扩展：
    // 1) DI 去抖 / 边沿检测
    // 2) 插枪 DI 与 ADC 的更细粒度融合
    // 3) 把插枪/急停状态进一步投影到 logic_view / HMI
    // 4) 若需要严格区分 ADC 读失败与 0V，需要在 IoSampleEvent 里补 valid 位
}

} // namespace control