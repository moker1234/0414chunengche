// services/control/logic_io.cpp
//
// IoSample 处理：
// - 缓存 DI/AI 采样
// - 生成 system / IO 类原始故障真源
// - 为未来 DI 去抖 / AI 阈值 / DO 输出预留入口
//
#include "logic_engine.h"

namespace control {

namespace {

// ------------------------------------------------------------------
// 第七批约定：
// 先把“SD 卡在位”接成一个 DI 位真源。
// 当前原始代码里没有看到现成的 sdcard online/present 来源，
// 所以先采用 IoSampleEvent.di_bits 的某一位作为“SD 卡插入检测”位。
//
// 约定：
// - bit = 1  => 检测到 SD 卡在位
// - bit = 0  => 未检测到 SD 卡在位（视为 sdcard_fault 原始故障）
//
// 如果你实际项目里 SD 卡插入位不是 DI0，后面只改这里的位号即可。
// ------------------------------------------------------------------
static constexpr int kDiBitSdcardPresent = 0;

static inline bool testDiBit_(uint64_t bits, int bit)
{
    if (bit < 0 || bit >= 64) return false;
    return ((bits >> bit) & 0x1ULL) != 0ULL;
}

} // namespace

void LogicEngine::onIoSample_(const IoSampleEvent& s,
                              LogicContext& ctx,
                              std::vector<Command>& out_cmds)
{
    (void)out_cmds;

    ctx.last_io_ts = s.ts_ms;
    ctx.di_bits = s.di_bits;
    ctx.ai = s.ai;

    // ------------------------------------------------------------
    // 第七批：system / IO 类原始故障真源
    // ------------------------------------------------------------

    // SD 卡插入检测：
    // 1 = 在位
    // 0 = 不在位 -> 记为 sdcard_fault 原始真源
    const bool sdcard_present = testDiBit_(s.di_bits, kDiBitSdcardPresent);
    ctx.logic_faults.sdcard_fault = !sdcard_present;

    // ------------------------------------------------------------
    // 维护 logic 聚合故障
    //
    // 说明：
    // 1) 当前 any_fault / env_any_alarm 原本主要由 snapshot / link / 其他链路更新；
    // 2) 这里先把 sdcard_fault 纳入 any_fault 聚合，
    //    避免即使 confirmed signal 已接好，但 any_fault 仍然不包含 sdcard_fault。
    // 3) env_any_alarm 仍然只保留环境类（气感/烟感/空调）语义，不把 sdcard 算进去。
    // ------------------------------------------------------------
    ctx.logic_faults.any_fault =
        ctx.logic_faults.system_estop ||
        ctx.logic_faults.sdcard_fault ||
        ctx.logic_faults.pcu_any_offline ||
        ctx.logic_faults.bms_any_offline ||
        ctx.logic_faults.ups_offline ||
        ctx.logic_faults.smoke_offline ||
        ctx.logic_faults.gas_offline ||
        ctx.logic_faults.air_offline;

    // 当前阶段先不在 IoSample 里修改 env_any_alarm：
    // env_any_alarm 继续由环境设备相关链路维护。
    //
    // 后续可在这里继续扩展：
    // 1) DI 去抖、边沿检测
    // 2) 其他系统输入故障（门禁、水浸、急停、储能舱状态等）
    // 3) AI 滤波、越限判断
    // 4) 生成 WriteDo 命令
}

} // namespace control