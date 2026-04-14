#pragma once

#include <cstdint>
#include "bms_logic_types.h"

namespace control {
    class FaultCenter;
    struct LogicContext;
}

namespace control::bms {

    /**
     * BMS 专用故障映射器
     *
     * 职责：
     * 1. 读取 BmsLogicCache / BmsPerInstanceCache
     * 2. 读取 LogicContext 中已经确认完成的 BMS confirmed faults
     * 3. 将运行态 / 显式故障位 / confirmed signals 统一翻译为 fault code
     *
     * 不负责：
     * - 不负责 HMI 写入
     * - 不负责分页
     * - 不负责历史记录
     * - 不负责重新计算 online/offline aging
     * - 不负责持续时间确认（那是 evaluator / condition engine 的职责）
     *
     * 第六批目标：
     * - 先统一输入风格：cache + confirmed signals
     * - 保留现有 F1/F2/runtime 映射不变
     * - 后续复杂持续时间型故障逐步迁到 evaluator，再由本 mapper 落码
     */
    class BmsFaultMapper {
    public:
        void applyToFaultPages(const BmsLogicCache& cache,
                               const control::LogicContext& ctx,
                               uint64_t now_ms,
                               control::FaultCenter& faults) const;

    private:
        void applyOne_(uint8_t inst,
                       const BmsPerInstanceCache* x,
                       const control::LogicContext& ctx,
                       uint64_t now_ms,
                       control::FaultCenter& faults) const;
    };

} // namespace control::bms