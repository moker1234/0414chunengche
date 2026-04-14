//
// Created by lxy on 2026/4/12.
//

#ifndef ENERGYSTORAGE_FAULT_CONDITION_STATE_H
#define ENERGYSTORAGE_FAULT_CONDITION_STATE_H

#pragma once

#include <cstdint>

namespace control::fault {

    /**
     * FaultConditionState
     *
     * 作用：
     * 1. 接收某条原始条件 raw_now（瞬时布尔值）
     * 2. 按 assert_delay_ms / clear_delay_ms 做持续时间确认
     * 3. 输出 confirmed_on（确认后的故障存在状态）
     *
     * 说明：
     * - raw_now=true 持续达到 assert_delay_ms 后，confirmed_on 才置 true
     * - raw_now=false 持续达到 clear_delay_ms 后，confirmed_on 才清 false
     * - clear_delay_ms=0 时，原始条件消失即可立即清除 confirmed_on
     *
     * 当前阶段只做最基础的“出现延时 + 消失延时”状态机，
     * 不引入复杂 latch / inhibit / reset reason 等扩展。
     */
    struct FaultConditionState
    {
        // 当前原始“出现条件”值（最近一次 update 传入）
        bool raw_on{false};

        // 当前原始“清除条件”值（最近一次 updateWithClear 传入）
        bool raw_clear{false};

        // 当前确认态：true 表示故障已确认存在
        bool confirmed_on{false};

        // 原始出现条件连续为 true 的起点时间
        uint64_t raw_on_since_ms{0};

        // 原始清除条件连续为 true 的起点时间
        uint64_t raw_clear_since_ms{0};

        // 最近一次 update 的时间
        uint64_t last_update_ms{0};

        // 最近一次 confirmed_on 变化时间
        uint64_t last_change_ms{0};

        // 参数（允许每条 condition 独立配置）
        uint32_t assert_delay_ms{0};
        uint32_t clear_delay_ms{0};

        // 调试辅助：最近一次 update 是否让 confirmed_on 发生了变化
        bool changed_last_update{false};

        // 调试辅助：最近一次变化是置 true 还是清 false
        bool last_change_to_on{false};

        void configure(uint32_t assert_delay, uint32_t clear_delay);

        // 重置到初始状态，但保留参数
        void reset();

        /**
         * 基础模式：
         * raw_now=true 持续 assert_delay_ms 后置 true；
         * raw_now=false 持续 clear_delay_ms 后清 false。
         *
         * 兼容第一批已有写法。
         */
        bool update(bool raw_now, uint64_t now_ms);

        /**
         * 增强模式：
         * - assert_now: 故障出现条件
         * - clear_now : 故障恢复条件
         *
         * 语义：
         * 1. 当当前未确认故障时，只看 assert_now
         * 2. 当当前已确认故障时，只看 clear_now
         * 3. 允许出现条件和清除条件不对称
         */
        bool updateWithClear(bool assert_now,
                             bool clear_now,
                             uint64_t now_ms);
    };

} // namespace control::fault

#endif // ENERGYSTORAGE_FAULT_CONDITION_STATE_H