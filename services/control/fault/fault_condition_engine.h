//
// Created by lxy on 2026/4/12.
//

#ifndef ENERGYSTORAGE_FAULT_CONDITION_ENGINE_H
#define ENERGYSTORAGE_FAULT_CONDITION_ENGINE_H

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fault_condition_state.h"

namespace control::fault
{
    /**
     * FaultConditionEngine
     *
     * 作用：
     * - 用 string key 管理多条 FaultConditionState
     * - 统一提供“持续时间确认”的更新入口
     * - 后续 BMS evaluator / 通用 evaluator 都共用它
     *
     * 当前阶段：
     * - 不做线程安全封装（默认运行于控制面单线程/单上下文）
     * - 不依赖 fault_map.jsonl
     * - 不依赖 fault_ids.h
     */
    class FaultConditionEngine
    {
    public:
        FaultConditionEngine() = default;

        /**
         * 基础模式：
         * raw_now=true 持续 assert_delay_ms 后成立；
         * raw_now=false 持续 clear_delay_ms 后清除。
         */
        bool updateCondition(const std::string& key,
                             bool raw_now,
                             uint64_t now_ms,
                             uint32_t assert_delay_ms,
                             uint32_t clear_delay_ms = 0);

        /**
         * 增强模式：
         * assert_now / clear_now 分离。
         */
        bool updateConditionWithClear(const std::string& key,
                                      bool assert_now,
                                      bool clear_now,
                                      uint64_t now_ms,
                                      uint32_t assert_delay_ms,
                                      uint32_t clear_delay_ms = 0);

        /**
         * 读取确认态
         * 不存在的 key 返回 false
         */
        bool getConfirmed(const std::string& key) const;

        /**
         * 获取某条 condition 当前状态
         * 不存在则返回 nullptr
         */
        const FaultConditionState* getState(const std::string& key) const;
        FaultConditionState* getState(const std::string& key);

        /**
         * 确保某条 condition 存在，并返回其状态对象
         */
        FaultConditionState& ensureState(const std::string& key);

        /**
         * 重置某条 condition（保留 key，参数仍可后续重新 configure）
         */
        void resetCondition(const std::string& key);

        /**
         * 清空全部 condition
         */
        void clear();

        /**
         * 获取所有 key，便于调试输出
         */
        std::vector<std::string> keys() const;

        /**
         * 只读查看内部状态表
         */
        const std::map<std::string, FaultConditionState>& states() const { return states_; }

    private:
        std::map<std::string, FaultConditionState> states_;
    };
} // namespace control::fault

#endif // ENERGYSTORAGE_FAULT_CONDITION_ENGINE_H
