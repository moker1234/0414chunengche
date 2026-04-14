//
// Created by lxy on 2026/3/2.
//

#ifndef ENERGYSTORAGE_FAULT_ADDR_LAYOUT_H
#define ENERGYSTORAGE_FAULT_ADDR_LAYOUT_H

#pragma once
#include <cstdint>

namespace control::fault {

    // ============================================================
    // 一阶段故障页地址布局
    //
    // 说明：
    // 1) fault_hmi_layout.jsonl 虽然按 12 行页面能力设计，
    //    但当前项目一阶段只对接“前 5 行”。
    // 2) 因此程序侧分页与 HMI 输出统一固定为 5 行。
    // 3) HMI 当前页故障代码读取目标：
    //      0x4125 ~ 0x4129
    // ============================================================
    static constexpr uint16_t FAULTS_PER_PAGE = 5;
    static constexpr uint16_t FAULT_PAGE_MAX  = 999;

    // ===== 当前故障页 =====
    static constexpr uint16_t ADDR_CUR_TOTAL_PAGES = 0x411B;
    static constexpr uint16_t ADDR_CUR_PAGE_INDEX  = 0x411C;

    // 当前页第1~5行序号
    static constexpr uint16_t ADDR_CUR_SEQ_BASE    = 0x411D; // 0x411D ~ 0x4121

    // 当前页第1~5行故障代码（本阶段 HMI 重点读取区）
    static constexpr uint16_t ADDR_CUR_CODE_BASE   = 0x4125; // 0x4125 ~ 0x4129

    // 当前页第1~5行触发时间：
    // 32位秒级时间戳，每条占2个16位寄存器，共5条=10寄存器
    static constexpr uint16_t ADDR_CUR_ON_TIME_BASE = 0x412D; // 0x412D ~ 0x4136

    // ===== 历史故障页 =====
    static constexpr uint16_t ADDR_HIS_TOTAL_PAGES   = 0x4141;
    static constexpr uint16_t ADDR_HIS_PAGE_INDEX    = 0x4142;

    // 历史页第1~5行序号
    static constexpr uint16_t ADDR_HIS_SEQ_BASE      = 0x4143; // 0x4143 ~ 0x4147

    // 历史页第1~5行故障代码
    static constexpr uint16_t ADDR_HIS_CODE_BASE     = 0x414F; // 0x414F ~ 0x4153

    // 历史页触发/清除时间：
    // 32位秒级时间戳，每条占2个16位寄存器，共5条=10寄存器
    static constexpr uint16_t ADDR_HIS_ON_TIME_BASE  = 0x415B; // 0x415B ~ 0x4164
    static constexpr uint16_t ADDR_HIS_OFF_TIME_BASE = 0x4173; // 0x4173 ~ 0x417C

    // 历史页状态（0=已清除，1=仍活动）
    static constexpr uint16_t ADDR_HIS_STATE_BASE    = 0x418B; // 0x418B ~ 0x418F

    // ===== HMI 按键地址 =====
    static constexpr uint16_t COIL_ENTER_HISTORY = 0x1005;
    static constexpr uint16_t COIL_CUR_NEXT      = 0x1006;
    static constexpr uint16_t COIL_CUR_PREV      = 0x1007;
    static constexpr uint16_t COIL_HIS_NEXT      = 0x1008;
    static constexpr uint16_t COIL_HIS_PREV      = 0x1009;

} // namespace control::fault

#endif // ENERGYSTORAGE_FAULT_ADDR_LAYOUT_H