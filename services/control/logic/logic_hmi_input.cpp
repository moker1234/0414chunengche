// services/control/logic/logic_hmi_input.cpp
//
// HMI 上行写入处理：
// - 记录写入状态
// - 故障页翻页
// - coil 按键采用“按下后松开才生效”的状态机
//
#include "logic_engine.h"

#include <cstdio>

#include "../fault/fault_addr_layout.h"
#include "../utils/logger/logger.h"

namespace control {

    namespace {

        static uint16_t firstRegOrZero_(const HmiWriteEvent& w)
        {
            return w.regs.empty() ? 0u : w.regs.front();
        }

        static uint8_t firstBitOrZero_(const HmiWriteEvent& w)
        {
            return w.bits.empty() ? 0u : (w.bits.front() ? 1u : 0u);
        }

        static bool isBoolWrite_(const HmiWriteEvent& w)
        {
            return !w.bits.empty();
        }

        static bool isRegWrite_(const HmiWriteEvent& w)
        {
            return !w.regs.empty();
        }

    } // namespace

    void LogicEngine::onHmiWrite_(const HmiWriteEvent& w,
                                  LogicContext& ctx,
                                  std::vector<Command>& out_cmds)
    {
        ctx.last_hmi_write_ts = w.ts_ms;
        ctx.last_hmi_addr = w.start_addr;

        // 第八批 + 第十四批：
        // 只要收到 HMI 上行写入，就视为 HMI 曾经在线，
        // 并立即清除当前 raw hmi_comm_fault 真源。
        ctx.hmi_seen_once = true;
        ctx.logic_faults.hmi_comm_fault = false;

        // 第十四批：就地同步一次 any_fault，
        // 避免在下一次 snapshot / fault refresh 之前，总故障口径短暂滞后。
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
            ctx.logic_faults.air_offline;

        // ------------------------------------------------------------
        // 1) coil 写：按键状态机
        // ------------------------------------------------------------
        if (isBoolWrite_(w))
        {
            const uint16_t curr = static_cast<uint16_t>(firstBitOrZero_(w));
            ctx.last_hmi_value = curr;

            handleHmiCoilWrite_(w, ctx, out_cmds);
            return;
        }

        // ------------------------------------------------------------
        // 2) reg 写：
        // ------------------------------------------------------------
        if (isRegWrite_(w))
        {
            const uint16_t curr = firstRegOrZero_(w);
            ctx.last_hmi_value = curr;
            ctx.hmi_reg_state[w.start_addr] = curr;

            // 故障页翻页：屏幕写当前页号
            // if (w.start_addr == fault::ADDR_FAULT_PAGE_SELECT)
            // {
            //     ctx.fault_center.setCurrentPage(curr);
            //
            //     if (ctx.hmi && ctx.fault_map_loaded) {
            //         ctx.fault_center.flushToHmi(*ctx.hmi);
            //     }
            //     return;
            // }

            // 其他寄存器写入暂时只缓存，不做控制动作
            return;
        }

        // 空写入保护
        LOG_COMM_D("[CTRL][HMI] ignore empty write addr=0x%04X", w.start_addr);
    }

    void LogicEngine::handleHmiCoilWrite_(const HmiWriteEvent& w,
                                          LogicContext& ctx,
                                          std::vector<Command>& out_cmds)
    {
        const uint16_t addr = w.start_addr;
        const uint16_t curr = static_cast<uint16_t>(firstBitOrZero_(w));
        const uint16_t prev = ctx.hmi_coil_state.count(addr)
                                  ? ctx.hmi_coil_state[addr]
                                  : 0u;

        ctx.hmi_coil_state[addr] = curr;

        auto& st = ctx.hmi_button_states[addr];

        LOG_COMM_D("[CTRL][HMI][COIL] addr=0x%04X prev=%u curr=%u is_down=%d armed=%d",
                   addr, prev, curr,
                   st.is_down ? 1 : 0,
                   st.armed ? 1 : 0);

        // ------------------------------------------------------------
        // 规则：
        // 1) 收到 1（FF00） -> 只记为“已按下，等待松开”
        // 2) 收到 0（0000）且此前确实按下过 -> 认定本次按键生效
        // ------------------------------------------------------------

        // A. 按下
        if (curr != 0u)
        {
            // 只有从未按下态进入时，才重新武装
            if (!st.is_down)
            {
                st.is_down = true;
                st.armed = true;
                st.press_ts_ms = w.ts_ms;

                LOG_COMM_D("[CTRL][HMI][BTN] press armed addr=0x%04X ts=%llu",
                           addr,
                           static_cast<unsigned long long>(w.ts_ms));
            }

            // 已经按下时收到重复 1，直接忽略
            return;
        }

        // B. 松开
        if (curr == 0u)
        {
            st.release_ts_ms = w.ts_ms;

            // 只有“先按下，再松开”才生效
            if (st.is_down && st.armed)
            {
                st.is_down = false;
                st.armed = false;

                onHmiButtonClick_(addr, ctx, out_cmds);
                return;
            }

            // 单独来的 0 / 异常松开，只清状态，不生效
            st.is_down = false;
            st.armed = false;
            return;
        }
    }

    void LogicEngine::onHmiButtonClick_(uint16_t addr,
                                        LogicContext& ctx,
                                        std::vector<Command>& out_cmds)
    {

        LOGINFO("[LOGIC][HMI][BUTTON_CLICK] begin addr=0x%04X cur_page=%u cur_total=%u",
          addr,
          (unsigned)ctx.fault_center.debugCurrentPageIndex(),
          (unsigned)ctx.fault_center.debugCurrentTotalPages());

        switch (addr) {
        case fault::COIL_ENTER_HISTORY:
            ctx.fault_center.enterHistoryView();
            break;

        case fault::COIL_CUR_NEXT:
            ctx.fault_center.nextCurrentPage();
            break;

        case fault::COIL_CUR_PREV:
            ctx.fault_center.prevCurrentPage();
            break;

        case fault::COIL_HIS_NEXT:
            ctx.fault_center.nextHistoryPage();
            break;

        case fault::COIL_HIS_PREV:
            ctx.fault_center.prevHistoryPage();
            break;

        default:
            return;
        }

        if (ctx.hmi && ctx.fault_map_loaded) {
            ctx.fault_center.flushToHmi(*ctx.hmi);
        }

        LOGINFO("[LOGIC][HMI][BUTTON_CLICK] addr=0x%04X triggered", addr);
    }

} // namespace control