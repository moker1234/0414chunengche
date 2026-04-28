// services/control/logic/logic_hmi_input.cpp
//
// HMI 上行写入处理：
// - 记录写入状态
// - 故障页翻页
// - coil 按键采用“按下后松开才生效”的状态机
//
#include "logic_engine.h"

#include <cstdio>
#include <string>

// #include "../fault/fault_addr_layout.h"
#include "../utils/logger/logger.h"
#include "../../normal/hmi_map_model.h"

namespace control
{
    namespace
    {
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

        static bool startsWith_(const std::string& s, const char* prefix)
        {
            if (!prefix) return false;
            const std::string p(prefix);
            return s.rfind(p, 0) == 0;
        }

        /*
         * path 兼容归一化。
         *
         * 新版推荐：
         *   hmi.ctrl.btn.xxx
         *   hmi.fault.btn.xxx
         *   hmi.param.xxx
         *
         * 旧版/过渡版可能还有：
         *   hmi.mainten.btn.xxx
         *
         * 这里统一转成控制层更稳定的 path，避免 jsonl 新旧版本切换时按钮失效。
         */
        static std::string normalizeHmiPath_(const std::string& path)
        {
            /*
             * 维护界面旧按钮不再做兼容归一化。
             *
             * 原来这里会把：
             *   hmi.mainten.btn.main_pwd_enter -> hmi.ctrl.btn.maintain_password_enter
             *   hmi.mainten.btn.box_num_set    -> hmi.ctrl.btn.box_num_set
             *   hmi.mainten.btn.ups_off        -> hmi.ctrl.btn.ups_shutdown
             *   hmi.mainten.btn.aircon_set     -> hmi.ctrl.btn.aircon_set
             *   hmi.mainten.btn.pwd_set        -> hmi.ctrl.btn.password_set
             *
             * 现在维护界面改为直接使用 int_rw 参数：
             *   hmi.param.maintain_password
             *   hmi.param.password_new_1
             *   hmi.param.box_num
             *   hmi.param.ups_shutdown_time
             *   hmi.param.aircon_set_temp
             *   hmi.param.aircon_set_humidity
             *
             * 因此旧维护按钮不再映射到控制按钮，避免误触发旧业务。
             */
            return path;
        }

        enum class HmiWriteLookupKind_
        {
            AnyRw = 0,
            BoolRw,
            IntRw,
        };
        static bool isDeprecatedMaintainButtonPath_(const std::string& path)
        {
            /*
             * 旧 HMI 维护按钮。
             *
             * hmi.mainten.btn.* 是 jsonl 里当前仍可能存在的旧 path；
             * hmi.ctrl.btn.* 是旧版本 normalizeHmiPath_() 映射后的过渡 path。
             *
             * 这些按钮全部降级为 deprecated：
             * - 不置 requested 标志
             * - 不发 UPS/空调命令
             * - 不修改密码
             * - 不影响故障翻页/上电/下电等其他按钮
             */
            return
                path == "hmi.mainten.btn.main_pwd_enter" ||
                path == "hmi.mainten.btn.box_num_set" ||
                path == "hmi.mainten.btn.ups_off" ||
                path == "hmi.mainten.btn.aircon_set" ||
                path == "hmi.mainten.btn.pwd_set" ||

                path == "hmi.ctrl.btn.maintain_password_enter" ||
                path == "hmi.ctrl.btn.box_num_set" ||
                path == "hmi.ctrl.btn.ups_shutdown" ||
                path == "hmi.ctrl.btn.aircon_set" ||
                path == "hmi.ctrl.btn.password_set";
        }

        struct HmiMappedWriteItem_
        {
            uint16_t addr{0}; // 实际写入地址
            uint16_t base_addr{0}; // normal_map_logic 中该变量的起始地址
            uint16_t word_offset{0}; // addr - base_addr
            uint16_t words{1};

            normal::HmiMapType type{normal::HmiMapType::Unknown};

            std::string raw_path; // jsonl 原始 path
            std::string path; // normalize 后的 path
            std::string name;
            std::string value_type;

            bool valid() const
            {
                return !path.empty() && path != "_";
            }

            bool isBoolRw() const
            {
                return type == normal::HmiMapType::BoolRw;
            }

            bool isIntRw() const
            {
                return type == normal::HmiMapType::IntRw;
            }
        };

        static bool isEmptyOrPlaceholderPath_(const std::string& path)
        {
            return path.empty() || path == "_";
        }

        static bool matchLookupKind_(normal::HmiMapType type,
                                     HmiWriteLookupKind_ kind)
        {
            switch (kind)
            {
            case HmiWriteLookupKind_::BoolRw:
                return type == normal::HmiMapType::BoolRw;

            case HmiWriteLookupKind_::IntRw:
                return type == normal::HmiMapType::IntRw;

            case HmiWriteLookupKind_::AnyRw:
            default:
                return type == normal::HmiMapType::BoolRw ||
                    type == normal::HmiMapType::IntRw;
            }
        }

        static bool fillMappedWriteItem_(const normal::HmiMapItem& it,
                                         uint16_t write_addr,
                                         HmiWriteLookupKind_ kind,
                                         HmiMappedWriteItem_& out)
        {
            if (!it.has_addr)
            {
                return false;
            }

            if (!matchLookupKind_(it.type, kind))
            {
                return false;
            }

            if (isEmptyOrPlaceholderPath_(it.path))
            {
                return false;
            }

            const uint16_t words = (it.words == 0) ? 1 : it.words;

            const uint32_t base = it.addr;
            const uint32_t end_exclusive = base + static_cast<uint32_t>(words);
            const uint32_t addr_u32 = write_addr;

            if (addr_u32 < base || addr_u32 >= end_exclusive)
            {
                return false;
            }

            HmiMappedWriteItem_ m;
            m.addr = write_addr;
            m.base_addr = it.addr;
            m.word_offset = static_cast<uint16_t>(addr_u32 - base);
            m.words = words;
            m.type = it.type;
            m.raw_path = it.path;
            m.path = normalizeHmiPath_(it.path);
            m.name = it.name;
            m.value_type = it.value_type;

            if (!m.valid())
            {
                return false;
            }

            out = std::move(m);
            return true;
        }

        static bool findHmiWriteItemByAddress_(const LogicContext& ctx,
                                               uint16_t addr,
                                               HmiWriteLookupKind_ kind,
                                               HmiMappedWriteItem_& out)
        {
            out = HmiMappedWriteItem_{};

            if (!ctx.hmi_map_loaded || !ctx.hmi_map)
            {
                return false;
            }

            /*
             * 1) 优先精确地址反查。
             * by_addr 只索引 item.addr，即变量起始地址。
             */
            if (const auto* owners = ctx.hmi_map->findByAddr(addr))
            {
                for (std::size_t idx : *owners)
                {
                    const normal::HmiMapItem* it = ctx.hmi_map->itemAt(idx);
                    if (!it) continue;

                    if (fillMappedWriteItem_(*it, addr, kind, out))
                    {
                        return true;
                    }
                }
            }

            /*
             * 2) 再做范围反查。
             *
             * 目的：
             * int_rw 中有 2-word 参数，例如：
             *   0x2000/0x2001 -> hmi.param.maintain_password
             *   0x2006/0x2007 -> hmi.param.password_old
             *
             * 如果屏幕只写了第二个 word，by_addr 精确查不到，
             * 这里仍然可以按 [base, base + words) 找回变量。
             */
            for (std::size_t idx : ctx.hmi_map->rw_items)
            {
                const normal::HmiMapItem* it = ctx.hmi_map->itemAt(idx);
                if (!it) continue;

                if (fillMappedWriteItem_(*it, addr, kind, out))
                {
                    return true;
                }
            }

            return false;
        }

        static void cacheWrittenRegs_(LogicContext& ctx,
                                      const HmiWriteEvent& w)
        {
            for (std::size_t i = 0; i < w.regs.size(); ++i)
            {
                const uint32_t addr =
                    static_cast<uint32_t>(w.start_addr) + static_cast<uint32_t>(i);

                if (addr > 0xFFFFu)
                {
                    break;
                }

                ctx.hmi_reg_state[static_cast<uint16_t>(addr)] = w.regs[i];
            }
        }

        static uint16_t regStateOrZero_(const LogicContext& ctx,
                                        uint16_t addr)
        {
            auto it = ctx.hmi_reg_state.find(addr);
            if (it == ctx.hmi_reg_state.end())
            {
                return 0;
            }

            return it->second;
        }

        static int32_t readMappedRegValue_(const LogicContext& ctx,
                                           const HmiMappedWriteItem_& item)
        {
            /*
             * 当前 normal_map_logic.jsonl 中 int_rw 最大重点是 1-word / 2-word。
             *
             * 2-word 按高字在前：
             *   base     -> high
             *   base + 1 -> low
             */
            if (item.words >= 2)
            {
                const uint16_t hi = regStateOrZero_(ctx, item.base_addr);
                const uint16_t lo = regStateOrZero_(
                    ctx,
                    static_cast<uint16_t>(item.base_addr + 1u)
                );

                const uint32_t combined =
                    (static_cast<uint32_t>(hi) << 16) |
                    static_cast<uint32_t>(lo);

                return static_cast<int32_t>(combined);
            }

            return static_cast<int32_t>(regStateOrZero_(ctx, item.base_addr));
        }

        static void mirrorKnownHmiParam_(LogicContext& ctx,
                                         const std::string& path,
                                         int32_t value)
        {
            /*
             * 维护参数不再镜像到 LogicContext 的旧字段。
             *
             * 原因：
             * 1. hmi.param.maintain_password 是控制器返回给 HMI 的当前密码，
             *    由 MaintainService 同步到 HMI int_rw 表，不再由 LogicEngine 解释。
             *
             * 2. hmi.param.password_new_1 已由 MaintainService 在 ControlLoop 入口消费，
             *    写入 maintain.json 后再同步回 hmi.param.maintain_password。
             *
             * 3. hmi.param.password_old / password_new_2 已废弃。
             *
             * 4. box_num / ups_shutdown_time / aircon_set_temp / aircon_set_humidity
             *    后续也应进入 MaintainService，不再靠旧按钮触发。
             *
             * 注意：
             * cacheHmiParam_() 仍然会把所有 HMI 参数写入：
             *   ctx.hmi_param_values[path]
             *   ctx.hmi_param_addr[path]
             *   ctx.hmi_param_name[path]
             *   ctx.hmi_param_ts[path]
             *
             * 这里只是不再同步到旧的专用字段，避免旧维护业务复活。
             */
            (void)ctx;
            (void)path;
            (void)value;
        }

        static void cacheHmiParam_(LogicContext& ctx,
                                   uint16_t addr,
                                   int32_t value,
                                   const std::string& path,
                                   const std::string& name,
                                   uint64_t ts_ms)
        {
            ctx.hmi_param_values[path] = value;
            ctx.hmi_param_addr[path] = addr;
            ctx.hmi_param_name[path] = name;
            ctx.hmi_param_ts[path] = ts_ms;

            ctx.last_hmi_param_ts = ts_ms;
            ctx.last_hmi_param_addr = addr;
            ctx.last_hmi_param_value = static_cast<uint16_t>(value & 0xFFFF);
            ctx.last_hmi_param_path = path;
            ctx.last_hmi_param_name = name;

            mirrorKnownHmiParam_(ctx, path, value);
        }
    } // namespace

    void LogicEngine::onHmiWrite_(const HmiWriteEvent& w,
                                  LogicContext& ctx,
                                  std::vector<Command>& out_cmds)
    {
        ctx.last_hmi_comm_ts = w.ts_ms;
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

            // 保存本次写入的所有寄存器，支持 2-word 参数。
            cacheWrittenRegs_(ctx, w);

            /*
             * 第七批关键变化：
             *
             * HMI 写入反查不再依赖 NormalHmiWriter。
             * 直接从 ctx.hmi_map / HmiMapModel 反查 int_rw 地址。
             */
            HmiMappedWriteItem_ mapped_item{};
            const bool mapped =
                findHmiWriteItemByAddress_(ctx,
                                           w.start_addr,
                                           HmiWriteLookupKind_::IntRw,
                                           mapped_item);

            if (!mapped)
            {
                LOG_COMM_D("[CTRL][HMI][REG] unmapped addr=0x%04X value=%u hmi_map_loaded=%d",
                           w.start_addr,
                           (unsigned)curr,
                           ctx.hmi_map_loaded ? 1 : 0);
                return;
            }

            if (startsWith_(mapped_item.path, "hmi.param."))
            {
                const int32_t value = readMappedRegValue_(ctx, mapped_item);

                cacheHmiParam_(ctx,
                               mapped_item.base_addr,
                               value,
                               mapped_item.path,
                               mapped_item.name,
                               w.ts_ms);

                LOGINFO("[CTRL][HMI][PARAM] addr=0x%04X base=0x%04X off=%u words=%u path=%s name=%s value=%d",
                        w.start_addr,
                        mapped_item.base_addr,
                        (unsigned)mapped_item.word_offset,
                        (unsigned)mapped_item.words,
                        mapped_item.path.c_str(),
                        mapped_item.name.c_str(),
                        (int)value);
                return;
            }

            /*
             * int_rw 里如果后续放了非 hmi.param.* 的 path，
             * 这里先只记录，不直接动作，避免误触发控制。
             */
            LOG_COMM_D("[CTRL][HMI][REG] mapped but not param addr=0x%04X base=0x%04X off=%u path=%s name=%s value=%u",
                       w.start_addr,
                       mapped_item.base_addr,
                       (unsigned)mapped_item.word_offset,
                       mapped_item.path.c_str(),
                       mapped_item.name.c_str(),
                       (unsigned)curr);
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

        /*
         * 第七批关键变化：
         *
         * coil 写入反查不再依赖 NormalHmiWriter。
         * 直接从 ctx.hmi_map / HmiMapModel 反查 bool_rw 地址。
         */
        HmiMappedWriteItem_ mapped_item{};
        const bool mapped =
            findHmiWriteItemByAddress_(ctx,
                                       addr,
                                       HmiWriteLookupKind_::BoolRw,
                                       mapped_item);

        if (!mapped)
        {
            LOG_COMM_D("[CTRL][HMI][COIL] ignore unmapped addr=0x%04X prev=%u curr=%u hmi_map_loaded=%d",
                       addr,
                       prev,
                       curr,
                       ctx.hmi_map_loaded ? 1 : 0);
            return;
        }

        if (!mapped_item.isBoolRw())
        {
            LOG_COMM_D("[CTRL][HMI][COIL] ignore non-bool-rw addr=0x%04X path=%s name=%s",
                       addr,
                       mapped_item.path.c_str(),
                       mapped_item.name.c_str());
            return;
        }

        auto& st = ctx.hmi_button_states[addr];

        LOG_COMM_D("[CTRL][HMI][COIL] addr=0x%04X path=%s raw_path=%s name=%s prev=%u curr=%u is_down=%d armed=%d",
                   addr,
                   mapped_item.path.c_str(),
                   mapped_item.raw_path.c_str(),
                   mapped_item.name.c_str(),
                   prev,
                   curr,
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
            if (!st.is_down)
            {
                st.is_down = true;
                st.armed = true;
                st.press_ts_ms = w.ts_ms;

                LOG_COMM_D("[CTRL][HMI][BTN] press armed addr=0x%04X path=%s ts=%llu",
                           addr,
                           mapped_item.path.c_str(),
                           static_cast<unsigned long long>(w.ts_ms));
            }

            return;
        }

        // B. 松开
        if (curr == 0u)
        {
            st.release_ts_ms = w.ts_ms;

            if (st.is_down && st.armed)
            {
                st.is_down = false;
                st.armed = false;

                onHmiButtonClick_(addr,
                                  mapped_item.path,
                                  mapped_item.name,
                                  ctx,
                                  out_cmds);
                return;
            }

            st.is_down = false;
            st.armed = false;
            return;
        }
    }

    void LogicEngine::onHmiButtonClick_(uint16_t addr,
                                        const std::string& path,
                                        const std::string& name,
                                        LogicContext& ctx,
                                        std::vector<Command>& out_cmds)
    {
        (void)out_cmds;

        ctx.last_hmi_button_ts = ctx.last_event_ts;
        ctx.last_hmi_button_addr = addr;
        ctx.last_hmi_button_path = path;
        ctx.last_hmi_button_name = name;

        LOGINFO(
            "[LOGIC][HMI][BUTTON_CLICK] begin addr=0x%04X path=%s name=%s cur_page=%u cur_total=%u his_page=%u his_total=%u history_view=%d",
            addr,
            path.c_str(),
            name.c_str(),
            (unsigned)ctx.fault_center.debugCurrentPageIndex(),
            (unsigned)ctx.fault_center.debugCurrentTotalPages(),
            (unsigned)ctx.history_page_no,
            (unsigned)(ctx.fault_history_cache ? ctx.fault_history_cache->totalPages() : 0),
            ctx.history_view_active ? 1 : 0);

        auto refreshHistoryCacheForPage = [&](uint16_t page)
        {
            if (!ctx.fault_history_cache)
            {
                return false;
            }

            if (!ctx.fault_history_cache->refreshMeta())
            {
                return false;
            }

            const uint16_t total_pages = ctx.fault_history_cache->totalPages();
            if (total_pages == 0)
            {
                ctx.history_page_no = 1;
                return ctx.fault_history_cache->refreshWindow(1);
            }

            if (page < 1) page = 1;
            if (page > total_pages) page = total_pages;

            ctx.history_page_no = page;
            return ctx.fault_history_cache->ensurePageLoaded(ctx.history_page_no);
        };

        bool fault_page_changed = false;

        // 旧维护按钮
        // if (isDeprecatedMaintainButtonPath_(path))
        // {
        //     LOGINFO("[LOGIC][HMI][DEPRECATED_MAINTAIN_BTN] ignored addr=0x%04X path=%s name=%s bit=%u",
        //             addr,
        //             path.c_str(),
        //             name.c_str(),
        //             (unsigned)firstBitOrZero_(w));
        //     return;
        // }

        // ============================================================
        // 1) 业务控制按键：来自 normal_map_logic.jsonl
        // ============================================================
        if (path == "hmi.ctrl.btn.hv_on")
        {
            ctx.hmi_hv_on_requested = true;
            ctx.hmi_hv_off_requested = false;
            ctx.hmi_hv_on_request_ts = ctx.last_event_ts;

            LOGINFO("[LOGIC][HMI][CTRL] hv_on requested by addr=0x%04X name=%s",
                    addr,
                    name.c_str());
            return;
        }

        if (path == "hmi.ctrl.btn.hv_off")
        {
            ctx.hmi_hv_off_requested = true;
            ctx.hmi_hv_on_requested = false;
            ctx.hmi_hv_off_request_ts = ctx.last_event_ts;

            LOGINFO("[LOGIC][HMI][CTRL] hv_off requested by addr=0x%04X name=%s",
                    addr,
                    name.c_str());
            return;
        }
        //         if (path == "hmi.ctrl.btn.fault_reset")   /// 故障复位按键，废除
        // {
        //     ctx.hmi_fault_reset_requested = true;
        //     ctx.hmi_fault_reset_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] fault_reset requested addr=0x%04X name=%s",
        //             addr,
        //             name.c_str());
        //     return;
        // }
        //
        // if (path == "hmi.fault.btn.home_ack") // 首页故障确认按键，废除
        // {
        //     ctx.hmi_fault_home_ack_requested = true;
        //     ctx.hmi_fault_home_ack_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] home_ack requested addr=0x%04X name=%s",
        //             addr,
        //             name.c_str());
        //     return;
        // }
        //
        // if (path == "hmi.fault.btn.ack_all")  // 故障全部确认按键，废除
        // {
        //     ctx.hmi_fault_ack_all_requested = true;
        //     ctx.hmi_fault_ack_all_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] ack_all requested addr=0x%04X name=%s",
        //             addr,
        //             name.c_str());
        //     return;
        // }
        //

        // 废弃的维护按键
        // if (path == "hmi.ctrl.btn.maintain_password_enter")    // 维护密码确认 按键，废除
        // {
        //     ctx.hmi_maintain_password_enter_requested = true;
        //     ctx.hmi_maintain_password_enter_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] maintain_password_enter requested addr=0x%04X name=%s pwd=%d",
        //             addr,
        //             name.c_str(),
        //             (int)ctx.hmi_param_maintain_password);
        //     return;
        // }
        //
        // if (path == "hmi.ctrl.btn.box_num_set")     // 箱号设置指令 按键，废除
        // {
        //     ctx.hmi_box_num_set_requested = true;
        //     ctx.hmi_box_num_set_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] box_num_set requested addr=0x%04X name=%s box=%d",
        //             addr,
        //             name.c_str(),
        //             (int)ctx.hmi_param_box_num);
        //     return;
        // }
        //
        // if (path == "hmi.ctrl.btn.ups_shutdown")            // UPS关机 按键，废除
        // {
        //     ctx.hmi_ups_shutdown_requested = true;
        //     ctx.hmi_ups_shutdown_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] ups_shutdown requested addr=0x%04X name=%s delay=%d",
        //             addr,
        //             name.c_str(),
        //             (int)ctx.hmi_param_ups_shutdown_time);
        //     return;
        // }
        //
        // if (path == "hmi.ctrl.btn.aircon_set")             //空调温度湿度设置 按键，废除
        // {
        //     ctx.hmi_aircon_set_requested = true;
        //     ctx.hmi_aircon_set_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] aircon_set requested addr=0x%04X name=%s temp=%d humidity=%d",
        //             addr,
        //             name.c_str(),
        //             (int)ctx.hmi_param_aircon_set_temp,
        //             (int)ctx.hmi_param_aircon_set_humidity);
        //     return;
        // }
        //
        // if (path == "hmi.ctrl.btn.password_set")            //密码设置指令 按键，废除
        // {
        //     ctx.hmi_password_set_requested = true;
        //     ctx.hmi_password_set_request_ts = ctx.last_event_ts;
        //
        //     LOGINFO("[LOGIC][HMI][CTRL] password_set requested addr=0x%04X name=%s old=%d new1=%d new2=%d",
        //             addr,
        //             name.c_str(),
        //             (int)ctx.hmi_param_password_old,
        //             (int)ctx.hmi_param_password_new_1,
        //             (int)ctx.hmi_param_password_new_2);
        //     return;
        // }

        // ============================================================
        // 2) 故障页按键：来自 normal_map_logic.jsonl
        // ============================================================
        if (path == "hmi.fault.btn.his_page_enter")
        {
            ctx.history_view_active = true;
            ctx.fault_center.enterHistoryView();

            if (ctx.fault_history_cache)
            {
                const bool ok = refreshHistoryCacheForPage(1);

                LOGINFO("[LOGIC][HMI][ENTER_HISTORY] ok=%d page=%u total=%u window=%u~%u",
                        ok ? 1 : 0,
                        (unsigned)ctx.history_page_no,
                        (unsigned)ctx.fault_history_cache->totalPages(),
                        (unsigned)ctx.fault_history_cache->windowStartPage(),
                        (unsigned)ctx.fault_history_cache->windowEndPage());
            }
            else
            {
                ctx.history_page_no = 1;
            }

            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.cur_page_next")
        {
            ctx.history_view_active = false;
            ctx.fault_center.nextCurrentPage();
            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.cur_page_prev")
        {
            ctx.history_view_active = false;
            ctx.fault_center.prevCurrentPage();
            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.his_page_next")
        {
            ctx.history_view_active = true;
            ctx.fault_center.enterHistoryView();

            if (ctx.fault_history_cache)
            {
                if (ctx.fault_history_cache->refreshMeta())
                {
                    const uint16_t total_pages = ctx.fault_history_cache->totalPages();

                    if (total_pages == 0)
                    {
                        ctx.history_page_no = 1;
                        ctx.fault_history_cache->refreshWindow(1);
                    }
                    else
                    {
                        uint16_t target = ctx.history_page_no;
                        if (target < total_pages)
                        {
                            ++target;
                        }
                        else
                        {
                            target = total_pages;
                        }

                        refreshHistoryCacheForPage(target);
                    }
                }
                else
                {
                    LOG_THROTTLE_MS("his_next_refresh_meta_fail", 1000, LOGINFO,
                                    "[LOGIC][HMI][HIS_NEXT] refreshMeta failed");
                }
            }
            else
            {
                ctx.fault_center.nextHistoryPage();
            }

            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.his_page_prev")
        {
            ctx.history_view_active = true;
            ctx.fault_center.enterHistoryView();

            if (ctx.fault_history_cache)
            {
                if (ctx.fault_history_cache->refreshMeta())
                {
                    const uint16_t total_pages = ctx.fault_history_cache->totalPages();

                    if (total_pages == 0)
                    {
                        ctx.history_page_no = 1;
                        ctx.fault_history_cache->refreshWindow(1);
                    }
                    else
                    {
                        uint16_t target = ctx.history_page_no;
                        if (target > 1)
                        {
                            --target;
                        }
                        else
                        {
                            target = 1;
                        }

                        refreshHistoryCacheForPage(target);
                    }
                }
                else
                {
                    LOG_THROTTLE_MS("his_prev_refresh_meta_fail", 1000, LOGINFO,
                                    "[LOGIC][HMI][HIS_PREV] refreshMeta failed");
                }
            }
            else
            {
                ctx.fault_center.prevHistoryPage();
            }

            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.cur_page_first")
        {
            ctx.history_view_active = false;
            ctx.fault_center.toFirstCurrentPage();
            fault_page_changed = true;
        }
        else if (path == "hmi.fault.btn.his_page_first")
        {
            ctx.history_view_active = true;
            ctx.fault_center.enterHistoryView();

            if (ctx.fault_history_cache)
            {
                if (ctx.fault_history_cache->refreshMeta())
                {
                    refreshHistoryCacheForPage(1);
                }
            }

            fault_page_changed = true;
        }
        else
        {
            LOG_COMM_D("[LOGIC][HMI][BUTTON_CLICK] ignored unsupported path addr=0x%04X path=%s name=%s",
                       addr,
                       path.c_str(),
                       name.c_str());
            return;
        }

        if (fault_page_changed && ctx.hmi && ctx.fault_map_loaded)
        {
            applyFaultHmi_(ctx);
        }

        LOGINFO(
            "[LOGIC][HMI][BUTTON_CLICK] end addr=0x%04X path=%s cur_page=%u cur_total=%u his_page=%u his_total=%u history_view=%d",
            addr,
            path.c_str(),
            (unsigned)ctx.fault_center.debugCurrentPageIndex(),
            (unsigned)ctx.fault_center.debugCurrentTotalPages(),
            (unsigned)ctx.history_page_no,
            (unsigned)(ctx.fault_history_cache ? ctx.fault_history_cache->totalPages() : 0),
            ctx.history_view_active ? 1 : 0);
    }
} // namespace control
