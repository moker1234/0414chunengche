// services/control/logic_tick.cpp
//
// Tick 周期逻辑：
// - BMS 周期命令
// - 故障页投影
// - logic_view 刷新
// - HMI 周期下行刷新
//
#include "logic_engine.h"

#include "../utils/logger/logger.h"
#include "fault/fault_addr_layout.h"

namespace control {
    namespace {
        // ===================== PCU runtime timeout =====================
        constexpr uint32_t PCU_RX_TIMEOUT_MS = 1500;
        constexpr uint32_t PCU_HB_STALE_MS   = 3000;

        // ===================== BMS runtime timeout =====================
        constexpr uint32_t BMS_INSTANCE_TIMEOUT_MS      = 3000;
        constexpr uint32_t BMS_ST1_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST2_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST3_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST4_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST5_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST6_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_ST7_TIMEOUT_MS           = 3000;
        constexpr uint32_t BMS_CURRENT_LIMIT_TIMEOUT_MS = 3000;
        constexpr uint32_t BMS_ELEC_ENERGY_TIMEOUT_MS   = 5000;
        constexpr uint32_t BMS_TM2B_TIMEOUT_MS          = 5000;
        constexpr uint32_t BMS_FIRE2B_TIMEOUT_MS        = 5000;
        constexpr uint32_t BMS_FAULT1_TIMEOUT_MS        = 5000;
        constexpr uint32_t BMS_FAULT2_TIMEOUT_MS        = 5000;
    }

    void LogicEngine::onTick_(const TickEvent& t,
                              LogicContext& ctx,
                              std::vector<Command>& out_cmds)
    {
    //     LOG_THROTTLE_MS("logic_tick_alive", 1000, LOG_COMM_D,  // 20260409 检查online
    // "[LOGIC][TICK] ts=%llu pcu0{on=%d rx=%d hb=%d} pcu1{on=%d rx=%d hb=%d} bms_items=%zu",
    // (unsigned long long)t.ts_ms,
    // ctx.pcu0_state.online ? 1 : 0,
    // ctx.pcu0_state.rx_alive ? 1 : 0,
    // ctx.pcu0_state.hb_alive ? 1 : 0,
    // ctx.pcu1_state.online ? 1 : 0,
    // ctx.pcu1_state.rx_alive ? 1 : 0,
    // ctx.pcu1_state.hb_alive ? 1 : 0,
    // ctx.bms_cache.items.size());
        // -----------------------------------------------------------------
        // A) 周期性 BMS 命令：不能依赖 HMI 存在与否
        // -----------------------------------------------------------------
        if (bms_cmd_mgr_inited_)
        {
            bms_cmd_mgr_.rebuildDesiredFromCache(ctx.bms_cache, t.ts_ms);
            bms_cmd_mgr_.emitPeriodicCommands(t.ts_ms, out_cmds, 100);

            // LOG_THROTTLE_MS("ctrl_bms_tx_tick", 1000, LOG_SYS_I,
            //                 "[CTRL][BMS] periodic cmd tick online=%d any_fault=%d cmds=%zu",
            //                 ctx.logic_view.value("bms_count_online", 0),
            //                 ctx.logic_view.value("bms_any_fault", 0),
            //                 out_cmds.size());
        }

        // -----------------------------------------------------------------
        // -----------------------------------------------------------------
        // B) runtime aging / online 判定
        // -----------------------------------------------------------------
        updatePcuRuntimeHealth_(ctx, t.ts_ms);
        updateBmsRuntimeHealth_(ctx, t.ts_ms);

        // -----------------------------------------------------------------
        // C) logic 聚合故障真源
        // -----------------------------------------------------------------
        ctx.logic_faults.last_eval_ts_ms = t.ts_ms;

        // BMS
        bool bms_any_offline = false;
        for (const auto& kv : ctx.bms_cache.items) {
            if (!kv.second.online) {
                bms_any_offline = true;
                break;
            }
        }
        ctx.logic_faults.bms_any_offline = bms_any_offline;

        // PCU
        ctx.logic_faults.pcu_any_offline =
            (!ctx.pcu0_state.online) || (!ctx.pcu1_state.online);

        // 其他设备 offline
        ctx.logic_faults.ups_offline   = ctx.ups_faults.seen_once   && !ctx.ups_faults.online;
        ctx.logic_faults.smoke_offline = ctx.smoke_faults.seen_once && !ctx.smoke_faults.online;
        ctx.logic_faults.gas_offline   = ctx.gas_faults.seen_once   && !ctx.gas_faults.online;
        ctx.logic_faults.air_offline   = ctx.air_faults.seen_once   && !ctx.air_faults.online;

        // 环境类告警
        ctx.logic_faults.env_any_alarm =
            ctx.ups_faults.alarm_any ||
            ctx.smoke_faults.alarm_any ||
            ctx.gas_faults.alarm_any ||
            ctx.air_faults.alarm_any;

        // E-Stop
        ctx.logic_faults.system_estop = ctx.e_stop_latched;

        // 汇总
        ctx.logic_faults.any_fault =
            ctx.logic_faults.pcu_any_offline ||
            ctx.logic_faults.bms_any_offline ||
            ctx.logic_faults.ups_offline ||
            ctx.logic_faults.smoke_offline ||
            ctx.logic_faults.gas_offline ||
            ctx.logic_faults.air_offline ||
            ctx.logic_faults.env_any_alarm ||
            ctx.logic_faults.system_estop;

        // -----------------------------------------------------------------
        // D) 故障页投影
        // -----------------------------------------------------------------
        applyFaultPages_(ctx, t.ts_ms);

        // 第八批：联调日志，直接观察“当前页前5条故障码”是否已形成
        {
            const auto visible = ctx.fault_center.debugCurrentVisibleCodes();
            const auto rows = ctx.fault_center.debugCurrentRows();
            const uint16_t cur_page = ctx.fault_center.debugCurrentPageIndex();
            const uint16_t cur_total = ctx.fault_center.debugCurrentTotalPages();

            auto row_or_empty = [&](size_t idx) -> FaultCenterCurrentRow {
                return (idx < rows.size()) ? rows[idx] : FaultCenterCurrentRow{};
            };

            const auto r0 = row_or_empty(0);
            const auto r1 = row_or_empty(1);
            const auto r2 = row_or_empty(2);
            const auto r3 = row_or_empty(3);
            const auto r4 = row_or_empty(4);

            // LOG_THROTTLE_MS(
            //     "fault_current_rows", 1000, LOG_SYS_I,
            //     "[FAULT][CUR] visible=%zu page=%u/%u "
            //     "r1{seq=%u code=0x%04X on=%u} "
            //     "r2{seq=%u code=0x%04X on=%u} "
            //     "r3{seq=%u code=0x%04X on=%u} "
            //     "r4{seq=%u code=0x%04X on=%u} "
            //     "r5{seq=%u code=0x%04X on=%u}",
            //     visible.size(),
            //     static_cast<unsigned>(cur_total == 0 ? 0 : cur_page + 1),
            //     static_cast<unsigned>(cur_total),
            //     r0.seq_no, r0.code, r0.on_time,
            //     r1.seq_no, r1.code, r1.on_time,
            //     r2.seq_no, r2.code, r2.on_time,
            //     r3.seq_no, r3.code, r3.on_time,
            //     r4.seq_no, r4.code, r4.on_time
            // );
        }

        // -----------------------------------------------------------------
        // E) 逻辑视图 / HMI 输出
        // -----------------------------------------------------------------
        rebuildLogicView_(ctx);
        applyHmiOutputs_(ctx);
    }
    bool LogicEngine::calcFreshByTimeout_(uint64_t now_ms,
                                          uint64_t last_ms,
                                          uint32_t timeout_ms)
    {
        if (last_ms == 0) return false;
        if (now_ms < last_ms) return false;
        return (now_ms - last_ms) <= static_cast<uint64_t>(timeout_ms);
    }

    double LogicEngine::calcAgeMs_(uint64_t now_ms,
                                   uint64_t last_ms)
    {
        if (last_ms == 0) return 0.0;
        if (now_ms < last_ms) return 0.0;
        return static_cast<double>(now_ms - last_ms);
    }

    const char* LogicEngine::pcuOfflineReasonText_(int code)
    {
        switch (code) {
            case 0: return "None";
            case 1: return "NoData";
            case 2: return "RxTimeout";
            case 3: return "HeartbeatStale";
            default: return "Unknown";
        }
    }

    const char* LogicEngine::bmsOfflineReasonText_(int code)
    {
        switch (code) {
            case 0: return "None";
            case 1: return "NoData";
            case 2: return "RxTimeout";
            default: return "Unknown";
        }
    }

    void LogicEngine::refreshFromSnapshotOnly(const agg::SystemSnapshot& snap,
                                              uint64_t ts_ms,
                                              LogicContext& ctx)
    {
        ctx.last_event_ts = ts_ms;
        ctx.latest_snapshot = snap;

        // 关键原则：
        // latest-snapshot 100ms 刷新线程只负责：
        //   1) 用最新 snapshot 刷新普通设备显示
        //   2) 重建 logic_view
        //   3) 刷 HMI
        //
        // 不负责：
        //   - 重建 PCU runtime state
        //   - 重建 BMS runtime cache
        //   - 运行 PCU/BMS aging
        //
        // 否则会再次把 snapshot 链和 runtime 链缠在一起。

        applyFaultPages_(ctx, ts_ms);
        rebuildLogicView_(ctx);

        if (ctx.hmi) {
            if (ctx.fault_map_loaded) {
                ctx.fault_pages.flushToHmi(*ctx.hmi);
            }

            if (ctx.normal_writer.loaded()) {
                ctx.normal_writer.flushFromModel(ctx.latest_snapshot, ctx.logic_view, *ctx.hmi);
            }
        }

        if (out2json) {
            model_exporter_.exportLatest(ctx.latest_snapshot, ctx.logic_view, ts_ms);
        }
    }

    void LogicEngine::updateOnePcuRuntimeHealth_(PcuOnlineState& st,
                                                 uint64_t now_ms,
                                                 uint32_t rx_timeout_ms,
                                                 uint32_t hb_stale_timeout_ms)
    {
        const bool old_online = st.online;

        st.last_rx_age_ms = calcAgeMs_(now_ms, st.last_rx_ms);
        st.last_hb_change_age_ms = calcAgeMs_(now_ms, st.last_hb_change_ms);

        st.rx_alive = st.seen_once &&
                      calcFreshByTimeout_(now_ms, st.last_rx_ms, rx_timeout_ms);

        st.hb_alive = st.has_last_heartbeat &&
                      calcFreshByTimeout_(now_ms, st.last_hb_change_ms, hb_stale_timeout_ms);

        if (!st.seen_once) {
            st.online = false;
            st.offline_reason_code = 1;
        } else if (!st.rx_alive) {
            st.online = false;
            st.offline_reason_code = 2;
        } else if (!st.hb_alive) {
            st.online = false;
            st.offline_reason_code = 3;
        } else {
            st.online = true;
            st.offline_reason_code = 0;
        }

        st.offline_reason_text = pcuOfflineReasonText_(st.offline_reason_code);

        if (st.online != old_online) {
            st.last_online_change_ms = now_ms;
            if (!st.online) {
                st.last_offline_ms = now_ms;
                st.disconnect_count += 1;
            }
        }
    }

    void LogicEngine::updatePcuRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    {
        updateOnePcuRuntimeHealth_(ctx.pcu0_state,
                                   now_ms,
                                   PCU_RX_TIMEOUT_MS,
                                   PCU_HB_STALE_MS);

        updateOnePcuRuntimeHealth_(ctx.pcu1_state,
                                   now_ms,
                                   PCU_RX_TIMEOUT_MS,
                                   PCU_HB_STALE_MS);
    }

    void LogicEngine::updateBmsRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    {
        for (auto& kv : ctx.bms_cache.items)
        {
            auto& x = kv.second;
            const bool old_online = x.online;

            x.st1_age_ms = calcAgeMs_(now_ms, x.last_st1_ms);
            x.st2_age_ms = calcAgeMs_(now_ms, x.last_st2_ms);
            x.st3_age_ms = calcAgeMs_(now_ms, x.last_st3_ms);
            x.st4_age_ms = calcAgeMs_(now_ms, x.last_st4_ms);
            x.st5_age_ms = calcAgeMs_(now_ms, x.last_st5_ms);
            x.st6_age_ms = calcAgeMs_(now_ms, x.last_st6_ms);
            x.st7_age_ms = calcAgeMs_(now_ms, x.last_st7_ms);

            x.elec_energy_age_ms   = calcAgeMs_(now_ms, x.last_elec_energy_ms);
            x.current_limit_age_ms = calcAgeMs_(now_ms, x.last_current_limit_ms);
            x.tm2b_age_ms          = calcAgeMs_(now_ms, x.last_tm2b_ms);
            x.fire2b_age_ms        = calcAgeMs_(now_ms, x.last_fire2b_ms);
            x.fault1_age_ms        = calcAgeMs_(now_ms, x.last_fault1_ms);
            x.fault2_age_ms        = calcAgeMs_(now_ms, x.last_fault2_ms);

            x.st1_online = calcFreshByTimeout_(now_ms, x.last_st1_ms, BMS_ST1_TIMEOUT_MS);
            x.st2_online = calcFreshByTimeout_(now_ms, x.last_st2_ms, BMS_ST2_TIMEOUT_MS);
            x.st3_online = calcFreshByTimeout_(now_ms, x.last_st3_ms, BMS_ST3_TIMEOUT_MS);
            x.st4_online = calcFreshByTimeout_(now_ms, x.last_st4_ms, BMS_ST4_TIMEOUT_MS);
            x.st5_online = calcFreshByTimeout_(now_ms, x.last_st5_ms, BMS_ST5_TIMEOUT_MS);
            x.st6_online = calcFreshByTimeout_(now_ms, x.last_st6_ms, BMS_ST6_TIMEOUT_MS);
            x.st7_online = calcFreshByTimeout_(now_ms, x.last_st7_ms, BMS_ST7_TIMEOUT_MS);

            x.elec_energy_online   = calcFreshByTimeout_(now_ms, x.last_elec_energy_ms,   BMS_ELEC_ENERGY_TIMEOUT_MS);
            x.current_limit_online = calcFreshByTimeout_(now_ms, x.last_current_limit_ms, BMS_CURRENT_LIMIT_TIMEOUT_MS);
            x.tm2b_online          = calcFreshByTimeout_(now_ms, x.last_tm2b_ms,          BMS_TM2B_TIMEOUT_MS);
            x.fire2b_online        = calcFreshByTimeout_(now_ms, x.last_fire2b_ms,        BMS_FIRE2B_TIMEOUT_MS);
            x.fault1_online        = calcFreshByTimeout_(now_ms, x.last_fault1_ms,        BMS_FAULT1_TIMEOUT_MS);
            x.fault2_online        = calcFreshByTimeout_(now_ms, x.last_fault2_ms,        BMS_FAULT2_TIMEOUT_MS);

            x.runtime_fault_stale = false;
            if ((x.last_st2_ms > 0 && !x.st2_online) ||
                (x.last_current_limit_ms > 0 && !x.current_limit_online) ||
                (x.last_fault1_ms > 0 && !x.fault1_online) ||
                (x.last_fault2_ms > 0 && !x.fault2_online))
            {
                x.runtime_fault_stale = true;
            }

            if (!x.seen_once) {
                x.online = false;
                x.offline_reason_code = 1;
            } else if (!calcFreshByTimeout_(now_ms, x.last_rx_ms, BMS_INSTANCE_TIMEOUT_MS)) {
                x.online = false;
                x.offline_reason_code = 2;
            } else {
                x.online = true;
                x.offline_reason_code = 0;
            }

            x.offline_reason_text = bmsOfflineReasonText_(x.offline_reason_code);

            if (x.online != old_online) {
                x.last_online_change_ms = now_ms;
                if (!x.online) {
                    x.last_offline_ms = now_ms;
                    x.disconnect_count += 1;
                }
            }
        }
    }
    static std::string joinCodesU16_(const std::vector<uint16_t>& codes)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < codes.size(); ++i)
        {
            if (i != 0) oss << ",";
            oss << "0x" << std::hex << std::uppercase << codes[i] << std::dec;
        }
        if (codes.empty())
        {
            oss << "<none>";
        }
        return oss.str();
    }
    /*
     * @brief 应用故障映射
     *
     * @param ctx 逻辑上下文引用
     * @param now_ms 当前时间戳
     * @return void
     */
    void LogicEngine::applyFaultPages_(LogicContext& ctx, uint64_t now_ms)
    {
        // 1) BMS 专用故障映射
        //    第六批开始：
        //    除了继续消费 bms_cache 中的 runtime/F1/F2 真源，
        //    也显式消费 ctx.bms_confirmed_faults 中的 confirmed signals。
        bms_fault_mapper_.applyToFaultPages(ctx.bms_cache, ctx, now_ms, ctx.fault_center);

        // 2) 再叠加 fault_map.jsonl 中的通用 runtime 规则
        LOG_THROTTLE_MS("fault_apply_pages_enter", 1000, LOGINFO, // 20260414
    "[PROVE][FAULT_APPLY] enter map_loaded=%d rules=%zu hmi=%p",
    ctx.fault_map_loaded ? 1 : 0,
    fault_runtime_mapper_.size(),
    (void*)ctx.hmi);

        fault_runtime_mapper_.applyAll(ctx, ctx.fault_center);

        LOG_THROTTLE_MS("fault_apply_pages_after_mapper", 1000, LOGINFO,
            "[PROVE][FAULT_APPLY] after mapper visible=%zu page=%u/%u",
            ctx.fault_center.debugCurrentVisibleCodes().size(),
            (unsigned)ctx.fault_center.debugCurrentPageIndex(),
            (unsigned)ctx.fault_center.debugCurrentTotalPages());
        {
            const auto visible = ctx.fault_center.debugCurrentVisibleCodes();
            const auto rows = ctx.fault_center.debugCurrentRows();
            const uint16_t cur_page = ctx.fault_center.debugCurrentPageIndex();
            const uint16_t cur_total = ctx.fault_center.debugCurrentTotalPages();
// 20260414
            {
                const auto visible = ctx.fault_center.debugCurrentVisibleCodes();
                const uint16_t total_pages = ctx.fault_center.debugCurrentTotalPages();

                for (uint16_t p = 0; p < total_pages; ++p)
                {
                    const size_t begin = static_cast<size_t>(p) * control::fault::FAULTS_PER_PAGE;
                    const size_t end = std::min(begin + static_cast<size_t>(control::fault::FAULTS_PER_PAGE),
                                                visible.size());

                    std::ostringstream oss;
                    for (size_t i = begin; i < end; ++i)
                    {
                        if (i != begin) oss << ",";
                        oss << "0x" << std::hex << std::uppercase << visible[i] << std::dec;
                    }
                    if (begin >= end)
                    {
                        oss << "<empty>";
                    }

                    LOGINFO("[RESULT][FAULT_PAGE_ALL] page=%u/%u codes=%s",
                            (unsigned)(p + 1),
                            (unsigned)total_pages,
                            oss.str().c_str());
                }
            }


        }


        fault_runtime_mapper_.applyAll(ctx, ctx.fault_center);

    }


    // const char* LogicEngine::pcuOfflineReasonText_(uint32_t code)
    // {
    //     switch (code) {
    //     case 0: return "Online";
    //     case 1: return "NoData";
    //     case 2: return "RxTimeout";
    //     case 3: return "HeartbeatStale";
    //     default: return "Unknown";
    //     }
    // }
    // void LogicEngine::updatePcuRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    // {
    //     constexpr uint64_t PCU_RX_TIMEOUT_MS = 1500;
    //     constexpr uint64_t PCU_HB_STALE_MS   = 3000;
    //
    //     auto update_one = [now_ms](PcuOnlineState& st) {
    //         const bool old_online = st.online;
    //
    //         if (!st.seen_once) {
    //             st.rx_alive = false;
    //             st.hb_alive = false;
    //             st.online = false;
    //             st.offline_reason_code = 1;
    //         } else {
    //             st.rx_alive =
    //                 (st.last_rx_ms > 0) &&
    //                 (now_ms >= st.last_rx_ms) &&
    //                 ((now_ms - st.last_rx_ms) <= PCU_RX_TIMEOUT_MS);
    //
    //             st.hb_alive =
    //                 st.has_last_heartbeat &&
    //                 (st.last_hb_change_ms > 0) &&
    //                 (now_ms >= st.last_hb_change_ms) &&
    //                 ((now_ms - st.last_hb_change_ms) <= PCU_HB_STALE_MS);
    //
    //             st.online = st.rx_alive && st.hb_alive;
    //
    //             if (st.online) {
    //                 st.offline_reason_code = 0;
    //             } else if (!st.rx_alive) {
    //                 st.offline_reason_code = 2;
    //             } else if (!st.hb_alive) {
    //                 st.offline_reason_code = 3;
    //             } else {
    //                 st.offline_reason_code = 1;
    //             }
    //         }
    //
    //         if (old_online != st.online) {
    //             st.last_online_change_ms = now_ms;
    //         }
    //     };
    //
    //     update_one(ctx.pcu0_state);
    //     update_one(ctx.pcu1_state);
    // }
    // void LogicEngine::updateBmsRuntimeHealth_(LogicContext& ctx, uint64_t now_ms)
    // {
    //     constexpr uint64_t BMS_INSTANCE_TIMEOUT_MS = 3000;
    //
    //     for (auto& kv : ctx.bms_cache.items) {
    //         auto& x = kv.second;
    //
    //         const bool old_online = x.online;
    //
    //         if (x.last_rx_ms == 0 || now_ms < x.last_rx_ms) {
    //             x.online = false;
    //         } else {
    //             x.online = ((now_ms - x.last_rx_ms) <= BMS_INSTANCE_TIMEOUT_MS);
    //         }
    //
    //         (void)old_online;
    //     }
    // }

} // namespace control