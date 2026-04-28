#include "logger.h"
#include "logic_engine.h"

namespace control {

    void LogicEngine::refreshFaultPagesOnly(uint64_t ts_ms, LogicContext& ctx)
    {
        ctx.last_event_ts = ts_ms;

        // 1) 先刷新 BMS confirmed faults
        bms_fault_evaluator_.evaluateAll(ctx.bms_cache, ctx, ts_ms);

        // 2) 再刷新通用 / VCU confirmed faults
        fault_logic_evaluator_.evaluateAll(ctx, ts_ms);

        // 3) 统一进入 fault page 映射链
        applyFaultPages_(ctx, ts_ms);

        // 4) 历史故障缓存刷新
        //
        // 设计原则：
        // - SQLite 是历史真源
        // - FaultCenter 只负责在“历史变化”时打 dirty/version
        // - 这里利用独立 fault refresh 线程（100ms）来驱动缓存刷新
        //
        // 刷新时机：
        // A. 历史有新变化（出现/清除/恢复） -> 刷新 meta + 热点 + 当前窗口
        // B. 当前页不在窗口缓存内         -> 补拉当前页窗口
        if (ctx.fault_history_cache) {
            uint64_t hist_ver = 0;
            const bool history_changed =
                ctx.fault_center.consumeHistoryDirty(&hist_ver);

            if (history_changed) {
                const bool ok_meta = ctx.fault_history_cache->refreshMeta();
                const bool ok_hot  = ok_meta && ctx.fault_history_cache->refreshHotCache();
                const bool ok_win  = ok_hot  && ctx.fault_history_cache->refreshWindow(ctx.history_page_no);

                if (ok_win) {
                    ctx.history_cache_version = hist_ver;
                } else {
                    LOG_THROTTLE_MS("fault_history_cache_refresh_fail", 1000, LOGINFO,
                                    "[FAULT][HIS_CACHE] refresh fail ver=%llu page=%u",
                                    (unsigned long long)hist_ver,
                                    (unsigned)ctx.history_page_no);
                }

                LOG_THROTTLE_MS("fault_history_cache_refresh", 1000, LOGINFO,
                                "[FAULT][HIS_CACHE] dirty=%d ver=%llu total_rows=%u total_pages=%u window=%u~%u page=%u",
                                history_changed ? 1 : 0,
                                (unsigned long long)hist_ver,
                                (unsigned)ctx.fault_history_cache->totalRows(),
                                (unsigned)ctx.fault_history_cache->totalPages(),
                                (unsigned)ctx.fault_history_cache->windowStartPage(),
                                (unsigned)ctx.fault_history_cache->windowEndPage(),
                                (unsigned)ctx.history_page_no);
            }
            else {
                // 没有历史变化时，不重查 meta/hot；
                // 只有当前页不在窗口里，才按需补拉窗口。
                if (!ctx.fault_history_cache->hasPageInWindow(ctx.history_page_no)) {
                    const bool ok_meta = ctx.fault_history_cache->refreshMeta();
                    const bool ok_win =
                        ok_meta && ctx.fault_history_cache->refreshWindow(ctx.history_page_no);

                    LOG_THROTTLE_MS("fault_history_cache_window", 1000, LOGINFO,
                                    "[FAULT][HIS_CACHE] window_reload ok=%d total_rows=%u total_pages=%u window=%u~%u page=%u",
                                    ok_win ? 1 : 0,
                                    (unsigned)ctx.fault_history_cache->totalRows(),
                                    (unsigned)ctx.fault_history_cache->totalPages(),
                                    (unsigned)ctx.fault_history_cache->windowStartPage(),
                                    (unsigned)ctx.fault_history_cache->windowEndPage(),
                                    (unsigned)ctx.history_page_no);
                }
            }
        }

        // 5) 故障页输出统一收口
        if (ctx.hmi && ctx.fault_map_loaded) {
            applyFaultHmi_(ctx);
        }

        // LOG_THROTTLE_MS("fault_refresh_only", 1000, LOGINFO,
        // "[PROVE][FAULT_NEW_CHAIN] refreshFaultPagesOnly ts=%llu",
        // (unsigned long long)ts_ms);
    }

} // namespace control