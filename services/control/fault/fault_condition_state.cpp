//
// Created by lxy on 2026/4/12.
//


#include "fault_condition_state.h"

namespace control::fault {

void FaultConditionState::configure(uint32_t assert_delay, uint32_t clear_delay)
{
    assert_delay_ms = assert_delay;
    clear_delay_ms = clear_delay;
}

void FaultConditionState::reset()
{
    raw_on = false;
    raw_clear = false;
    confirmed_on = false;
    raw_on_since_ms = 0;
    raw_clear_since_ms = 0;
    last_update_ms = 0;
    last_change_ms = 0;
    changed_last_update = false;
    last_change_to_on = false;
}

bool FaultConditionState::update(bool raw_now, uint64_t now_ms)
{
    // 兼容第一批语义：
    // 出现条件 = raw_now
    // 清除条件 = !raw_now
    return updateWithClear(raw_now, !raw_now, now_ms);
}

bool FaultConditionState::updateWithClear(bool assert_now,
                                          bool clear_now,
                                          uint64_t now_ms)
{
    changed_last_update = false;
    last_update_ms = now_ms;

    // ------------------------------------------------------------
    // 当前未确认故障：只看“出现条件”
    // ------------------------------------------------------------
    if (!confirmed_on) {
        raw_clear = false;
        raw_clear_since_ms = 0;

        if (assert_now) {
            if (!raw_on) {
                raw_on = true;
                raw_on_since_ms = now_ms;
            }

            const uint64_t held_ms =
                (now_ms >= raw_on_since_ms) ? (now_ms - raw_on_since_ms) : 0ULL;

            if (assert_delay_ms == 0 || held_ms >= static_cast<uint64_t>(assert_delay_ms)) {
                confirmed_on = true;
                last_change_ms = now_ms;
                changed_last_update = true;
                last_change_to_on = true;

                // 切换后把 clear 侧计时清空，避免污染
                raw_clear = false;
                raw_clear_since_ms = 0;
            }
        } else {
            raw_on = false;
            raw_on_since_ms = 0;
        }

        return changed_last_update;
    }

    // ------------------------------------------------------------
    // 当前已确认故障：只看“清除条件”
    // ------------------------------------------------------------
    raw_on = assert_now;
    if (!assert_now) {
        raw_on_since_ms = 0;
    }

    if (clear_now) {
        if (!raw_clear) {
            raw_clear = true;
            raw_clear_since_ms = now_ms;
        }

        const uint64_t held_ms =
            (now_ms >= raw_clear_since_ms) ? (now_ms - raw_clear_since_ms) : 0ULL;

        if (clear_delay_ms == 0 || held_ms >= static_cast<uint64_t>(clear_delay_ms)) {
            confirmed_on = false;
            last_change_ms = now_ms;
            changed_last_update = true;
            last_change_to_on = false;

            raw_on = false;
            raw_on_since_ms = 0;
            raw_clear = false;
            raw_clear_since_ms = 0;
        }
    } else {
        raw_clear = false;
        raw_clear_since_ms = 0;
    }

    return changed_last_update;
}

} // namespace control::fault
