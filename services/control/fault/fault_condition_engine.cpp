//
// Created by lxy on 2026/4/12.
//


#include "fault_condition_engine.h"
#include "../../utils/logger/logger.h"

namespace control::fault {

bool FaultConditionEngine::updateCondition(const std::string& key,
                                           bool raw_now,
                                           uint64_t now_ms,
                                           uint32_t assert_delay_ms,
                                           uint32_t clear_delay_ms)
{
    auto& st = states_[key];
    st.configure(assert_delay_ms, clear_delay_ms);

    const bool changed = st.update(raw_now, now_ms);
    if (changed) {
        LOGINFO("[FAULT][COND] key=%s confirmed=%d raw=%d assert_ms=%u clear_ms=%u ts=%llu",  // 故障结构十批输出
                key.c_str(),
                st.confirmed_on ? 1 : 0,
                raw_now ? 1 : 0,
                (unsigned)assert_delay_ms,
                (unsigned)clear_delay_ms,
                (unsigned long long)now_ms);
    }

    return changed;
}

bool FaultConditionEngine::updateConditionWithClear(const std::string& key,
                                                    bool assert_now,
                                                    bool clear_now,
                                                    uint64_t now_ms,
                                                    uint32_t assert_delay_ms,
                                                    uint32_t clear_delay_ms)
{
    auto& st = states_[key];
    st.configure(assert_delay_ms, clear_delay_ms);

    const bool changed = st.updateWithClear(assert_now, clear_now, now_ms);
    if (changed) {
        LOGINFO("[FAULT][COND] key=%s confirmed=%d assert_now=%d clear_now=%d assert_ms=%u clear_ms=%u ts=%llu",  // 故障结构十批输出
                key.c_str(),
                st.confirmed_on ? 1 : 0,
                assert_now ? 1 : 0,
                clear_now ? 1 : 0,
                (unsigned)assert_delay_ms,
                (unsigned)clear_delay_ms,
                (unsigned long long)now_ms);
    }

    return changed;
}

    bool FaultConditionEngine::getConfirmed(const std::string& key) const
    {
        auto it = states_.find(key);
        if (it == states_.end()) return false;
        return it->second.confirmed_on;
    }

    const FaultConditionState* FaultConditionEngine::getState(const std::string& key) const
    {
        auto it = states_.find(key);
        if (it == states_.end()) return nullptr;
        return &it->second;
    }

    FaultConditionState* FaultConditionEngine::getState(const std::string& key)
    {
        auto it = states_.find(key);
        if (it == states_.end()) return nullptr;
        return &it->second;
    }

    FaultConditionState& FaultConditionEngine::ensureState(const std::string& key)
    {
        return states_[key];
    }

    void FaultConditionEngine::resetCondition(const std::string& key)
    {
        auto it = states_.find(key);
        if (it == states_.end()) return;
        it->second.reset();
    }

    void FaultConditionEngine::clear()
    {
        states_.clear();
    }

    std::vector<std::string> FaultConditionEngine::keys() const
    {
        std::vector<std::string> out;
        out.reserve(states_.size());

        for (const auto& kv : states_) {
            out.push_back(kv.first);
        }

        return out;
    }

} // namespace control::fault