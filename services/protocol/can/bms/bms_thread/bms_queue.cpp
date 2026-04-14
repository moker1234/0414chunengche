//
// Created by lxy on 2026/2/14.
//

#include "bms_queue.h"

namespace proto::bms {

    void BmsQueue::push(const BmsFrame& f)
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_[f.key()] = f;  // overwrite -> latest wins
        }
        cv_.notify_one();
    }

    std::vector<BmsFrame> BmsQueue::popAll()
    {
        std::vector<BmsFrame> out;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            out.reserve(latest_.size());
            for (auto& kv : latest_) out.push_back(kv.second);
            latest_.clear();
        }
        return out;
    }

    bool BmsQueue::waitForData(uint32_t timeout_ms)
    {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!latest_.empty()) return true;

        if (timeout_ms == 0) return false;

        return cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] {
            return !latest_.empty();
        });
    }

    size_t BmsQueue::size() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return latest_.size();
    }

    void BmsQueue::clear()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_.clear();
    }

} // namespace proto::bms
