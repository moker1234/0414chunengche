//
// Created by lxy on 2026/2/8.
//

// services/protocol/j1939/opensae_port.cpp
#include "opensae_port.h"

#include <chrono>
#include <mutex>
#include <unordered_map>

namespace j1939 {

    static std::mutex g_mtx;
    static std::unordered_map<int, CanTxFunc> g_tx;

    void bindCanTx(int can_index, CanTxFunc tx) {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_tx[can_index] = std::move(tx);
    }

    void unbindCanTx(int can_index) {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_tx.erase(can_index);
    }

    uint64_t nowMs() {
        using namespace std::chrono;
        return (uint64_t)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }

    bool sendCanFrame(int can_index, const can_frame& fr) {
        CanTxFunc fn;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            auto it = g_tx.find(can_index);
            if (it == g_tx.end()) return false;
            fn = it->second;
        }
        if (!fn) return false;
        fn(fr);
        return true;
    }

    bool sendCanFrame(int can_index, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
        can_frame fr{};
        fr.can_id  = can_id;
        fr.can_dlc = dlc > 8 ? 8 : dlc;
        if (data && fr.can_dlc > 0) {
            for (uint8_t i = 0; i < fr.can_dlc; ++i) fr.data[i] = data[i];
        }
        return sendCanFrame(can_index, fr);
    }

    bool loadBlob(const std::string& /*key*/, std::vector<uint8_t>& /*out*/) {
        // 第一阶段：不做持久化（后续可改为文件/kv/flash）
        return false;
    }

    bool saveBlob(const std::string& /*key*/, const uint8_t* /*data*/, size_t /*len*/) {
        // 第一阶段：不做持久化
        return false;
    }

} // namespace j1939
