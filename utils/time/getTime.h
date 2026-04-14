//
// Created by lxy on 2026/2/7.
//

#ifndef ENERGYSTORAGE_GETTIME_H
#define ENERGYSTORAGE_GETTIME_H
#include <chrono>
#include <cstdint>

// 获取当前时间（ms）
// 注意：steady_clock 是单调递增的，而 system_clock 是 wall time
inline uint64_t nowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

// 获取当前时间（ms）
// 注意：steady_clock 是单调递增的，而 system_clock 是 wall time
inline uint64_t monotonicNowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 获取当前时间（ms）
// 注意：system_clock 是 wall time，而 steady_clock 是单调递增的
inline uint64_t unixNowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// 获取当前时间（s）
// 注意：system_clock 是 wall time，而 steady_clock 是单调递增的
inline uint32_t unixNowSec32()
{
    return static_cast<uint32_t>(unixNowMs() / 1000ULL);
}
#endif //ENERGYSTORAGE_GETTIME_H