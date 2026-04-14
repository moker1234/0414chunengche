//
// Created by forlinx on 2025/12/17.
//

#ifndef ENERGYSTORAGE_LOGGER_H
#define ENERGYSTORAGE_LOGGER_H

#pragma once
#include <cstdarg>
#include <cstdint>

#define LOGI LOGINFO
#define LOGD LOGDEBUG
#define LOGT LOGTRACE

/*
 * ===================== 日志级别 =====================
 *
 * 数值越小，级别越高（越重要）
 */
enum class LogLevel : int {
    Error = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
    Trace = 4,
};

/*
 * ===================== Tag（分类）=====================
 * GEN：历史默认日志
 * SYS：线程/状态机/生命周期
 * COMM：通信报文（高频、默认紧凑）
 * HEALTH：设备健康（低频、默认带定位）
 */
enum class LogTag : int {
    GEN = 0,
    SYS,
    COMM,
    HEALTH,
};

/*
 * ===================== 编译期日志级别 =====================
 *
 * 在 CMakeLists.txt 中设置：
 *   add_compile_definitions(LOG_LEVEL=LogLevel::Debug)
 */
#ifndef LOG_LEVEL
#define LOG_LEVEL LogLevel::Debug
#endif

/*
 * ===================== COMM 是否显示 file:line =====================
 * 0：COMM 紧凑输出（默认）
 * 1：COMM 也输出 file:line:func（定位期用）
 */
#ifndef LOG_COMM_VERBOSE
#define LOG_COMM_VERBOSE 0
#endif

/*
 * ===================== 内部函数（不要直接调用） =====================
 */
void log_print(LogLevel level,
               const char* file,
               int line,
               const char* func,
               const char* fmt, ...);

void log_print_tag(LogLevel level,
                   LogTag tag,
                   const char* file,
                   int line,
                   const char* func,
                   const char* fmt, ...);

// 用于限频/时间戳
uint64_t log_now_ms();

// 用于 hex 打印（COMM RX/TX）
void log_hex_print(LogLevel level,
                   LogTag tag,
                   const char* file,
                   int line,
                   const char* func,
                   const char* prefix,
                   const uint8_t* data,
                   std::size_t len);

/*
 * ===================== 文件名裁剪 =====================
 *  "/home/user/a/b.cpp" -> "b.cpp"
 */
constexpr const char* log_basename(const char* path) {
    const char* p = path;
    for (const char* s = path; *s; ++s) {
        if (*s == '/' || *s == '\\') {
            p = s + 1;
        }
    }
    return p;
}

/*
 * ===================== 基础日志宏（历史兼容） =====================
 * 注意：低于 LOG_LEVEL 的日志不会进入函数
 */

#define LOGERR(fmt, ...) \
    do { \
        if constexpr (LogLevel::Error <= LOG_LEVEL) \
            log_print(LogLevel::Error, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOGWARN(fmt, ...) \
    do { \
        if constexpr (LogLevel::Warn <= LOG_LEVEL) \
            log_print(LogLevel::Warn, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOGINFO(fmt, ...) \
    do { \
        if constexpr (LogLevel::Info <= LOG_LEVEL) \
            log_print(LogLevel::Info, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOGDEBUG(fmt, ...) \
    do { \
        if constexpr (LogLevel::Debug <= LOG_LEVEL) \
            log_print(LogLevel::Debug, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOGTRACE(fmt, ...) \
    do { \
        if constexpr (LogLevel::Trace <= LOG_LEVEL) \
            log_print(LogLevel::Trace, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

/*
 * ===================== 分类日志宏（推荐使用） =====================
 */

// SYS
#define LOG_SYS_I(fmt, ...) \
    do { \
        if constexpr (LogLevel::Info <= LOG_LEVEL) \
            log_print_tag(LogLevel::Info, LogTag::SYS, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOG_SYS_W(fmt, ...) \
    do { \
        if constexpr (LogLevel::Warn <= LOG_LEVEL) \
            log_print_tag(LogLevel::Warn, LogTag::SYS, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)
#define LOG_SYS_D(fmt, ...) \
    do { \
        if constexpr (LogLevel::Debug <= LOG_LEVEL) \
            log_print_tag(LogLevel::Debug, LogTag::SYS, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

// HEALTH（建议默认带定位）
#define LOG_HEALTH_I(fmt, ...) \
    do { \
        if constexpr (LogLevel::Info <= LOG_LEVEL) \
            log_print_tag(LogLevel::Info, LogTag::HEALTH, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOG_HEALTH_W(fmt, ...) \
    do { \
        if constexpr (LogLevel::Warn <= LOG_LEVEL) \
            log_print_tag(LogLevel::Warn, LogTag::HEALTH, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

// COMM（正常 TX/RX 用 Debug；异常用 Warn）
#define LOG_COMM_D(fmt, ...) \
    do { \
        if constexpr (LogLevel::Debug <= LOG_LEVEL) \
            log_print_tag(LogLevel::Debug, LogTag::COMM, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOG_COMM_T(fmt, ...) \
    do { \
        if constexpr (LogLevel::Trace <= LOG_LEVEL) \
            log_print_tag(LogLevel::Trace, LogTag::COMM, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

#define LOG_COMM_W(fmt, ...) \
    do { \
        if constexpr (LogLevel::Warn <= LOG_LEVEL) \
            log_print_tag(LogLevel::Warn, LogTag::COMM, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__); \
    } while (0)

// COMM hex（默认 Debug）
#define LOG_COMM_HEX(prefix, data, len) \
    do { \
        if constexpr (LogLevel::Debug <= LOG_LEVEL) \
            log_hex_print(LogLevel::Debug, LogTag::COMM, __FILE__, __LINE__, __func__, prefix, \
                          reinterpret_cast<const uint8_t*>(data), (size_t)(len)); \
    } while (0)

/*
 * ===================== 限频日志（每个调用点一个静态计时器） =====================
 * key 参数保留兼容（不参与计算；每个调用点本身已经独立静态存储）
 */
#define LOG_THROTTLE_MS(key, interval_ms, LOGMACRO, fmt, ...) \
    do { \
        (void)(key); \
        static uint64_t _last_ms = 0; \
        const uint64_t _now_ms = log_now_ms(); \
        if (_now_ms - _last_ms >= (uint64_t)(interval_ms)) { \
            _last_ms = _now_ms; \
            LOGMACRO(fmt, ##__VA_ARGS__); \
        } \
    } while (0)

#endif //ENERGYSTORAGE_LOGGER_H
