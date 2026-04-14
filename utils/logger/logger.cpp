#include "logger.h"

#include <cstdio>
#include <ctime>
#include <mutex>

/*
 * ===================== 线程安全 =====================
 */
static std::mutex g_log_mutex;

/*
 * ===================== 日志级别字符串 =====================
 */
static const char* levelToStr(LogLevel lv) {
    switch (lv) {
    case LogLevel::Error: return "E";
    case LogLevel::Warn:  return "W";
    case LogLevel::Info:  return "I";
    case LogLevel::Debug: return "D";
    case LogLevel::Trace: return "T";
    default:              return "?";
    }
}

static const char* tagToStr(LogTag tag) {
    switch (tag) {
    case LogTag::GEN:    return "GEN";
    case LogTag::SYS:    return "SYS";
    case LogTag::COMM:   return "COMM";
    case LogTag::HEALTH: return "HEALTH";
    default:             return "UNK";
    }
}

/*
 * ===================== 时间戳（毫秒） =====================
 */
static void printTimestamp(FILE* fp) {
    struct timespec ts {};
    clock_gettime(CLOCK_REALTIME, &ts);

    std::tm tm {};
    localtime_r(&ts.tv_sec, &tm);

    std::fprintf(fp,
        "%02d:%02d:%02d.%03ld",
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        ts.tv_nsec / 1000000
    );
}

// monotonic ms（用于 throttle/timeout）
uint64_t log_now_ms() {
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

/*
 * ===================== 是否紧凑输出（不带 file:line:func） =====================
 */
static bool is_compact(LogTag tag, LogLevel level) {
    // COMM：默认紧凑；定位期可打开 LOG_COMM_VERBOSE
    if (tag == LogTag::COMM) {
        if (LOG_COMM_VERBOSE) return false;

        // COMM 的 WARN/ERR 默认不紧凑（异常要定位）
        if (level == LogLevel::Warn || level == LogLevel::Error) return false;
        return true;
    }

    // HEALTH：默认不紧凑（便于知道谁判的状态）
    if (tag == LogTag::HEALTH) return false;

    // SYS/GEN：不紧凑
    return false;
}

/*
 * ===================== 核心日志函数（带 tag）=====================
 */
void log_print_tag(LogLevel level,
                   LogTag tag,
                   const char* file,
                   int line,
                   const char* func,
                   const char* fmt, ...) {

    std::lock_guard<std::mutex> lock(g_log_mutex);

    FILE* out = stderr;

    const bool compact = is_compact(tag, level);

    // Header
    std::fprintf(out, "[");
    printTimestamp(out);
    std::fprintf(out, "][%s][%s]",
                 levelToStr(level),
                 tagToStr(tag));

    if (!compact) {
        std::fprintf(out, "[%s:%d][%s] ",
                     log_basename(file),
                     line,
                     func);
    } else {
        std::fprintf(out, " ");
    }

    // Body
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(out, fmt, ap);
    va_end(ap);

    std::fprintf(out, "\n");
    std::fflush(out);
}

/*
 * ===================== 兼容旧接口（无 tag）=====================
 */
void log_print(LogLevel level,
               const char* file,
               int line,
               const char* func,
               const char* fmt, ...) {

    std::lock_guard<std::mutex> lock(g_log_mutex);

    FILE* out = stderr;

    std::fprintf(out, "[");
    printTimestamp(out);
    std::fprintf(out, "][%s][%s:%d][%s] ",
                 levelToStr(level),
                 log_basename(file),
                 line,
                 func);

    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(out, fmt, ap);
    va_end(ap);

    std::fprintf(out, "\n");
    std::fflush(out);
}

/*
 * ===================== Hex 打印（一行，默认截断）=====================
 */
void log_hex_print(LogLevel level,
                   LogTag tag,
                   const char* file,
                   int line,
                   const char* func,
                   const char* prefix,
                   const uint8_t* data,
                   size_t len) {

    std::lock_guard<std::mutex> lock(g_log_mutex);

    FILE* out = stderr;
    const bool compact = is_compact(tag, level);

    std::fprintf(out, "[");
    printTimestamp(out);
    std::fprintf(out, "][%s][%s]",
                 levelToStr(level),
                 tagToStr(tag));

    if (!compact) {
        std::fprintf(out, "[%s:%d][%s] ",
                     log_basename(file),
                     line,
                     func);
    } else {
        std::fprintf(out, " ");
    }

    std::fprintf(out, "%s ", (prefix ? prefix : "HEX"));

    // 截断，避免刷屏
    constexpr size_t MAX_DUMP = 64;
    const size_t n = (len > MAX_DUMP) ? MAX_DUMP : len;

    for (size_t i = 0; i < n; ++i) {
        std::fprintf(out, "%02X", data[i]);
        if (i + 1 < n) std::fprintf(out, " ");
    }
    if (len > MAX_DUMP) {
        std::fprintf(out, " ...(+%zu)", len - MAX_DUMP);
    }

    std::fprintf(out, "\n");
    std::fflush(out);
}
