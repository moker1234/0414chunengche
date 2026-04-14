//
// Created by lxy on 2026/1/20.
// Final version: support CR / LF / CRLF, anti-sticky & anti-overflow
//
/*
 * ASCII 协议帧组装实现
 */
/* 解释整个文件的作用
 * 该文件实现了ASCII协议帧组装的定义，包括构造函数、字节接收函数和帧获取函数。
 * 构造函数用于初始化帧组装器，字节接收函数用于将接收的字节添加到缓冲区中，帧获取函数用于从缓冲区中提取完整的帧。
 */
#include "ascii_frame_assembler.h"
#include "logger.h"

#include <algorithm>

using namespace parser;

namespace {
    // 单帧最大长度（防止异常设备导致内存无限增长）
    constexpr size_t MAX_ASCII_FRAME_LEN = 256;
}

void AsciiFrameAssembler::onBytes(int link_index,
                                  const std::vector<uint8_t>& bytes) {
    auto& buf = buffers_[link_index];
    buf.insert(buf.end(), bytes.begin(), bytes.end());

    // 防御性处理：缓冲区异常增长（几乎不会发生，但工程上必须有）
    if (buf.size() > MAX_ASCII_FRAME_LEN * 2) {
        LOGD("[ASCII] buffer overflow, idx=%d size=%zu, drop buffer",
             link_index, buf.size());
        buf.clear();
        return;
    }

#if 0   // === 打开这个块可查看拼帧细节（联调用） ===
    bool has_cr = std::find(buf.begin(), buf.end(), 0x0D) != buf.end();
    bool has_lf = std::find(buf.begin(), buf.end(), 0x0A) != buf.end();
    LOGD("[ASCII][BUF] idx=%d len=%zu hasCR=%d hasLF=%d last=%02X",
         link_index,
         buf.size(),
         has_cr ? 1 : 0,
         has_lf ? 1 : 0,
         buf.empty() ? 0 : buf.back());
#endif
}

bool AsciiFrameAssembler::tryGetFrame(int link_index,
                                      std::vector<uint8_t>& out) {
    auto& buf = buffers_[link_index];
    if (buf.empty()) return false;

    // 1. 优先查找 CR
    auto it = std::find(buf.begin(), buf.end(), 0x0D);

    // 2. 如果没有 CR，再查找 LF
    if (it == buf.end()) {
        it = std::find(buf.begin(), buf.end(), 0x0A);
        if (it == buf.end()) {
            // 还没等到结束符
            return false;
        }
    }

    const size_t frame_len = std::distance(buf.begin(), it);

    // 防御：异常帧长度
    if (frame_len == 0 || frame_len > MAX_ASCII_FRAME_LEN) {
        LOGD("[ASCII] invalid frame len=%zu idx=%d, drop one byte",
             frame_len, link_index);
        buf.erase(buf.begin());   // 丢 1 字节，继续自恢复
        return false;
    }

    // 3. 输出帧（不含 CR / LF）
    out.assign(buf.begin(), it);

    // 4. 擦除结束符
    size_t erase_len = frame_len + 1; // 吃掉 CR 或 LF

    // 如果是 CRLF，一起吃掉 LF
    if (buf[frame_len] == 0x0D &&
        buf.size() > frame_len + 1 &&
        buf[frame_len + 1] == 0x0A) {
        erase_len++;
    }

    buf.erase(buf.begin(), buf.begin() + erase_len);

    return true;
}
