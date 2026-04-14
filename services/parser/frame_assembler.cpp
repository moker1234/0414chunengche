//
// Created by lxy on 2026/1/19.
//

#include "frame_assembler.h"

using namespace parser;

/*
 * 目标支持：
 *  - 03 正常响应： [addr][03][byte_cnt][data...][crc_lo][crc_hi]
 *  - 06 正常响应： [addr][06][reg_hi][reg_lo][val_hi][val_lo][crc_lo][crc_hi]
 *  - 10 正常响应： [addr][10][start_hi][start_lo][qty_hi][qty_lo][crc_lo][crc_hi]
 *  - 异常响应：    [addr][func|0x80][err][crc_lo][crc_hi]
 *
 * 注：这里仅做“按长度切帧”，CRC 校验交给各 Protocol::parse()。
 */

void FrameAssembler::onBytes(int link_index,
                             const std::vector<uint8_t>& bytes) {
    auto& buf = buffers_[link_index];
    buf.insert(buf.end(), bytes.begin(), bytes.end());
}

static inline bool isExceptionFunc(uint8_t func) {
    return (func & 0x80) != 0;
}

bool FrameAssembler::tryGetFrame(int link_index,
                                 std::vector<uint8_t>& out) {
    auto& buf = buffers_[link_index];
    if (buf.size() < 5) return false; // 最短异常帧 5 字节

    // 基本同步：如果第2字节不像功能码，丢1字节继续
    // （这里不做复杂同步，仅做轻量容错）
    const uint8_t func = buf[1];

    // 1) 异常帧：固定 5 字节
    if (isExceptionFunc(func)) {
        if (buf.size() < 5) return false;
        out.assign(buf.begin(), buf.begin() + 5);
        buf.erase(buf.begin(), buf.begin() + 5);
        return true;
    }

    // 2) 03：按 byte_cnt 切
    if (func == 0x03) {
        if (buf.size() < 5) return false;

        uint8_t byte_cnt = buf[2];

        // ★ 过滤回显/噪声：byte_cnt 不合理则丢 1 字节找新帧头
        if (byte_cnt == 0 || byte_cnt > 0x7D) {
            buf.erase(buf.begin());
            return false;
        }

        size_t expect_len = 3u + byte_cnt + 2u;
        if (buf.size() < expect_len) return false;

        out.assign(buf.begin(), buf.begin() + expect_len);
        buf.erase(buf.begin(), buf.begin() + expect_len);
        return true;
    }

    // 3) 06：固定 8 字节
    if (func == 0x06) {
        if (buf.size() < 8) return false;
        out.assign(buf.begin(), buf.begin() + 8);
        buf.erase(buf.begin(), buf.begin() + 8);
        return true;
    }

    // 4) 10：固定 8 字节（写多个寄存器响应）
    if (func == 0x10) {
        if (buf.size() < 8) return false;
        out.assign(buf.begin(), buf.begin() + 8);
        buf.erase(buf.begin(), buf.begin() + 8);
        return true;
    }

    // 未识别：丢 1 字节
    buf.erase(buf.begin());
    return false;
}
