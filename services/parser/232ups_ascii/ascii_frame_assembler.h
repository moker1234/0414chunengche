//
// Created by lxy on 2026/1/20.
// Final version: support CR / LF / CRLF, anti-sticky & anti-overflow

/*
 * ASCII 协议帧组装定义
 */
/* 解释整个文件的作用
 * 该文件实现了ASCII协议帧组装的定义，包括构造函数、字节接收函数和帧获取函数。
 * 构造函数用于初始化帧组装器，字节接收函数用于将接收的字节添加到缓冲区中，帧获取函数用于从缓冲区中提取完整的帧。
 */
#ifndef ENERGYSTORAGE_ASCII_FRAME_ASSEMBLER_H
#define ENERGYSTORAGE_ASCII_FRAME_ASSEMBLER_H


#pragma once
#include <map>
#include <vector>
#include <cstdint>

namespace parser {

    /**
     * ASCII Frame Assembler
     * - 以 0x0D (CR) 作为帧结束
     * - 不做校验
     */
    class AsciiFrameAssembler {
    public:
        void onBytes(int link_index, const std::vector<uint8_t>& bytes);

        // out 包含完整一帧（含 CR 或不含 CR 都可以，这里选择“不含 CR”更好解析）
        bool tryGetFrame(int link_index, std::vector<uint8_t>& out);

    private:
        std::map<int, std::vector<uint8_t>> buffers_;
    };

} // namespace parser


#endif //ENERGYSTORAGE_ASCII_FRAME_ASSEMBLER_H
