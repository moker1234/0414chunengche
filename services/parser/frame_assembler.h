// services/parser/frame_assembler.h
//
// Created by lxy on 2026/1/19.
//

#ifndef ENERGYSTORAGE_FRAME_ASSEMBLER_H
#define ENERGYSTORAGE_FRAME_ASSEMBLER_H

#pragma once
#include <map>
#include <vector>
#include <cstdint>

namespace parser {

    /**
     * RS485 帧拼接器（Modbus RTU）
     * 支持：
     *  - 0x03 读保持寄存器： [addr][03][byte_cnt][data...][crc_lo][crc_hi]
     *  - 0x06 写单寄存器：   固定 8 字节
     *  - 0x10 写多寄存器：   正常响应固定 8 字节
     *  - 异常响应：          固定 5 字节 [addr][func|0x80][exc][crc_lo][crc_hi]
     *
     * 注意：这里只做“长度切帧”，不做 CRC 校验（校验在 Protocol::parse 内做）
     */
    class FrameAssembler {
    public:
        void onBytes(int link_index,
                     const std::vector<uint8_t>& bytes);

        bool tryGetFrame(int link_index,
                         std::vector<uint8_t>& out);

    private:
        std::map<int, std::vector<uint8_t>> buffers_;
    };

} // namespace parser

#endif // ENERGYSTORAGE_FRAME_ASSEMBLER_H
