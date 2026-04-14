//
// Created by lxy on 2026/1/16.
//
/*
 * @brief 十六进制转储工具
 * @details 将二进制数据转换为十六进制字符串，用于日志调试。
 */
/* 解释整个文件的作用
 * 该文件实现了十六进制转储工具的定义，包括函数 hexDump。
 * 函数 hexDump 用于将二进制数据转换为十六进制字符串，每个字节用空格分隔。
 */
#ifndef ENERGYSTORAGE_HEX_DUMP_H
#define ENERGYSTORAGE_HEX_DUMP_H
#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>

/*
 * ===================== hexDump =====================
 *
 * - 仅用于日志（Trace / Debug）
 * - 非线程安全问题：thread_local
 * - 每次调用都会覆盖上一次结果
 *
 * 示例：
 *   hexDump({0x01,0x03,0x00}) -> "01 03 00"
 */
inline const char* hexDump(const std::vector<uint8_t>& data) {
    thread_local std::string buf;
    buf.clear();

    if (data.empty()) {
        return "<empty>";
    }

    // 预分配，避免反复扩容
    buf.reserve(data.size() * 3);

    char tmp[4]; // "FF "

    for (uint8_t b : data) {
        std::snprintf(tmp, sizeof(tmp), "%02X ", b);
        buf.append(tmp);
    }

    // 去掉最后一个空格
    if (!buf.empty()) {
        buf.pop_back();
    }

    return buf.c_str();
}

#endif //ENERGYSTORAGE_HEX_DUMP_H