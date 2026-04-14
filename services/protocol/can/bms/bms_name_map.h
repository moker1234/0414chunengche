//
// Created by lxy on 2026/3/10.
//

#ifndef ENERGYSTORAGE_BMS_NAME_MAP_H
#define ENERGYSTORAGE_BMS_NAME_MAP_H


#pragma once

#include <string>

namespace proto::bms {

    /**
     * 协议原名 -> 工程内部 canonical 名
     *
     * 目的：
     * 1. 统一 msg.xlsx / generated / filesink / logic 使用的消息名
     * 2. 避免后续出现 Fult / Fault、带后缀 / 不带后缀 混用
     */
    class BmsNameMap {
    public:
        static std::string canonicalMsgName(const char* proto_name);
    };

} // namespace proto::bms


#endif //ENERGYSTORAGE_BMS_NAME_MAP_H