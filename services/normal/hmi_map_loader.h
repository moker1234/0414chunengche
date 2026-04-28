//
// Created by lxy on 2026/4/27.
//

#ifndef ENERGYSTORAGE_HMI_MAP_LOADER_H
#define ENERGYSTORAGE_HMI_MAP_LOADER_H


// services/normal/hmi_map_loader.h
#pragma once

#include <cstdint>
#include <string>

#include "hmi_map_model.h"

namespace normal {

    /*
     * 统一 HMI 映射加载器。
     *
     * 只负责：
     *   normal_map_logic.jsonl -> HmiMapModel
     *
     * 不负责：
     *   写 HMI
     *   刷故障页
     *   处理 HMI 按钮
     */
    class HmiMapLoader {
    public:
        bool loadJsonl(const std::string& path,
                       HmiMapModel& out,
                       std::string* err = nullptr) const;

        static bool parseType(const std::string& s, HmiMapType& out);
        static bool parseU16HexOrDec(const std::string& s, uint16_t& out);
    };

} // namespace normal


#endif //ENERGYSTORAGE_HMI_MAP_LOADER_H
