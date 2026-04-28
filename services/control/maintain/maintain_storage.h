//
// Created by lxy on 2026/4/28.
//

#ifndef ENERGYSTORAGE_MAINTAIN_STORAGE_H
#define ENERGYSTORAGE_MAINTAIN_STORAGE_H


// services/control/maintain/maintain_storage.h
#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

namespace control::maintain {

struct MaintainConfig
{
    uint32_t version{1};

    uint32_t password{123456};

    uint16_t box_id{0};

    uint16_t ups_shutdown_time{0};

    int16_t aircon_set_temp{25};
    uint16_t aircon_set_humidity{60};
};

class MaintainStorage
{
public:
    MaintainStorage() = default;

    bool load(const std::string& path,
              MaintainConfig& out,
              std::string* err = nullptr) const;

    bool saveAtomic(const std::string& path,
                    const MaintainConfig& cfg,
                    std::string* err = nullptr) const;

private:
    static bool parseU32_(const nlohmann::json& j,
                          const char* key,
                          uint32_t& out);

    static bool parseU16_(const nlohmann::json& j,
                          const char* key,
                          uint16_t& out);

    static bool parseI16_(const nlohmann::json& j,
                          const char* key,
                          int16_t& out);
};

} // namespace control::maintain

#endif //ENERGYSTORAGE_MAINTAIN_STORAGE_H
