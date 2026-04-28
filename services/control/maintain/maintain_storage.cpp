//
// Created by lxy on 2026/4/28.
//
// services/control/maintain/maintain_storage.cpp

#include "maintain_storage.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace control::maintain {

bool MaintainStorage::parseU32_(const nlohmann::json& j,
                                const char* key,
                                uint32_t& out)
{
    if (!key || !j.contains(key)) {
        return false;
    }

    try {
        const auto& v = j.at(key);

        if (v.is_number_unsigned()) {
            out = v.get<uint32_t>();
            return true;
        }

        if (v.is_number_integer()) {
            const int64_t x = v.get<int64_t>();
            if (x < 0) return false;
            out = static_cast<uint32_t>(x);
            return true;
        }

        if (v.is_string()) {
            const std::string s = v.get<std::string>();
            if (s.empty()) return false;

            std::size_t pos = 0;
            const unsigned long x = std::stoul(s, &pos, 0);
            if (pos != s.size()) return false;

            out = static_cast<uint32_t>(x);
            return true;
        }
    } catch (...) {
        return false;
    }

    return false;
}

bool MaintainStorage::parseU16_(const nlohmann::json& j,
                                const char* key,
                                uint16_t& out)
{
    uint32_t tmp = 0;
    if (!parseU32_(j, key, tmp)) {
        return false;
    }

    if (tmp > 0xFFFFu) {
        return false;
    }

    out = static_cast<uint16_t>(tmp);
    return true;
}

bool MaintainStorage::parseI16_(const nlohmann::json& j,
                                const char* key,
                                int16_t& out)
{
    if (!key || !j.contains(key)) {
        return false;
    }

    try {
        const auto& v = j.at(key);

        int64_t x = 0;

        if (v.is_number_integer()) {
            x = v.get<int64_t>();
        } else if (v.is_number_unsigned()) {
            x = static_cast<int64_t>(v.get<uint32_t>());
        } else if (v.is_string()) {
            const std::string s = v.get<std::string>();
            if (s.empty()) return false;

            std::size_t pos = 0;
            x = std::stoll(s, &pos, 0);
            if (pos != s.size()) return false;
        } else {
            return false;
        }

        if (x < -32768 || x > 32767) {
            return false;
        }

        out = static_cast<int16_t>(x);
        return true;
    } catch (...) {
        return false;
    }
}

bool MaintainStorage::load(const std::string& path,
                           MaintainConfig& out,
                           std::string* err) const
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        if (err) {
            *err = "open failed: " + path + ", errno=" +
                   std::to_string(errno) + "(" + std::strerror(errno) + ")";
        }
        return false;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception& e) {
        if (err) {
            *err = std::string("json parse failed: ") + e.what();
        }
        return false;
    }

    if (!j.is_object()) {
        if (err) {
            *err = "maintain config root is not object";
        }
        return false;
    }

    MaintainConfig cfg{};

    uint32_t u32 = 0;
    if (parseU32_(j, "version", u32)) {
        cfg.version = u32;
    }

    if (parseU32_(j, "password", u32)) {
        cfg.password = u32;
    }

    uint16_t u16 = 0;
    if (parseU16_(j, "box_id", u16)) {
        cfg.box_id = u16;
    }

    if (parseU16_(j, "ups_shutdown_time", u16)) {
        cfg.ups_shutdown_time = u16;
    }

    int16_t i16 = 0;
    if (parseI16_(j, "aircon_set_temp", i16)) {
        cfg.aircon_set_temp = i16;
    }

    if (parseU16_(j, "aircon_set_humidity", u16)) {
        cfg.aircon_set_humidity = u16;
    }

    out = cfg;
    return true;
}

bool MaintainStorage::saveAtomic(const std::string& path,
                                 const MaintainConfig& cfg,
                                 std::string* err) const
{
    const std::string tmp_path = path + ".tmp";

    {
        std::ofstream ofs(tmp_path, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) {
            if (err) {
                *err = "open tmp failed: " + tmp_path + ", errno=" +
                       std::to_string(errno) + "(" + std::strerror(errno) + ")";
            }
            return false;
        }

        nlohmann::json j;
        j["version"] = cfg.version;
        j["password"] = cfg.password;
        j["box_id"] = cfg.box_id;
        j["ups_shutdown_time"] = cfg.ups_shutdown_time;
        j["aircon_set_temp"] = cfg.aircon_set_temp;
        j["aircon_set_humidity"] = cfg.aircon_set_humidity;

        ofs << j.dump(4) << "\n";
        if (!ofs.good()) {
            if (err) {
                *err = "write tmp failed: " + tmp_path;
            }
            return false;
        }
    }

    if (std::rename(tmp_path.c_str(), path.c_str()) != 0) {
        if (err) {
            *err = "rename failed: " + tmp_path + " -> " + path +
                   ", errno=" + std::to_string(errno) +
                   "(" + std::strerror(errno) + ")";
        }

        std::remove(tmp_path.c_str());
        return false;
    }

    return true;
}

} // namespace control::maintain
