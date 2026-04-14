//
// Created by lxy on 2026/2/3.
//

#include "config_loader.h"
#include <fstream>
#include <unistd.h>
#include <linux/limits.h>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

static LinkRole parseRole(const std::string& s) {
    if (s == "master_poll") return LinkRole::MasterPoll;
    if (s == "slave_hmi")   return LinkRole::SlaveHmi;
    return LinkRole::Unknown;
}
static Rs485ProtoType parseRs485Type(const std::string& s) {
    if (s == "gas")            return Rs485ProtoType::Gas;
    if (s == "smoke")          return Rs485ProtoType::Smoke;
    if (s == "air_conditioner")return Rs485ProtoType::AirConditioner;
    if (s == "hmi")            return Rs485ProtoType::Hmi;
    return Rs485ProtoType::Unknown;
}

static std::string getCwd() {
    char buf[PATH_MAX] = {0};
    if (getcwd(buf, sizeof(buf))) return std::string(buf);
    return {};
}

static bool readJsonFile(const std::string& path, json& j, std::string& err) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        err = "open failed: " + path + " cwd=" + getCwd();
        return false;
    }
    try { ifs >> j; }
    catch (const std::exception& e) { err = e.what(); return false; }
    return true;
}

static uint32_t parseU32HexOrDec(const nlohmann::json& v, uint32_t def = 0)
{
    try {
        if (v.is_number_unsigned()) return v.get<uint32_t>();
        if (v.is_number_integer())  return (uint32_t)v.get<int64_t>();
        if (v.is_number_float())    return (uint32_t)v.get<double>();
        if (v.is_string()) {
            const std::string s = v.get<std::string>();
            // base=0: 支持 "0x.." / "123"
            return (uint32_t)std::stoul(s, nullptr, 0);
        }
    } catch (...) {
        // fallthrough
    }
    return def;
}
static uint8_t parseU8HexOrDec(const nlohmann::json& v, uint8_t def = 0)
{
    return (uint8_t)parseU32HexOrDec(v, def);
}

bool ConfigLoader::loadIoMap(const std::string& path, IoMapConfig& out, std::string& err) {
    json j;
    if (!readJsonFile(path, j, err)) return false;

    out.can.clear();
    out.rs485.clear();
    out.rs232.clear();

    // ===== CAN =====
    // 允许两种写法：
    // 1) "can": { "links": [ {...}, ... ] }
    // 2) "can": [ {...}, ... ]   （兼容）
    if (j.contains("can")) {
        json links;
        if (j["can"].is_object()) {
            links = j["can"].value("links", json::array());
        } else if (j["can"].is_array()) {
            links = j["can"];
        } else {
            links = json::array();
        }

        if (links.is_array()) {
            for (const auto& it : links) {
                CanPortCfg c;
                c.name   = it.value("name", std::string{});
                c.ifname = it.value("ifname", std::string("can0"));
                c.bitrate = it.value("bitrate", 500000);
                c.enable = it.value("enable", true);
                if (!c.ifname.empty() && c.enable) out.can.push_back(c);
            }
        }
    }

    // 没配 CAN 就给默认 can0（避免程序没有任何 CAN）
    if (out.can.empty()) {
        out.can.push_back({"can0", "can0", true});
    }

    // ===== SERIAL =====
    auto serial = j.value("serial", json::object());
    auto rs485  = serial.value("rs485", json::array());
    auto rs232  = serial.value("rs232", json::array());

    if (rs485.is_array()) {
        for (const auto& it : rs485) {
            SerialPortCfg p;
            p.name     = it.value("name", std::string{});
            p.device   = it.value("device", std::string{});
            p.baudrate = it.value("baudrate", 9600);
            p.enable   = it.value("enable", true);
            if (!p.device.empty() && p.enable) out.rs485.push_back(p);
        }
    }

    if (rs232.is_array()) {
        for (const auto& it : rs232) {
            SerialPortCfg p;
            p.name     = it.value("name", std::string{});
            p.device   = it.value("device", std::string{});
            p.baudrate = it.value("baudrate", 2400);
            p.enable   = it.value("enable", true);
            if (!p.device.empty() && p.enable) out.rs232.push_back(p);
        }
    }

    return true;
}


bool ConfigLoader::loadSystem(const std::string& path, SystemConfig& out, std::string& err) {
    json j;
    if (!readJsonFile(path, j, err)) return false;

    // ===== helper: parse "0x.." string or number =====
    auto parseU32HexOrDec = [&](const json& v, uint32_t def = 0u) -> uint32_t {
        try {
            if (v.is_number_unsigned()) return v.get<uint32_t>();
            if (v.is_number_integer())  return static_cast<uint32_t>(v.get<int64_t>());
            if (v.is_string()) {
                const auto s = v.get<std::string>();
                if (s.empty()) return def;
                // base=0 支持 0x 前缀；也支持纯十进制字符串
                return static_cast<uint32_t>(std::stoul(s, nullptr, 0));
            }
        } catch (...) {
            // ignore
        }
        return def;
    };

    auto parseU8HexOrDec = [&](const json& v, uint8_t def = 0u) -> uint8_t {
        const uint32_t u = parseU32HexOrDec(v, def);
        return static_cast<uint8_t>(u & 0xFFu);
    };

    out.rs485_links.clear();
    out.rs232_links.clear();
    out.can_links.clear();

    // ===================== RS485 =====================
    auto rs485 = j.value("rs485", json::object());
    for (auto& it : rs485.value("links", json::array())) {
        Rs485LinkCfg c;
        c.link_index = it.value("link_index", 0);
        c.name       = it.value("name", "");
        c.role       = parseRole(it.value("role", ""));

        auto dev = it.value("device", json::object());
        c.type     = parseRs485Type(dev.value("type", ""));
        c.slave_id = (uint8_t)dev.value("slave_id", 1);
        c.crc_order= dev.value("crc_order", "hilo");

        auto poll = it.value("poll", json::object());

        c.poll.enable = poll.value("enable", false);

        c.poll.interval_ms = poll.value("interval_ms", c.poll.interval_ms);
        if (c.poll.interval_ms == 0) c.poll.interval_ms = 1000;

        c.poll.timeout_ms = poll.value("timeout_ms", c.poll.timeout_ms);
        if (c.poll.timeout_ms == 0) c.poll.timeout_ms = 300;

        c.poll.disconnect_window_ms = poll.value("disconnect_window_ms", c.poll.disconnect_window_ms);

        c.poll.backoff_ms = poll.value("backoff_ms", c.poll.backoff_ms);
        if (c.poll.backoff_ms == 0) c.poll.backoff_ms = 1000;

        c.poll.resend_after_ms = poll.value("resend_after_ms", c.poll.resend_after_ms);
        if (c.poll.resend_after_ms == 0) c.poll.resend_after_ms = 300;

        c.poll.max_retries = poll.value("max_retries", c.poll.max_retries);
        if (c.poll.max_retries == 0) c.poll.max_retries = 2;

        out.rs485_links.push_back(c);
    }

    // ===================== RS232 =====================
    auto rs232 = j.value("rs232", json::object());
    for (auto& it : rs232.value("links", json::array())) {
        Rs232LinkCfg c;
        c.link_index = it.value("link_index", 0);
        c.name       = it.value("name", "");
        c.role       = parseRole(it.value("role", ""));
        auto dev     = it.value("device", json::object());
        c.type       = dev.value("type", "ups_ascii");

        auto poll = it.value("poll", json::object());

        c.poll.enable = poll.value("enable", false);

        c.poll.interval_ms = poll.value("interval_ms", c.poll.interval_ms);
        if (c.poll.interval_ms == 0) c.poll.interval_ms = 1000;

        c.poll.timeout_ms = poll.value("timeout_ms", c.poll.timeout_ms);
        if (c.poll.timeout_ms == 0) c.poll.timeout_ms = 600;  // UPS 默认可更大

        c.poll.disconnect_window_ms = poll.value("disconnect_window_ms", c.poll.disconnect_window_ms);

        c.poll.backoff_ms = poll.value("backoff_ms", c.poll.backoff_ms);
        if (c.poll.backoff_ms == 0) c.poll.backoff_ms = 1000;

        c.poll.resend_after_ms = poll.value("resend_after_ms", c.poll.resend_after_ms);
        if (c.poll.resend_after_ms == 0) c.poll.resend_after_ms = 300;

        c.poll.max_retries = poll.value("max_retries", c.poll.max_retries);
        if (c.poll.max_retries == 0) c.poll.max_retries = 2;

        out.rs232_links.push_back(c);
    }

    // ===================== CAN (NEW) =====================
    auto can = j.value("can", json::object());
    for (auto& it : can.value("links", json::array())) {
        CanLinkCfg c;
        c.can_index = it.value("can_index", 0);
        c.name      = it.value("name", "");
        c.role      = it.value("role", "periodic_tx_rx");
        c.enable    = it.value("enable", false);

        auto proto = it.value("protocol", json::object());
        c.protocol_type = proto.value("type", "");

        auto ids = proto.value("ids", json::object());
        c.id_emu_ctrl   = parseU32HexOrDec(ids.value("emu_ctrl", json()), 0u);
        c.id_emu_status = parseU32HexOrDec(ids.value("emu_status", json()), 0u);
        c.id_pcu_status = parseU32HexOrDec(ids.value("pcu_status", json()), 0u);

        auto tx = it.value("tx", json::object());
        c.tx_enable  = tx.value("enable", true);
        c.interval_ms = tx.value("interval_ms", c.interval_ms);
        if (c.interval_ms == 0) c.interval_ms = 200;

        c.send_ctrl   = tx.value("send_ctrl", true);
        c.send_status = tx.value("send_status", true);

        // ctrl defaults (byte values, allow "0x..")
        c.ctrl_enable_default = parseU8HexOrDec(tx.value("ctrl_enable_default", json()), c.ctrl_enable_default);
        c.plug_default        = parseU8HexOrDec(tx.value("plug_default", json()), c.plug_default);
        c.estop_default       = parseU8HexOrDec(tx.value("estop_default", json()), c.estop_default);
        c.batt1_estop_default = parseU8HexOrDec(tx.value("batt1_estop_default", json()), c.batt1_estop_default);
        c.batt2_estop_default = parseU8HexOrDec(tx.value("batt2_estop_default", json()), c.batt2_estop_default);

        auto rx = it.value("rx", json::object());
        c.rx_enable = rx.value("enable", true);
        c.rx_timeout_ms = rx.value("timeout_ms", c.rx_timeout_ms);
        if (c.rx_timeout_ms == 0) c.rx_timeout_ms = 1000;
        c.require_heartbeat_increment = rx.value("require_heartbeat_increment", false);

        out.can_links.push_back(c);
    }

    return true;
}


