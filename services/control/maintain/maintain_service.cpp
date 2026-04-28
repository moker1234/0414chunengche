//
// Created by lxy on 2026/4/28.
//
// services/control/maintain/maintain_service.cpp

#include "maintain_service.h"

#include <cstdio>
#include <sstream>
#include <utility>

#include "logger.h"
#include "config_loader.h"

#include "../../protocol/rs485/hmi/hmi_proto.h"
#include "../../protocol/rs232/ups_232_ascii_proto.h"
#include "../../protocol/rs485/air_conditioner_proto.h"

namespace control::maintain {

namespace {

static constexpr const char* kPathMaintainPassword   = "hmi.param.maintain_password";
static constexpr const char* kPathBoxNum             = "hmi.param.box_num";
static constexpr const char* kPathUpsShutdownTime    = "hmi.param.ups_shutdown_time";
static constexpr const char* kPathAirconSetTemp      = "hmi.param.aircon_set_temp";
static constexpr const char* kPathAirconSetHumidity  = "hmi.param.aircon_set_humidity";
static constexpr const char* kPathPasswordNew1       = "hmi.param.password_new_1";

/*
 * 空调协议寄存器。
 *
 * 注意：这是空调设备协议寄存器，不是 HMI 地址。
 * HMI 地址仍然全部从 normal_map_logic.jsonl 读取。
 *
 * 当前 AirConditionerProto 的 S4_Params 解析：
 *   0x0700 -> param.cool_point_c
 *   0x0708 -> param.high_hum_pct
 */
static constexpr uint16_t kAirRegCoolPointC  = 0x0700;
static constexpr uint16_t kAirRegHighHumPct  = 0x0708;

static const char* yesNo_(bool v)
{
    return v ? "1" : "0";
}

static uint16_t firstRegOrZero_(const ::control::HmiWriteEvent& w)
{
    return w.regs.empty() ? 0u : w.regs.front();
}

static bool isRegWrite_(const ::control::HmiWriteEvent& w)
{
    return !w.regs.empty();
}

} // namespace

void MaintainService::splitU32ToWordsHiLo_(uint32_t v,
                                            uint16_t& hi,
                                            uint16_t& lo)
{
    hi = static_cast<uint16_t>((v >> 16) & 0xFFFFu);
    lo = static_cast<uint16_t>(v & 0xFFFFu);
}

uint32_t MaintainService::joinWordsHiLo_(uint16_t hi, uint16_t lo)
{
    return (static_cast<uint32_t>(hi) << 16) |
           static_cast<uint32_t>(lo);
}

bool MaintainService::loadConfig(const std::string& path, std::string* err)
{
    ctx_.storage.config_path = path;

    MaintainConfig cfg{};
    std::string load_err;

    if (!storage_.load(path, cfg, &load_err)) {
        ctx_.storage.loaded_from_file = false;

        const MaintainConfig def{};
        ctx_.storage.current_password = def.password;
        ctx_.storage.box_id = def.box_id;
        ctx_.storage.ups_shutdown_time = def.ups_shutdown_time;
        ctx_.storage.aircon_set_temp = def.aircon_set_temp;
        ctx_.storage.aircon_set_humidity = def.aircon_set_humidity;

        LOG_SYS_W("[MAINTAIN][CFG] load failed path=%s err=%s, use default password=%u box_id=%u ups_time=%u air_temp=%d air_hum=%u",
                  path.c_str(),
                  load_err.c_str(),
                  (unsigned)ctx_.storage.current_password,
                  (unsigned)ctx_.storage.box_id,
                  (unsigned)ctx_.storage.ups_shutdown_time,
                  (int)ctx_.storage.aircon_set_temp,
                  (unsigned)ctx_.storage.aircon_set_humidity);

        if (err) {
            *err = load_err;
        }

        std::string sync_err;
        if (hmi_ && ctx_.map.maintain_password.valid) {
            (void)syncPasswordToHmi(&sync_err);
        }

        return true;
    }

    ctx_.storage.loaded_from_file = true;
    ctx_.storage.current_password = cfg.password;
    ctx_.storage.box_id = cfg.box_id;
    ctx_.storage.ups_shutdown_time = cfg.ups_shutdown_time;
    ctx_.storage.aircon_set_temp = cfg.aircon_set_temp;
    ctx_.storage.aircon_set_humidity = cfg.aircon_set_humidity;

    LOG_SYS_I("[MAINTAIN][CFG] loaded path=%s password=%u box_id=%u ups_time=%u air_temp=%d air_hum=%u",
              path.c_str(),
              (unsigned)ctx_.storage.current_password,
              (unsigned)ctx_.storage.box_id,
              (unsigned)ctx_.storage.ups_shutdown_time,
              (int)ctx_.storage.aircon_set_temp,
              (unsigned)ctx_.storage.aircon_set_humidity);

    if (err) {
        err->clear();
    }

    std::string sync_err;
    if (hmi_ && ctx_.map.maintain_password.valid) {
        (void)syncPasswordToHmi(&sync_err);
    }

    return true;
}

bool MaintainService::loadSystemConfig(const std::string& path, std::string* err)
{
    ctx_.links.clear();

    SystemConfig sys{};
    std::string load_err;

    if (!ConfigLoader::loadSystem(path, sys, load_err)) {
        if (err) {
            *err = load_err;
        }

        LOG_SYS_W("[MAINTAIN][SYS] load system failed path=%s err=%s",
                  path.c_str(),
                  load_err.c_str());
        return false;
    }

    for (const auto& l : sys.rs485_links) {
        if (l.role != LinkRole::MasterPoll) continue;
        if (l.type != Rs485ProtoType::AirConditioner) continue;

        ctx_.links.air_rs485_index = l.link_index;
        ctx_.links.air_slave_id = l.slave_id;
        break;
    }

    for (const auto& l : sys.rs232_links) {
        if (l.role != LinkRole::MasterPoll) continue;
        if (l.type != "ups_ascii") continue;

        ctx_.links.ups_rs232_index = l.link_index;
        break;
    }

    ctx_.links.system_loaded = true;

    LOG_SYS_I("[MAINTAIN][SYS] loaded path=%s air_rs485=%d air_slave=%u ups_rs232=%d",
              path.c_str(),
              ctx_.links.air_rs485_index,
              (unsigned)ctx_.links.air_slave_id,
              ctx_.links.ups_rs232_index);

    if (ctx_.links.air_rs485_index < 0) {
        LOG_SYS_W("[MAINTAIN][SYS] AirConditioner RS485 link not found");
    }

    if (ctx_.links.ups_rs232_index < 0) {
        LOG_SYS_W("[MAINTAIN][SYS] UPS RS232 link not found");
    }

    if (err) {
        err->clear();
    }

    return true;
}

bool MaintainService::resolveEndpoint_(const normal::HmiMapModel& model,
                                       const char* path,
                                       MaintainEndpoint& out,
                                       bool require_rw,
                                       std::string* warn) const
{
    out.clear();

    if (!path || !*path) {
        if (warn) *warn = "empty path";
        return false;
    }

    const auto* indexes = model.findByPath(path);
    if (!indexes || indexes->empty()) {
        if (warn) *warn = std::string("path not found: ") + path;
        return false;
    }

    const normal::HmiMapItem* fallback = nullptr;
    const normal::HmiMapItem* selected = nullptr;

    for (std::size_t idx : *indexes) {
        const normal::HmiMapItem* item = model.itemAt(idx);
        if (!item || !item->validAddress()) {
            continue;
        }

        if (!fallback) {
            fallback = item;
        }

        const bool is_rw =
            normal::isRwMapType(item->type) ||
            item->kind == normal::HmiMapItemKind::Rw;

        if (!require_rw || is_rw) {
            selected = item;
            break;
        }
    }

    if (!selected) {
        selected = fallback;
    }

    if (!selected) {
        if (warn) *warn = std::string("path has no valid address: ") + path;
        return false;
    }

    const bool selected_is_rw =
        normal::isRwMapType(selected->type) ||
        selected->kind == normal::HmiMapItemKind::Rw;

    if (require_rw && !selected_is_rw) {
        if (warn) {
            std::ostringstream oss;
            oss << "path found but not rw: " << path
                << " addr=0x" << std::hex << selected->addr
                << " type=" << normal::hmiMapTypeName(selected->type)
                << " kind=" << normal::hmiMapItemKindName(selected->kind);
            *warn = oss.str();
        }
        return false;
    }

    out.valid = true;
    out.addr = selected->addr;
    out.words = selected->words == 0 ? 1 : selected->words;
    out.type = selected->type;
    out.kind = selected->kind;
    out.path = selected->path;
    out.name = selected->name;
    out.value_type = selected->value_type;

    return true;
}

void MaintainService::logEndpoint_(const char* tag,
                                   const MaintainEndpoint& ep) const
{
    if (!ep.valid) {
        LOG_SYS_W("[MAINTAIN][MAP] %-22s missing", tag ? tag : "unknown");
        return;
    }

    LOG_SYS_I("[MAINTAIN][MAP] %-22s addr=0x%04X words=%u type=%s kind=%s value_type=%s path=%s name=%s",
              tag ? tag : "unknown",
              (unsigned)ep.addr,
              (unsigned)ep.words,
              normal::hmiMapTypeName(ep.type),
              normal::hmiMapItemKindName(ep.kind),
              ep.value_type.c_str(),
              ep.path.c_str(),
              ep.name.c_str());
}

bool MaintainService::bindHmiMap(std::shared_ptr<const normal::HmiMapModel> model,
                                 std::string* err)
{
    ctx_.map.clear();

    if (!model) {
        if (err) *err = "null HmiMapModel";
        LOG_SYS_W("[MAINTAIN][MAP] bind failed: null HmiMapModel");
        return false;
    }

    ctx_.map.model = std::move(model);

    std::string warn;

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathMaintainPassword, ctx_.map.maintain_password, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathBoxNum, ctx_.map.box_num, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathUpsShutdownTime, ctx_.map.ups_shutdown_time, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathAirconSetTemp, ctx_.map.aircon_set_temp, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathAirconSetHumidity, ctx_.map.aircon_set_humidity, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    warn.clear();
    if (!resolveEndpoint_(*ctx_.map.model, kPathPasswordNew1, ctx_.map.password_new_1, true, &warn)) {
        LOG_SYS_W("[MAINTAIN][MAP] %s", warn.c_str());
    }

    ctx_.map.bound = true;

    logEndpoint_("maintain_password", ctx_.map.maintain_password);
    logEndpoint_("box_num", ctx_.map.box_num);
    logEndpoint_("ups_shutdown_time", ctx_.map.ups_shutdown_time);
    logEndpoint_("aircon_set_temp", ctx_.map.aircon_set_temp);
    logEndpoint_("aircon_set_humidity", ctx_.map.aircon_set_humidity);
    logEndpoint_("password_new_1", ctx_.map.password_new_1);

    LOG_SYS_I("[MAINTAIN][MAP] bind done loaded_from_file=%s password=%u box_id=%u ups_time=%u air_temp=%d air_hum=%u",
              yesNo_(ctx_.storage.loaded_from_file),
              (unsigned)ctx_.storage.current_password,
              (unsigned)ctx_.storage.box_id,
              (unsigned)ctx_.storage.ups_shutdown_time,
              (int)ctx_.storage.aircon_set_temp,
              (unsigned)ctx_.storage.aircon_set_humidity);

    if (err) err->clear();

    if (hmi_ && ctx_.map.maintain_password.valid) {
        std::string sync_err;
        if (!syncPasswordToHmi(&sync_err)) {
            LOG_SYS_W("[MAINTAIN][HMI] sync password after map bind failed: %s",
                      sync_err.c_str());
        }
    }

    return true;
}

void MaintainService::bindHmi(::HMIProto* hmi)
{
    hmi_ = hmi;

    if (!hmi_) {
        LOG_SYS_W("[MAINTAIN][HMI] bind null HMIProto");
        return;
    }

    LOG_SYS_I("[MAINTAIN][HMI] bind HMIProto slave=%u",
              (unsigned)hmi_->slaveAddr());

    std::string sync_err;
    if (!syncPasswordToHmi(&sync_err)) {
        LOG_SYS_W("[MAINTAIN][HMI] sync password on bind failed: %s",
                  sync_err.c_str());
    }
}

bool MaintainService::syncPasswordToHmi(std::string* err)
{
    if (!hmi_) {
        if (err) *err = "HMIProto not bound";
        return false;
    }

    const auto& ep = ctx_.map.maintain_password;
    if (!ep.valid) {
        if (err) *err = "maintain_password endpoint invalid";
        return false;
    }

    if (ep.words < 2) {
        if (err) {
            std::ostringstream oss;
            oss << "maintain_password words invalid, words=" << ep.words
                << " path=" << ep.path
                << " addr=0x" << std::hex << ep.addr;
            *err = oss.str();
        }
        return false;
    }

    uint16_t hi = 0;
    uint16_t lo = 0;
    splitU32ToWordsHiLo_(ctx_.storage.current_password, hi, lo);

    hmi_->setIntRw(ep.addr, hi);
    hmi_->setIntRw(static_cast<uint16_t>(ep.addr + 1), lo);

    LOG_SYS_I("[MAINTAIN][HMI] password synced path=%s addr=0x%04X words=%u password=%u hi=0x%04X lo=0x%04X",
              ep.path.c_str(),
              (unsigned)ep.addr,
              (unsigned)ep.words,
              (unsigned)ctx_.storage.current_password,
              (unsigned)hi,
              (unsigned)lo);

    if (err) err->clear();
    return true;
}

bool MaintainService::isAddrInEndpoint_(const MaintainEndpoint& ep,
                                        uint16_t addr) const
{
    if (!ep.valid) {
        return false;
    }

    const uint16_t words = ep.words == 0 ? 1 : ep.words;
    return addr >= ep.addr &&
           addr < static_cast<uint16_t>(ep.addr + words);
}

bool MaintainService::isEndpointBaseWrite_(const MaintainEndpoint& ep,
                                           const ::control::HmiWriteEvent& w) const
{
    return ep.valid && isRegWrite_(w) && w.start_addr == ep.addr;
}

MaintainConfig MaintainService::makeConfigFromContext_() const
{
    MaintainConfig cfg{};
    cfg.version = 1;
    cfg.password = ctx_.storage.current_password;
    cfg.box_id = ctx_.storage.box_id;
    cfg.ups_shutdown_time = ctx_.storage.ups_shutdown_time;
    cfg.aircon_set_temp = ctx_.storage.aircon_set_temp;
    cfg.aircon_set_humidity = ctx_.storage.aircon_set_humidity;
    return cfg;
}

bool MaintainService::saveCurrentConfig_(std::string* err) const
{
    if (ctx_.storage.config_path.empty()) {
        if (err) *err = "maintain config path is empty";
        return false;
    }

    return storage_.saveAtomic(ctx_.storage.config_path,
                               makeConfigFromContext_(),
                               err);
}

bool MaintainService::applyNewPassword_(uint32_t new_password,
                                        uint64_t ts_ms,
                                        std::string* err)
{
    const MaintainStorageState old = ctx_.storage;

    ctx_.storage.current_password = new_password;
    ctx_.storage.last_update_ts_ms = ts_ms;

    std::string save_err;
    if (!saveCurrentConfig_(&save_err)) {
        ctx_.storage = old;

        LOG_SYS_W("[MAINTAIN][PWD] save failed old=%u new=%u err=%s",
                  (unsigned)old.current_password,
                  (unsigned)new_password,
                  save_err.c_str());

        std::string sync_err;
        (void)syncPasswordToHmi(&sync_err);

        if (err) *err = save_err;
        return false;
    }

    ctx_.storage.loaded_from_file = true;

    std::string sync_err;
    if (!syncPasswordToHmi(&sync_err)) {
        LOG_SYS_W("[MAINTAIN][PWD] saved but sync HMI failed old=%u new=%u err=%s",
                  (unsigned)old.current_password,
                  (unsigned)new_password,
                  sync_err.c_str());

        if (err) *err = sync_err;
        return true;
    }

    LOG_SYS_I("[MAINTAIN][PWD] password changed old=%u new=%u ts=%llu",
              (unsigned)old.current_password,
              (unsigned)new_password,
              (unsigned long long)ts_ms);

    if (err) err->clear();
    return true;
}

bool MaintainService::applyBoxId_(uint16_t box_id,
                                  uint64_t ts_ms,
                                  std::string* err)
{
    const MaintainStorageState old = ctx_.storage;

    ctx_.storage.box_id = box_id;
    ctx_.storage.last_update_ts_ms = ts_ms;

    std::string save_err;
    if (!saveCurrentConfig_(&save_err)) {
        ctx_.storage = old;
        if (err) *err = save_err;

        LOG_SYS_W("[MAINTAIN][BOX] save failed old=%u new=%u err=%s",
                  (unsigned)old.box_id,
                  (unsigned)box_id,
                  save_err.c_str());
        return false;
    }

    ctx_.storage.loaded_from_file = true;

    LOG_SYS_I("[MAINTAIN][BOX] box_id changed old=%u new=%u ts=%llu",
              (unsigned)old.box_id,
              (unsigned)box_id,
              (unsigned long long)ts_ms);

    if (err) err->clear();
    return true;
}

std::string MaintainService::formatUpsShutdownDelay_(uint16_t minutes)
{
    /*
     * UPS 协议当前支持：
     *   S.2 / S.3 / S01..S10
     *
     * HMI 这里是 UINT，不适合表达 .2 / .3。
     * 所以本批采用整数分钟：
     *   1..10 -> "01".."10"
     *   0     -> 不下发，只保存
     */
    if (minutes < 1 || minutes > 10) {
        return {};
    }

    char buf[4] = {0};
    std::snprintf(buf, sizeof(buf), "%02u", (unsigned)minutes);
    return std::string(buf);
}

bool MaintainService::emitUpsShutdownCommand_(uint16_t minutes,
                                              std::vector<::control::Command>& out_cmds,
                                              std::string* err) const
{
    if (minutes == 0) {
        if (err) err->clear();
        LOG_SYS_I("[MAINTAIN][UPS] shutdown_time=0, save only, command not sent");
        return true;
    }

    if (ctx_.links.ups_rs232_index < 0) {
        if (err) *err = "UPS RS232 link not bound";
        return false;
    }

    const std::string delay = formatUpsShutdownDelay_(minutes);
    if (delay.empty()) {
        if (err) {
            *err = "UPS shutdown time invalid, allowed 1..10 minutes";
        }
        return false;
    }

    std::vector<uint8_t> bytes = Ups232AsciiProto::buildShutdownCmd(delay);
    if (bytes.empty()) {
        if (err) *err = "UPS buildShutdownCmd returned empty";
        return false;
    }

    ::control::Command cmd;
    cmd.type = ::control::Command::Type::SendSerialRaw;
    cmd.serial_raw.link_type = dev::LinkType::RS232;
    cmd.serial_raw.index = ctx_.links.ups_rs232_index;
    cmd.serial_raw.bytes = std::move(bytes);

    out_cmds.push_back(std::move(cmd));

    LOG_SYS_I("[MAINTAIN][UPS] shutdown command queued rs232=%d minutes=%u delay=%s",
              ctx_.links.ups_rs232_index,
              (unsigned)minutes,
              delay.c_str());

    if (err) err->clear();
    return true;
}

bool MaintainService::emitAirconWriteSingle_(uint16_t reg,
                                             uint16_t value,
                                             const char* reason,
                                             std::vector<::control::Command>& out_cmds,
                                             std::string* err) const
{
    if (ctx_.links.air_rs485_index < 0) {
        if (err) *err = "AirConditioner RS485 link not bound";
        return false;
    }

    AirConditionerProto proto(ctx_.links.air_slave_id);
    std::vector<uint8_t> bytes = proto.buildWriteCmd(reg, value);
    if (bytes.empty()) {
        if (err) *err = "AirConditioner buildWriteCmd returned empty";
        return false;
    }

    ::control::Command cmd;
    cmd.type = ::control::Command::Type::SendSerialRaw;
    cmd.serial_raw.link_type = dev::LinkType::RS485;
    cmd.serial_raw.index = ctx_.links.air_rs485_index;
    cmd.serial_raw.bytes = std::move(bytes);

    out_cmds.push_back(std::move(cmd));

    LOG_SYS_I("[MAINTAIN][AIR] write command queued rs485=%d slave=%u reg=0x%04X value=%u reason=%s",
              ctx_.links.air_rs485_index,
              (unsigned)ctx_.links.air_slave_id,
              (unsigned)reg,
              (unsigned)value,
              reason ? reason : "");

    if (err) err->clear();
    return true;
}

bool MaintainService::applyUpsShutdownTime_(uint16_t minutes,
                                            uint64_t ts_ms,
                                            std::vector<::control::Command>& out_cmds,
                                            std::string* err)
{
    const MaintainStorageState old = ctx_.storage;

    ctx_.storage.ups_shutdown_time = minutes;
    ctx_.storage.last_update_ts_ms = ts_ms;

    std::string save_err;
    if (!saveCurrentConfig_(&save_err)) {
        ctx_.storage = old;
        if (err) *err = save_err;

        LOG_SYS_W("[MAINTAIN][UPS] save failed old=%u new=%u err=%s",
                  (unsigned)old.ups_shutdown_time,
                  (unsigned)minutes,
                  save_err.c_str());
        return false;
    }

    ctx_.storage.loaded_from_file = true;

    std::string cmd_err;
    if (!emitUpsShutdownCommand_(minutes, out_cmds, &cmd_err)) {
        LOG_SYS_W("[MAINTAIN][UPS] saved but command not queued minutes=%u err=%s",
                  (unsigned)minutes,
                  cmd_err.c_str());

        if (err) *err = cmd_err;
        return true;
    }

    LOG_SYS_I("[MAINTAIN][UPS] shutdown_time changed old=%u new=%u ts=%llu",
              (unsigned)old.ups_shutdown_time,
              (unsigned)minutes,
              (unsigned long long)ts_ms);

    if (err) err->clear();
    return true;
}

bool MaintainService::applyAirconTemp_(int16_t temp_c,
                                       uint64_t ts_ms,
                                       std::vector<::control::Command>& out_cmds,
                                       std::string* err)
{
    const MaintainStorageState old = ctx_.storage;

    ctx_.storage.aircon_set_temp = temp_c;
    ctx_.storage.last_update_ts_ms = ts_ms;

    std::string save_err;
    if (!saveCurrentConfig_(&save_err)) {
        ctx_.storage = old;
        if (err) *err = save_err;

        LOG_SYS_W("[MAINTAIN][AIR] temp save failed old=%d new=%d err=%s",
                  (int)old.aircon_set_temp,
                  (int)temp_c,
                  save_err.c_str());
        return false;
    }

    ctx_.storage.loaded_from_file = true;

    const uint16_t raw = static_cast<uint16_t>(temp_c);
    std::string cmd_err;
    if (!emitAirconWriteSingle_(kAirRegCoolPointC,
                                raw,
                                "set_temp",
                                out_cmds,
                                &cmd_err)) {
        LOG_SYS_W("[MAINTAIN][AIR] temp saved but command not queued temp=%d err=%s",
                  (int)temp_c,
                  cmd_err.c_str());

        if (err) *err = cmd_err;
        return true;
    }

    LOG_SYS_I("[MAINTAIN][AIR] temp changed old=%d new=%d ts=%llu",
              (int)old.aircon_set_temp,
              (int)temp_c,
              (unsigned long long)ts_ms);

    if (err) err->clear();
    return true;
}

bool MaintainService::applyAirconHumidity_(uint16_t humidity_pct,
                                           uint64_t ts_ms,
                                           std::vector<::control::Command>& out_cmds,
                                           std::string* err)
{
    const MaintainStorageState old = ctx_.storage;

    ctx_.storage.aircon_set_humidity = humidity_pct;
    ctx_.storage.last_update_ts_ms = ts_ms;

    std::string save_err;
    if (!saveCurrentConfig_(&save_err)) {
        ctx_.storage = old;
        if (err) *err = save_err;

        LOG_SYS_W("[MAINTAIN][AIR] humidity save failed old=%u new=%u err=%s",
                  (unsigned)old.aircon_set_humidity,
                  (unsigned)humidity_pct,
                  save_err.c_str());
        return false;
    }

    ctx_.storage.loaded_from_file = true;

    std::string cmd_err;
    if (!emitAirconWriteSingle_(kAirRegHighHumPct,
                                humidity_pct,
                                "set_humidity",
                                out_cmds,
                                &cmd_err)) {
        LOG_SYS_W("[MAINTAIN][AIR] humidity saved but command not queued humidity=%u err=%s",
                  (unsigned)humidity_pct,
                  cmd_err.c_str());

        if (err) *err = cmd_err;
        return true;
    }

    LOG_SYS_I("[MAINTAIN][AIR] humidity changed old=%u new=%u ts=%llu",
              (unsigned)old.aircon_set_humidity,
              (unsigned)humidity_pct,
              (unsigned long long)ts_ms);

    if (err) err->clear();
    return true;
}

bool MaintainService::handlePasswordSetWrite_(const ::control::HmiWriteEvent& w)
{
    const auto& ep = ctx_.map.password_new_1;

    if (!ep.valid) {
        return false;
    }

    if (!isAddrInEndpoint_(ep, w.start_addr)) {
        return false;
    }

    if (!isRegWrite_(w)) {
        LOG_SYS_W("[MAINTAIN][PWD] ignore non-reg write addr=0x%04X path=%s",
                  (unsigned)w.start_addr,
                  ep.path.c_str());
        return true;
    }

    if (ep.words < 2) {
        LOG_SYS_W("[MAINTAIN][PWD] endpoint words invalid addr=0x%04X words=%u path=%s",
                  (unsigned)ep.addr,
                  (unsigned)ep.words,
                  ep.path.c_str());
        return true;
    }

    if (w.start_addr == ep.addr && w.regs.size() >= 2) {
        const uint16_t hi = w.regs[0];
        const uint16_t lo = w.regs[1];
        const uint32_t new_password = joinWordsHiLo_(hi, lo);

        ctx_.password_write.clearPending();

        std::string err;
        if (!applyNewPassword_(new_password, w.ts_ms, &err)) {
            LOG_SYS_W("[MAINTAIN][PWD] apply direct multi-reg failed addr=0x%04X err=%s",
                      (unsigned)w.start_addr,
                      err.c_str());
        }

        return true;
    }

    const uint16_t word = firstRegOrZero_(w);
    const uint16_t hi_addr = ep.addr;
    const uint16_t lo_addr = static_cast<uint16_t>(ep.addr + 1);

    if (w.start_addr == hi_addr) {
        ctx_.password_write.pending_hi_valid = true;
        ctx_.password_write.pending_hi = word;
        ctx_.password_write.pending_hi_ts_ms = w.ts_ms;

        LOG_SYS_I("[MAINTAIN][PWD] high word received addr=0x%04X hi=0x%04X path=%s",
                  (unsigned)w.start_addr,
                  (unsigned)word,
                  ep.path.c_str());

        return true;
    }

    if (w.start_addr == lo_addr) {
        if (!ctx_.password_write.pending_hi_valid) {
            LOG_SYS_W("[MAINTAIN][PWD] low word received without high word addr=0x%04X lo=0x%04X path=%s",
                      (unsigned)w.start_addr,
                      (unsigned)word,
                      ep.path.c_str());
            return true;
        }

        if (w.ts_ms >= ctx_.password_write.pending_hi_ts_ms) {
            const uint64_t age = w.ts_ms - ctx_.password_write.pending_hi_ts_ms;
            if (age > ctx_.password_write.pending_timeout_ms) {
                LOG_SYS_W("[MAINTAIN][PWD] pending high word timeout age=%llu ms hi=0x%04X lo=0x%04X",
                          (unsigned long long)age,
                          (unsigned)ctx_.password_write.pending_hi,
                          (unsigned)word);
                ctx_.password_write.clearPending();
                return true;
            }
        }

        const uint16_t hi = ctx_.password_write.pending_hi;
        const uint16_t lo = word;
        const uint32_t new_password = joinWordsHiLo_(hi, lo);

        ctx_.password_write.clearPending();

        std::string err;
        if (!applyNewPassword_(new_password, w.ts_ms, &err)) {
            LOG_SYS_W("[MAINTAIN][PWD] apply password failed addr=0x%04X err=%s",
                      (unsigned)w.start_addr,
                      err.c_str());
        }

        return true;
    }

    return true;
}

bool MaintainService::handleBoxNumWrite_(const ::control::HmiWriteEvent& w)
{
    const auto& ep = ctx_.map.box_num;
    if (!isEndpointBaseWrite_(ep, w)) {
        return false;
    }

    const uint16_t box = firstRegOrZero_(w);

    std::string err;
    if (!applyBoxId_(box, w.ts_ms, &err)) {
        LOG_SYS_W("[MAINTAIN][BOX] apply failed addr=0x%04X value=%u err=%s",
                  (unsigned)w.start_addr,
                  (unsigned)box,
                  err.c_str());
    }

    return true;
}

bool MaintainService::handleUpsShutdownWrite_(const ::control::HmiWriteEvent& w,
                                              std::vector<::control::Command>& out_cmds)
{
    const auto& ep = ctx_.map.ups_shutdown_time;
    if (!isEndpointBaseWrite_(ep, w)) {
        return false;
    }

    const uint16_t minutes = firstRegOrZero_(w);

    std::string err;
    if (!applyUpsShutdownTime_(minutes, w.ts_ms, out_cmds, &err)) {
        LOG_SYS_W("[MAINTAIN][UPS] apply failed addr=0x%04X value=%u err=%s",
                  (unsigned)w.start_addr,
                  (unsigned)minutes,
                  err.c_str());
    }

    return true;
}

bool MaintainService::handleAirconTempWrite_(const ::control::HmiWriteEvent& w,
                                             std::vector<::control::Command>& out_cmds)
{
    const auto& ep = ctx_.map.aircon_set_temp;
    if (!isEndpointBaseWrite_(ep, w)) {
        return false;
    }

    const int16_t temp_c = static_cast<int16_t>(firstRegOrZero_(w));

    std::string err;
    if (!applyAirconTemp_(temp_c, w.ts_ms, out_cmds, &err)) {
        LOG_SYS_W("[MAINTAIN][AIR] temp apply failed addr=0x%04X value=%d err=%s",
                  (unsigned)w.start_addr,
                  (int)temp_c,
                  err.c_str());
    }

    return true;
}

bool MaintainService::handleAirconHumidityWrite_(const ::control::HmiWriteEvent& w,
                                                 std::vector<::control::Command>& out_cmds)
{
    const auto& ep = ctx_.map.aircon_set_humidity;
    if (!isEndpointBaseWrite_(ep, w)) {
        return false;
    }

    const uint16_t humidity = firstRegOrZero_(w);

    std::string err;
    if (!applyAirconHumidity_(humidity, w.ts_ms, out_cmds, &err)) {
        LOG_SYS_W("[MAINTAIN][AIR] humidity apply failed addr=0x%04X value=%u err=%s",
                  (unsigned)w.start_addr,
                  (unsigned)humidity,
                  err.c_str());
    }

    return true;
}

bool MaintainService::onHmiWrite(const ::control::HmiWriteEvent& w,
                                 std::vector<::control::Command>& out_cmds)
{
    if (handlePasswordSetWrite_(w)) {
        return true;
    }

    if (handleBoxNumWrite_(w)) {
        return true;
    }

    if (handleUpsShutdownWrite_(w, out_cmds)) {
        return true;
    }

    if (handleAirconTempWrite_(w, out_cmds)) {
        return true;
    }

    if (handleAirconHumidityWrite_(w, out_cmds)) {
        return true;
    }

    return false;
}

} // namespace control::maintain