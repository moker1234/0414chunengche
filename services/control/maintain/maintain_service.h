//
// Created by lxy on 2026/4/28.
//

#ifndef ENERGYSTORAGE_MAINTAIN_SERVICE_H
#define ENERGYSTORAGE_MAINTAIN_SERVICE_H
// services/control/maintain/maintain_service.h
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "maintain_context.h"
#include "maintain_storage.h"

#include "../control_events.h"
#include "../control_commands.h"

class HMIProto;

namespace control::maintain {

class MaintainService
{
public:
    MaintainService() = default;

    bool loadConfig(const std::string& path, std::string* err = nullptr);

    // 第五批：从 system.json 识别 UPS / AIR 所在通信口。
    bool loadSystemConfig(const std::string& path, std::string* err = nullptr);

    bool bindHmiMap(std::shared_ptr<const normal::HmiMapModel> model,
                    std::string* err = nullptr);

    void bindHmi(::HMIProto* hmi);

    bool syncPasswordToHmi(std::string* err = nullptr);

    // 返回 true 表示该写入已被维护模块消费。
    bool onHmiWrite(const ::control::HmiWriteEvent& w,
                    std::vector<::control::Command>& out_cmds);

    const MaintainContext& context() const
    {
        return ctx_;
    }

private:
    bool resolveEndpoint_(const normal::HmiMapModel& model,
                          const char* path,
                          MaintainEndpoint& out,
                          bool require_rw,
                          std::string* warn = nullptr) const;

    void logEndpoint_(const char* tag,
                      const MaintainEndpoint& ep) const;

    static void splitU32ToWordsHiLo_(uint32_t v,
                                     uint16_t& hi,
                                     uint16_t& lo);

    static uint32_t joinWordsHiLo_(uint16_t hi, uint16_t lo);

    bool isAddrInEndpoint_(const MaintainEndpoint& ep,
                           uint16_t addr) const;

    bool isEndpointBaseWrite_(const MaintainEndpoint& ep,
                              const ::control::HmiWriteEvent& w) const;

    bool handlePasswordSetWrite_(const ::control::HmiWriteEvent& w);

    bool handleBoxNumWrite_(const ::control::HmiWriteEvent& w);
    bool handleUpsShutdownWrite_(const ::control::HmiWriteEvent& w,
                                 std::vector<::control::Command>& out_cmds);
    bool handleAirconTempWrite_(const ::control::HmiWriteEvent& w,
                                std::vector<::control::Command>& out_cmds);
    bool handleAirconHumidityWrite_(const ::control::HmiWriteEvent& w,
                                    std::vector<::control::Command>& out_cmds);

    bool applyNewPassword_(uint32_t new_password,
                           uint64_t ts_ms,
                           std::string* err = nullptr);

    bool applyBoxId_(uint16_t box_id,
                     uint64_t ts_ms,
                     std::string* err = nullptr);

    bool applyUpsShutdownTime_(uint16_t minutes,
                               uint64_t ts_ms,
                               std::vector<::control::Command>& out_cmds,
                               std::string* err = nullptr);

    bool applyAirconTemp_(int16_t temp_c,
                          uint64_t ts_ms,
                          std::vector<::control::Command>& out_cmds,
                          std::string* err = nullptr);

    bool applyAirconHumidity_(uint16_t humidity_pct,
                              uint64_t ts_ms,
                              std::vector<::control::Command>& out_cmds,
                              std::string* err = nullptr);

    MaintainConfig makeConfigFromContext_() const;
    bool saveCurrentConfig_(std::string* err = nullptr) const;

    bool emitUpsShutdownCommand_(uint16_t minutes,
                                 std::vector<::control::Command>& out_cmds,
                                 std::string* err = nullptr) const;

    bool emitAirconWriteSingle_(uint16_t reg,
                                uint16_t value,
                                const char* reason,
                                std::vector<::control::Command>& out_cmds,
                                std::string* err = nullptr) const;

    static std::string formatUpsShutdownDelay_(uint16_t minutes);

private:
    MaintainContext ctx_;
    MaintainStorage storage_;

    // 不拥有 HMIProto，只保存控制面绑定进来的指针。
    ::HMIProto* hmi_{nullptr};
};

} // namespace control::maintain

#endif //ENERGYSTORAGE_MAINTAIN_SERVICE_H
