//
// Created by lxy on 2026/01/25.
//

#ifndef ENERGYSTORAGE_AIR_CONDITIONER_PROTO_H
#define ENERGYSTORAGE_AIR_CONDITIONER_PROTO_H

#pragma once
#include "../protocol_base.h"
#include <cstdint>
#include <vector>
#include <string>

/*
 * Air Conditioner (RS485 / Modbus RTU)
 *
 * CRC: LoHi (标准 Modbus RTU：CRC低字节在前)
 * Allowed funcs: 0x03, 0x06, 0x10
 *
 * 分段读（轮询循环）：
 *  S0: 0x0000 x1    版本
 *  S1: 0x0100 x6    运行状态
 *  S2: 0x0500 x9    传感器
 *  S3: 0x0600 x33   告警(0x0600~0x0620)
 *  S4: 0x0700 x11   系统参数(0x0700~0x070A)
 *  S5: 0x0801 x1    监控开关机
 */

class AirConditionerProto : public ProtocolBase {
public:
    explicit AirConditionerProto(uint8_t addr);

    // ===== 轮询读取（分段）=====
    std::vector<uint8_t> buildReadCmd() override;

    // ===== 解析：每次只输出本次响应对应的分段数据 =====
    bool parse(const std::vector<uint8_t>& rx, DeviceData& out) override;

    // ===== 写单寄存器（06）=====
    std::vector<uint8_t> buildWriteCmd(uint16_t reg, uint16_t value) override;

    // ===== 写多个寄存器（10）=====
    // 注意：ProtocolBase 可能没有虚函数接口，这里作为 AirConditionerProto 专用 API
    std::vector<uint8_t> buildWriteMultiCmd(uint16_t start_reg,
                                            const std::vector<uint16_t>& values);

    uint8_t slaveAddr() const override { return addr_; }

private:
    enum class PollStage : uint8_t {
        S0_Version = 0,
        S1_RunState,
        S2_Sensors,
        S3_Alarms,
        S4_Params,
    };

    struct Segment {
        uint16_t start;
        uint16_t count;
    };

private:
    uint8_t addr_{1};

    PollStage next_stage_{PollStage::S1_RunState};

    // 高频轮询运行状态/传感器/告警，低频轮询参数/遥控/版本。
    uint8_t poll_seq_index_{0};

    // 内部“全量缓存”：分段更新，保证每次输出都是“尽可能完整”
    // 注意：对外输出一律使用工程值：
    // - temp.*_c       单位 ℃
    // - current_a      单位 A
    // - ac_voltage_v   单位 V
    // - dc_voltage_v   单位 V
    DeviceData cache_;

private:
    // ===== Modbus RTU LoHi =====
    static std::vector<uint8_t> buildReadHolding_LoHi(uint8_t slave,
                                                      uint16_t start,
                                                      uint16_t count);

    static std::vector<uint8_t> buildWriteSingle_LoHi(uint8_t slave,
                                                      uint16_t reg,
                                                      uint16_t value);

    static std::vector<uint8_t> buildWriteMulti_LoHi(uint8_t slave,
                                                     uint16_t start_reg,
                                                     const std::vector<uint16_t>& values);

    static bool verifyCRC_LoHi(const std::vector<uint8_t>& frame);

    static bool parseHoldingRegs03_LoHi(const std::vector<uint8_t>& rx,
                                        uint8_t expect_slave,
                                        std::vector<uint16_t>& regs_out);

    static bool parseWriteEcho06_10_LoHi(const std::vector<uint8_t>& rx,
                                        uint8_t expect_slave,
                                        uint8_t expect_func);

    // ===== 数据工具 =====
    static int16_t  toS16(uint16_t u) { return static_cast<int16_t>(u); }
    static double   divSafe(int16_t v, double k) { return double(v) / k; }

    static Segment segmentOf(PollStage s);
    void advanceStage_();

    static bool inferStageByRegCount_(std::size_t reg_count, PollStage& out_stage);

    static void fillSegmentData_(PollStage s,
                                 const std::vector<uint16_t>& regs,
                                 DeviceData& out);

    // 告警字段名（用于 status map）
    static const char* alarmKeyByOffset_(uint16_t offset); // offset=0..32
};

#endif // ENERGYSTORAGE_AIR_CONDITIONER_PROTO_H
