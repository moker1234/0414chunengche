//
// Created by lxy on 2026/01/25.
//

#include "air_conditioner_proto.h"
#include "../modbus/modbus_crc.h"
#include "logger.h"

#include <cstdio>

#include "hex_dump.h"

AirConditionerProto::AirConditionerProto(uint8_t addr)
    : addr_(addr) {
    cache_.device_name = "AirConditioner";
}

AirConditionerProto::Segment AirConditionerProto::segmentOf(PollStage s) {
    switch (s) {
        case PollStage::S0_Version:  return {0x0000, 1};
        case PollStage::S1_RunState: return {0x0100, 6};
        case PollStage::S2_Sensors:  return {0x0500, 9};
        case PollStage::S3_Alarms:   return {0x0600, 0x21}; // 33 regs: 0x0600~0x0620
        case PollStage::S4_Params:   return {0x0700, 0x0B}; // 11 regs: 0x0700~0x070A
        case PollStage::S5_Remote:   return {0x0801, 1};
        default:                     return {0x0000, 1};
    }
}

void AirConditionerProto::advanceStage_() {
    uint8_t v = static_cast<uint8_t>(next_stage_);
    v = (v + 1) % static_cast<uint8_t>(PollStage::COUNT);
    next_stage_ = static_cast<PollStage>(v);
}

std::vector<uint8_t> AirConditionerProto::buildReadCmd() {
    const PollStage s = next_stage_;
    const Segment seg = segmentOf(s);

    last_sent_stage_ = s;
    advanceStage_();

    // 03 读保持寄存器
    return buildReadHolding_LoHi(addr_, seg.start, seg.count);
}

bool AirConditionerProto::parse(const std::vector<uint8_t>& rx, DeviceData& out) {
    // LOGD("[AC][RX] %s", hexDump(rx));
    // LOG_COMM_HEX("RX dev=AirConditioner ...", rx.data(), rx.size());
    if (rx.size() < 5) return false;
    if (rx[0] != addr_) return false;

    const uint8_t func = rx[1];

    // 处理异常响应：[addr][func|0x80][err][crc_lo][crc_hi]
    if (func & 0x80) {
        if (!verifyCRC_LoHi(rx)) return false;
        const uint8_t err = rx[2];
        cache_.status["modbus_exception"] = 1;
        cache_.num["modbus_exception_code"] = double(err);
        out = cache_;
        return true; // 仍然上报，让上层看到异常
    }

    if (func == 0x03) {
    // LOG_COMM_HEX("RX dev=AirConditioner ...", rx.data(), rx.size());
        std::vector<uint16_t> regs;
        if (!parseHoldingRegs03_LoHi(rx, addr_, regs)) return false;

        // 分段更新缓存
        applySegmentToCache_(last_sent_stage_, regs);

        out = cache_;
        return true;
    }

    if (func == 0x06) {
        if (!parseWriteEcho06_10_LoHi(rx, addr_, 0x06)) return false;
        cache_.status["write_ok_06"] = 1;
        out = cache_;
        return true;
    }

    if (func == 0x10) {
        if (!parseWriteEcho06_10_LoHi(rx, addr_, 0x10)) return false;
        cache_.status["write_ok_10"] = 1;
        out = cache_;
        return true;
    }

    return false;
}

std::vector<uint8_t> AirConditionerProto::buildWriteCmd(uint16_t reg, uint16_t value) {
    return buildWriteSingle_LoHi(addr_, reg, value);
}

std::vector<uint8_t> AirConditionerProto::buildWriteMultiCmd(uint16_t start_reg,
                                                             const std::vector<uint16_t>& values) {
    return buildWriteMulti_LoHi(addr_, start_reg, values);
}

/* ====================== Modbus RTU LoHi ====================== */

std::vector<uint8_t> AirConditionerProto::buildReadHolding_LoHi(uint8_t slave,
                                                                uint16_t start,
                                                                uint16_t count) {
    std::vector<uint8_t> frame{
        slave,
        0x03,
        uint8_t(start >> 8), uint8_t(start & 0xFF),
        uint8_t(count >> 8), uint8_t(count & 0xFF)
    };

    const uint16_t crc = modbusCRC::calc(frame);
    frame.push_back(uint8_t(crc & 0xFF));   // CRC_L
    frame.push_back(uint8_t(crc >> 8));     // CRC_H
    // LOGD("[AC][TX] %s", hexDump(frame));
    // LOG_COMM_HEX("TX dev=AirConditioner ...", frame.data(), frame.size());
    return frame;
}

std::vector<uint8_t> AirConditionerProto::buildWriteSingle_LoHi(uint8_t slave,
                                                                uint16_t reg,
                                                                uint16_t value) {
    std::vector<uint8_t> frame{
        slave,
        0x06,
        uint8_t(reg >> 8), uint8_t(reg & 0xFF),
        uint8_t(value >> 8), uint8_t(value & 0xFF)
    };

    const uint16_t crc = modbusCRC::calc(frame);
    frame.push_back(uint8_t(crc & 0xFF));   // CRC_L
    frame.push_back(uint8_t(crc >> 8));     // CRC_H
    return frame;
}

std::vector<uint8_t> AirConditionerProto::buildWriteMulti_LoHi(uint8_t slave,
                                                               uint16_t start_reg,
                                                               const std::vector<uint16_t>& values) {
    // [addr][10][startHi][startLo][qtyHi][qtyLo][byteCnt][data...][crcLo][crcHi]
    const uint16_t qty = static_cast<uint16_t>(values.size());
    const uint8_t byteCnt = static_cast<uint8_t>(qty * 2);

    std::vector<uint8_t> frame;
    frame.reserve(7 + byteCnt + 2);

    frame.push_back(slave);
    frame.push_back(0x10);
    frame.push_back(uint8_t(start_reg >> 8));
    frame.push_back(uint8_t(start_reg & 0xFF));
    frame.push_back(uint8_t(qty >> 8));
    frame.push_back(uint8_t(qty & 0xFF));
    frame.push_back(byteCnt);

    for (uint16_t v : values) {
        frame.push_back(uint8_t(v >> 8));
        frame.push_back(uint8_t(v & 0xFF));
    }

    const uint16_t crc = modbusCRC::calc(frame);
    frame.push_back(uint8_t(crc & 0xFF));   // CRC_L
    frame.push_back(uint8_t(crc >> 8));     // CRC_H
    return frame;
}

bool AirConditionerProto::verifyCRC_LoHi(const std::vector<uint8_t>& frame) {
    if (frame.size() < 4) return false;

    const uint16_t recv =
        uint16_t(frame[frame.size() - 2]) |
        (uint16_t(frame[frame.size() - 1]) << 8);

    const uint16_t calc = modbusCRC::calc(frame.data(),
                                          static_cast<uint16_t>(frame.size() - 2));
    return recv == calc;
}

bool AirConditionerProto::parseHoldingRegs03_LoHi(const std::vector<uint8_t>& rx,
                                                  uint8_t expect_slave,
                                                  std::vector<uint16_t>& regs_out) {
    // [addr][03][byteCnt][data...][crcLo][crcHi]
    if (rx.size() < 5) return false;
    if (rx[0] != expect_slave) return false;
    if (rx[1] != 0x03) return false;
    if (!verifyCRC_LoHi(rx)) return false;

    const uint8_t byteCnt = rx[2];
    if ((byteCnt % 2) != 0) return false;

    const size_t expect_len = 3u + byteCnt + 2u;
    if (rx.size() != expect_len) return false;

    const size_t regCnt = byteCnt / 2;
    regs_out.clear();
    regs_out.reserve(regCnt);

    size_t idx = 3;
    for (size_t i = 0; i < regCnt; ++i) {
        const uint16_t v = (uint16_t(rx[idx]) << 8) | uint16_t(rx[idx + 1]);
        regs_out.push_back(v);
        idx += 2;
    }
    return true;
}

bool AirConditionerProto::parseWriteEcho06_10_LoHi(const std::vector<uint8_t>& rx,
                                                   uint8_t expect_slave,
                                                   uint8_t expect_func) {
    // 06/10 正常响应固定 8 字节
    if (rx.size() != 8) return false;
    if (rx[0] != expect_slave) return false;
    if (rx[1] != expect_func) return false;
    if (!verifyCRC_LoHi(rx)) return false;
    return true;
}

/* ====================== 分段应用到缓存 ====================== */

const char* AirConditionerProto::alarmKeyByOffset_(uint16_t off) {
    // 0x0600 + off
    // 简化：给出最主要的键名（你也可以后续扩展为完整表）
    switch (off) {
        case 0x00: return "alarm.high_temp";
        case 0x01: return "alarm.low_temp";
        case 0x02: return "alarm.high_hum";
        case 0x03: return "alarm.low_hum";
        case 0x04: return "alarm.coil_freeze";
        case 0x05: return "alarm.exhaust_high_temp";
        case 0x06: return "alarm.coil_sensor_fault";
        case 0x07: return "alarm.outdoor_sensor_fault";
        case 0x08: return "alarm.condense_sensor_fault";
        case 0x09: return "alarm.indoor_sensor_fault";
        case 0x0A: return "alarm.exhaust_sensor_fault";
        case 0x0B: return "alarm.hum_sensor_fault";
        case 0x0C: return "alarm.inner_fan_fault";
        case 0x0D: return "alarm.outer_fan_fault";
        case 0x0E: return "alarm.compressor_fault";
        case 0x0F: return "alarm.heater_fault";
        case 0x10: return "alarm.emergency_fan_fault";
        case 0x11: return "alarm.high_pressure";
        case 0x12: return "alarm.low_pressure";
        case 0x13: return "alarm.water";
        case 0x14: return "alarm.smoke";
        case 0x15: return "alarm.door";
        case 0x16: return "alarm.high_pressure_lock";
        case 0x17: return "alarm.low_pressure_lock";
        case 0x18: return "alarm.exhaust_lock";
        case 0x19: return "alarm.ac_over_v";
        case 0x1A: return "alarm.ac_under_v";
        case 0x1B: return "alarm.ac_power_loss";
        case 0x1C: return "alarm.phase_loss";
        case 0x1D: return "alarm.freq_abnormal";
        case 0x1E: return "alarm.reverse_phase";
        case 0x1F: return "alarm.dc_over_v";
        case 0x20: return "alarm.dc_under_v";
        default:   return "alarm.unknown";
    }
}

void AirConditionerProto::applySegmentToCache_(PollStage s,
                                               const std::vector<uint16_t>& regs) {
    cache_.device_name = "AirConditioner";
    cache_.status["modbus_exception"] = 0; // 收到正常帧时清理异常标志

    switch (s) {
        case PollStage::S0_Version: {
            if (regs.size() >= 1) {
                cache_.num["version"] = double(regs[0]);
            }
        } break;

        case PollStage::S1_RunState: {
            // 0x0100~0x0105
            if (regs.size() >= 6) {
                cache_.num["run.overall"]   = double(regs[0]);
                cache_.num["run.inner_fan"] = double(regs[1]);
                cache_.num["run.outer_fan"] = double(regs[2]);
                cache_.num["run.compressor"]= double(regs[3]);
                cache_.num["run.heater"]    = double(regs[4]);
                cache_.num["run.em_fan"]    = double(regs[5]);
            }
        } break;

        case PollStage::S2_Sensors: {
            // 0x0500~0x0508
            if (regs.size() >= 9) {
                // 温度：x10（可能存在负值，按 int16 解读）
                const int16_t coil10   = toS16(regs[0]);
                const int16_t out10    = toS16(regs[1]);
                const int16_t cond10   = toS16(regs[2]);
                const int16_t in10     = toS16(regs[3]);
                const int16_t hum      = toS16(regs[4]); // x1
                const int16_t exhaust10= toS16(regs[5]);
                const int16_t cur1000  = toS16(regs[6]); // x1000
                const int16_t acv      = toS16(regs[7]); // x1
                const int16_t dcv10    = toS16(regs[8]); // x10

                // 失效值约定：温度=2000（即 200.0℃），湿度=120
                cache_.status["sensor.temp_invalid"] =
                    (coil10==2000 || out10==2000 || cond10==2000 || in10==2000 || exhaust10==2000) ? 1 : 0;
                cache_.status["sensor.hum_invalid"] = (hum==120) ? 1 : 0;

                cache_.num["temp.coil_c"]     = div_enabled_? divSafe(coil10, 10.0):coil10;
                cache_.num["temp.outdoor_c"]  = div_enabled_? divSafe(out10, 10.0):out10;
                cache_.num["temp.condense_c"] = div_enabled_? divSafe(cond10, 10.0):cond10;
                cache_.num["temp.indoor_c"]   = div_enabled_? divSafe(in10, 10.0):in10;
                cache_.num["humidity_percent"]= double(hum);
                cache_.num["temp.exhaust_c"]  = div_enabled_? divSafe(exhaust10, 10.0):exhaust10;

                cache_.num["current_a"]       = div_enabled_? divSafe(cur1000, 1000.0):cur1000;
                cache_.num["ac_voltage_v"]    = double(acv);
                cache_.num["dc_voltage_v"]    = div_enabled_? divSafe(dcv10, 10.0):dcv10;
            }
        } break;

        case PollStage::S3_Alarms: {
            // 0x0600~0x0620 (33 regs) 值：0x00正常 0x01告警
            int any = 0;
            for (size_t i = 0; i < regs.size(); ++i) {
                const uint16_t v = regs[i];
                const char* key = alarmKeyByOffset_(uint16_t(i));
                cache_.status[key] = (v != 0) ? 1 : 0;
                if (v != 0) any = 1;
            }
                //打印所有告警的0x0600~0x0620


            cache_.status["alarm.any"] = any;
        } break;

        case PollStage::S4_Params: {
            // 0x0700~0x070A (11 regs)
            if (regs.size() >= 11) {
                cache_.num["param.cool_point_c"]   = double(toS16(regs[0])); // x1
                cache_.num["param.cool_hys_c"]     = double(toS16(regs[1]));
                cache_.num["param.heat_point_c"]   = double(toS16(regs[2]));
                cache_.num["param.heat_hys_c"]     = double(toS16(regs[3]));
                cache_.num["param.high_temp_c"]    = double(toS16(regs[6]));
                cache_.num["param.low_temp_c"]     = double(toS16(regs[7]));
                cache_.num["param.high_hum_pct"]   = double(toS16(regs[8]));
                // 0x0709 未列出，跳过
                cache_.num["param.inner_fan_stop_c"]= double(toS16(regs[10])); // 0x070A
            }
        } break;

        case PollStage::S5_Remote: {
            // 0x0801：0x01 开机，0x00 关机
            if (regs.size() >= 1) {
                cache_.num["remote.power"] = double(regs[0]);
            }
        } break;

        default:
            break;
    }
}
