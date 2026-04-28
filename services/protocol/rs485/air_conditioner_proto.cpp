//
// Created by lxy on 2026/01/25.
//

#include "air_conditioner_proto.h"
#include "../modbus/modbus_crc.h"
#include "logger.h"

#include <cstdio>

#include "hex_dump.h"

AirConditionerProto::AirConditionerProto(uint8_t addr)
    : addr_(addr)
{
}

AirConditionerProto::Segment AirConditionerProto::segmentOf(PollStage s) {
    switch (s) {
    case PollStage::S0_Version:
        return {0x0000, 1};

    case PollStage::S1_RunState:
        return {0x0100, 6};

    case PollStage::S2_Sensors:
        return {0x0500, 9};

    case PollStage::S3_Alarms:
        return {0x0600, 0x21}; // 33 regs: 0x0600~0x0620

    case PollStage::S4_Params:
        return {0x0700, 0x0B}; // 11 regs: 0x0700~0x070A

    default:
        return {0x0100, 6};
    }
}

void AirConditionerProto::advanceStage_() {
    static constexpr PollStage kSeq[] = {
        // 高频：运行状态、传感器、告警
        PollStage::S1_RunState,
        PollStage::S2_Sensors,
        PollStage::S3_Alarms,

        PollStage::S1_RunState,
        PollStage::S2_Sensors,
        PollStage::S3_Alarms,

        // 低频：参数
        PollStage::S4_Params,

        PollStage::S1_RunState,
        PollStage::S2_Sensors,
        PollStage::S3_Alarms,

        PollStage::S1_RunState,
        PollStage::S2_Sensors,
        PollStage::S3_Alarms,

        // 低频：版本
        PollStage::S0_Version
    };

    constexpr uint8_t kSeqCount =
        static_cast<uint8_t>(sizeof(kSeq) / sizeof(kSeq[0]));

    next_stage_ = kSeq[poll_seq_index_ % kSeqCount];
    poll_seq_index_ = static_cast<uint8_t>((poll_seq_index_ + 1) % kSeqCount);
}

std::vector<uint8_t> AirConditionerProto::buildReadCmd() {
    const PollStage s = next_stage_;
    const Segment seg = segmentOf(s);

    advanceStage_();

    // 03 读保持寄存器
    return buildReadHolding_LoHi(addr_, seg.start, seg.count);
}
bool AirConditionerProto::inferStageByRegCount_(std::size_t reg_count,
                                                PollStage& out_stage)
{
    switch (reg_count) {
    case 1:
        out_stage = PollStage::S0_Version;
        return true;

    case 6:
        out_stage = PollStage::S1_RunState;
        return true;

    case 9:
        out_stage = PollStage::S2_Sensors;
        return true;

    case 33:
        out_stage = PollStage::S3_Alarms;
        return true;

    case 11:
        out_stage = PollStage::S4_Params;
        return true;

    default:
        return false;
    }
}

bool AirConditionerProto::parse(const std::vector<uint8_t>& rx, DeviceData& out) {
    if (rx.size() < 5) return false;
    if (rx[0] != addr_) return false;

    const uint8_t func = rx[1];

    // Modbus 异常响应：说明设备有响应，但不是有效数据。
    // 不上报 DeviceData，避免污染 Aggregator / Logic / SQLite。
    if (func & 0x80) {
        return verifyCRC_LoHi(rx);
    }

    if (func == 0x03) {
        std::vector<uint16_t> regs;
        if (!parseHoldingRegs03_LoHi(rx, addr_, regs)) {
            return false;
        }

        PollStage stage{};
        if (!inferStageByRegCount_(regs.size(), stage)) {
            LOG_COMM_D("[AC][PARSE][DROP] unknown regs.size=%zu", regs.size());
            return false;
        }

        DeviceData d;
        d.device_name = "AirConditioner";
        d.value["__air_stage"] = static_cast<int32_t>(stage);
        d.value["__air_regs_count"] = static_cast<int32_t>(regs.size());

        fillSegmentData_(stage, regs, d);

        // 如果该长度合法但本段没有解析出任何字段，也认为无有效业务数据。
        // 这样可以避免空 DeviceData 把设备状态误判成有新数据。
        if (d.num.empty() && d.status.empty() && d.value.size() <= 2) {
            LOG_COMM_D("[AC][PARSE][DROP] empty segment stage=%d regs.size=%zu",
                       static_cast<int>(stage),
                       regs.size());
            return false;
        }

        out = std::move(d);
        return true;
    }

    // 写单寄存器 / 写多寄存器应答不是空调运行数据。
    if (func == 0x06) {
        return parseWriteEcho06_10_LoHi(rx, addr_, 0x06);
    }

    if (func == 0x10) {
        return parseWriteEcho06_10_LoHi(rx, addr_, 0x10);
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
    switch (off) {
    case 0x00: return "alarm.high_temp_alarm";
    case 0x01: return "alarm.low_temp_alarm";
    case 0x02: return "alarm.high_humidity_alarm";
    case 0x03: return "alarm.low_humidity_alarm";
    case 0x04: return "alarm.coil_freeze_protect";
    case 0x05: return "alarm.exhaust_high_temp_alarm";

    case 0x06: return "alarm.coil_temp_sensor_fault";
    case 0x07: return "alarm.outdoor_temp_sensor_fault";
    case 0x08: return "alarm.condenser_temp_sensor_fault";
    case 0x09: return "alarm.indoor_temp_sensor_fault";
    case 0x0A: return "alarm.exhaust_temp_sensor_fault";
    case 0x0B: return "alarm.humidity_sensor_fault";

    case 0x0C: return "alarm.internal_fan_fault";
    case 0x0D: return "alarm.external_fan_fault";
    case 0x0E: return "alarm.compressor_fault";
    case 0x0F: return "alarm.heater_fault";
    case 0x10: return "alarm.emergency_fan_fault";

    case 0x11: return "alarm.high_pressure_alarm";
    case 0x12: return "alarm.low_pressure_alarm";
    case 0x13: return "alarm.water_alarm";
    case 0x14: return "alarm.smoke_alarm";
    case 0x15: return "alarm.gating_alarm";

    case 0x16: return "alarm.high_pressure_lock";
    case 0x17: return "alarm.low_pressure_lock";
    case 0x18: return "alarm.exhaust_lock";

    case 0x19: return "alarm.ac_over_voltage_alarm";
    case 0x1A: return "alarm.ac_under_voltage_alarm";
    case 0x1B: return "alarm.ac_power_loss";
    case 0x1C: return "alarm.lose_phase_alarm";
    case 0x1D: return "alarm.freq_fault";
    case 0x1E: return "alarm.anti_phase_alarm";
    case 0x1F: return "alarm.dc_over_voltage_alarm";
    case 0x20: return "alarm.dc_under_voltage_alarm";

    default:   return "alarm.unknown";
    }
}

void AirConditionerProto::fillSegmentData_(PollStage s,
                                           const std::vector<uint16_t>& regs,
                                           DeviceData& out)
{
    out.device_name = "AirConditioner";

    switch (s) {
        case PollStage::S0_Version: {
            if (regs.size() >= 1) {
                out.num["version"] = static_cast<double>(regs[0]);
            }
        } break;

        case PollStage::S1_RunState: {
            if (regs.size() >= 6) {
                out.num["run.overall"]    = static_cast<double>(regs[0]);
                out.num["run.inner_fan"]  = static_cast<double>(regs[1]);
                out.num["run.outer_fan"]  = static_cast<double>(regs[2]);
                out.num["run.compressor"] = static_cast<double>(regs[3]);
                out.num["run.heater"]     = static_cast<double>(regs[4]);
                out.num["run.em_fan"]     = static_cast<double>(regs[5]);
            }
        } break;

        case PollStage::S2_Sensors: {
            if (regs.size() >= 9) {
                const int16_t coil10    = toS16(regs[0]);
                const int16_t out10     = toS16(regs[1]);
                const int16_t cond10    = toS16(regs[2]);
                const int16_t in10      = toS16(regs[3]);
                const int16_t hum       = toS16(regs[4]);
                const int16_t exhaust10 = toS16(regs[5]);
                const int16_t cur1000   = toS16(regs[6]);
                const int16_t acv       = toS16(regs[7]);
                const int16_t dcv10     = toS16(regs[8]);

                // 协议层只输出工程值。
                out.num["temp.coil_c"]      = divSafe(coil10, 10.0);
                out.num["temp.outdoor_c"]   = divSafe(out10, 10.0);
                out.num["temp.condense_c"]  = divSafe(cond10, 10.0);
                out.num["temp.indoor_c"]    = divSafe(in10, 10.0);
                out.num["humidity_percent"] = static_cast<double>(hum);
                out.num["temp.exhaust_c"]   = divSafe(exhaust10, 10.0);

                out.num["current_a"]        = divSafe(cur1000, 1000.0);
                out.num["ac_voltage_v"]     = static_cast<double>(acv);
                out.num["dc_voltage_v"]     = divSafe(dcv10, 10.0);
            }
        } break;

        case PollStage::S3_Alarms: {
            int any = 0;

            for (size_t i = 0; i < regs.size() && i <= 0x20; ++i) {
                const uint16_t v = regs[i];
                const char* key = alarmKeyByOffset_(static_cast<uint16_t>(i));

                out.status[key] = (v != 0) ? 1u : 0u;

                if (v != 0) {
                    any = 1;
                }
            }

            out.status["alarm.any"] = any ? 1u : 0u;
        } break;

        case PollStage::S4_Params: {
            if (regs.size() >= 11) {
                out.num["param.cool_point_c"]      = static_cast<double>(toS16(regs[0]));
                out.num["param.cool_hys_c"]        = static_cast<double>(toS16(regs[1]));
                out.num["param.heat_point_c"]      = static_cast<double>(toS16(regs[2]));
                out.num["param.heat_hys_c"]        = static_cast<double>(toS16(regs[3]));
                out.num["param.high_temp_c"]       = static_cast<double>(toS16(regs[6]));
                out.num["param.low_temp_c"]        = static_cast<double>(toS16(regs[7]));
                out.num["param.high_hum_pct"]      = static_cast<double>(toS16(regs[8]));
                out.num["param.inner_fan_stop_c"]  = static_cast<double>(toS16(regs[10]));
            }
        } break;


        default:
            break;
    }
}