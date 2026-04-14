//
// Created by lxy on 2026/2/13.
//

#ifndef ENERGYSTORAGE_BMS_PROTO_H
#define ENERGYSTORAGE_BMS_PROTO_H

#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>

#include "can_protocol_base.h"
#include "proto_bms_table_runtime.h"

namespace proto::bms {

    /**
     * BMS 协议解析器（表驱动）
     *
     * 本批目标：
     * 1. 支持 canhub 改写后的 4 路实例 ID
     * 2. 输出实例元信息 __bms.instance / __bms.instance_index
     * 3. 输出原始帧 __bms.raw_hex
     * 4. 保留全量解析字段，为后续 logic 接入做准备
     */
    class BmsProto final : public proto::CanProtocolBase {
    public:
        explicit BmsProto(std::string device_name = "BMS");

        std::string name() const override { return "BMS_TABLE"; }

        bool parse(const can_frame& fr, DeviceData& out) override;

        void setKeyPrefix(std::string prefix) { key_prefix_ = std::move(prefix); }

    private:
        static uint32_t getId29(const can_frame& fr);

        static uint64_t extractBitsLsb(const uint8_t data[8], int start_lsb, int len);
        static std::string bytesToHex(const uint8_t data[8], uint8_t dlc);

    private:
        std::string device_name_;
        std::string key_prefix_{"bms"}; // 输出 key 前缀：bms.<Msg>.<Sig>
    };

} // namespace proto::bms


#endif //ENERGYSTORAGE_BMS_PROTO_H