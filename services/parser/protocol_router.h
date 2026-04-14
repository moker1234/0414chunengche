//
// Created by lxy on 2026/1/19.
//

#ifndef ENERGYSTORAGE_PROTOCOL_ROUTER_H
#define ENERGYSTORAGE_PROTOCOL_ROUTER_H

#pragma once
#include <memory>
#include <map>

#include "can_dispatcher.h"
#include "../protocol/rs485/gas_detector_proto.h"
#include "../protocol/rs485/smoke_sensor_proto.h"
#include "../protocol/rs485/air_conditioner_proto.h"
#include "../protocol/rs232/ups_232_ascii_proto.h"
#include "../protocol/can/can_dispatcher.h"   // 确保能看到 CanParseResult
#include "../protocol/can/bms/bms_thread/bms_queue.h"

#include "crc_policy.h"
#include "hmi/hmi_proto.h"

namespace parser {

    enum class Rs485DeviceType {
        Gas,
        Smoke,
        AirConditioner,
        HMI
    };

    struct Rs485DeviceCfg {
        Rs485DeviceType type;
        uint8_t addr;
        CrcOrder crc;
    };

    class ProtocolRouter {
    public:
        ProtocolRouter();

        bool parseRs485(int index,
                        const std::vector<uint8_t>& frame,
                        DeviceData& out);

        std::vector<uint8_t> buildQuery(int index);

        bool parseRs232(int index,
                        const std::vector<uint8_t>& frame,
                        DeviceData& out);

        std::vector<uint8_t> buildQueryRs232(int index);

        // ===== HMI 从站入口：输入 Modbus RTU request，输出 response（可能为空=不回包）=====
        bool handleRs485Slave(int index,
                      const std::vector<uint8_t>& frame,
                      std::vector<uint8_t>& resp);

        bool isRs485SlavePort(int index) const;

        // ===== HMI 从站入口：返回 HMIProto 指针 =====
        HMIProto* getHmiProto(int index);

        proto::CanParseResult parseCan(int can_index,
                      const can_frame& frame,
                      DeviceData& out);

        // 为了线程
        void setBmsQueue(int bms_can_index, proto::bms::BmsQueue* q);
    private:
        std::map<int, Rs485DeviceCfg> cfg_;
        std::map<int, std::unique_ptr<ProtocolBase>> proto_;
        std::map<int, std::unique_ptr<ProtocolBase>> proto_rs232_;

        proto::CanDispatcher can_disp_;

    };



} // namespace parser

#endif // ENERGYSTORAGE_PROTOCOL_ROUTER_H
