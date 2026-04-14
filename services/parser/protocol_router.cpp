//
// Created by lxy on 2026/1/19.
//

#include "protocol_router.h"

#include "../protocol/rs485/hmi/hmi_proto.h"
#include "logger.h"

using namespace parser;

#include "config_loader.h"
ProtocolRouter::ProtocolRouter() {
    SystemConfig sys{};
    std::string err;
    if (!ConfigLoader::loadSystem("/home/zlg/userdata/config/system.json", sys, err)) {
        LOGERR("[ROUTER] load system.json failed: %s", err.c_str());
        // 回退到原硬编码（建议保留）
        cfg_[0] = {Rs485DeviceType::Gas,            0x01, CrcOrder::HiLo};
        cfg_[1] = {Rs485DeviceType::Smoke,          0x02, CrcOrder::HiLo};
        cfg_[2] = {Rs485DeviceType::AirConditioner, 0x01, CrcOrder::LoHi};
        cfg_[3] = {Rs485DeviceType::HMI,            0x01, CrcOrder::LoHi};

        proto_[0] = std::make_unique<GasDetectorProto>(cfg_[0].addr);
        proto_[1] = std::make_unique<SmokeSensorProto>(cfg_[1].addr);
        proto_[2] = std::make_unique<AirConditionerProto>(cfg_[2].addr);
        proto_[3] = std::make_unique<HMIProto>(cfg_[3].addr);

        proto_rs232_[0] = std::make_unique<Ups232AsciiProto>();
        return;
    }

    // 先清空
    cfg_.clear();
    proto_.clear();
    proto_rs232_.clear();

    for (const auto& l : sys.rs485_links) {
        // crc_order 字符串转枚举
        CrcOrder crc = (l.crc_order == "lohi") ? CrcOrder::LoHi : CrcOrder::HiLo;

        if (l.type == Rs485ProtoType::Gas) {
            cfg_[l.link_index] = {Rs485DeviceType::Gas, l.slave_id, crc};
            proto_[l.link_index] = std::make_unique<GasDetectorProto>(l.slave_id);
        } else if (l.type == Rs485ProtoType::Smoke) {
            cfg_[l.link_index] = {Rs485DeviceType::Smoke, l.slave_id, crc};
            proto_[l.link_index] = std::make_unique<SmokeSensorProto>(l.slave_id);
        } else if (l.type == Rs485ProtoType::AirConditioner) {
            cfg_[l.link_index] = {Rs485DeviceType::AirConditioner, l.slave_id, crc};
            proto_[l.link_index] = std::make_unique<AirConditionerProto>(l.slave_id);
        } else if (l.type == Rs485ProtoType::Hmi) {
            cfg_[l.link_index] = {Rs485DeviceType::HMI, l.slave_id, crc};
            proto_[l.link_index] = std::make_unique<HMIProto>(l.slave_id);
        } else {
            LOGERR("[ROUTER] unknown rs485 type at link_index=%d", l.link_index);
        }
    }

    // rs232
    for (const auto& l : sys.rs232_links) {
        if (l.type == "ups_ascii") {
            proto_rs232_[l.link_index] = std::make_unique<Ups232AsciiProto>();
        }
    }

    can_disp_.init(sys);
}

bool ProtocolRouter::parseRs485(int index,
                                const std::vector<uint8_t>& frame,
                                DeviceData& out) {
    if (frame.size() < 2) return false;

    // ★ 关键：优先按 “串口index” 路由（避免不同端口同地址冲突）
    auto it = proto_.find(index);
    if (it != proto_.end() && it->second) {
        return it->second->parse(frame, out);
    }

    // 兜底：按 slave addr 匹配（保留兼容）
    const uint8_t slave = frame[0];
    for (auto& kv : proto_) {
        if (kv.second && kv.second->slaveAddr() == slave) {
            return kv.second->parse(frame, out);
        }
    }
    return false;
}

std::vector<uint8_t> ProtocolRouter::buildQuery(int index) {
    auto it = proto_.find(index);
    if (it == proto_.end() || !it->second) return {};
    return it->second->buildReadCmd();
}

bool ProtocolRouter::parseRs232(int index,
                                const std::vector<uint8_t>& frame,
                                DeviceData& out) {
    // LOGD("[ROUTER] parse RS232 idx=%d len=%zu", index, frame.size());

    auto it = proto_rs232_.find(index);
    if (it == proto_rs232_.end() || !it->second) return false;
    return it->second->parse(frame, out);
}

std::vector<uint8_t> ProtocolRouter::buildQueryRs232(int index) {
    auto it = proto_rs232_.find(index);
    if (it == proto_rs232_.end() || !it->second) return {};
    return it->second->buildReadCmd();
}

bool ProtocolRouter::handleRs485Slave(int index,
                                      const std::vector<uint8_t>& frame,
                                      std::vector<uint8_t>& resp)
{
    resp.clear();

    auto it = proto_.find(index);
    if (it == proto_.end() || !it->second) return false;

    // ✅ 统一走虚接口：由具体协议决定是否“吞掉”/是否回包
    return it->second->handleSlaveRequest(frame, resp);
}
bool ProtocolRouter::isRs485SlavePort(int index) const {
    auto it = cfg_.find(index);
    if (it == cfg_.end()) return false;
    return it->second.type == Rs485DeviceType::HMI;
}


HMIProto* ProtocolRouter::getHmiProto(int index) {
    auto it = proto_.find(index);
    if (it == proto_.end() || !it->second) return nullptr;
    if (!isRs485SlavePort(index)) return nullptr; // 你现在用 type==HMI 判断
    return static_cast<HMIProto*>(it->second.get());
}

void ProtocolRouter::setBmsQueue(int bms_can_index, proto::bms::BmsQueue* q)
{
    can_disp_.setBmsAsync(bms_can_index, q);
}


proto::CanParseResult ProtocolRouter::parseCan(int can_index,
                              const can_frame& frame,
                              DeviceData& out)
{
    return can_disp_.handle(can_index, frame, out);
}


