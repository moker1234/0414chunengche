//
// Created by lxy on 2025/12/28.
//

#include "can_dispatcher.h"

#include <string>
#include <cstdio>

#include "logger.h"
#include "getTime.h"

#include "bms/bms_thread/bms_queue.h"
#include "pcu/proto_pcu.h"

namespace proto {

namespace {

static std::string pcuRuntimeName_(uint8_t pcu_instance)
{
    // 配置实例号：1/2
    // 程序内部名：PCU_0/PCU_1
    if (pcu_instance == 1) return "PCU_0";
    if (pcu_instance == 2) return "PCU_1";
    return {};
}
    static std::string canFrameRawHex_(const can_frame& fr)
{
    char buf[4];
    std::string s;
    s.reserve(8 * 3);

    const uint8_t n = (fr.can_dlc > 8) ? 8 : fr.can_dlc;
    for (uint8_t i = 0; i < n; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", fr.data[i]);
        if (!s.empty()) s.push_back(' ');
        s.append(buf);
    }

    return s;
}

} // namespace

void CanDispatcher::init(const SystemConfig& sys)
{
    proto_by_can_.clear();
    pcu_instance_by_can_.clear();

    // 注意：
    // BMS 不再在这里创建同步 BmsProto。
    // BMS 的实际上行链路是：
    //   CanDispatcher::handle()
    //      -> BmsQueue
    //      -> BmsWorker
    //      -> BmsProto
    //
    // 这样可以避免 “同步 BmsProto” 与 “BmsWorker 专线” 两套路径并存。
    for (const auto& l : sys.can_links) {
        if (!l.enable) continue;

        if (l.protocol_type == "emu_pcu_v1") {
            /*
             * ProtoPCU 只负责 PCU->EMU 状态反馈解析。
             * EMU->PCU 控制帧/状态帧由 LogicEngine 使用 buildEmuCtrl/buildEmuStatus 生成。
             */
            proto_by_can_[l.can_index] =
                std::make_unique<proto::pcu::ProtoPCU>(l.id_pcu_status);

            if (l.pcu_instance >= 1 && l.pcu_instance <= 2) {
                pcu_instance_by_can_[l.can_index] = l.pcu_instance;
            } else {
                LOG_SYS_W("[CAN_DISP][PCU] can_index=%d has no valid pcu_instance, "
                          "PCU device name will not be normalized safely",
                          l.can_index);
            }

            LOG_SYS_I("[CAN_DISP] bind can_index=%d protocol=%s sync=1 pcu_instance=%u rx_id=0x%08X",
                      l.can_index,
                      l.protocol_type.c_str(),
                      static_cast<unsigned>(l.pcu_instance),
                      static_cast<unsigned>(l.id_pcu_status));
        }
        else if (l.protocol_type == "bms_table_v1") {
            // BMS 只走异步队列，不创建 proto_by_can_ 项。
            // setBmsAsync() 会在 AppManager::init() 中注入 BmsQueue。
            LOG_SYS_I("[CAN_DISP] reserve can_index=%d protocol=%s async=1",
                      l.can_index, l.protocol_type.c_str());
        }
        else {
            LOGERR("[CAN_DISP] unknown protocol_type=%s can_index=%d",
                   l.protocol_type.c_str(), l.can_index);
        }
    }
}

void CanDispatcher::stampParsedMeta_(int can_index,
                                     const can_frame& fr,
                                     DeviceData& out) const
{
    const uint32_t id29 = static_cast<uint32_t>(fr.can_id & CAN_EFF_MASK);

    // ===== 通用 CAN 元信息 =====
    out.value["__can_index"] = can_index;
    out.value["__can_id"] = static_cast<int32_t>(id29);
    out.value["__can_dlc"] = static_cast<int32_t>(fr.can_dlc);

    out.str["__can.raw_hex"] = canFrameRawHex_(fr);

    // PCU 协议层已经会写 __pcu.raw_hex。
    // 这里做一次兜底，避免后续如果换协议实现时丢元信息。
    if ((out.device_name == "PCU" || out.device_name == "PCU_CTRL") &&
        out.str.find("__pcu.raw_hex") == out.str.end()) {
        out.str["__pcu.raw_hex"] = out.str["__can.raw_hex"];
    }

    if (out.device_name != "PCU" && out.device_name != "PCU_CTRL") {
        return;
    }

    // ===== PCU 实例元信息 =====
    auto it = pcu_instance_by_can_.find(can_index);
    if (it == pcu_instance_by_can_.end()) {
        LOG_THROTTLE_MS("pcu_no_instance_meta", 1000, LOG_COMM_W,
                        "[CAN_DISP][PCU] parsed PCU on can_index=%d id=0x%08X but no pcu_instance configured raw=%s",
                        can_index,
                        static_cast<unsigned>(id29),
                        out.str["__can.raw_hex"].c_str());
        return;
    }

    const uint8_t inst = it->second;
    if (inst < 1 || inst > 2) {
        LOG_THROTTLE_MS("pcu_bad_instance_meta", 1000, LOG_COMM_W,
                        "[CAN_DISP][PCU] parsed PCU on can_index=%d id=0x%08X invalid pcu_instance=%u raw=%s",
                        can_index,
                        static_cast<unsigned>(id29),
                        static_cast<unsigned>(inst),
                        out.str["__can.raw_hex"].c_str());
        return;
    }

    const std::string runtime_name = pcuRuntimeName_(inst);
    if (runtime_name.empty()) {
        return;
    }

    out.value["__pcu.instance"] = static_cast<int32_t>(inst);
    out.value["__pcu.runtime_index"] = static_cast<int32_t>(inst - 1);
    out.value["__pcu.can_index"] = can_index;

    out.str["__pcu.instance_name"] = runtime_name;                 // PCU_0 / PCU_1
    out.str["__pcu.display_name"]  = "PCU" + std::to_string(inst); // PCU1 / PCU2

    if (out.str.find("__pcu.msg") == out.str.end()) {
        out.str["__pcu.msg"] = "PCU_STATUS";
    }

    LOG_THROTTLE_MS("pcu_rx_parsed_probe", 2000, LOG_COMM_D,
                    "[PCU][RX][PARSED] can=%d inst=%u name=%s id=0x%08X hb=%d estop=%u state=%d cab=%d raw=%s",
                    can_index,
                    static_cast<unsigned>(inst),
                    runtime_name.c_str(),
                    static_cast<unsigned>(id29),
                    static_cast<int>(out.value.count("heartbeat") ? out.value.at("heartbeat") : -1),
                    static_cast<unsigned>(out.status.count("estop") ? out.status.at("estop") : 0u),
                    static_cast<int>(out.value.count("pcu_state") ? out.value.at("pcu_state") : -1),
                    static_cast<int>(out.value.count("cabinet_id") ? out.value.at("cabinet_id") : -1),
                    out.str["__can.raw_hex"].c_str());
}

bool CanDispatcher::parse(int can_index, const can_frame& fr, DeviceData& out)
{
    auto it = proto_by_can_.find(can_index);
    if (it == proto_by_can_.end() || !it->second) {
        LOG_THROTTLE_MS("can_parse_no_proto", 1000, LOG_COMM_D,
                        "[CAN_DISP] parse no proto can_index=%d id=0x%08X",
                        can_index,
                        static_cast<unsigned>(fr.can_id & CAN_EFF_MASK));
        return false;
    }

    if (!it->second->parse(fr, out)) {
        return false;
    }

    stampParsedMeta_(can_index, fr, out);
    return true;
}

void CanDispatcher::setBmsAsync(int bms_can_index, bms::BmsQueue* q)
{
    bms_can_index_ = bms_can_index;
    bms_q_ = q;

    LOG_SYS_I("[CAN_DISP] BMS async enabled can_index=%d q=%p",
              bms_can_index_, static_cast<void*>(bms_q_));
}

CanParseResult CanDispatcher::handle(int can_index, const can_frame& fr, DeviceData& out)
{
    // ===== BMS：异步入队，不做同步解析 =====
    if (bms_q_ && can_index == bms_can_index_) {
        // BMS 协议只接收 29bit 扩展帧；非扩展帧直接吞掉，避免刷 ParseError。
        if ((fr.can_id & CAN_EFF_FLAG) == 0) {
            LOG_THROTTLE_MS("bms_drop_std_frame", 1000, LOG_COMM_D,
                            "[BMS][QUEUE_DROP] std frame can=%d id=0x%08X dlc=%u",
                            can_index,
                            static_cast<unsigned>(fr.can_id),
                            static_cast<unsigned>(fr.can_dlc));
            return CanParseResult::Consumed;
        }

        // BMS 表驱动协议目前按 8 字节帧处理；短帧直接吞掉。
        if (fr.can_dlc < 8) {
            LOG_THROTTLE_MS("bms_drop_short_frame", 1000, LOG_COMM_D,
                            "[BMS][QUEUE_DROP] short frame can=%d id=0x%08X dlc=%u",
                            can_index,
                            static_cast<unsigned>(fr.can_id & CAN_EFF_MASK),
                            static_cast<unsigned>(fr.can_dlc));
            return CanParseResult::Consumed;
        }

        proto::bms::BmsFrame bf;
        bf.can_index = can_index;
        bf.id29 = static_cast<uint32_t>(fr.can_id & CAN_EFF_MASK);
        bf.dlc  = fr.can_dlc;
        bf.ts_ms = nowMs();

        for (int i = 0; i < 8; ++i) {
            bf.data[static_cast<size_t>(i)] = fr.data[i];
        }

        bms_q_->push(bf);
        return CanParseResult::Consumed;
    }

    // ===== 其它 CAN 协议：同步解析 =====
    auto it = proto_by_can_.find(can_index);
    if (it == proto_by_can_.end() || !it->second) {
        return CanParseResult::NoProto;
    }

    if (it->second->parse(fr, out)) {
        stampParsedMeta_(can_index, fr, out);
        return CanParseResult::Parsed;
    }

    return CanParseResult::ParseFail;
}

} // namespace proto