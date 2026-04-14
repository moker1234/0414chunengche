//
// Created by forlinx on 2025/12/28.
//

#include "can_dispatcher.h"
#include "logger.h"

// 第一个协议实现
#include "bms_thread/bms_queue.h"
#include "pcu/proto_pcu.h"
#include "getTime.h"

namespace proto {

    void CanDispatcher::init(const SystemConfig& sys)
    {
        proto_by_can_.clear();

        // 需要你在 SystemConfig 增加 sys.can_links（见下方 config_loader 修改说明）
        for (const auto& l : sys.can_links) {
            if (!l.enable) continue;

            if (l.protocol_type == "emu_pcu_v1") {
                proto_by_can_[l.can_index] = std::make_unique<proto::pcu::ProtoPCU>(
                    l.id_pcu_status,   // rx id
                    l.id_emu_ctrl,     // tx id (可不用，但保留)
                    l.id_emu_status    // tx id (可不用，但保留)
                );

                LOG_SYS_I("[CAN_DISP] bind can_index=%d protocol=%s",
                          l.can_index, l.protocol_type.c_str());
            } else if (l.protocol_type == "bms_table_v1") {
                // ★ BMS：基于 generated/proto_bms_table.* 的表驱动解析
                // 注意：BMS 协议不需要固定 rx id（表里有所有 MsgID），所以不依赖 l.id_* 字段
                proto_by_can_[l.can_index] = std::make_unique<proto::bms::BmsProto>("BMS");

                LOG_SYS_I("[CAN_DISP] bind can_index=%d protocol=%s",
                          l.can_index, l.protocol_type.c_str());
            } else {
                LOGERR("[CAN_DISP] unknown protocol_type=%s can_index=%d",
                       l.protocol_type.c_str(), l.can_index);
            }
        }
    }

    bool CanDispatcher::parse(int can_index, const can_frame& fr, DeviceData& out)
    {
        auto it = proto_by_can_.find(can_index);
        if (it == proto_by_can_.end() || !it->second)
        {
            printf("parse can_index=%d failed, proto=%p, end=%p\n", can_index, it->second.get(), proto_by_can_.end());
            return false;
        }
        return it->second->parse(fr, out);
    }

    void CanDispatcher::setBmsAsync(int bms_can_index, bms::BmsQueue* q)
    {
        bms_can_index_ = bms_can_index;
        bms_q_ = q;
        LOG_COMM_D("[CAN_DISP] BMS async enabled on can_index=%d (q=%p)",
                  bms_can_index_, (void*)bms_q_);
    }

    CanParseResult CanDispatcher::handle(int can_index, const can_frame& fr, DeviceData& out)
    {
        // ===== BMS：异步入队（不做同步解析）=====
        if (bms_q_ && can_index == bms_can_index_) {
            // 只处理扩展帧（29bit）
            if ((fr.can_id & CAN_EFF_FLAG) == 0 || fr.can_dlc < 8) {
                return CanParseResult::Consumed; // 不报错，直接吞掉（避免刷 ParseError）
            }

            proto::bms::BmsFrame bf;
            bf.can_index = can_index;
            bf.id29 = static_cast<uint32_t>(fr.can_id & CAN_EFF_MASK);
            bf.dlc  = fr.can_dlc;
            bf.ts_ms = nowMs();
            for (int i = 0; i < 8; ++i) bf.data[(size_t)i] = fr.data[i];

            // LOG_COMM_D("[BMS][QUEUE_PUSH] can=%d id=0x%08X dlc=%d q=%p",
            //            can_index, (unsigned)(fr.can_id & CAN_EFF_MASK), fr.can_dlc, (void*)bms_q_);
            bms_q_->push(bf);
            // LOG_COMM_D("[BMS][QUEUE_PUSH] can=%d id=0x%08X dlc=%d q=%p",
            //            can_index, (unsigned)(fr.can_id & CAN_EFF_MASK), fr.can_dlc, (void*)bms_q_);
            return CanParseResult::Consumed;
        }

        // ===== 其它 CAN 协议：同步解析 =====
        auto it = proto_by_can_.find(can_index);
        if (it == proto_by_can_.end() || !it->second) return CanParseResult::NoProto;

        if (it->second->parse(fr, out)) return CanParseResult::Parsed;
        return CanParseResult::ParseFail;
    }

} // namespace proto
