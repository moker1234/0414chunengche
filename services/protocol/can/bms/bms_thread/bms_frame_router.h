#pragma once

#include <cstdint>
#include <string>

namespace proto::bms {

/**
 * BmsFrameRouter
 *
 * 唯一职责：
 * 1. 解释 canhub 改写后的 BMS 运行态 ID
 * 2. 从运行态 ID 提取 BMS 实例号和报文号
 * 3. 在 “协议模板 ID / generated 表 ID / canhub 运行态 ID” 之间转换
 *
 * canhub 运行态规则：
 *   低 12bit = [instance:4bit][msg_code:8bit]
 *
 * 示例：
 *   0x1883E104 -> instance=1, msg_code=0x04 -> B2V_ST1
 *   0x1883E204 -> instance=2, msg_code=0x04 -> B2V_ST1
 *   0x18FFC20E -> instance=2, msg_code=0x0E -> TM2B_Info
 *
 * msg_code 规则：
 *   0x01 -> V2B_CMD
 *   0x02 -> B2V_Fult1_32960
 *   0x03 -> B2V_Fult2
 *   0x04 -> B2V_ST1
 *   0x05 -> B2V_ST2
 *   0x06 -> B2V_ST3
 *   0x07 -> B2V_ST4
 *   0x08 -> B2V_ST5
 *   0x09 -> B2V_ST6
 *   0x0A -> B2V_ST7
 *   0x0B -> B2V_ElecEnergy
 *   0x0C -> B2V_CurrentLimit
 *   0x0D -> B2TM_Info
 *   0x0E -> TM2B_Info
 *   0x0F -> Fire2B_Info / Fire2B_state
 */
class BmsFrameRouter {
public:
    static constexpr uint32_t kCanId29Mask = 0x1FFFFFFFu;

    static uint32_t extractInstanceIndex(uint32_t id29);
    static uint32_t extractMsgCode(uint32_t id29);

    static bool isValidInstance(uint32_t idx);
    static bool decodeRuntimeId(uint32_t runtime_id29,
                                uint32_t& instance,
                                uint32_t& msg_code);
    static bool isRuntimeId(uint32_t id29);

    static std::string makeInstanceName(uint32_t idx);

    // msg_code -> generated 表可查找的消息 ID。
    // 注意：当前 generated 表里 Fire2B_state 是 0x18FFF10F。
    static uint32_t protoIdFromMsgCode(uint32_t msg_code);

    // 任何已知 ID -> msg_code。
    // 支持：
    //   1) 原始协议模板 ID，例如 0x1883EFF3
    //   2) canhub 运行态 ID，例如 0x1883E104
    //   3) 当前 generated 表中 Fire2B_state 的 0x18FFF10F
    static uint32_t msgCodeFromProtoId(uint32_t proto_id29);

    // 任何已知 ID -> generated 表可查找的消息 ID。
    // 例如：
    //   0x1883E204 -> 0x1883EFF3
    //   0x18FFC20E -> 0x18FFC13A
    //   0x18FFF20F -> 0x18FFF10F
    static uint32_t normalizeToProtoId(uint32_t id29);

    // 用模板 ID / generated 表 ID / 运行态 ID + 实例号，得到 canhub 运行态 ID。
    // 例如：
    //   0x1802F3EF + inst=1 -> 0x1802F101
    //   0x1802F3EF + inst=4 -> 0x1802F401
    //   0x1883EFF3 + inst=2 -> 0x1883E204
    static uint32_t rewriteRuntimeId(uint32_t proto_id29,
                                     uint32_t instance_index);

private:
    static uint32_t clean29_(uint32_t id29)
    {
        return id29 & kCanId29Mask;
    }

    static uint32_t low12_(uint32_t id29)
    {
        return clean29_(id29) & 0xFFFu;
    }
};

} // namespace proto::bms