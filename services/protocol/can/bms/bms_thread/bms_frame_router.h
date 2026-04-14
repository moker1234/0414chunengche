#pragma once

#include <cstdint>
#include <string>

namespace proto::bms {

    /**
     * BmsFrameRouter：
     * 负责从 canhub 改写后的 29bit CAN ID 中提取：
     * 1. BMS 实例号（1~4）
     * 2. 报文编码（最后两位）
     *
     * 当前规则（与你现在目标 JSON 保持一致）：
     * - 最后三位中：
     *   - 第一位：实例号 1/2/3/4
     *   - 后两位：报文码 01~0F
     *
     * 例如：
     *   0x1883E104 -> instance=1, msg_code=0x04
     *   0x1883E204 -> instance=2, msg_code=0x04
     *   0x18FFC20E -> instance=2, msg_code=0x0E
     */
    class BmsFrameRouter {
    public:
        static uint32_t extractInstanceIndex(uint32_t id29);
        static uint32_t extractMsgCode(uint32_t id29);

        static bool isValidInstance(uint32_t idx);
        static std::string makeInstanceName(uint32_t idx);

        // 用 canonical 模板 ID + 实例号，得到 canhub 改写后的实际 ID
        // 例如：
        //   0x1883EFF3 + inst=2 -> 0x1883E204
        static uint32_t rewriteRuntimeId(uint32_t proto_id29, uint32_t instance_index);


        static bool decodeRuntimeId(uint32_t runtime_id29, uint32_t& instance, uint32_t& msg_code);
        static uint32_t protoIdFromMsgCode(uint32_t msg_code);
        static uint32_t msgCodeFromProtoId(uint32_t proto_id29);
    private:
        static uint32_t low12(uint32_t id29) { return (id29 & 0xFFFu); }
    };

} // namespace proto::bms