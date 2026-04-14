#include "bms_frame_router.h"

namespace proto::bms {
    static uint32_t msgCodeFromProtoId_(uint32_t proto_id29)
    {
        switch (proto_id29) {
        case 0x1802F3EFu: return 0x01; // V2B_CMD
        case 0x1881EFF3u: return 0x02; // B2V_Fault1
        case 0x1882EFF3u: return 0x03; // B2V_Fault2
        case 0x1883EFF3u: return 0x04; // B2V_ST1
        case 0x1884EFF3u: return 0x05; // B2V_ST2
        case 0x1885EFF3u: return 0x06; // B2V_ST3
        case 0x1886EFF3u: return 0x07; // B2V_ST4
        case 0x1887EFF3u: return 0x08; // B2V_ST5
        case 0x1888EFF3u: return 0x09; // B2V_ST6
        case 0x1889EFF3u: return 0x0A; // B2V_ST7
        case 0x18C3EFF3u: return 0x0B; // B2V_ElecEnergy
        case 0x18C4EFF3u: return 0x0C; // B2V_CurrentLimit
        case 0x18FF45F4u: return 0x0D; // B2TM_Info
        case 0x18FFC13Au: return 0x0E; // TM2B_Info
        case 0x18FFF7F6u: return 0x0F; // Fire2B_Info
        default: return 0u;
        }
    }

    uint32_t BmsFrameRouter::extractInstanceIndex(uint32_t id29)
    {
        const uint32_t v = low12(id29);      // 例如 0x10E / 0x204 / 0x30A
        const uint32_t idx = (v >> 8) & 0xFu;
        return (idx >= 1 && idx <= 4) ? idx : 0u;
    }

    uint32_t BmsFrameRouter::extractMsgCode(uint32_t id29)
    {
        return low12(id29) & 0xFFu;
    }

    bool BmsFrameRouter::isValidInstance(uint32_t idx)
    {
        return idx >= 1 && idx <= 4;
    }

    std::string BmsFrameRouter::makeInstanceName(uint32_t idx)
    {
        if (!isValidInstance(idx)) return "BMS_0";
        return "BMS_" + std::to_string(idx);
    }

    // uint32_t BmsFrameRouter::rewriteRuntimeId(uint32_t proto_id29, uint32_t instance_index)
    // {
    //     if (!isValidInstance(instance_index)) {
    //         return proto_id29;
    //     }
    //
    //     // 协议原模板的最后 12bit 形如：
    //     //   ...FF3 / ...F4 / ...13A / ...7F6
    //     // canhub 运行态实际需要：
    //     //   最后三位 = [instance][msg_code]
    //     //
    //     // 这里统一取协议模板最低 8bit 作为 msg_code，再把高 4bit 填成实例号。
    //     const uint32_t msg_code = proto_id29 & 0xFFu;
    //     const uint32_t new_low12 = ((instance_index & 0xFu) << 8) | msg_code;
    //     return (proto_id29 & ~0xFFFu) | new_low12;
    // }
    uint32_t BmsFrameRouter::rewriteRuntimeId(uint32_t proto_id29, uint32_t instance_index)
    {
        if (!isValidInstance(instance_index)) {
            return proto_id29;
        }

        const uint32_t msg_code = msgCodeFromProtoId_(proto_id29);
        if (msg_code == 0u) {
            return proto_id29;
        }

        const uint32_t new_low12 = ((instance_index & 0xFu) << 8) | msg_code;
        return (proto_id29 & ~0xFFFu) | new_low12;
    }

} // namespace proto::bms