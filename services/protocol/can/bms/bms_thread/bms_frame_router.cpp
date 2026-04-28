#include "bms_frame_router.h"

namespace proto::bms {

uint32_t BmsFrameRouter::extractInstanceIndex(uint32_t id29)
{
    const uint32_t low12 = low12_(id29);
    const uint32_t idx = (low12 >> 8) & 0xFu;

    return isValidInstance(idx) ? idx : 0u;
}

uint32_t BmsFrameRouter::extractMsgCode(uint32_t id29)
{
    return low12_(id29) & 0xFFu;
}

bool BmsFrameRouter::isValidInstance(uint32_t idx)
{
    return idx >= 1u && idx <= 4u;
}

bool BmsFrameRouter::decodeRuntimeId(uint32_t runtime_id29,
                                     uint32_t& instance,
                                     uint32_t& msg_code)
{
    instance = extractInstanceIndex(runtime_id29);
    msg_code = extractMsgCode(runtime_id29);

    if (!isValidInstance(instance)) {
        instance = 0u;
        msg_code = 0u;
        return false;
    }

    if (protoIdFromMsgCode(msg_code) == 0u) {
        instance = 0u;
        msg_code = 0u;
        return false;
    }

    return true;
}

bool BmsFrameRouter::isRuntimeId(uint32_t id29)
{
    uint32_t inst = 0;
    uint32_t code = 0;
    return decodeRuntimeId(id29, inst, code);
}

std::string BmsFrameRouter::makeInstanceName(uint32_t idx)
{
    if (!isValidInstance(idx)) {
        return "BMS_0";
    }

    return "BMS_" + std::to_string(idx);
}

uint32_t BmsFrameRouter::protoIdFromMsgCode(uint32_t msg_code)
{
    switch (msg_code) {
    case 0x01: return 0x1802F3EFu; // V2B_CMD

    case 0x02: return 0x1881EFF3u; // B2V_Fult1_32960
    case 0x03: return 0x1882EFF3u; // B2V_Fult2
    case 0x04: return 0x1883EFF3u; // B2V_ST1
    case 0x05: return 0x1884EFF3u; // B2V_ST2
    case 0x06: return 0x1885EFF3u; // B2V_ST3
    case 0x07: return 0x1886EFF3u; // B2V_ST4
    case 0x08: return 0x1887EFF3u; // B2V_ST5
    case 0x09: return 0x1888EFF3u; // B2V_ST6
    case 0x0A: return 0x1889EFF3u; // B2V_ST7

    case 0x0B: return 0x18C3EFF3u; // B2V_ElecEnergy
    case 0x0C: return 0x18C4EFF3u; // B2V_CurrentLimit

    case 0x0D: return 0x18FF45F4u; // B2TM_Info
    case 0x0E: return 0x18FFC13Au; // TM2B_Info

    // 注意：
    // 你的原始规则里 Fire2B_Info 原 ID 是 0x18FFF7F6，
    // canhub 后 BMS1 是 0x18FFF10F。
    // 但当前 generated/proto_bms_table.cpp 里 Fire2B_state 的 ID 是 0x18FFF10F。
    // 为了让 ProtoBmsTableRuntime::findMessage() 能查到表，这里返回当前 generated 表中的 ID。
    // 如果后续重新生成表并改回 0x18FFF7F6，只改这里即可。
    case 0x0F: return 0x18FFF10Fu; // Fire2B_state / Fire2B_Info

    default:
        return 0u;
    }
}

uint32_t BmsFrameRouter::msgCodeFromProtoId(uint32_t proto_id29)
{
    const uint32_t id = clean29_(proto_id29);

    // 1) 原始协议模板 ID / generated 表 ID
    switch (id) {
    case 0x1802F3EFu: return 0x01; // V2B_CMD

    case 0x1881EFF3u: return 0x02; // B2V_Fult1_32960
    case 0x1882EFF3u: return 0x03; // B2V_Fult2
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

    // 同时识别“原始规则 ID”和“当前 generated 表 ID”
    case 0x18FFF7F6u: return 0x0F; // Fire2B_Info 原始 ID
    case 0x18FFF10Fu: return 0x0F; // Fire2B_state 当前 generated ID / BMS1 runtime ID

    default:
        break;
    }

    // 2) canhub 运行态 ID
    uint32_t inst = 0;
    uint32_t code = 0;
    if (decodeRuntimeId(id, inst, code)) {
        return code;
    }

    return 0u;
}

uint32_t BmsFrameRouter::normalizeToProtoId(uint32_t id29)
{
    const uint32_t id = clean29_(id29);
    const uint32_t msg_code = msgCodeFromProtoId(id);
    const uint32_t proto_id = protoIdFromMsgCode(msg_code);

    return proto_id != 0u ? proto_id : id;
}

uint32_t BmsFrameRouter::rewriteRuntimeId(uint32_t proto_id29,
                                          uint32_t instance_index)
{
    const uint32_t id = clean29_(proto_id29);

    if (!isValidInstance(instance_index)) {
        return id;
    }

    const uint32_t msg_code = msgCodeFromProtoId(id);
    if (msg_code == 0u) {
        return id;
    }

    const uint32_t table_id = protoIdFromMsgCode(msg_code);
    if (table_id == 0u) {
        return id;
    }

    const uint32_t new_low12 =
        ((instance_index & 0xFu) << 8) |
        (msg_code & 0xFFu);

    return (table_id & ~0xFFFu) | new_low12;
}

} // namespace proto::bms