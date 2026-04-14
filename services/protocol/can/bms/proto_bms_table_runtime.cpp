#include "proto_bms_table_runtime.h"

#include <cmath>
#include <cstring>

#include "bms_name_map.h"
#include "bms_thread/bms_frame_router.h"

namespace proto::bms {

const proto_bms_gen::MessageDef* ProtoBmsTableRuntime::findMessage(uint32_t runtime_id29)
{
    // 1) 先直接按原 ID 查（兼容单实例 / 未改写场景）
    if (const auto* msg = proto_bms_gen::findMessage(runtime_id29)) {
        return msg;
    }

    // 2) 再按“去实例化”后的协议模板 ID 查
    const uint32_t proto_id29 = normalizeToProtoId(runtime_id29);
    return proto_bms_gen::findMessage(proto_id29);
}

const proto_bms_gen::SignalDef* ProtoBmsTableRuntime::findSignal(const proto_bms_gen::MessageDef* msg,
                                                                 const char* sig_name)
{
    if (!msg || !sig_name) return nullptr;
    for (uint16_t i = 0; i < msg->signal_len; ++i) {
        const auto& s = msg->signals[i];
        if (s.name && std::strcmp(s.name, sig_name) == 0) {
            return &s;
        }
    }
    return nullptr;
}

bool ProtoBmsTableRuntime::enumText(const proto_bms_gen::SignalDef* sig,
                                    uint32_t raw,
                                    const char*& out_text)
{
    if (!sig || !sig->enums || sig->enum_len == 0) return false;
    for (uint16_t i = 0; i < sig->enum_len; ++i) {
        if (sig->enums[i].raw == raw) {
            out_text = sig->enums[i].text;
            return true;
        }
    }
    return false;
}

uint64_t ProtoBmsTableRuntime::signExtend(uint64_t raw_u64, uint32_t bit_len)
{
    if (bit_len == 0 || bit_len >= 64) return raw_u64;

    const uint64_t sign_bit = (1ULL << (bit_len - 1));
    if ((raw_u64 & sign_bit) == 0) {
        return raw_u64;
    }

    const uint64_t mask = ~((1ULL << bit_len) - 1ULL);
    return raw_u64 | mask;
}

double ProtoBmsTableRuntime::rawToPhys(const proto_bms_gen::SignalDef* sig, uint64_t raw_u64)
{
    if (!sig) return 0.0;

    uint64_t v = raw_u64;

    // 简单策略：
    // - factor/offset 允许负值，所以支持有符号扩展
    // - 长度为 1 时仍允许按数值返回
    if (sig->offset < 0.0) {
        v = signExtend(raw_u64, sig->length);
    }

    const double raw = static_cast<double>(static_cast<int64_t>(v));
    return raw * sig->factor + sig->offset;
}

bool ProtoBmsTableRuntime::isBoolLikeSignal(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) return false;
    return sig->length == 1;
}

bool ProtoBmsTableRuntime::isEnumLikeSignal(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) return false;
    return sig->enums && sig->enum_len > 0;
}

    static uint32_t protoIdFromMsgCode_(uint32_t msg_code)
{
    switch (msg_code) {
    case 0x01: return 0x1802F3EFu; // V2B_CMD
    case 0x02: return 0x1881EFF3u; // B2V_Fault1
    case 0x03: return 0x1882EFF3u; // B2V_Fault2
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
    case 0x0F: return 0x18FFF7F6u; // Fire2B_Info
    default: return 0u;
    }
}
// uint32_t ProtoBmsTableRuntime::normalizeToProtoId(uint32_t runtime_id29)
// {
//     const uint32_t inst = BmsFrameRouter::extractInstanceIndex(runtime_id29);
//     if (!BmsFrameRouter::isValidInstance(inst)) {
//         return runtime_id29;
//     }
//
//     const uint32_t msg_code = BmsFrameRouter::extractMsgCode(runtime_id29);
//
//     // 协议模板低 12bit 的常见形式：
//     // BMU->VCU:   EFF3 / EFF4 / ...
//     // B2TM:       45F4
//     // TM2B:       C13A
//     // Fire2B:     F7F6
//     //
//     // 你的 canhub 运行态样例已经表明：
//     //   B2V_*           -> 低 8bit 直接保留 0x02..0x0C
//     //   B2TM_Info       -> 0x18FF410D/420D/430D/440D
//     //   TM2B_Info       -> 0x18FFC10E/20E/30E/40E
//     //   Fire2B_Info     -> 0x18FFF10F/20F/30F/40F
//     //
//     // 所以这里统一策略：
//     // 低 8bit 保留 msg_code，实例位清零；
//     // 其余高位保持不变。
//     return (runtime_id29 & ~0xF00u) & ~0xFFu | msg_code;
// }
    uint32_t ProtoBmsTableRuntime::normalizeToProtoId(uint32_t runtime_id29)
{
    const uint32_t inst = BmsFrameRouter::extractInstanceIndex(runtime_id29);
    if (!BmsFrameRouter::isValidInstance(inst)) {
        return runtime_id29;
    }

    const uint32_t msg_code = BmsFrameRouter::extractMsgCode(runtime_id29);
    const uint32_t proto_id29 = protoIdFromMsgCode_(msg_code);
    return (proto_id29 != 0u) ? proto_id29 : runtime_id29;
}

std::string ProtoBmsTableRuntime::canonicalMsgName(const proto_bms_gen::MessageDef* msg)
{
    if (!msg) return "UNKNOWN";
    return BmsNameMap::canonicalMsgName(msg->name);
}

    bool ProtoBmsTableRuntime::shouldParseAsPhysical(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) return false;

    // 1bit 明确是状态位
    if (isBoolLikeSignal(sig)) return false;

    constexpr double kEps = 1e-12;

    // 只要有非恒等换算，必定是物理量
    if (std::fabs(sig->factor - 1.0) > kEps) return true;
    if (std::fabs(sig->offset - 0.0) > kEps) return true;

    // 没有枚举，默认按数值量
    if (!(sig->enums && sig->enum_len > 0)) return true;

    // 对于位宽较大的量，即使挂了 enum，也优先按数值量处理
    // 这样可以覆盖 ST3 的 16bit 绝缘值
    if (sig->length >= 8) return true;

    return false;
}

} // namespace proto::bms