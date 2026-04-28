#include "proto_bms_table_runtime.h"

#include <cmath>
#include <cstring>

#include "bms_name_map.h"
#include "bms_thread/bms_frame_router.h"

namespace proto::bms {

const proto_bms_gen::MessageDef* ProtoBmsTableRuntime::findMessage(uint32_t runtime_id29)
{
    // 1) 先直接按传入 ID 查。
    //    兼容：
    //    - 原始协议模板 ID
    //    - generated 表中的 ID
    //    - 当前 Fire2B_state 的 0x18FFF10F
    if (const auto* msg = proto_bms_gen::findMessage(runtime_id29 & BmsFrameRouter::kCanId29Mask)) {
        return msg;
    }

    // 2) 再通过 BmsFrameRouter 统一反推 generated 表可查 ID。
    const uint32_t proto_id29 = normalizeToProtoId(runtime_id29);
    if (proto_id29 == (runtime_id29 & BmsFrameRouter::kCanId29Mask)) {
        return nullptr;
    }

    return proto_bms_gen::findMessage(proto_id29);
}

const proto_bms_gen::SignalDef* ProtoBmsTableRuntime::findSignal(const proto_bms_gen::MessageDef* msg,
                                                                 const char* sig_name)
{
    if (!msg || !sig_name) {
        return nullptr;
    }

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
    if (!sig || !sig->enums || sig->enum_len == 0) {
        return false;
    }

    for (uint16_t i = 0; i < sig->enum_len; ++i) {
        if (sig->enums[i].raw == raw) {
            out_text = sig->enums[i].text;
            return true;
        }
    }

    return false;
}

uint64_t ProtoBmsTableRuntime::signExtend(uint64_t raw_u64,
                                          uint32_t bit_len)
{
    if (bit_len == 0 || bit_len >= 64) {
        return raw_u64;
    }

    const uint64_t sign_bit = (1ULL << (bit_len - 1));
    if ((raw_u64 & sign_bit) == 0) {
        return raw_u64;
    }

    const uint64_t mask = ~((1ULL << bit_len) - 1ULL);
    return raw_u64 | mask;
}

double ProtoBmsTableRuntime::rawToPhys(const proto_bms_gen::SignalDef* sig,
                                       uint64_t raw_u64)
{
    if (!sig) {
        return 0.0;
    }

    uint64_t v = raw_u64;

    // 简单策略：
    // - 只要 offset 为负，按有符号扩展处理。
    // - 当前项目的 BMS 电流类字段依赖 factor/offset，必须保留该逻辑。
    if (sig->offset < 0.0) {
        v = signExtend(raw_u64, sig->length);
    }

    const double raw = static_cast<double>(static_cast<int64_t>(v));
    return raw * sig->factor + sig->offset;
}

bool ProtoBmsTableRuntime::isBoolLikeSignal(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) {
        return false;
    }

    return sig->length == 1;
}

bool ProtoBmsTableRuntime::isEnumLikeSignal(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) {
        return false;
    }

    return sig->enums && sig->enum_len > 0;
}

uint32_t ProtoBmsTableRuntime::normalizeToProtoId(uint32_t runtime_id29)
{
    return BmsFrameRouter::normalizeToProtoId(runtime_id29);
}

std::string ProtoBmsTableRuntime::canonicalMsgName(const proto_bms_gen::MessageDef* msg)
{
    if (!msg) {
        return "UNKNOWN";
    }

    return BmsNameMap::canonicalMsgName(msg->name);
}

bool ProtoBmsTableRuntime::shouldParseAsPhysical(const proto_bms_gen::SignalDef* sig)
{
    if (!sig) {
        return false;
    }

    // 1bit 明确是状态位
    if (isBoolLikeSignal(sig)) {
        return false;
    }

    constexpr double kEps = 1e-12;

    // 只要有非恒等换算，必定是物理量
    if (std::fabs(sig->factor - 1.0) > kEps) {
        return true;
    }

    if (std::fabs(sig->offset - 0.0) > kEps) {
        return true;
    }

    // 没有枚举，默认按数值量
    if (!(sig->enums && sig->enum_len > 0)) {
        return true;
    }

    // 对于位宽较大的量，即使挂了 enum，也优先按数值量处理。
    // 这样可以覆盖 ST3 的 16bit 绝缘值等字段。
    if (sig->length >= 8) {
        return true;
    }

    return false;
}

} // namespace proto::bms