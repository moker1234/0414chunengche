#include "bms_proto.h"

#include <cstdio>
#include <cstring>

#include "logger.h"
#include "bms_name_map.h"
#include "bms_thread/bms_frame_router.h"

namespace proto::bms {

BmsProto::BmsProto(std::string device_name)
    : device_name_(std::move(device_name))
{
}

uint32_t BmsProto::getId29(const can_frame& fr)
{
    return (fr.can_id & CAN_EFF_MASK);
}

uint64_t BmsProto::extractBitsLsb(const uint8_t data[8], int start_lsb, int len)
{
    if (start_lsb < 0 || len <= 0 || start_lsb >= 64) return 0;
    if (start_lsb + len > 64) len = 64 - start_lsb;

    uint64_t u = 0;
    for (int i = 0; i < 8; ++i) {
        u |= (static_cast<uint64_t>(data[i]) << (i * 8));
    }

    const uint64_t mask = (len == 64) ? ~0ULL : ((1ULL << len) - 1ULL);
    return (u >> start_lsb) & mask;
}

std::string BmsProto::bytesToHex(const uint8_t data[8], uint8_t dlc)
{
    std::string s;
    s.reserve(dlc * 3);

    char buf[4];
    for (uint8_t i = 0; i < dlc; ++i) {
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        if (!s.empty()) s.push_back(' ');
        s.append(buf);
    }
    return s;
}

bool BmsProto::parse(const can_frame& fr, DeviceData& out)
{
    // 只处理扩展帧
    if ((fr.can_id & CAN_EFF_FLAG) == 0) {
        return false;
    }
    if (fr.can_dlc < 8) {
        return false;
    }

    const uint32_t id29 = getId29(fr);
    // if ( id29 == 0x1883E104)
    // {
    //     printf("st1");
    // }
    // if ( id29 == 0x1884E205)
    // {
    //     printf("st3");
    // }
    // if ( id29 == 0x1884E305)
    // {
    //     printf("st3");
    // }
    // if ( id29 == 0x1884E405)
    // {
    //     printf("st3");
    // }
    const auto* msg = ProtoBmsTableRuntime::findMessage(id29);
    if (!msg) {
        return false;
    }

    // 清空复用内容
    out.num.clear();
    out.value.clear();
    out.status.clear();
    out.str.clear();

    out.device_name = device_name_;

    const uint32_t bms_index = BmsFrameRouter::extractInstanceIndex(id29);
    const std::string instance_name = BmsFrameRouter::makeInstanceName(bms_index);

    const char* proto_msg_name = (msg->name && msg->name[0]) ? msg->name : "UNKNOWN";
    const std::string msg_name = BmsNameMap::canonicalMsgName(proto_msg_name);

    // ===== 内部字段：给 Aggregator / Snapshot / 后续 Logic 使用 =====
    out.str["__bms.instance"] = instance_name;
    out.value["__bms.instance_index"] = static_cast<int32_t>(bms_index);

    out.str["__bms.msg"] = msg_name;
    out.str["__bms.proto_msg"] = proto_msg_name;

    out.value["__bms.id"] = static_cast<int32_t>(id29);
    out.value["__bms.cycle_ms"] = static_cast<int32_t>(msg->cycle_ms);

    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "0x%08X", id29);
        out.str["__bms.id_hex"] = buf;
    }

    out.str["__bms.raw_hex"] = bytesToHex(fr.data, fr.can_dlc);

    // 输出 key：
    // 1) canonical：bms.<MsgName>.<SigName>
    // 2) short：<SigName>（便于后续 logic 过渡期用）
    const std::string msg_prefix = key_prefix_ + "." + msg_name + ".";

for (uint16_t i = 0; i < msg->signal_len; ++i) {
    const auto& sig = msg->signals[i];
    if (!sig.name || !sig.name[0]) continue;

    const std::string full_key = msg_prefix + sig.name;
    const std::string short_key = sig.name;

    const uint64_t raw_u64 = extractBitsLsb(fr.data, sig.startbit_lsb, sig.length);

    // 1) 1bit -> value（0/1）
    if (ProtoBmsTableRuntime::isBoolLikeSignal(&sig)) {
        const int32_t v = raw_u64 ? 1 : 0;
        out.value[full_key] = v;
        out.value[short_key] = v;

        const char* txt = nullptr;
        if (ProtoBmsTableRuntime::enumText(&sig, static_cast<uint32_t>(raw_u64), txt) && txt) {
            out.str[full_key + ".text"] = txt;
            out.str[short_key + ".text"] = txt;
        }
        continue;
    }

    // 2) 需要按 factor/offset 转换的字段，优先走物理量
    if (ProtoBmsTableRuntime::shouldParseAsPhysical(&sig)) {
        const double phys = ProtoBmsTableRuntime::rawToPhys(&sig, raw_u64);
        out.num[full_key] = phys;
        if (out.num.find(short_key) == out.num.end()) {
            out.num[short_key] = phys;
        }

        // 可选：保留 text，不影响主值
        const char* txt = nullptr;
        if (ProtoBmsTableRuntime::enumText(&sig, static_cast<uint32_t>(raw_u64), txt) && txt) {
            out.str[full_key + ".text"] = txt;
            out.str[short_key + ".text"] = txt;
        }
        continue;
    }

    // 3) 纯枚举/状态字段 -> value + text
    if (ProtoBmsTableRuntime::isEnumLikeSignal(&sig)) {
        const int32_t v = static_cast<int32_t>(raw_u64);
        out.value[full_key] = v;
        out.value[short_key] = v;

        const char* txt = nullptr;
        if (ProtoBmsTableRuntime::enumText(&sig, static_cast<uint32_t>(raw_u64), txt) && txt) {
            out.str[full_key + ".text"] = txt;
            out.str[short_key + ".text"] = txt;
        }
        continue;
    }

    // 4) 其它普通数值字段 -> num
    const double phys = ProtoBmsTableRuntime::rawToPhys(&sig, raw_u64);
    out.num[full_key] = phys;
    if (out.num.find(short_key) == out.num.end()) {
        out.num[short_key] = phys;
    }
}

    return true;
}

} // namespace proto::bms