#include "bms_snapshot.h"

#include <cstdio>

namespace snapshot {

static std::string hexId(uint32_t id) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08X", id);
    return std::string(buf);
}

    json BmsGroupHealth::toJson() const
{
    json j;
    j["online"] = online;
    j["last_rx_ms"] = last_rx_ms;
    j["last_ok_ms"] = last_ok_ms;
    j["disconnect_window_ms"] = disconnect_window_ms;
    j["last_offline_ms"] = last_offline_ms;
    j["disconnect_count"] = disconnect_count;
    return j;
}

json BmsGroupData::toJson() const
{
    json j;
    j["ts_ms"] = ts_ms;
    j["rx_count"] = rx_count;

    if (can_id != 0) {
        j["can_id"] = hexId(can_id);
    }
    if (!raw_hex.empty()) {
        j["raw_hex"] = raw_hex;
    }

    // 第一批先不强制落 cycle_ms / group health；
    // 但结构上保留，后续 health 扩展时不用再改 snapshot 类型。
    // 如需调试可打开下面两行：
    // if (cycle_ms >= 0) j["cycle_ms"] = cycle_ms;
    // j["health"] = health.toJson();

    return j;
}

json BmsInstanceMeta::toJson() const
{
    json j;
    j["bms_index"] = bms_index;
    if (!last_msg_name.empty()) {
        j["last_msg_name"] = last_msg_name;
    }
    return j;
}

    json BmsInstanceHealth::toJson() const
{
    json j;
    j["online"] = online;
    j["last_ok_ms"] = last_ok_ms;
    j["disconnect_window_ms"] = disconnect_window_ms;
    j["last_offline_ms"] = last_offline_ms;
    j["disconnect_count"] = disconnect_count;
    return j;
}

json BmsInstanceData::toJson() const
{
    json j;
    j["meta"] = meta.toJson();
    j["health"] = health.toJson();

    json g = json::object();
    for (const auto& kv : groups) {
        g[kv.first] = kv.second.toJson();
    }
    j["groups"] = std::move(g);
    return j;
}

json BmsInstanceData::toJsonGroupsOnly() const
{
    json g = json::object();
    for (const auto& kv : groups) {
        g[kv.first] = kv.second.toJson();
    }
    return g;
}

json BmsSnapshot::toJson() const
{
    json j;
    j["ts_ms"] = ts_ms;

    json ji = json::object();
    for (const auto& kv : items) {
        ji[kv.first] = kv.second.toJson();
    }
    j["items"] = std::move(ji);
    return j;
}

json BmsSnapshot::toJsonItemsOnly() const
{
    json ji = json::object();
    for (const auto& kv : items) {
        ji[kv.first] = kv.second.toJson();
    }
    return ji;
}

BmsInstanceData& BmsSnapshot::ensureInstance(const std::string& instance_name, uint32_t bms_index)
{
    auto& inst = items[instance_name];
    if (inst.meta.bms_index == 0 && bms_index != 0) {
        inst.meta.bms_index = bms_index;
    }
    return inst;
}

} // namespace snapshot