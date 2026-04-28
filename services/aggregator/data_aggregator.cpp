#include "data_aggregator.h"

#include <chrono>
#include <cmath>
#include <cctype>

#include "logger.h"

using namespace agg;

uint64_t DataAggregator::nowMs_() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count()
    );
}

DataAggregator::DataAggregator() {
    snap_.timestamp_ms = nowMs_();
}

namespace {

static bool isPcuStateDeviceName_(const std::string& name)
{
    return name == "PCU" ||
           name == "PCU_0" ||
           name == "PCU_1";
}

static bool isPcuCtrlDeviceName_(const std::string& name)
{
    return name == "PCU_CTRL" ||
           name == "PCU_0_CTRL" ||
           name == "PCU_1_CTRL" ||
           name == "PCU_TX" ||
           name == "PCU_0_TX" ||
           name == "PCU_1_TX";
}

static std::string pcuBaseNameFromInstance_(int inst)
{
    if (inst == 1) return "PCU_0";
    if (inst == 2) return "PCU_1";
    return {};
}

static std::string pcuBaseNameFromDeviceName_(const std::string& name)
{
    if (name == "PCU_0" ||
        name == "PCU_0_CTRL" ||
        name == "PCU_0_TX") {
        return "PCU_0";
    }

    if (name == "PCU_1" ||
        name == "PCU_1_CTRL" ||
        name == "PCU_1_TX") {
        return "PCU_1";
    }

    return {};
}

static int pcuInstanceFromDeviceData_(const DeviceData& d)
{
    if (auto it = d.value.find("__pcu.instance"); it != d.value.end()) {
        const int inst = static_cast<int>(it->second);
        if (inst >= 1 && inst <= 2) return inst;
    }

    if (auto it = d.value.find("pcu_instance"); it != d.value.end()) {
        const int inst = static_cast<int>(it->second);
        if (inst >= 1 && inst <= 2) return inst;
    }

    if (auto it = d.value.find("__pcu.runtime_index"); it != d.value.end()) {
        const int runtime_idx = static_cast<int>(it->second);
        if (runtime_idx == 0) return 1;
        if (runtime_idx == 1) return 2;
    }

    return 0;
}

    static std::string pcuBaseNameFromDeviceData_(const DeviceData& d)
{
    /*
     * 第八批收敛：
     *   1. 优先使用 __pcu.instance
     *   2. 其次使用已经归一化的 device_name
     *   3. 不再把裸 PCU / PCU_CTRL / PCU_TX 回退成 items["PCU"]
     */

    const int inst = pcuInstanceFromDeviceData_(d);
    {
        const std::string by_inst = pcuBaseNameFromInstance_(inst);
        if (!by_inst.empty()) return by_inst;
    }

    {
        const std::string by_name = pcuBaseNameFromDeviceName_(d.device_name);
        if (!by_name.empty()) return by_name;
    }

    return {};
}

static DeviceData makePcuItemData_(const DeviceData& d,
                                   const std::string& item_name)
{
    DeviceData out = d;
    out.device_name = item_name;

    out.str["__inst_name"] = item_name;
    out.str["kind"] = "pcu";

    return out;
}

static void copyPcuGroup_(const DeviceData& d,
                          PcuGroupData& g,
                          uint64_t ts)
{
    g.num.clear();
    g.value.clear();
    g.status.clear();
    g.str.clear();

    for (const auto& [k, v] : d.num)    g.num[k] = v;
    for (const auto& [k, v] : d.value)  g.value[k] = v;
    for (const auto& [k, v] : d.status) g.status[k] = v;
    for (const auto& [k, v] : d.str)    g.str[k] = v;

    g.ts_ms = ts;
}

} // namespace

uint32_t DataAggregator::extractBmsIndexFromCanId_(uint32_t can_id)
{
    // 规则示例：
    // 0x1883E104 -> 最后三位 104 -> instance=1
    // 0x18FFC20E -> 最后三位 20E -> instance=2
    const uint32_t low12 = (can_id & 0xFFFu);
    const uint32_t idx = (low12 >> 8) & 0xFu;
    if (idx >= 1 && idx <= 4) return idx;
    return 0;
}

std::string DataAggregator::makeBmsInstanceName_(uint32_t idx)
{
    if (idx >= 1 && idx <= 4) {
        return "BMS_" + std::to_string(idx);
    }
    return "BMS_0";
}

uint32_t DataAggregator::parseBmsIndexFromInstanceName_(const std::string& s)
{
    // 允许 "BMS_1" ~ "BMS_4"
    constexpr const char* kPrefix = "BMS_";
    if (s.rfind(kPrefix, 0) != 0) return 0;

    const std::string tail = s.substr(4);
    if (tail.empty()) return 0;
    for (char c : tail) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return 0;
    }

    const int v = std::stoi(tail);
    if (v >= 1 && v <= 4) return static_cast<uint32_t>(v);
    return 0;
}

void DataAggregator::onBmsDeviceData_(const DeviceData& d, uint64_t ts)
{
    // 1) msg name
    const auto it_msg = d.str.find("__bms.msg");
    const std::string msg_name = (it_msg != d.str.end() && !it_msg->second.empty())
        ? it_msg->second
        : "UNKNOWN";

    // 2) instance 优先取 __bms.instance，其次从 __bms.id 推导
    std::string instance_name;
    uint32_t bms_index = 0;

    if (auto it = d.str.find("__bms.instance"); it != d.str.end()) {
        instance_name = it->second;
        bms_index = parseBmsIndexFromInstanceName_(instance_name);
    }

    uint32_t can_id = 0;
    if (auto it = d.value.find("__bms.id"); it != d.value.end()) {
        can_id = static_cast<uint32_t>(it->second);
    }

    if (bms_index == 0 && can_id != 0) {
        bms_index = extractBmsIndexFromCanId_(can_id);
        if (bms_index != 0) {
            instance_name = makeBmsInstanceName_(bms_index);
        }
    }

    if (instance_name.empty()) {
        instance_name = "BMS_0";
    }

    // 3) raw_hex
    std::string raw_hex;
    if (auto it = d.str.find("__bms.raw_hex"); it != d.str.end()) {
        raw_hex = it->second;
    }

    // 4) cycle
    int32_t cycle_ms = -1;
    if (auto it = d.value.find("__bms.cycle_ms"); it != d.value.end()) {
        cycle_ms = static_cast<int32_t>(it->second);
    }

    // 5) BMS 专用快照：保存报文级轻量数据
    // 注意：
    // - BmsSnapshot 是 BMS 专用链路
    // - 这里只保存 msg / can_id / raw_hex / rx_count / health
    // - 不在 BmsSnapshot 中扩展 num/value/status 信号级 JSON
    bms_snap_.ts_ms = ts;

    auto& inst = bms_snap_.ensureInstance(instance_name, bms_index);
    inst.meta.last_msg_name = msg_name;

    // 收到有效 BMS 报文时，只更新“最近收到数据”的最小 health。
    // 最终 online/offline、disconnect_window、last_offline、disconnect_count
    // 由 Logic runtime 通过 updateBmsRuntimeHealth() 低频回写。
    inst.health.online = true;
    inst.health.last_ok_ms = ts;

    auto& grp = inst.groups[msg_name];
    grp.ts_ms = ts;
    grp.rx_count += 1;
    grp.can_id = can_id;
    grp.cycle_ms = cycle_ms;
    grp.raw_hex = raw_hex;

    grp.health.online = true;
    grp.health.last_rx_ms = ts;
    grp.health.last_ok_ms = ts;

    // 6) 普通 SystemSnapshot 只保留 BMS 轻量 health 投影
    // 关键变化：
    // - 不再 item.data = d
    // - 不再把 BMS 的 num/value/status/str 放进普通 SystemSnapshot
    // - HMI BMS 数值继续走 logic_view
    // - BMS 报文级存档继续走 BmsSnapshot
    SnapshotItem& item = snap_.items[instance_name];

    item.ts_ms = ts;
    item.health = DeviceHealth::ONLINE;
    item.online = true;
    item.last_ok_ms = ts;

    // 只保留极小元信息，便于普通 JSON/调试识别；
    // SystemSnapshot::toJson() 会对 BMS_x 特判，不输出 item.data 的大字段。
    item.data = DeviceData{};
    item.data.device_name = instance_name;
    item.data.str["kind"] = "bms_shadow";
    item.data.str["last_msg_name"] = msg_name;
    item.data.value["bms_index"] = static_cast<int32_t>(bms_index);
}


void DataAggregator::onDeviceData(const DeviceData& d)
{
    std::lock_guard<std::mutex> lk(mtx_);

    const uint64_t ts = nowMs_();
    snap_.timestamp_ms = ts;

    /* ======================= BMS（4 路实例独立快照）======================= */
    if (d.device_name == "BMS") {
        onBmsDeviceData_(d, ts);
        return;
    }

    /*
     * ======================= PCU state / ctrl 合并 =======================
     *
     * 目标结构：
     *
     *   items["PCU_0"].pcu.state   <- PCU_0 / PCU + __pcu.instance=1
     *   items["PCU_0"].pcu.ctrl    <- PCU_0_CTRL / PCU_CTRL + __pcu.instance=1
     *
     *   items["PCU_1"].pcu.state   <- PCU_1 / PCU + __pcu.instance=2
     *   items["PCU_1"].pcu.ctrl    <- PCU_1_CTRL / PCU_CTRL + __pcu.instance=2
     *
     * 不再生成：
     *   items["PCU_0_CTRL"]
     *   items["PCU_1_CTRL"]
     *
     * 注意：
     *   PCU state 是 PCU->EMU 接收状态，可以证明设备在线。
     *   PCU ctrl 是 EMU->PCU 发送镜像，不能证明 PCU 在线。
     */
    if (isPcuStateDeviceName_(d.device_name))
    {
        const std::string item_name = pcuBaseNameFromDeviceData_(d);
        if (item_name.empty()) {
            LOG_THROTTLE_MS("agg_pcu_state_unresolved",
                            1000,
                            LOG_COMM_W,
                            "[AGG][PCU] drop unresolved state device=%s",
                            d.device_name.c_str());
            return;
        }

        SnapshotItem& item = snap_.items[item_name];

        item.ts_ms = ts;
        item.data = makePcuItemData_(d, item_name);

        item.health = DeviceHealth::ONLINE;
        item.online = true;
        item.last_ok_ms = ts;

        if (!item.pcu.has_value()) {
            item.pcu.emplace();
        }

        auto& p = *item.pcu;
        copyPcuGroup_(d, p.state, ts);

        return;
    }

    if (isPcuCtrlDeviceName_(d.device_name))
    {
        const std::string item_name = pcuBaseNameFromDeviceData_(d);
        if (item_name.empty()) {
            LOG_THROTTLE_MS("agg_pcu_ctrl_unresolved",
                            1000,
                            LOG_COMM_W,
                            "[AGG][PCU] drop unresolved ctrl device=%s msg=%s",
                            d.device_name.c_str(),
                            d.str.count("__pcu.msg") ? d.str.at("__pcu.msg").c_str() : "");
            return;
        }

        SnapshotItem& item = snap_.items[item_name];

        /*
         * TX/CTRL 镜像只表示“本机发过什么”，不能当作 PCU 在线依据。
         * 所以这里不写：
         *   item.online = true;
         *   item.health = ONLINE;
         *   item.last_ok_ms = ts;
         *
         * 如果 state 已经使 PCU 在线，则保持原状态；
         * 如果只有 ctrl 没有 state，则 item 仍保持默认 OFFLINE。
         */
        item.ts_ms = ts;

        if (item.data.device_name.empty()) {
            item.data.device_name = item_name;
            item.data.str["kind"] = "pcu_tx_only";
            item.data.str["__inst_name"] = item_name;
        }

        if (!item.pcu.has_value()) {
            item.pcu.emplace();
        }

        auto& p = *item.pcu;
        copyPcuGroup_(d, p.ctrl, ts);

        return;
    }

    SnapshotItem& item = snap_.items[d.device_name];
    item.ts_ms = ts;
    item.data  = d;

    /* ======================= Health ======================= */
    item.health = DeviceHealth::ONLINE;
    item.online = true;
    item.last_ok_ms = ts;

    /* ======================= UPS ======================= */
    if (d.device_name == "UPS") {
        auto it = d.value.find("__ups_cmd");
        if (it != d.value.end()) {
            std::string key =
                it->second == 1 ? "Q1" :
                it->second == 2 ? "Q6" :
                it->second == 3 ? "WA" : "UNKNOWN";

            UpsGroupData& grp = item.ups_groups[key];
            grp.num.clear();
            grp.value.clear();
            grp.status.clear();

            for (const auto& [k, v] : d.num)    grp.num[k] = v;
            for (const auto& [k, v] : d.value)  grp.value[k] = v;
            for (const auto& [k, v] : d.status) grp.status[k] = v;

            grp.ts_ms = ts;
        }
        return;
    }

    /* ======================= GasDetector ======================= */
    if (d.device_name == "GasDetector") {
        if (!d.gas.valid) {
            return;
        }

        if (d.gas.type_code > 5) {
            return;
        }

        GasType gt = GasType::Unknown;
        switch (d.gas.type_code) {
        case 0: gt = GasType::Combustible; break;
        case 1: gt = GasType::CO;          break;
        case 2: gt = GasType::O2;          break;
        case 3: gt = GasType::Temperature; break;
        case 4: gt = GasType::Humidity;    break;
        case 5: gt = GasType::CO2;         break;
        default: return;
        }

        GasChannelState& ch = item.gas_channels[gt];

        ch.valid        = true;
        ch.raw          = d.gas.raw;
        ch.value        = d.gas.value;
        ch.status       = d.gas.status;
        ch.decimal_code = d.gas.decimal_code;
        ch.unit_code    = d.gas.unit_code;
        ch.type_code    = d.gas.type_code;

        ch.fault_any    = (d.gas.status & 0x0001u) != 0;
        ch.low_alarm    = (d.gas.status & 0x0002u) != 0;
        ch.high_alarm   = (d.gas.status & 0x0004u) != 0;
        ch.alarm_any    = ch.low_alarm || ch.high_alarm;

        ch.ts_ms        = ts;

        snap_.gas_alarm = false;
        for (const auto& kv : item.gas_channels) {
            const auto& c = kv.second;
            if (c.valid && item.online && (c.fault_any || c.alarm_any)) {
                snap_.gas_alarm = true;
                break;
            }
        }

        return;
    }

    /* ======================= SmokeSensor ======================= */
    if (d.device_name == "SmokeSensor") {
        if (auto it = d.num.find("smoke_percent"); it != d.num.end()) {
            snap_.smoke_percent = it->second;
        }

        if (auto it = d.num.find("temp"); it != d.num.end()) {
            snap_.smoke_temperature = static_cast<int>(it->second);
        }

        bool smoke_alarm = false;

        if (auto it = d.status.find("TSS_smoke_alarm"); it != d.status.end()) {
            smoke_alarm = (it->second != 0u);
        } else if (auto it = d.num.find("alarm"); it != d.num.end()) {
            smoke_alarm = (static_cast<int>(it->second) != 0);
        }

        snap_.smoke_alarm = smoke_alarm;

        // Smoke 温度是 system_temperature 的备用来源。
        // updateSystemTemperature_();

        return;
    }

    /* ======================= AirConditioner ======================= */
    if (d.device_name == "AirConditioner") {
        if (!item.aircon.has_value())
            item.aircon.emplace();

        auto& ac = *item.aircon;

        for (const auto& [k, v] : d.num) {
            if (k == "version") {
                ac.version.fields[k] = v;
                ac.version.ts_ms = ts;
            }
            else if (k.rfind("run.", 0) == 0) {
                ac.run_state.fields[k] = v;
                ac.run_state.ts_ms = ts;
            }
            else if (
                k.rfind("temp.", 0) == 0 ||
                k == "humidity_percent" ||
                k == "current_a" ||
                k == "ac_voltage_v" ||
                k == "dc_voltage_v"
            ) {
                ac.sensor_state.fields[k] = v;
                ac.sensor_state.ts_ms = ts;
            }
            else if (k.rfind("param.", 0) == 0) {
                ac.sys_para.fields[k] = v;
                ac.sys_para.ts_ms = ts;
            }
            else if (k.rfind("remote.", 0) == 0) {
                ac.remote_para.fields[k] = v;
                ac.remote_para.ts_ms = ts;
            }
        }

        for (const auto& [k, v] : d.status) {
            if (k.rfind("alarm.", 0) == 0) {
                ac.warn_state.fields[k] = static_cast<double>(v);
                ac.warn_state.ts_ms = ts;
            }
        }

        if (auto it = ac.sensor_state.fields.find("temp.indoor_c");
            it != ac.sensor_state.fields.end()) {
            snap_.ac_indoor_temp = it->second;
        }

        if (auto it = ac.sensor_state.fields.find("humidity_percent");
            it != ac.sensor_state.fields.end()) {
            snap_.ac_humidity = it->second;
        }

        if (auto it = ac.remote_para.fields.find("remote.power");
            it != ac.remote_para.fields.end()) {
            snap_.ac_power = static_cast<int>(it->second);
        }

        if (auto it = ac.run_state.fields.find("run.overall");
            it != ac.run_state.fields.end()) {
            snap_.ac_run_state = static_cast<int>(it->second);
        }

        if (auto it = ac.warn_state.fields.find("alarm.any");
            it != ac.warn_state.fields.end()) {
            snap_.ac_alarm = (it->second != 0);
        }

        // updateSystemTemperature_();
        return;
    }
}

void DataAggregator::updateSystemTemperature_() {
    if (snap_.ac_indoor_temp > -40 && snap_.ac_indoor_temp < 150) {
        snap_.system_temperature = (int)snap_.ac_indoor_temp;
        return;
    }
    if (snap_.smoke_temperature != 0) {
        snap_.system_temperature = snap_.smoke_temperature;
        return;
    }
    snap_.system_temperature = 0;
}

SystemSnapshot DataAggregator::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return snap_;
}

snapshot::BmsSnapshot DataAggregator::bmsSnapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return bms_snap_;
}



void DataAggregator::updateHealthFromScheduler(
    const std::string& device_name,
    bool online,
    uint64_t last_ok_ms,
    uint32_t disconnect_window_ms,
    uint64_t last_offline_ms,
    uint32_t disconnect_count
)
{
    std::lock_guard<std::mutex> lk(mtx_);
    auto& item = snap_.items[device_name];

    item.health = online ? DeviceHealth::ONLINE : DeviceHealth::OFFLINE;
    item.online = online;
    item.last_ok_ms = last_ok_ms;
    item.disconnect_window_ms = disconnect_window_ms;
    item.last_offline_ms = last_offline_ms;
    item.disconnect_count = disconnect_count;

    // LOG_COMM_D("[AGG][HEALTH_WRITE] dev=%s online=%d last_ok=%llu off=%llu dc=%u win=%u",
    //            device_name.c_str(),
    //            item.online ? 1 : 0,
    //            (unsigned long long)item.last_ok_ms,
    //            (unsigned long long)item.last_offline_ms,
    //            (unsigned)item.disconnect_count,
    //            (unsigned)item.disconnect_window_ms);
    if (device_name == "GasDetector" && !online) {
        snap_.gas_alarm = false;
    }
    if (device_name == "AirConditioner" && !online) {
        snap_.ac_alarm = false;
    }
}

bool DataAggregator::updateBmsRuntimeHealth(
    const std::vector<BmsRuntimeHealthUpdate>& updates
)
{
    if (updates.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lk(mtx_);

    bool changed = false;
    const uint64_t ts = nowMs_();

    auto parse_idx_from_name = [](const std::string& name) -> uint32_t {
        constexpr const char* kPrefix = "BMS_";
        if (name.rfind(kPrefix, 0) != 0) return 0;

        const std::string tail = name.substr(4);
        if (tail.empty()) return 0;

        try {
            const int v = std::stoi(tail);
            if (v >= 1 && v <= 4) {
                return static_cast<uint32_t>(v);
            }
        } catch (...) {
        }

        return 0;
    };

    auto mark_changed = [&changed](bool old_v, bool new_v) {
        if (old_v != new_v) changed = true;
    };

    auto mark_changed_u64 = [&changed](uint64_t old_v, uint64_t new_v) {
        if (old_v != new_v) changed = true;
    };

    auto mark_changed_u32 = [&changed](uint32_t old_v, uint32_t new_v) {
        if (old_v != new_v) changed = true;
    };

    for (const auto& u : updates) {
        if (u.instance_name.empty()) {
            continue;
        }

        uint32_t idx = u.bms_index;
        if (idx == 0) {
            idx = parse_idx_from_name(u.instance_name);
        }

        if (idx < 1 || idx > 4) {
            continue;
        }

        bms_snap_.ts_ms = ts;

        auto& inst = bms_snap_.ensureInstance(u.instance_name, idx);

        mark_changed(inst.health.online, u.online);
        mark_changed_u64(inst.health.last_ok_ms, u.last_ok_ms);
        mark_changed_u32(inst.health.disconnect_window_ms, u.disconnect_window_ms);
        mark_changed_u64(inst.health.last_offline_ms, u.last_offline_ms);
        mark_changed_u32(inst.health.disconnect_count, u.disconnect_count);

        inst.health.online = u.online;
        inst.health.last_ok_ms = u.last_ok_ms;
        inst.health.disconnect_window_ms = u.disconnect_window_ms;
        inst.health.last_offline_ms = u.last_offline_ms;
        inst.health.disconnect_count = u.disconnect_count;

        // BMS_1~BMS_4 在 SystemSnapshot 中只作为轻量 health 投影存在。
        // HMI 的 BMS 数值仍走 logic_view，不在这里扩展信号级 JSON。
        auto& item = snap_.items[u.instance_name];

        mark_changed(item.online, u.online);
        mark_changed_u64(item.last_ok_ms, u.last_ok_ms);
        mark_changed_u32(item.disconnect_window_ms, u.disconnect_window_ms);
        mark_changed_u64(item.last_offline_ms, u.last_offline_ms);
        mark_changed_u32(item.disconnect_count, u.disconnect_count);

        item.ts_ms = ts;
        item.health = u.online ? DeviceHealth::ONLINE : DeviceHealth::OFFLINE;
        item.online = u.online;
        item.last_ok_ms = u.last_ok_ms;
        item.disconnect_window_ms = u.disconnect_window_ms;
        item.last_offline_ms = u.last_offline_ms;
        item.disconnect_count = u.disconnect_count;

        // 如果实例已经离线，则至少把已存在的报文组 health 一并置离线。
        // 这里不做“每个组是否 stale”的细分回写，避免扩大 BmsSnapshot 语义。
        if (!u.online) {
            for (auto& kv : inst.groups) {
                auto& grp = kv.second;

                mark_changed(grp.health.online, false);
                mark_changed_u32(grp.health.disconnect_window_ms, u.disconnect_window_ms);
                mark_changed_u64(grp.health.last_offline_ms, u.last_offline_ms);
                mark_changed_u32(grp.health.disconnect_count, u.disconnect_count);

                grp.health.online = false;
                grp.health.disconnect_window_ms = u.disconnect_window_ms;
                grp.health.last_offline_ms = u.last_offline_ms;
                grp.health.disconnect_count = u.disconnect_count;
            }
        }
    }

    return changed;
}