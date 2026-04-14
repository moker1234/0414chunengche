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

bool DataAggregator::isBmsInternalKey_(const std::string& k)
{
    return k.rfind("__bms.", 0) == 0;
}

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

    // 5) 写入 BmsSnapshot
    bms_snap_.ts_ms = ts;

    auto& inst = bms_snap_.ensureInstance(instance_name, bms_index);
    inst.meta.last_msg_name = msg_name;
    inst.health.online = true;
    inst.health.last_ok_ms = ts;
    inst.health.disconnect_window_ms = 6000; // 或来自配置

    auto& grp = inst.groups[msg_name];
    grp.ts_ms = ts;
    grp.rx_count += 1;
    grp.can_id = can_id;
    grp.cycle_ms = cycle_ms;
    grp.raw_hex = raw_hex;

    grp.health.online = true;
    grp.health.last_rx_ms = ts;
    grp.health.last_ok_ms = ts;

    // 6) 同时把该实例的最近一帧轻量投影到 SystemSnapshot.items[instance]
    SnapshotItem& item = snap_.items[instance_name];
    item.ts_ms = ts;
    item.data = d;
    item.data.device_name = instance_name;

    item.health = DeviceHealth::ONLINE;
    item.online = true;
    item.last_ok_ms = ts;

    // 7) 先保留现有 system.bms_* 轻量投影，避免老的 logic/hmi 立即失效
    updateLegacyBmsProjection_(d, instance_name, ts);
}

void DataAggregator::updateLegacyBmsProjection_(const DeviceData& d,
                                                const std::string& instance_name,
                                                uint64_t ts)
{
    // 第一批先只用 BMS_1 更新 system.bms_*，
    // 避免旧逻辑/HMI 被 4 路混合值冲掉。
    if (instance_name != "BMS_1") {
        return;
    }

    auto get_num = [&](std::initializer_list<const char*> keys, double defv) -> double {
        for (const char* k : keys) {
            auto it = d.num.find(k);
            if (it != d.num.end()) return it->second;
        }
        return defv;
    };

    auto get_bool_like = [&](std::initializer_list<const char*> keys, bool defv) -> bool {
        for (const char* k : keys) {
            auto itv = d.value.find(k);
            if (itv != d.value.end()) return itv->second != 0;

            auto its = d.status.find(k);
            if (its != d.status.end()) return its->second != 0u;

            auto itn = d.num.find(k);
            if (itn != d.num.end()) return itn->second != 0.0;
        }
        return defv;
    };

    const double soc = get_num({
        "soc",
        "SOC",
        "bms.soc",
        "bms.B2V_ST2.SOC",
        "bms.B2V_ST2.B2V_ST2_SOC",
        "B2V_ST2_SOC"
    }, std::nan(""));

    const double pack_v = get_num({
        "pack_v",
        "pack_voltage_v",
        "PackVoltage",
        "bms.pack_voltage_v",
        "bms.B2V_ST2.PackVoltage",
        "bms.B2V_ST2.B2V_ST2_PackInsideVolt",
        "B2V_ST2_PackInsideVolt"
    }, std::nan(""));

    const double pack_i = get_num({
        "pack_i",
        "pack_current_a",
        "PackCurrent",
        "bms.pack_current_a",
        "bms.B2V_ST2.PackCurrent",
        "bms.B2V_ST2.B2V_ST2_PackCurrent",
        "B2V_ST2_PackCurrent"
    }, std::nan(""));

    const double fault_num = get_num({
        "fault_num",
        "FaultNum",
        "bms.fault_num"
    }, std::nan(""));

    const bool alarm_any = get_bool_like({
        "alarm.any",
        "alarm_any",
        "bms.alarm_any",
        "FaultAny",
        "fault.any"
    }, snap_.bms_alarm_any);

    if (!std::isnan(soc)) {
        snap_.bms_soc = soc;
    }
    if (!std::isnan(pack_v)) {
        snap_.bms_pack_v = pack_v;
    }
    if (!std::isnan(pack_i)) {
        snap_.bms_pack_i = pack_i;
    }
    if (!std::isnan(fault_num) && fault_num >= 0.0) {
        snap_.bms_fault_num = static_cast<uint32_t>(fault_num + 0.5);
    }

    snap_.bms_alarm_any = alarm_any;
    snap_.bms_ts_ms = ts;
}

void DataAggregator::onDeviceData(const DeviceData& d) {
    std::lock_guard<std::mutex> lk(mtx_);

    const uint64_t ts = nowMs_();
    snap_.timestamp_ms = ts;


    /* ======================= BMS（4 路实例独立快照）======================= */
    if (d.device_name == "BMS") {
        onBmsDeviceData_(d, ts);
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
        auto it_type = d.num.find("type_code");
        if (it_type == d.num.end())
            return;

        GasType gt = static_cast<GasType>((uint16_t)it_type->second);

        GasChannelState& ch = item.gas_channels[gt];
        ch.valid        = true;
        ch.raw          = (uint16_t)d.num.at("gas_raw");
        ch.value        = d.num.at("gas_value");
        ch.status       = (uint16_t)d.num.at("status");
        ch.decimal_code = (uint16_t)d.num.at("decimal");
        ch.unit_code    = (uint16_t)d.num.at("unit_code");
        ch.type_code    = (uint16_t)d.num.at("type_code");
        ch.ts_ms        = ts;

        snap_.gas_ppm   = ch.value;
        snap_.gas_alarm = (ch.status != 0);
    }

    /* ======================= PCU (按 state/ctrl 分组保存) ======================= */
    if (d.device_name == "PCU" ||
        d.device_name == "PCU_0" ||
        d.device_name == "PCU_1")
    {
        if (!item.pcu.has_value())
            item.pcu.emplace();

        auto& p = *item.pcu;

        p.state.num.clear();
        p.state.value.clear();
        p.state.status.clear();

        for (const auto& [k, v] : d.num)    p.state.num[k] = v;
        for (const auto& [k, v] : d.value)  p.state.value[k] = v;
        for (const auto& [k, v] : d.status) p.state.status[k] = v;

        p.state.ts_ms = ts;
        return;
    }

    if (d.device_name == "PCU_CTRL" ||
        d.device_name == "PCU_0_CTRL" ||
        d.device_name == "PCU_1_CTRL")
    {
        if (!item.pcu.has_value())
            item.pcu.emplace();

        auto& p = *item.pcu;

        p.ctrl.num.clear();
        p.ctrl.value.clear();
        p.ctrl.status.clear();

        for (const auto& [k, v] : d.num)    p.ctrl.num[k] = v;
        for (const auto& [k, v] : d.value)  p.ctrl.value[k] = v;
        for (const auto& [k, v] : d.status) p.ctrl.status[k] = v;

        p.ctrl.ts_ms = ts;
        return;
    }

    /* ======================= SmokeSensor ======================= */
    if (d.device_name == "SmokeSensor") {
        if (auto it = d.num.find("smoke_percent"); it != d.num.end())
            snap_.smoke_percent = it->second;
        if (auto it = d.num.find("temp"); it != d.num.end())
            snap_.smoke_temperature = (int)it->second;
        if (auto it = d.num.find("alarm"); it != d.num.end())
            snap_.smoke_alarm = ((int)it->second != 0);
        return;
    }

    /* ======================= AirConditioner ======================= */
    if (d.device_name == "AirConditioner") {
        if (!item.aircon.has_value())
            item.aircon.emplace();

        auto& ac = *item.aircon;

        for (const auto& [k, v] : d.num) {
            if (k.find("version") == 0) {
                ac.version.fields[k] = v;
                ac.version.ts_ms = ts;
            }
            else if (k.find("run.") == 0) {
                ac.run_state.fields[k] = v;
                ac.run_state.ts_ms = ts;
            }
            else if (
                k.find("temp.") == 0 ||
                k.find("humidity") != std::string::npos ||
                k.find("voltage") != std::string::npos ||
                k.find("current") != std::string::npos
            ) {
                ac.sensor_state.fields[k] = v;
                ac.sensor_state.ts_ms = ts;
            }
            else if (k.find("param.") == 0) {
                ac.sys_para.fields[k] = v;
                ac.sys_para.ts_ms = ts;
            }
            else if (k.find("remote.") == 0) {
                ac.remote_para.fields[k] = v;
                ac.remote_para.ts_ms = ts;
            }
        }

        for (const auto& [k, v] : d.status) {
            if (k.find("alarm.") == 0) {
                ac.warn_state.fields[k] = v;
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

        updateSystemTemperature_();
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
}