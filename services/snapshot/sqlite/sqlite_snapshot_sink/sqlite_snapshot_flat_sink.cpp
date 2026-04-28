//
// Created by lxy on 2026/3/18.
//

#include "sqlite_snapshot_flat_sink.h"

#include <cmath>
#include <filesystem>
#include <utility>

#include "../../utils/logger/logger.h"
#include "../aggregator/system_snapshot.h"

namespace fs = std::filesystem;

/* ================= helpers ================= */

template<typename MapT>
static double getNumOr_(const MapT& m,
                        const char* k,
                        double defv = 0.0)
{
    auto it = m.find(k);
    return (it != m.end()) ? static_cast<double>(it->second) : defv;
}

template<typename MapT>
static int32_t getValOr_(const MapT& m,
                         const char* k,
                         int32_t defv = 0)
{
    auto it = m.find(k);
    return (it != m.end()) ? static_cast<int32_t>(it->second) : defv;
}

template<typename MapT>
static uint32_t getStatusOr_(const MapT& m,
                             const char* k,
                             uint32_t defv = 0)
{
    auto it = m.find(k);
    return (it != m.end()) ? static_cast<uint32_t>(it->second) : defv;
}

int32_t SqliteSnapshotFlatSink::scaleX10_(double v)
{
    return static_cast<int32_t>(std::llround(v * 10.0));
}

int32_t SqliteSnapshotFlatSink::scaleX100_(double v)
{
    return static_cast<int32_t>(std::llround(v * 100.0));
}

int32_t SqliteSnapshotFlatSink::scaleX1000_(double v)
{
    return static_cast<int32_t>(std::llround(v * 1000.0));
}

bool SqliteSnapshotFlatSink::isBmsShadowItem_(const std::string& name)
{
    return name.rfind("BMS_", 0) == 0;
}

std::string SqliteSnapshotFlatSink::healthKeyJson_(const SnapshotItem& item)
{
    nlohmann::json j;
    j["state"]                = static_cast<int>(item.health);
    j["online"]               = item.online;
    j["last_ok_ms"]           = item.last_ok_ms;
    j["disconnect_window_ms"] = item.disconnect_window_ms;
    j["last_offline_ms"]      = item.last_offline_ms;
    j["disconnect_count"]     = item.disconnect_count;
    return j.dump();
}

nlohmann::json SqliteSnapshotFlatSink::deviceStateJson_(const std::string& name,
                                                        const SnapshotItem& item)
{
    nlohmann::json j = nlohmann::json::object();

    /*
     * BMS shadow：
     * 普通 SQLite snapshot 不保存 BMS 大字段。
     * BMS 报文级/历史数据由 SqliteBmsFlatSink 负责。
     */
    if (isBmsShadowItem_(name) || name == "BMS") {
        j["kind"] = "bms_shadow";
        j["online"] = item.online;
        j["last_ok_ms"] = item.last_ok_ms;
        return j;
    }

    if (name == "GasDetector") {
        j["gas_channels"] = nlohmann::json::object();

        for (const auto& [gt, ch] : item.gas_channels) {
            j["gas_channels"][std::to_string(static_cast<uint16_t>(gt))] = {
                {"valid", ch.valid},
                {"value", ch.value},
                {"raw", ch.raw},
                {"status", ch.status},
                {"decimal", ch.decimal_code},
                {"type_code", ch.type_code},
                {"unit_code", ch.unit_code},

                {"fault_any", ch.fault_any},
                {"low_alarm", ch.low_alarm},
                {"high_alarm", ch.high_alarm},
                {"alarm_any", ch.alarm_any},

                {"ts_ms", ch.ts_ms}
            };
        }

        return j;
    }

    if (name == "SmokeSensor") {
        j["num"] = item.data.num;
        return j;
    }

    if (name == "UPS") {
        for (const auto& [cmd, grp] : item.ups_groups) {
            j[cmd] = {
                {"num", grp.num},
                {"value", grp.value},
                {"status", grp.status},
                {"ts_ms", grp.ts_ms}
            };
        }
        return j;
    }

    if (name == "AirConditioner" && item.aircon.has_value()) {
        const auto& ac = *item.aircon;
        j["remote_para"]  = ac.remote_para.fields;
        j["run_state"]    = ac.run_state.fields;
        j["sensor_state"] = ac.sensor_state.fields;
        j["sys_para"]     = ac.sys_para.fields;
        j["version"]      = ac.version.fields;
        j["warn_state"]   = ac.warn_state.fields;
        return j;
    }

    if ((name == "PCU" || name.rfind("PCU_", 0) == 0) && item.pcu.has_value()) {
        const auto& p = *item.pcu;

        j["state"] = {
            {"num", p.state.num},
            {"value", p.state.value},
            {"status", p.state.status},
            {"ts_ms", p.state.ts_ms}
        };

        const bool has_ctrl =
            (p.ctrl.ts_ms != 0) ||
            (!p.ctrl.num.empty()) ||
            (!p.ctrl.value.empty()) ||
            (!p.ctrl.status.empty());

        if (has_ctrl) {
            j["ctrl"] = {
                {"num", p.ctrl.num},
                {"value", p.ctrl.value},
                {"status", p.ctrl.status},
                {"ts_ms", p.ctrl.ts_ms}
            };
        }

        return j;
    }

    j["num"] = item.data.num;
    j["value"] = item.data.value;
    j["status"] = item.data.status;
    return j;
}

/* ================= ctor / dtor ================= */

SqliteSnapshotFlatSink::SqliteSnapshotFlatSink(Config cfg)
    : cfg_(std::move(cfg))
{
    opened_ = openDb_();
    if (!opened_) {
        LOGERR("[SQLITE_SNAP_FLAT] open db failed: %s", cfg_.db_path.c_str());
        return;
    }

    if (!applyPragmas_()) {
        LOGERR("[SQLITE_SNAP_FLAT] apply pragmas failed");
    }

    if (!initSchema_()) {
        LOGERR("[SQLITE_SNAP_FLAT] init schema failed");
    }
}

SqliteSnapshotFlatSink::~SqliteSnapshotFlatSink()
{
    closeDb_();
}

/* ================= db ================= */

bool SqliteSnapshotFlatSink::openDb_()
{
    try {
        const fs::path p(cfg_.db_path);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }
    } catch (const std::exception& e) {
        LOGERR("[SQLITE_SNAP_FLAT] create parent dir failed: %s", e.what());
        return false;
    }

    const int rc = sqlite3_open(cfg_.db_path.c_str(), &db_);
    if (rc != SQLITE_OK || !db_) {
        LOGERR("[SQLITE_SNAP_FLAT] sqlite3_open failed rc=%d err=%s",
               rc, db_ ? sqlite3_errmsg(db_) : "null");
        return false;
    }

    sqlite3_busy_timeout(db_, static_cast<int>(cfg_.busy_timeout_ms));
    return true;
}

void SqliteSnapshotFlatSink::closeDb_()
{
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool SqliteSnapshotFlatSink::applyPragmas_()
{
    if (!db_) return false;

    const char* sqls[] = {
        "PRAGMA journal_mode=WAL;",
        "PRAGMA synchronous=FULL;",
        "PRAGMA wal_autocheckpoint=1000;",
        nullptr
    };

    char* err = nullptr;
    for (int i = 0; sqls[i] != nullptr; ++i) {
        const int rc = sqlite3_exec(db_, sqls[i], nullptr, nullptr, &err);
        if (rc != SQLITE_OK) {
            LOGERR("[SQLITE_SNAP_FLAT] pragma failed sql=%s err=%s",
                   sqls[i], err ? err : "unknown");
            if (err) sqlite3_free(err);
            return false;
        }
    }
    return true;
}

bool SqliteSnapshotFlatSink::initSchema_()
{
    if (!db_) return false;

    const char* sql = R"SQL(
CREATE TABLE IF NOT EXISTS snapshot_main (
    id                      INTEGER PRIMARY KEY,
    ts_ms                   INTEGER NOT NULL,

    gas_alarm               INTEGER NOT NULL DEFAULT 0,

    smoke_percent_x10       INTEGER,
    smoke_temperature_x10   INTEGER,
    smoke_alarm             INTEGER NOT NULL DEFAULT 0,

    ac_indoor_temp_x10      INTEGER,
    ac_humidity_x10         INTEGER,
    ac_power                INTEGER,
    ac_run_state            INTEGER,
    ac_alarm                INTEGER NOT NULL DEFAULT 0,

    system_temperature_x10  INTEGER
);

CREATE INDEX IF NOT EXISTS idx_snapshot_main_ts
ON snapshot_main(ts_ms DESC);

CREATE TABLE IF NOT EXISTS device_health (
    id                      INTEGER PRIMARY KEY,
    snapshot_id             INTEGER NOT NULL,
    ts_ms                   INTEGER NOT NULL,
    device_name             TEXT    NOT NULL,

    health_state            INTEGER NOT NULL,
    online                  INTEGER NOT NULL,

    last_ok_ms              INTEGER NOT NULL,
    disconnect_window_ms    INTEGER NOT NULL,
    last_offline_ms         INTEGER NOT NULL,
    disconnect_count        INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_device_health_name_ts
ON device_health(device_name, ts_ms DESC);


CREATE INDEX IF NOT EXISTS idx_gas_poll_event_name_ts
ON gas_poll_event(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS gas_channel_state (
    id              INTEGER PRIMARY KEY,
    snapshot_id     INTEGER NOT NULL,
    ts_ms           INTEGER NOT NULL,
    device_name     TEXT    NOT NULL,
    channel_index   INTEGER NOT NULL,
    channel_ts_ms   INTEGER NOT NULL,
    valid           INTEGER NOT NULL,

    type_code       INTEGER NOT NULL,
    unit_code       INTEGER NOT NULL,
    decimal_code    INTEGER NOT NULL,
    status_code     INTEGER NOT NULL,

    fault_any       INTEGER NOT NULL DEFAULT 0,
    low_alarm       INTEGER NOT NULL DEFAULT 0,
    high_alarm      INTEGER NOT NULL DEFAULT 0,
    alarm_any       INTEGER NOT NULL DEFAULT 0,

    raw_value       INTEGER NOT NULL,
    value_x1000     INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_gas_channel_state_name_ch_ts
ON gas_channel_state(device_name, channel_index, ts_ms DESC);

CREATE TABLE IF NOT EXISTS smoke_state (
    id                  INTEGER PRIMARY KEY,
    snapshot_id         INTEGER NOT NULL,
    ts_ms               INTEGER NOT NULL,
    device_name         TEXT    NOT NULL,
    alarm               INTEGER NOT NULL,
    fault_code          INTEGER NOT NULL,
    smoke_percent_x10   INTEGER NOT NULL,
    temp_c_x10          INTEGER NOT NULL,
    warn_level          INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_smoke_state_name_ts
ON smoke_state(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS ups_q1_state (
    id                      INTEGER PRIMARY KEY,
    snapshot_id             INTEGER NOT NULL,
    ts_ms                   INTEGER NOT NULL,
    device_name             TEXT    NOT NULL,

    ups_cmd                 INTEGER NOT NULL,

    input_v_x10             INTEGER NOT NULL,
    input_last_v_x10        INTEGER NOT NULL,
    input_freq_hz_x10       INTEGER NOT NULL,
    output_v_x10            INTEGER NOT NULL,
    load_pct_x10            INTEGER NOT NULL,
    battery_12v_x100        INTEGER NOT NULL,
    battery_cell_v_x100     INTEGER NOT NULL,
    temp_c_x10              INTEGER NOT NULL,

    ups_battery_low         INTEGER NOT NULL,
    ups_bypass              INTEGER NOT NULL,
    ups_fault               INTEGER NOT NULL,
    ups_mains_abnormal      INTEGER NOT NULL,
    ups_raw                 INTEGER NOT NULL,
    ups_standby             INTEGER NOT NULL,
    ups_testing             INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_ups_q1_state_name_ts
ON ups_q1_state(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS ups_q6_state (
    id                          INTEGER PRIMARY KEY,
    snapshot_id                 INTEGER NOT NULL,
    ts_ms                       INTEGER NOT NULL,
    device_name                 TEXT    NOT NULL,

    ups_cmd                     INTEGER NOT NULL,

    battery_v_pos_x10           INTEGER NOT NULL,
    input_freq_hz_x10           INTEGER NOT NULL,
    input_v_r_x10               INTEGER NOT NULL,
    input_v_s_x10               INTEGER NOT NULL,
    input_v_t_x10               INTEGER NOT NULL,
    output_freq_hz_x10          INTEGER NOT NULL,
    output_i_r_x10              INTEGER NOT NULL,
    output_v_r_x10              INTEGER NOT NULL,
    temp_c_x10                  INTEGER NOT NULL,

    battery_capacity            INTEGER,
    battery_remain_sec          INTEGER,
    battery_test_state          INTEGER,
    system_mode                 INTEGER,

    battery_capacity_valid      INTEGER NOT NULL,
    battery_remain_sec_valid    INTEGER NOT NULL,
    battery_v_neg_valid         INTEGER NOT NULL,
    battery_v_pos_valid         INTEGER NOT NULL,
    input_freq_valid            INTEGER NOT NULL,
    input_v_r_valid             INTEGER NOT NULL,
    input_v_s_valid             INTEGER NOT NULL,
    input_v_t_valid             INTEGER NOT NULL,
    output_freq_valid           INTEGER NOT NULL,
    output_i_r_valid            INTEGER NOT NULL,
    output_i_s_valid            INTEGER NOT NULL,
    output_i_t_valid            INTEGER NOT NULL,
    output_v_r_valid            INTEGER NOT NULL,
    output_v_s_valid            INTEGER NOT NULL,
    output_v_t_valid            INTEGER NOT NULL,
    temp_valid                  INTEGER NOT NULL,
    lcd_phase_v                 INTEGER NOT NULL,
    transformer_y               INTEGER NOT NULL,
    fault_bits                  INTEGER NOT NULL,
    warning_bits                INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_ups_q6_state_name_ts
ON ups_q6_state(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS pcu_state (
    id              INTEGER PRIMARY KEY,
    snapshot_id     INTEGER NOT NULL,
    ts_ms           INTEGER NOT NULL,
    device_name     TEXT    NOT NULL,

    can_index       INTEGER NOT NULL,
    cabinet_id      INTEGER NOT NULL,
    heartbeat       INTEGER NOT NULL,
    pcu_state       INTEGER NOT NULL,
    estop           INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_pcu_state_name_ts
ON pcu_state(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS pcu_tx_state (
    id                  INTEGER PRIMARY KEY,
    snapshot_id         INTEGER NOT NULL,
    ts_ms               INTEGER NOT NULL,
    device_name         TEXT    NOT NULL,

    can_index           INTEGER NOT NULL,
    pcu_instance        INTEGER NOT NULL,
    runtime_index       INTEGER NOT NULL,

    ctrl_id             INTEGER NOT NULL,
    status_id           INTEGER NOT NULL,

    ctrl_heartbeat      INTEGER NOT NULL,

    plug_state          INTEGER NOT NULL,
    estop               INTEGER NOT NULL,
    batt1_estop         INTEGER NOT NULL,
    batt2_estop         INTEGER NOT NULL,
    sys_enable          INTEGER NOT NULL,

    batt1_kw_x10        INTEGER NOT NULL,
    batt2_kw_x10        INTEGER NOT NULL,
    batt1_branches      INTEGER NOT NULL,
    batt2_branches      INTEGER NOT NULL,

    ctrl_raw_hex        TEXT,
    status_raw_hex      TEXT
);

CREATE INDEX IF NOT EXISTS idx_pcu_tx_state_name_ts
ON pcu_tx_state(device_name, ts_ms DESC);

CREATE TABLE IF NOT EXISTS aircon_state (
    id                              INTEGER PRIMARY KEY,
    snapshot_id                     INTEGER NOT NULL,
    ts_ms                           INTEGER NOT NULL,
    device_name                     TEXT    NOT NULL,

    ts_remote_para                  INTEGER NOT NULL,
    ts_run_state                    INTEGER NOT NULL,
    ts_sensor_state                 INTEGER NOT NULL,
    ts_sys_para                     INTEGER NOT NULL,
    ts_version                      INTEGER NOT NULL,
    ts_warn_state                   INTEGER NOT NULL,

    remote_power                    INTEGER NOT NULL,

    run_overall                     INTEGER NOT NULL,
    run_compressor                  INTEGER NOT NULL,
    run_em_fan                      INTEGER NOT NULL,
    run_heater                      INTEGER NOT NULL,
    run_inner_fan                   INTEGER NOT NULL,
    run_outer_fan                   INTEGER NOT NULL,

    ac_voltage_v_x10                INTEGER NOT NULL,
    current_a_x10                   INTEGER NOT NULL,
    dc_voltage_v_x10                INTEGER NOT NULL,
    humidity_percent_x10            INTEGER NOT NULL,
    temp_coil_c_x10                 INTEGER NOT NULL,
    temp_condense_c_x10             INTEGER NOT NULL,
    temp_exhaust_c_x10              INTEGER NOT NULL,
    temp_indoor_c_x10               INTEGER NOT NULL,
    temp_outdoor_c_x10              INTEGER NOT NULL,

    param_cool_hys_c_x10            INTEGER NOT NULL,
    param_cool_point_c_x10          INTEGER NOT NULL,
    param_heat_hys_c_x10            INTEGER NOT NULL,
    param_heat_point_c_x10          INTEGER NOT NULL,
    param_high_hum_pct_x10          INTEGER NOT NULL,
    param_high_temp_c_x10           INTEGER NOT NULL,
    param_inner_fan_stop_c_x10      INTEGER NOT NULL,
    param_low_temp_c_x10            INTEGER NOT NULL,

    version_code_x10                INTEGER NOT NULL,

    alarm_any                       INTEGER NOT NULL,
    alarm_ac_over_v                 INTEGER NOT NULL,
    alarm_ac_power_loss             INTEGER NOT NULL,
    alarm_ac_under_v                INTEGER NOT NULL,
    alarm_coil_freeze               INTEGER NOT NULL,
    alarm_coil_sensor_fault         INTEGER NOT NULL,
    alarm_compressor_fault          INTEGER NOT NULL,
    alarm_condense_sensor_fault     INTEGER NOT NULL,
    alarm_dc_over_v                 INTEGER NOT NULL,
    alarm_dc_under_v                INTEGER NOT NULL,
    alarm_door                      INTEGER NOT NULL,
    alarm_emergency_fan_fault       INTEGER NOT NULL,
    alarm_exhaust_high_temp         INTEGER NOT NULL,
    alarm_exhaust_lock              INTEGER NOT NULL,
    alarm_exhaust_sensor_fault      INTEGER NOT NULL,
    alarm_freq_abnormal             INTEGER NOT NULL,
    alarm_heater_fault              INTEGER NOT NULL,
    alarm_high_hum                  INTEGER NOT NULL,
    alarm_high_pressure             INTEGER NOT NULL,
    alarm_high_pressure_lock        INTEGER NOT NULL,
    alarm_high_temp                 INTEGER NOT NULL,
    alarm_hum_sensor_fault          INTEGER NOT NULL,
    alarm_indoor_sensor_fault       INTEGER NOT NULL,
    alarm_inner_fan_fault           INTEGER NOT NULL,
    alarm_low_hum                   INTEGER NOT NULL,
    alarm_low_pressure              INTEGER NOT NULL,
    alarm_low_pressure_lock         INTEGER NOT NULL,
    alarm_low_temp                  INTEGER NOT NULL,
    alarm_outdoor_sensor_fault      INTEGER NOT NULL,
    alarm_outer_fan_fault           INTEGER NOT NULL,
    alarm_phase_loss                INTEGER NOT NULL,
    alarm_reverse_phase             INTEGER NOT NULL,
    alarm_smoke                     INTEGER NOT NULL,
    alarm_water                     INTEGER NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_aircon_state_name_ts
ON aircon_state(device_name, ts_ms DESC);
)SQL";

    char* err = nullptr;
    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP_FLAT] init schema failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::beginTx_()
{
    char* err = nullptr;
    const int rc = sqlite3_exec(db_, "BEGIN IMMEDIATE TRANSACTION;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP_FLAT] begin tx failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

bool SqliteSnapshotFlatSink::commitTx_()
{
    char* err = nullptr;
    const int rc = sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, &err);
    if (rc != SQLITE_OK) {
        LOGERR("[SQLITE_SNAP_FLAT] commit tx failed err=%s", err ? err : "unknown");
        if (err) sqlite3_free(err);
        return false;
    }
    return true;
}

void SqliteSnapshotFlatSink::rollbackTx_()
{
    sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
}

/* ================= change detect ================= */

bool SqliteSnapshotFlatSink::deviceChanged_(const std::string& name,
                                            const SnapshotItem& item)
{
    const std::string cur = deviceStateJson_(name, item).dump();

    auto it = last_device_json_cache_.find(name);
    if (it == last_device_json_cache_.end() || it->second != cur) {
        last_device_json_cache_[name] = cur;
        return true;
    }
    return false;
}

bool SqliteSnapshotFlatSink::healthChanged_(const std::string& name,
                                            const SnapshotItem& item)
{
    const std::string cur = healthKeyJson_(item);

    auto it = last_device_health_cache_.find(name);
    if (it == last_device_health_cache_.end() || it->second != cur) {
        last_device_health_cache_[name] = cur;
        return true;
    }
    return false;
}

std::set<std::string> SqliteSnapshotFlatSink::collectChangedDevices_(const agg::SystemSnapshot& snap)
{
    std::set<std::string> out;

    for (const auto& [name, item] : snap.items) {
        if (isBmsShadowItem_(name)) continue;

        if (deviceChanged_(name, item) || healthChanged_(name, item)) {
            out.insert(name);
        }
    }

    return out;
}

/* ================= inserts ================= */

bool SqliteSnapshotFlatSink::insertSnapshotMain_(const agg::SystemSnapshot& snap,
                                                 int64_t& out_snapshot_id)
{
    const char* sql =
        "INSERT INTO snapshot_main("
        "ts_ms, "
        "gas_alarm, "
        "smoke_percent_x10, smoke_temperature_x10, smoke_alarm, "
        "ac_indoor_temp_x10, ac_humidity_x10, ac_power, ac_run_state, ac_alarm, "
        "system_temperature_x10"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare snapshot_main failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1,  static_cast<sqlite3_int64>(snap.timestamp_ms));
    sqlite3_bind_int(stmt,   2,  snap.gas_alarm ? 1 : 0);
    sqlite3_bind_int(stmt,   3,  scaleX10_(snap.smoke_percent));
    sqlite3_bind_int(stmt,   4,  snap.smoke_temperature * 10);
    sqlite3_bind_int(stmt,   5,  snap.smoke_alarm ? 1 : 0);
    sqlite3_bind_int(stmt,   6,  scaleX10_(snap.ac_indoor_temp));
    sqlite3_bind_int(stmt,   7,  scaleX10_(snap.ac_humidity));
    sqlite3_bind_int(stmt,   8,  snap.ac_power);
    sqlite3_bind_int(stmt,   9,  snap.ac_run_state);
    sqlite3_bind_int(stmt,   10, snap.ac_alarm ? 1 : 0);
    sqlite3_bind_int(stmt,   11, snap.system_temperature * 10);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert snapshot_main failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    out_snapshot_id = static_cast<int64_t>(sqlite3_last_insert_rowid(db_));
    return true;
}

bool SqliteSnapshotFlatSink::insertDeviceHealth_(int64_t snapshot_id,
                                                 uint64_t ts_ms,
                                                 const std::string& device_name,
                                                 const SnapshotItem& item)
{
    const char* sql =
        "INSERT INTO device_health("
        "snapshot_id, ts_ms, device_name,health_state, online, "
        "last_ok_ms, disconnect_window_ms,last_offline_ms, disconnect_count)"
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?);"
    ;

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare device_health failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3, device_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   4, static_cast<int>(item.health));
    sqlite3_bind_int(stmt,   5, item.online ? 1 : 0);
    sqlite3_bind_int64(stmt, 6, item.last_ok_ms);
    sqlite3_bind_int(stmt,   7, item.disconnect_window_ms);
    sqlite3_bind_int64(stmt, 8, item.last_offline_ms);
    sqlite3_bind_int(stmt,   9, item.disconnect_count);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert device_health failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}



bool SqliteSnapshotFlatSink::insertGasChannelDelta_(int64_t snapshot_id,
                                                    uint64_t ts_ms,
                                                    const std::string& device_name,
                                                    const SnapshotItem& item)
{
    const char* sql =
        "INSERT INTO gas_channel_state("
        "snapshot_id, ts_ms, device_name, channel_index, channel_ts_ms, valid, "
        "type_code, unit_code, decimal_code, status_code, "
        "fault_any, low_alarm, high_alarm, alarm_any, "
        "raw_value, value_x1000"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    for (const auto& [gt, ch] : item.gas_channels) {
        // 只写“本轮刚更新的通道”
        if (ch.ts_ms != item.ts_ms) continue;

        sqlite3_stmt* stmt = nullptr;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        if (rc != SQLITE_OK || !stmt) {
            LOGERR("[SQLITE_SNAP_FLAT] prepare gas_channel_state failed err=%s",
                   sqlite3_errmsg(db_));
            return false;
        }

        sqlite3_bind_int64(stmt, 1,  static_cast<sqlite3_int64>(snapshot_id));
        sqlite3_bind_int64(stmt, 2,  static_cast<sqlite3_int64>(ts_ms));
        sqlite3_bind_text(stmt,  3,  device_name.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt,   4,  static_cast<int>(static_cast<uint16_t>(gt)));
        sqlite3_bind_int64(stmt, 5,  static_cast<sqlite3_int64>(ch.ts_ms));
        sqlite3_bind_int(stmt,   6,  ch.valid ? 1 : 0);

        sqlite3_bind_int(stmt,   7,  static_cast<int>(ch.type_code));
        sqlite3_bind_int(stmt,   8,  static_cast<int>(ch.unit_code));
        sqlite3_bind_int(stmt,   9,  static_cast<int>(ch.decimal_code));
        sqlite3_bind_int(stmt,   10, static_cast<int>(ch.status));

        sqlite3_bind_int(stmt,   11, ch.fault_any ? 1 : 0);
        sqlite3_bind_int(stmt,   12, ch.low_alarm ? 1 : 0);
        sqlite3_bind_int(stmt,   13, ch.high_alarm ? 1 : 0);
        sqlite3_bind_int(stmt,   14, ch.alarm_any ? 1 : 0);

        sqlite3_bind_int(stmt,   15, static_cast<int>(ch.raw));
        sqlite3_bind_int(stmt,   16, scaleX1000_(ch.value));

        rc = sqlite3_step(stmt);
        sqlite3_finalize(stmt);

        if (rc != SQLITE_DONE) {
            LOGERR("[SQLITE_SNAP_FLAT] insert gas_channel_state failed dev=%s err=%s",
                   device_name.c_str(), sqlite3_errmsg(db_));
            return false;
        }
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertSmokeState_(int64_t snapshot_id,
                                               uint64_t ts_ms,
                                               const std::string& device_name,
                                               const SnapshotItem& item)
{
    const auto& n = item.data.num;

    const char* sql =
        "INSERT INTO smoke_state("
        "snapshot_id, ts_ms, device_name, alarm, fault_code, smoke_percent_x10, temp_c_x10, warn_level"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare smoke_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3, device_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   4, static_cast<int>(getNumOr_(n, "alarm")));
    sqlite3_bind_int(stmt,   5, static_cast<int>(getNumOr_(n, "fault")));
    sqlite3_bind_int(stmt,   6, scaleX10_(getNumOr_(n, "smoke_percent")));
    sqlite3_bind_int(stmt,   7, scaleX10_(getNumOr_(n, "temp")));
    sqlite3_bind_int(stmt,   8, static_cast<int>(getNumOr_(n, "warn_level")));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert smoke_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertUpsQ1State_(int64_t snapshot_id,
                                               uint64_t ts_ms,
                                               const std::string& device_name,
                                               const SnapshotItem& item)
{
    auto it = item.ups_groups.find("Q1");
    if (it == item.ups_groups.end()) return true;

    const auto& g = it->second;
    const char* sql =
        "INSERT INTO ups_q1_state("
        "snapshot_id, ts_ms, device_name, ups_cmd, "
        "input_v_x10, input_last_v_x10, input_freq_hz_x10, output_v_x10, load_pct_x10, "
        "battery_12v_x100, battery_cell_v_x100, temp_c_x10, "
        "ups_battery_low, ups_bypass, ups_fault, ups_mains_abnormal, ups_raw, ups_standby, ups_testing"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare ups_q1_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1,  static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2,  static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3,  device_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   4,  getValOr_(g.value, "__ups_cmd"));
    sqlite3_bind_int(stmt,   5,  scaleX10_(getNumOr_(g.num, "input.v")));
    sqlite3_bind_int(stmt,   6,  scaleX10_(getNumOr_(g.num, "input.last.v")));
    sqlite3_bind_int(stmt,   7,  scaleX10_(getNumOr_(g.num, "input.freq.hz")));
    sqlite3_bind_int(stmt,   8,  scaleX10_(getNumOr_(g.num, "output.v")));
    sqlite3_bind_int(stmt,   9,  scaleX10_(getNumOr_(g.num, "load.pct")));
    sqlite3_bind_int(stmt,   10, scaleX100_(getNumOr_(g.num, "battery.12v.v")));
    sqlite3_bind_int(stmt,   11, scaleX100_(getNumOr_(g.num, "battery.cell.v")));
    sqlite3_bind_int(stmt,   12, scaleX10_(getNumOr_(g.num, "temp.c")));
    sqlite3_bind_int(stmt,   13, static_cast<int>(getStatusOr_(g.status, "q1.battery_low")));
    sqlite3_bind_int(stmt,   14, static_cast<int>(getStatusOr_(g.status, "q1.bypass")));
    sqlite3_bind_int(stmt,   15, static_cast<int>(getStatusOr_(g.status, "q1.fault")));
    sqlite3_bind_int(stmt,   16, static_cast<int>(getStatusOr_(g.status, "q1.mains_abnormal")));
    sqlite3_bind_int(stmt,   17, static_cast<int>(getStatusOr_(g.status, "q1.status.bits")));
    sqlite3_bind_int(stmt,   18, static_cast<int>(getStatusOr_(g.status, "q1.backup_mode")));
    sqlite3_bind_int(stmt,   19, static_cast<int>(getStatusOr_(g.status, "q1.testing")));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert ups_q1_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertUpsQ6State_(int64_t snapshot_id,
                                               uint64_t ts_ms,
                                               const std::string& device_name,
                                               const SnapshotItem& item)
{
    auto it = item.ups_groups.find("Q6");
    if (it == item.ups_groups.end()) return true;

    const auto& g = it->second;
    const char* sql =
        "INSERT INTO ups_q6_state("
        "snapshot_id, ts_ms, device_name, ups_cmd, "
        "battery_v_pos_x10, input_freq_hz_x10, input_v_r_x10, input_v_s_x10, input_v_t_x10, "
        "output_freq_hz_x10, output_i_r_x10, output_v_r_x10, temp_c_x10, "
        "battery_capacity, battery_remain_sec, battery_test_state, system_mode, "
        "battery_capacity_valid, battery_remain_sec_valid, battery_v_neg_valid, battery_v_pos_valid, "
        "input_freq_valid, input_v_r_valid, input_v_s_valid, input_v_t_valid, output_freq_valid, "
        "output_i_r_valid, output_i_s_valid, output_i_t_valid, output_v_r_valid, output_v_s_valid, "
        "output_v_t_valid, temp_valid, lcd_phase_v, transformer_y, fault_bits, warning_bits"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare ups_q6_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1,  static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2,  static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3,  device_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   4,  getValOr_(g.value, "__ups_cmd"));
    sqlite3_bind_int(stmt,   5,  scaleX10_(getNumOr_(g.num, "battery.v.pos")));
    sqlite3_bind_int(stmt,   6,  scaleX10_(getNumOr_(g.num, "input.freq.hz")));
    sqlite3_bind_int(stmt,   7,  scaleX10_(getNumOr_(g.num, "input.v.r")));
    sqlite3_bind_int(stmt,   8,  scaleX10_(getNumOr_(g.num, "input.v.s")));
    sqlite3_bind_int(stmt,   9,  scaleX10_(getNumOr_(g.num, "input.v.t")));
    sqlite3_bind_int(stmt,   10, scaleX10_(getNumOr_(g.num, "output.freq.hz")));
    sqlite3_bind_int(stmt,   11, scaleX10_(getNumOr_(g.num, "output.i.r")));
    sqlite3_bind_int(stmt,   12, scaleX10_(getNumOr_(g.num, "output.v.r")));
    sqlite3_bind_int(stmt,   13, scaleX10_(getNumOr_(g.num, "temp.c")));
    sqlite3_bind_int(stmt,   14, getValOr_(g.value, "battery.capacity"));
    sqlite3_bind_int(stmt,   15, getValOr_(g.value, "battery.remain.sec"));
    sqlite3_bind_int(stmt,   16, getValOr_(g.value, "battery.test.state"));
    sqlite3_bind_int(stmt,   17, getValOr_(g.value, "system.mode"));
    sqlite3_bind_int(stmt,   18, static_cast<int>(getStatusOr_(g.status, "battery.capacity.valid")));
    sqlite3_bind_int(stmt,   19, static_cast<int>(getStatusOr_(g.status, "battery.remain.sec.valid")));
    sqlite3_bind_int(stmt,   20, static_cast<int>(getStatusOr_(g.status, "battery.v.neg.valid")));
    sqlite3_bind_int(stmt,   21, static_cast<int>(getStatusOr_(g.status, "battery.v.pos.valid")));
    sqlite3_bind_int(stmt,   22, static_cast<int>(getStatusOr_(g.status, "input.freq.hz.valid")));
    sqlite3_bind_int(stmt,   23, static_cast<int>(getStatusOr_(g.status, "input.v.r.valid")));
    sqlite3_bind_int(stmt,   24, static_cast<int>(getStatusOr_(g.status, "input.v.s.valid")));
    sqlite3_bind_int(stmt,   25, static_cast<int>(getStatusOr_(g.status, "input.v.t.valid")));
    sqlite3_bind_int(stmt,   26, static_cast<int>(getStatusOr_(g.status, "output.freq.hz.valid")));
    sqlite3_bind_int(stmt,   27, static_cast<int>(getStatusOr_(g.status, "output.i.r.valid")));
    sqlite3_bind_int(stmt,   28, static_cast<int>(getStatusOr_(g.status, "output.i.s.valid")));
    sqlite3_bind_int(stmt,   29, static_cast<int>(getStatusOr_(g.status, "output.i.t.valid")));
    sqlite3_bind_int(stmt,   30, static_cast<int>(getStatusOr_(g.status, "output.v.r.valid")));
    sqlite3_bind_int(stmt,   31, static_cast<int>(getStatusOr_(g.status, "output.v.s.valid")));
    sqlite3_bind_int(stmt,   32, static_cast<int>(getStatusOr_(g.status, "output.v.t.valid")));
    sqlite3_bind_int(stmt,   33, static_cast<int>(getStatusOr_(g.status, "temp.c.valid")));
    sqlite3_bind_int(stmt,   34, static_cast<int>(getStatusOr_(g.status, "lcd.phase.v")));
    sqlite3_bind_int(stmt,   35, static_cast<int>(getStatusOr_(g.status, "transformer.y")));
    sqlite3_bind_int(stmt,   36, static_cast<int>(getStatusOr_(g.status, "fault.bits")));
    sqlite3_bind_int(stmt,   37, static_cast<int>(getStatusOr_(g.status, "warning.bits")));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert ups_q6_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertPcuState_(int64_t snapshot_id,
                                             uint64_t ts_ms,
                                             const std::string& device_name,
                                             const SnapshotItem& item)
{
    if (!item.pcu.has_value()) return true;

    const auto& p = *item.pcu;
    const auto& v = p.state.value;
    const auto& s = p.state.status;

    const char* sql =
        "INSERT INTO pcu_state("
        "snapshot_id, ts_ms, device_name, can_index, cabinet_id, heartbeat, pcu_state, estop"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare pcu_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3, device_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt,   4, getValOr_(v, "__can_index"));
    sqlite3_bind_int(stmt,   5, getValOr_(v, "cabinet_id"));
    sqlite3_bind_int(stmt,   6, getValOr_(v, "heartbeat"));
    sqlite3_bind_int(stmt,   7, getValOr_(v, "pcu_state"));
    sqlite3_bind_int(stmt,   8, static_cast<int>(getStatusOr_(s, "estop")));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert pcu_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertPcuTxState_(int64_t snapshot_id,
                                               uint64_t ts_ms,
                                               const std::string& device_name,
                                               const SnapshotItem& item)
{
    if (!item.pcu.has_value()) return true;

    const auto& p = *item.pcu;

    const bool has_ctrl =
        (p.ctrl.ts_ms != 0) ||
        (!p.ctrl.num.empty()) ||
        (!p.ctrl.value.empty()) ||
        (!p.ctrl.status.empty()) ||
        (!p.ctrl.str.empty());

    if (!has_ctrl) {
        return true;
    }

    const auto& v  = p.ctrl.value;
    const auto& s  = p.ctrl.status;
    const auto& st = p.ctrl.str;

    const char* sql =
        "INSERT INTO pcu_tx_state("
        "snapshot_id, ts_ms, device_name, "
        "can_index, pcu_instance, runtime_index, "
        "ctrl_id, status_id, ctrl_heartbeat, "
        "plug_state, estop, batt1_estop, batt2_estop, sys_enable, "
        "batt1_kw_x10, batt2_kw_x10, batt1_branches, batt2_branches, "
        "ctrl_raw_hex, status_raw_hex"
        ") VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare pcu_tx_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    auto getStrOr_ = [&](const char* key) -> const std::string& {
        static const std::string empty;
        auto it = st.find(key);
        return (it == st.end()) ? empty : it->second;
    };

    const std::string& ctrl_raw_hex = getStrOr_("__pcu.ctrl_raw_hex");
    const std::string& status_raw_hex = getStrOr_("__pcu.status_raw_hex");

    sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, 2, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text(stmt,  3, device_name.c_str(), -1, SQLITE_TRANSIENT);

    sqlite3_bind_int(stmt,   4, getValOr_(v, "__can_index"));
    sqlite3_bind_int(stmt,   5, getValOr_(v, "__pcu.instance"));
    sqlite3_bind_int(stmt,   6, getValOr_(v, "__pcu.runtime_index"));

    sqlite3_bind_int(stmt,   7, getValOr_(v, "__pcu.ctrl_id"));
    sqlite3_bind_int(stmt,   8, getValOr_(v, "__pcu.status_id"));
    sqlite3_bind_int(stmt,   9, getValOr_(v, "ctrl_heartbeat"));

    sqlite3_bind_int(stmt,  10, static_cast<int>(getStatusOr_(s, "plug_state")));
    sqlite3_bind_int(stmt,  11, static_cast<int>(getStatusOr_(s, "estop")));
    sqlite3_bind_int(stmt,  12, static_cast<int>(getStatusOr_(s, "batt1_estop")));
    sqlite3_bind_int(stmt,  13, static_cast<int>(getStatusOr_(s, "batt2_estop")));
    sqlite3_bind_int(stmt,  14, static_cast<int>(getStatusOr_(s, "sys_enable")));

    sqlite3_bind_int(stmt,  15, getValOr_(v, "batt1_kw_x10"));
    sqlite3_bind_int(stmt,  16, getValOr_(v, "batt2_kw_x10"));
    sqlite3_bind_int(stmt,  17, getValOr_(v, "batt1_branches"));
    sqlite3_bind_int(stmt,  18, getValOr_(v, "batt2_branches"));

    if (!ctrl_raw_hex.empty()) {
        sqlite3_bind_text(stmt, 19, ctrl_raw_hex.c_str(), -1, SQLITE_TRANSIENT);
    } else {
        sqlite3_bind_null(stmt, 19);
    }

    if (!status_raw_hex.empty()) {
        sqlite3_bind_text(stmt, 20, status_raw_hex.c_str(), -1, SQLITE_TRANSIENT);
    } else {
        sqlite3_bind_null(stmt, 20);
    }

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert pcu_tx_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

bool SqliteSnapshotFlatSink::insertAirconState_(int64_t snapshot_id,
                                                uint64_t ts_ms,
                                                const std::string& device_name,
                                                const SnapshotItem& item)
{
    if (!item.aircon.has_value()) return true;

    const auto& ac = *item.aircon;

    const auto& rp = ac.remote_para.fields;
    const auto& rs = ac.run_state.fields;
    const auto& ss = ac.sensor_state.fields;
    const auto& sp = ac.sys_para.fields;
    const auto& vv = ac.version.fields;
    const auto& ws = ac.warn_state.fields;

    auto getWarn01_ = [&](std::initializer_list<const char*> keys) -> int {
        for (const char* k : keys) {
            auto it = ws.find(k);
            if (it != ws.end()) {
                return (std::llround(it->second) != 0) ? 1 : 0;
            }
        }
        return 0;
    };

    const char* sql =
        "INSERT INTO aircon_state("
        "snapshot_id, ts_ms, device_name, "
        "ts_remote_para, ts_run_state, ts_sensor_state, ts_sys_para, ts_version, ts_warn_state, "
        "remote_power, "
        "run_overall, run_compressor, run_em_fan, run_heater, run_inner_fan, run_outer_fan, "
        "ac_voltage_v_x10, current_a_x10, dc_voltage_v_x10, humidity_percent_x10, "
        "temp_coil_c_x10, temp_condense_c_x10, temp_exhaust_c_x10, temp_indoor_c_x10, temp_outdoor_c_x10, "
        "param_cool_hys_c_x10, param_cool_point_c_x10, param_heat_hys_c_x10, param_heat_point_c_x10, "
        "param_high_hum_pct_x10, param_high_temp_c_x10, param_inner_fan_stop_c_x10, param_low_temp_c_x10, "
        "version_code_x10, "
        "alarm_any, alarm_ac_over_v, alarm_ac_power_loss, alarm_ac_under_v, alarm_coil_freeze, "
        "alarm_coil_sensor_fault, alarm_compressor_fault, alarm_condense_sensor_fault, alarm_dc_over_v, "
        "alarm_dc_under_v, alarm_door, alarm_emergency_fan_fault, alarm_exhaust_high_temp, alarm_exhaust_lock, "
        "alarm_exhaust_sensor_fault, alarm_freq_abnormal, alarm_heater_fault, alarm_high_hum, alarm_high_pressure, "
        "alarm_high_pressure_lock, alarm_high_temp, alarm_hum_sensor_fault, alarm_indoor_sensor_fault, "
        "alarm_inner_fan_fault, alarm_low_hum, alarm_low_pressure, alarm_low_pressure_lock, alarm_low_temp, "
        "alarm_outdoor_sensor_fault, alarm_outer_fan_fault, alarm_phase_loss, alarm_reverse_phase, alarm_smoke, alarm_water"
        ") VALUES("
        "?, ?, ?, "
        "?, ?, ?, ?, ?, ?, "
        "?, "
        "?, ?, ?, ?, ?, ?, "
        "?, ?, ?, ?, ?, ?, ?, ?, ?, "
        "?, ?, ?, ?, ?, ?, ?, ?, "
        "?, "
        "?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?"
        ");";

    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK || !stmt) {
        LOGERR("[SQLITE_SNAP_FLAT] prepare aircon_state failed err=%s",
               sqlite3_errmsg(db_));
        return false;
    }

    int idx = 1;
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(snapshot_id));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ts_ms));
    sqlite3_bind_text (stmt, idx++, device_name.c_str(), -1, SQLITE_TRANSIENT);

    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.remote_para.ts_ms));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.run_state.ts_ms));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.sensor_state.ts_ms));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.sys_para.ts_ms));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.version.ts_ms));
    sqlite3_bind_int64(stmt, idx++, static_cast<sqlite3_int64>(ac.warn_state.ts_ms));

    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rp, "remote.power"))));

    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.overall"))));
    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.compressor"))));
    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.em_fan"))));
    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.heater"))));
    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.inner_fan"))));
    sqlite3_bind_int(stmt, idx++, static_cast<int>(std::llround(getNumOr_(rs, "run.outer_fan"))));

    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "ac_voltage_v")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "current_a")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "dc_voltage_v")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "humidity_percent")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "temp.coil_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "temp.condense_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "temp.exhaust_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "temp.indoor_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(ss, "temp.outdoor_c")));

    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.cool_hys_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.cool_point_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.heat_hys_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.heat_point_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.high_hum_pct")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.high_temp_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.inner_fan_stop_c")));
    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(sp, "param.low_temp_c")));

    sqlite3_bind_int(stmt, idx++, scaleX10_(getNumOr_(vv, "version")));

    // aircon_state 表的列名可以保留旧命名；
    // 但写入时优先读取标准 AIR signal，兼容旧 alarm 短名。
    sqlite3_bind_int(stmt, idx++, getWarn01_({"alarm.any"}));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.ac_over_voltage_alarm",
        "alarm.ac_over_v"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.ac_power_loss"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.ac_under_voltage_alarm",
        "alarm.ac_under_v"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.coil_freeze_protect",
        "alarm.coil_freeze"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.coil_temp_sensor_fault",
        "alarm.coil_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.compressor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.condenser_temp_sensor_fault",
        "alarm.condense_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.dc_over_voltage_alarm",
        "alarm.dc_over_v"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.dc_under_voltage_alarm",
        "alarm.dc_under_v"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.gating_alarm",
        "alarm.door"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.emergency_fan_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.exhaust_high_temp_alarm",
        "alarm.exhaust_high_temp"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.exhaust_lock"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.exhaust_temp_sensor_fault",
        "alarm.exhaust_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.freq_fault",
        "alarm.freq_abnormal"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.heater_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.high_humidity_alarm",
        "alarm.high_hum"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.high_pressure_alarm",
        "alarm.high_pressure"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.high_pressure_lock"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.high_temp_alarm",
        "alarm.high_temp"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.humidity_sensor_fault",
        "alarm.hum_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.indoor_temp_sensor_fault",
        "alarm.indoor_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.internal_fan_fault",
        "alarm.inner_fan_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.low_humidity_alarm",
        "alarm.low_hum"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.low_pressure_alarm",
        "alarm.low_pressure"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.low_pressure_lock"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.low_temp_alarm",
        "alarm.low_temp"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.outdoor_temp_sensor_fault",
        "alarm.outdoor_sensor_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.external_fan_fault",
        "alarm.outer_fan_fault"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.lose_phase_alarm",
        "alarm.phase_loss"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.anti_phase_alarm",
        "alarm.reverse_phase"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.smoke_alarm",
        "alarm.smoke"
    }));

    sqlite3_bind_int(stmt, idx++, getWarn01_({
        "alarm.water_alarm",
        "alarm.water"
    }));

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);

    if (rc != SQLITE_DONE) {
        LOGERR("[SQLITE_SNAP_FLAT] insert aircon_state failed dev=%s err=%s",
               device_name.c_str(), sqlite3_errmsg(db_));
        return false;
    }

    return true;
}

/* ================= main entry ================= */

void SqliteSnapshotFlatSink::onSnapshot(const agg::SystemSnapshot& snap)
{
    if (!opened_ || !db_) return;

    const auto changed_devices = collectChangedDevices_(snap);
    if (!cfg_.write_main_every_snapshot && changed_devices.empty()) {
        return;
    }

    if (!beginTx_()) return;

    int64_t snapshot_id = 0;
    if (!insertSnapshotMain_(snap, snapshot_id)) {
        rollbackTx_();
        return;
    }

    for (const auto& [name, item] : snap.items) {
        if (isBmsShadowItem_(name) || name == "BMS") continue;

        // 设备健康：这里每个快照都写一条，便于追溯
        if (!insertDeviceHealth_(snapshot_id, snap.timestamp_ms, name, item)) {
            rollbackTx_();
            return;
        }
    }

    for (const auto& [name, item] : snap.items) {
        if (isBmsShadowItem_(name) || name == "BMS") continue;
        if (changed_devices.find(name) == changed_devices.end()) continue;

        if (name == "GasDetector") {
            if ( !insertGasChannelDelta_(snapshot_id, snap.timestamp_ms, name, item)) {
                rollbackTx_();
                return;
            }
        }
        else if (name == "SmokeSensor") {
            if (!insertSmokeState_(snapshot_id, snap.timestamp_ms, name, item)) {
                rollbackTx_();
                return;
            }
        }
        else if (name == "UPS") {
            if (!insertUpsQ1State_(snapshot_id, snap.timestamp_ms, name, item) ||
                !insertUpsQ6State_(snapshot_id, snap.timestamp_ms, name, item)) {
                rollbackTx_();
                return;
            }
        }
        else if (name == "AirConditioner") {
            if (!insertAirconState_(snapshot_id, snap.timestamp_ms, name, item)) {
                rollbackTx_();
                return;
            }
        }
        else if (name == "PCU" || name.rfind("PCU_", 0) == 0) {
            if (!insertPcuState_(snapshot_id, snap.timestamp_ms, name, item) ||
                !insertPcuTxState_(snapshot_id, snap.timestamp_ms, name, item)) {
                rollbackTx_();
                return;
                }
        }
    }

    if (!commitTx_()) {
        rollbackTx_();
    }
}