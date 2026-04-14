#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// 显示映射的值类型（对“屏幕区”做四分）
// - bool_read : 离散输入(02) 只读
// - bool_rw   : 线圈(01/05/0F) 可读可写
// - int_read  : 输入寄存器(04) 只读
// - int_rw    : 保持寄存器(03/06/10) 可读可写
//
// 兼容旧配置：bool/u16/s16/bit 仍可解析（映射到 read 区）
enum class HmiValType {
    BoolRead,
    BoolRw,
    IntRead,
    IntRw,

    // legacy / advanced
    U16,   // == IntRead
    S16,   // == IntRead(有符号打包)
    Bit    // == BoolRead(从整数取 bit)
};

struct HmiPathItem {
    // "_" 表示占位（不写）
    std::string path;

    // 可选字段：每个 item 覆盖 block
    std::optional<double> def;
    std::optional<double> scale;
    std::optional<double> offset;
    std::optional<double> minv;
    std::optional<double> maxv;

    // compare (for bool/bit)
    std::optional<double> eq;
    std::optional<double> gt;
    std::optional<double> ge;
    std::optional<double> lt;
    std::optional<double> le;

    // bit index for Bit mode
    std::optional<int> bit_index;

    // debug/注释
    std::string name;
};

struct HmiMapBlock {
    uint16_t base{0};
    HmiValType type{HmiValType::IntRead};

    // Bit 模式：统一 src
    std::string src;

    // block defaults
    std::optional<double> def;
    std::optional<double> scale;
    std::optional<double> offset;
    std::optional<double> minv;
    std::optional<double> maxv;
    std::optional<int> bit_index;

    std::vector<HmiPathItem> items;
};

struct HmiDisplayMap {
    std::vector<HmiMapBlock> blocks;
};

class HmiDisplayMapLoader {
public:
    bool loadJsonl(const std::string& path, HmiDisplayMap& out, std::string* err = nullptr);

private:
    static bool parseHexU16(const std::string& s, uint16_t& out);
    static bool parseType(const std::string& s, HmiValType& out);

    static void fillOpt(const nlohmann::json& j, const char* key, std::optional<double>& dst);
    static void fillOptInt(const nlohmann::json& j, const char* key, std::optional<int>& dst);
};
