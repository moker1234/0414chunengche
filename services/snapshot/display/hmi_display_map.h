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

    /*
     * 新版 normal_map_logic.jsonl 支持每个 item 显式带地址：
     *
     * {
     *   "地址hex": "0x4000",
     *   "长度/int16": 1,
     *   "变量描述": "...",
     *   "变量类型": "UINT16",
     *   "缩放": 1,
     *   "path": "system.logic_view.xxx"
     * }
     *
     * 第一批只负责解析保存这些字段；
     * 第二批 NormalHmiWriter 再改为优先使用 item.addr。
     */
    bool has_addr{false};
    uint16_t addr{0};

    // 来自 “长度/int16”，默认 1
    uint16_t words{1};

    // 来自 “变量类型”
    std::string var_type;

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
    // 旧版来自 name；新版来自 “变量描述”
    std::string name;
};

struct HmiMapBlock {
    /*
     * 旧版格式：
     *   {"type":"int_read","base":"0x4000","items":[...]}
     *
     * 新版格式：
     *   {"type":"int_read","items":[{"地址hex":"0x4000",...}]}
     *
     * 为兼容旧 NormalHmiWriter，第一批在新版格式中也会把 base 设为
     * 第一个显式地址。但真正正确的写屏地址，要等第二批使用 item.addr。
     */
    bool has_base{false};
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
    bool loadJsonl(const std::string& path,
                   HmiDisplayMap& out,
                   std::string* err = nullptr);

    static bool parseHexU16(const std::string& s, uint16_t& out);
private:
    static bool parseType(const std::string& s, HmiValType& out);

    static void fillOpt(const nlohmann::json& j,
                        const char* key,
                        std::optional<double>& dst);

    static void fillOptInt(const nlohmann::json& j,
                           const char* key,
                           std::optional<int>& dst);
};