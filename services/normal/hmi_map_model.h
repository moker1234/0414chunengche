//
// Created by lxy on 2026/4/27.
//

#ifndef ENERGYSTORAGE_HMI_MAP_MODEL_H
#define ENERGYSTORAGE_HMI_MAP_MODEL_H

// services/normal/hmi_map_model.h
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace normal {

/*
 * normal_map_logic.jsonl 的统一类型。
 *
 * 注意：
 * - BoolRead / IntRead 用于 HMI 只读显示区
 * - BoolRw / IntRw 用于 HMI 写入/读写区
 * - U16 / S16 / Bit 是兼容旧 display_map 风格
 */
enum class HmiMapType : uint8_t {
    Unknown = 0,

    BoolRead,
    BoolRw,
    IntRead,
    IntRw,

    // legacy / compatible
    U16,
    S16,
    Bit,
};

inline const char* hmiMapTypeName(HmiMapType t)
{
    switch (t) {
    case HmiMapType::BoolRead: return "bool_read";
    case HmiMapType::BoolRw:   return "bool_rw";
    case HmiMapType::IntRead:  return "int_read";
    case HmiMapType::IntRw:    return "int_rw";
    case HmiMapType::U16:      return "u16";
    case HmiMapType::S16:      return "s16";
    case HmiMapType::Bit:      return "bit";
    default:                   return "unknown";
    }
}

inline bool isBoolMapType(HmiMapType t)
{
    return t == HmiMapType::BoolRead ||
           t == HmiMapType::BoolRw ||
           t == HmiMapType::Bit;
}

inline bool isIntMapType(HmiMapType t)
{
    return t == HmiMapType::IntRead ||
           t == HmiMapType::IntRw ||
           t == HmiMapType::U16 ||
           t == HmiMapType::S16;
}

inline bool isRwMapType(HmiMapType t)
{
    return t == HmiMapType::BoolRw ||
           t == HmiMapType::IntRw;
}

inline bool isReadOutputMapType(HmiMapType t)
{
    return t == HmiMapType::BoolRead ||
           t == HmiMapType::IntRead ||
           t == HmiMapType::U16 ||
           t == HmiMapType::S16 ||
           t == HmiMapType::Bit;
}

/*
 * item 用途分类。
 *
 * 注意：
 * - FaultNum：故障页数值显示，后续只允许 FaultPageManager 写
 * - Rw：HMI 写入反查用
 * - Normal：普通 HMI 输出候选
 */
enum class HmiMapItemKind : uint8_t {
    Empty = 0,
    Normal,
    FaultNum,
    FaultButton,
    Rw,
    OtherHmi,
};

inline const char* hmiMapItemKindName(HmiMapItemKind k)
{
    switch (k) {
    case HmiMapItemKind::Empty:       return "empty";
    case HmiMapItemKind::Normal:      return "normal";
    case HmiMapItemKind::FaultNum:    return "fault_num";
    case HmiMapItemKind::FaultButton: return "fault_button";
    case HmiMapItemKind::Rw:          return "rw";
    case HmiMapItemKind::OtherHmi:    return "other_hmi";
    default:                          return "unknown";
    }
}

/*
 * 一行 JSONL block。
 *
 * 旧格式可能有：
 *   {"type":"int_read","base":"0x4000","items":[...]}
 *
 * 新格式可能没有 base，而每个 item 显式带：
 *   "地址hex": "0x411B"
 */
struct HmiMapBlock {
    HmiMapType type{HmiMapType::Unknown};
    std::string type_text;

    bool has_base{false};
    uint16_t base{0};

    // Bit 类型兼容字段
    std::string src;

    // block 默认参数
    std::optional<double> def;
    std::optional<double> scale;
    std::optional<double> offset;
    std::optional<double> minv;
    std::optional<double> maxv;
    std::optional<int> bit_index;

    // 调试信息
    uint32_t line_no{0};

    // 本 block 展开后在 model.items 中的索引
    std::vector<std::size_t> item_indexes;
};

/*
 * 展开后的单个 HMI 点位。
 *
 * 后续 NormalHmiWriter / FaultPageManager / HmiInputMapper
 * 都应该消费这个结构，而不是各自重新解析 jsonl。
 */
struct HmiMapItem {
    HmiMapType type{HmiMapType::Unknown};
    HmiMapItemKind kind{HmiMapItemKind::Empty};

    std::size_t block_index{0};
    std::size_t item_index{0};
    uint32_t line_no{0};

    bool block_has_base{false};
    uint16_t block_base{0};

    bool has_addr{false};
    uint16_t addr{0};

    // 一个变量占几个 16-bit 寄存器；bool 也统一为 1
    uint16_t words{1};

    // 变量路径，例如：
    //   system.logic_view.pcu1_online
    //   hmi.fault.num.cur_row1_code
    std::string path;

    // 中文描述 / 旧 name
    std::string name;

    // 变量类型，例如 BOOL / UINT16 / UDINT
    std::string value_type;

    // Bit 类型兼容字段，通常来自 block.src
    std::string src;

    // 有些后续写入需要知道原始 type 字符串
    std::string type_text;

    // 有效参数，item 没有时由 loader 继承 block 默认值
    std::optional<double> def;
    std::optional<double> scale;
    std::optional<double> offset;
    std::optional<double> minv;
    std::optional<double> maxv;

    // compare，用于 bool/bit 判断
    std::optional<double> eq;
    std::optional<double> gt;
    std::optional<double> ge;
    std::optional<double> lt;
    std::optional<double> le;

    std::optional<int> bit_index;

    bool hasPath() const
    {
        return !path.empty() && path != "_";
    }

    bool validAddress() const
    {
        return has_addr;
    }
};

/*
 * 统一 HMI 映射模型。
 *
 * 本批只新增模型，不接入业务。
 * 第二批 NormalHmiWriter 会从 normal_items 取普通输出。
 * 第三批 FaultPageManager 会从 fault_num_items 推导 FaultHmiLayout。
 */
struct HmiMapModel {
    std::vector<HmiMapBlock> blocks;
    std::vector<HmiMapItem> items;

    // 输出视图：保存 items 的索引，避免复制大对象
    std::vector<std::size_t> normal_items;
    std::vector<std::size_t> fault_num_items;
    std::vector<std::size_t> fault_button_items;
    std::vector<std::size_t> rw_items;

    // 反查视图
    std::unordered_map<std::string, std::vector<std::size_t>> by_path;
    std::unordered_map<uint16_t, std::vector<std::size_t>> by_addr;

    void clear()
    {
        blocks.clear();
        items.clear();
        normal_items.clear();
        fault_num_items.clear();
        fault_button_items.clear();
        rw_items.clear();
        by_path.clear();
        by_addr.clear();
    }

    bool empty() const
    {
        return items.empty();
    }

    const HmiMapItem* itemAt(std::size_t idx) const
    {
        if (idx >= items.size()) {
            return nullptr;
        }
        return &items[idx];
    }

    const std::vector<std::size_t>* findByPath(const std::string& path) const
    {
        auto it = by_path.find(path);
        if (it == by_path.end()) {
            return nullptr;
        }
        return &it->second;
    }

    const std::vector<std::size_t>* findByAddr(uint16_t addr) const
    {
        auto it = by_addr.find(addr);
        if (it == by_addr.end()) {
            return nullptr;
        }
        return &it->second;
    }

    const HmiMapItem* firstByPath(const std::string& path) const
    {
        const auto* v = findByPath(path);
        if (!v || v->empty()) {
            return nullptr;
        }
        return itemAt((*v)[0]);
    }

    const HmiMapItem* firstByAddr(uint16_t addr) const
    {
        const auto* v = findByAddr(addr);
        if (!v || v->empty()) {
            return nullptr;
        }
        return itemAt((*v)[0]);
    }
};

} // namespace normal



#endif //ENERGYSTORAGE_HMI_MAP_MODEL_H
