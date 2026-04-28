// Created by lxy on 2026/1/20.
/*
 * UPS 232 ASCII 协议实现
 */

#include "ups_232_ascii_proto.h"
#include "logger.h"
#include "hex_dump.h"

#include <cctype>
#include <cstdlib>
#include <sstream>

using std::string;
using std::vector;
// 检查是否为 10-10 分钟的浮点数
static inline bool isDashDouble(const std::string& tok) {
    // ---.- 或 ----. 或 --- 等也算缺失
    if (tok.empty()) return false;
    if (tok.find('-') == std::string::npos) return false;
    return tok.find_first_not_of("-._") == std::string::npos;
}
// 检查是否为 10-10 分钟的整数
static inline bool isDashInt(const std::string& tok) {
    if (tok.empty()) return false;
    if (tok.find('-') == std::string::npos) return false;
    return tok.find_first_not_of("-") == std::string::npos;
}
// 检查是否为 8 位 0/1 字符串
static inline bool isBits01_8(const std::string& tok)
{
    if (tok.size() != 8) return false;
    for (char c : tok) {
        if (c != '0' && c != '1') return false;
    }
    return true;
}
// 检查是否为 32 位十六进制字符串
static inline bool parseHexU32Token_(const std::string& tok, uint32_t& out)
{
    if (tok.empty()) return false;
    for (char c : tok) {
        if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
    }

    try {
        unsigned long v = std::stoul(tok, nullptr, 16);
        if (v > 0xFFFFFFFFul) return false;
        out = static_cast<uint32_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}
// 检查是否为 10-10 分钟的整数
static inline bool isValidShutdownDelay_(const std::string& n)
{
    // UPS 协议允许 .2 / .3 / 01..10，单位 min
    if (n == ".2" || n == ".3") return true;

    if (n.size() != 2) return false;
    if (!std::isdigit(static_cast<unsigned char>(n[0])) ||
        !std::isdigit(static_cast<unsigned char>(n[1]))) {
        return false;
        }

    int v = (n[0] - '0') * 10 + (n[1] - '0');
    return v >= 1 && v <= 10;
}



Ups232AsciiProto::Ups232AsciiProto() = default;

/* =======================
 * Poll / Command builder
 * ======================= */

std::vector<uint8_t> Ups232AsciiProto::buildReadCmd() {
    std::string cmd;
    switch (next_) {
        case PollKind::Q1: cmd = "Q1"; next_ = PollKind::Q6; break;
        case PollKind::Q6: cmd = "Q6"; next_ = PollKind::WA; break;
        case PollKind::WA: cmd = "WA"; next_ = PollKind::Q1; break;
    }
    cmd.push_back(char(0x0D)); // CR
    return {cmd.begin(), cmd.end()};
}

std::vector<uint8_t> Ups232AsciiProto::buildShutdownCmd(const std::string& n)
{
    if (!isValidShutdownDelay_(n)) {
        LOGERR("[UPS][CMD] invalid shutdown delay: %s", n.c_str());
        return {};
    }

    std::string cmd = "S";
    cmd += n;
    cmd.push_back(char(0x0D));
    return {cmd.begin(), cmd.end()};
}

std::vector<uint8_t> Ups232AsciiProto::buildWriteCmd(uint16_t, uint16_t)
{
    // 禁止默认 S.3。
    // UPS 关机命令必须由上层明确调用 buildShutdownCmd(".2/.3/01..10")。
    LOGERR("[UPS][CMD] buildWriteCmd disabled; use buildShutdownCmd(delay) explicitly");
    return {};
}

/* =======================
 * Parse entry
 * ======================= */

bool Ups232AsciiProto::parse(const std::vector<uint8_t>& rx, DeviceData& out) {
    // LOG_COMM_HEX("RX RS232#0 dev=Ups232Ascii ...", rx.data(), rx.size());

    out.device_name = "UPS";
    // 注意：timestamp 应由 Parser/Assembler 统一填充，协议层不要写 time(nullptr)

    std::string s = toString(rx);
    s = stripWrapper(s);
    if (s.empty()) return false;

    auto tokens = splitTokens(s);
    if (tokens.empty()) return false;

    /*
     * 协议区分规则（实践验证过，稳定）：
     * Q1 : 8  tokens
     * WA : 13 tokens（... + bits8）
     * Q6 : >= 21 tokens
     */
    bool parse_results = false;
    if (tokens.size() == 8) {
        markCmd_(PollKind::Q1, out);
        parse_results = parseQ1(tokens, out);
    }

    if (tokens.size() == 13) {
        markCmd_(PollKind::WA, out);
        parse_results = parseWA(tokens, out);
    }

    // 默认按 Q6
    if ((tokens.size() != 13) && (tokens.size() != 8))
    {
        markCmd_(PollKind::Q6, out);
        parse_results = parseQ6(tokens, out);
    }
    return parse_results;
}

/* =======================
 * Internal cmd mark
 * ======================= */

void Ups232AsciiProto::markCmd_(PollKind kind, DeviceData& out) {
    // 这个字段是给 Aggregator 做 UPS(Q1/Q6/WA) 分组用的
    // 后续你在 SystemSnapshot::toJson() 里可以统一过滤掉 "__" 前缀字段
    out.value["__ups_cmd"] = static_cast<int32_t>(kind);

    // 日志可用（但我们不写到 out.str，避免你后面清洗麻烦）
    LOGTRACE("[UPS][CMD] %s", cmdToString_(kind));
}

const char* Ups232AsciiProto::cmdToString_(PollKind k) {
    switch (k) {
        case PollKind::Q1: return "Q1";
        case PollKind::Q6: return "Q6";
        case PollKind::WA: return "WA";
        default:           return "UNKNOWN";
    }
}

/* =======================
 * String helpers
 * ======================= */

std::string Ups232AsciiProto::toString(const std::vector<uint8_t>& rx) {
    return std::string(rx.begin(), rx.end());
}

std::string Ups232AsciiProto::stripWrapper(const std::string& s) {
    std::string t = s;

    while (!t.empty() && std::isspace((unsigned char)t.front())) t.erase(t.begin());
    if (!t.empty() && t.front() == '(') t.erase(t.begin());

    while (!t.empty() && std::isspace((unsigned char)t.back())) t.pop_back();
    if (!t.empty() && t.back() == ')') t.pop_back();
    while (!t.empty() && std::isspace((unsigned char)t.back())) t.pop_back();

    return t;
}

std::vector<std::string> Ups232AsciiProto::splitTokens(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream iss(s);
    std::string tok;
    while (iss >> tok) out.push_back(tok);
    return out;
}

/* =======================
 * Token parse helpers
 * ======================= */
bool Ups232AsciiProto::parseDoubleToken(const std::string& tok, double& out)
{
    if (tok.empty()) return false;
    if (isDashDouble(tok)) return false;

    char* end = nullptr;
    out = std::strtod(tok.c_str(), &end);

    // 必须完整消费 token，避免 "12abc" 被解析成 12
    return end && end != tok.c_str() && *end == '\0';
}

bool Ups232AsciiProto::parseIntToken(const std::string& tok, int& out)
{
    if (tok.empty()) return false;
    if (isDashInt(tok)) return false;

    std::size_t p = 0;
    if (tok[0] == '+' || tok[0] == '-') p = 1;
    if (p >= tok.size()) return false;

    for (; p < tok.size(); ++p) {
        if (!std::isdigit(static_cast<unsigned char>(tok[p]))) return false;
    }

    char* end = nullptr;
    long v = std::strtol(tok.c_str(), &end, 10);
    if (!end || *end != '\0') return false;

    out = static_cast<int>(v);
    return true;
}

bool Ups232AsciiProto::fillBits8(const std::string& bits01,
                                 DeviceData& out,
                                 const std::string& prefix)
{
    if (!isBits01_8(bits01)) {
        LOGERR("[UPS][PARSE] invalid 8-bit status token: %s", bits01.c_str());
        return false;
    }

    uint32_t raw_val = 0;
    for (char c : bits01) {
        raw_val = (raw_val << 1) | static_cast<uint32_t>(c == '1');
    }

    // prefix 建议使用 "q1." 或 "wa."，避免 Q1/WA 状态位混淆。
    out.status[prefix + "status.bits"]    = raw_val;
    out.status[prefix + "mains_abnormal"] = (bits01[0] == '1'); // bit7
    out.status[prefix + "battery_low"]    = (bits01[1] == '1'); // bit6
    out.status[prefix + "bypass"]         = (bits01[2] == '1'); // bit5
    out.status[prefix + "fault"]          = (bits01[3] == '1'); // bit4
    out.status[prefix + "backup_mode"]    = (bits01[4] == '1'); // bit3
    out.status[prefix + "testing"]        = (bits01[5] == '1'); // bit2

    return true;
}

/* =======================
 * Q1
 * ======================= */

bool Ups232AsciiProto::parseQ1(const std::vector<std::string>& t, DeviceData& out)
{
    if (t.size() != 8) return false;

    double vin = 0.0;
    double vin_last = 0.0;
    double vout = 0.0;
    double fin = 0.0;
    double cell = 0.0;
    double temp = 0.0;
    int load = 0;

    if (!parseDoubleToken(t[0], vin)) return false;
    if (!parseDoubleToken(t[1], vin_last)) return false;
    if (!parseDoubleToken(t[2], vout)) return false;
    if (!parseIntToken(t[3], load)) return false;
    if (!parseDoubleToken(t[4], fin)) return false;
    if (!parseDoubleToken(t[5], cell)) return false;
    if (!parseDoubleToken(t[6], temp)) return false;
    if (!fillBits8(t[7], out, "q1.")) return false;

    out.num["input.v"]        = vin;
    out.num["input.last.v"]   = vin_last;
    out.num["output.v"]       = vout;
    out.num["load.pct"]       = static_cast<double>(load);
    out.num["input.freq.hz"]  = fin;
    out.num["battery.cell.v"] = cell;
    out.num["battery.12v.v"]  = cell * 6.0;
    out.num["temp.c"]         = temp;

    return true;
}

/* =======================
 * WA
 * ======================= */

bool Ups232AsciiProto::parseWA(const std::vector<std::string>& t, DeviceData& out)
{
    if (t.size() != 13) return false;

    double w_r = 0.0;
    double w_s = 0.0;
    double w_t = 0.0;
    double va_r = 0.0;
    double va_s = 0.0;
    double va_t = 0.0;
    double w_total = 0.0;
    double va_total = 0.0;
    double a_r = 0.0;
    double a_s = 0.0;
    double a_t = 0.0;
    int load = 0;

    if (!parseDoubleToken(t[0], w_r)) return false;
    if (!parseDoubleToken(t[1], w_s)) return false;
    if (!parseDoubleToken(t[2], w_t)) return false;
    if (!parseDoubleToken(t[3], va_r)) return false;
    if (!parseDoubleToken(t[4], va_s)) return false;
    if (!parseDoubleToken(t[5], va_t)) return false;
    if (!parseDoubleToken(t[6], w_total)) return false;
    if (!parseDoubleToken(t[7], va_total)) return false;
    if (!parseDoubleToken(t[8], a_r)) return false;
    if (!parseDoubleToken(t[9], a_s)) return false;
    if (!parseDoubleToken(t[10], a_t)) return false;
    if (!parseIntToken(t[11], load)) return false;
    if (!fillBits8(t[12], out, "wa.")) return false;

    out.num["power.kw.r"]          = w_r;
    out.num["power.kw.s"]          = w_s;
    out.num["power.kw.t"]          = w_t;
    out.num["apparent.kva.r"]      = va_r;
    out.num["apparent.kva.s"]      = va_s;
    out.num["apparent.kva.t"]      = va_t;
    out.num["power.total.kw"]      = w_total;
    out.num["apparent.total.kva"]  = va_total;
    out.num["output.i.r"]          = a_r;
    out.num["output.i.s"]          = a_s;
    out.num["output.i.t"]          = a_t;
    out.num["load.pct"]            = static_cast<double>(load);

    return true;
}

/* =======================
 * Q6
 * ======================= */

bool Ups232AsciiProto::parseQ6(const std::vector<std::string>& t, DeviceData& out)
{
    // 实测机型 token = 20：
    // 0..15: 数值
    // 16: KB
    // 17: fault hex，4 个 fault container，每个 8 bit
    // 18: warning hex，低 32 位 warning bits
    // 19: YO
    if (t.size() < 20) return false;

    auto D = [&](std::size_t i, const char* k) -> bool {
        const std::string key(k);

        if (i >= t.size()) return false;

        if (isDashDouble(t[i])) {
            out.status[key + ".valid"] = 0;
            return true;
        }

        double v = 0.0;
        if (!parseDoubleToken(t[i], v)) {
            out.status[key + ".valid"] = 0;
            return false;
        }

        out.status[key + ".valid"] = 1;
        out.num[k] = v;
        return true;
    };

    auto I = [&](std::size_t i, const char* k) -> bool {
        const std::string key(k);

        if (i >= t.size()) return false;

        if (isDashInt(t[i])) {
            out.status[key + ".valid"] = 0;
            return true;
        }

        int v = 0;
        if (!parseIntToken(t[i], v)) {
            out.status[key + ".valid"] = 0;
            return false;
        }

        out.status[key + ".valid"] = 1;
        out.value[k] = v;
        return true;
    };

    if (!D(0,  "input.v.r")) return false;
    if (!D(1,  "input.v.s")) return false;
    if (!D(2,  "input.v.t")) return false;
    if (!D(3,  "input.freq.hz")) return false;

    if (!D(4,  "output.v.r")) return false;
    if (!D(5,  "output.v.s")) return false;
    if (!D(6,  "output.v.t")) return false;
    if (!D(7,  "output.freq.hz")) return false;

    if (!D(8,  "output.i.r")) return false;
    if (!D(9,  "output.i.s")) return false;
    if (!D(10, "output.i.t")) return false;

    if (!D(11, "battery.v.pos")) return false;
    if (!D(12, "battery.v.neg")) return false;
    if (!D(13, "temp.c")) return false;

    if (!I(14, "battery.remain.sec")) return false;
    if (!I(15, "battery.capacity")) return false;

    // KB：K=system.mode，B=battery.test.state
    if (t[16].size() < 2 ||
        !std::isdigit(static_cast<unsigned char>(t[16][0])) ||
        !std::isdigit(static_cast<unsigned char>(t[16][1]))) {
        return false;
    }

    out.value["system.mode"] = t[16][0] - '0';
    out.value["battery.test.state"] = t[16][1] - '0';

    uint32_t fault_raw = 0;
    uint32_t warning_bits = 0;

    if (!parseHexU32Token_(t[17], fault_raw)) return false;
    if (!parseHexU32Token_(t[18], warning_bits)) return false;

    // Q6 处于 Q6 分组里，因此这里保留 fault.bits / warning.bits；
    // JSON 路径会是 items.UPS.data.Q6.status.fault.bits。
    out.status["fault.bits"]   = fault_raw;
    out.status["warning.bits"] = warning_bits;

    out.value["fault.raw"] = static_cast<int32_t>(fault_raw);

    // 4 个 fault container，每个 container 是一个 fault code。
    out.value["fault.code.1"] = static_cast<int32_t>((fault_raw >> 24) & 0xFFu);
    out.value["fault.code.2"] = static_cast<int32_t>((fault_raw >> 16) & 0xFFu);
    out.value["fault.code.3"] = static_cast<int32_t>((fault_raw >> 8)  & 0xFFu);
    out.value["fault.code.4"] = static_cast<int32_t>((fault_raw >> 0)  & 0xFFu);

    // YO
    if (t[19].size() < 2) return false;
    if ((t[19][0] != '0' && t[19][0] != '1') ||
        (t[19][1] != '0' && t[19][1] != '1')) {
        return false;
    }

    out.status["transformer.y"] = (t[19][0] == '1');
    out.status["lcd.phase.v"]   = (t[19][1] == '1');

    return true;
}
