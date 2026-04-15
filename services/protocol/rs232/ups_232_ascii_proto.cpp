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
static inline bool isDashDouble(const std::string& tok) {
    // ---.- 或 ----. 或 --- 等也算缺失
    if (tok.empty()) return false;
    if (tok.find('-') == std::string::npos) return false;
    return tok.find_first_not_of("-._") == std::string::npos;
}

static inline bool isDashInt(const std::string& tok) {
    if (tok.empty()) return false;
    if (tok.find('-') == std::string::npos) return false;
    return tok.find_first_not_of("-") == std::string::npos;
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

std::vector<uint8_t> Ups232AsciiProto::buildShutdownCmd(const std::string& n) {
    std::string cmd = "S";
    cmd += n;
    cmd.push_back(char(0x0D));
    return {cmd.begin(), cmd.end()};
}

std::vector<uint8_t> Ups232AsciiProto::buildWriteCmd(uint16_t, uint16_t) {
    // 默认：0.3 分钟
    return buildShutdownCmd(".3");
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

bool Ups232AsciiProto::parseDoubleToken(const std::string& tok, double& out) {
    if (tok.empty()) return false;
    if (tok.find('-') != std::string::npos &&
        tok.find_first_not_of("-._") == std::string::npos)
        return false;

    char* end = nullptr;
    out = std::strtod(tok.c_str(), &end);
    return end && end != tok.c_str();
}

bool Ups232AsciiProto::parseIntToken(const std::string& tok, int& out) {
    if (tok.empty()) return false;
    if (isDashInt(tok)) return false;

    // 必须全部是数字（允许前导 + 或 -）
    size_t p = 0;
    if (tok[0] == '+' || tok[0] == '-') p = 1;
    if (p >= tok.size()) return false;
    for (; p < tok.size(); ++p) if (!std::isdigit((unsigned char)tok[p])) return false;

    long v = std::strtol(tok.c_str(), nullptr, 10);
    out = (int)v;
    return true;
}

void Ups232AsciiProto::fillBits8(const std::string& bits01,
                                DeviceData& out,
                                const std::string& prefix) {
    if (bits01.size() != 8) return;

    // 将 8 位 0/1 字符串 (如 "10001001") 转换为 10 进制整数
    uint32_t raw_val = std::stoul(bits01, nullptr, 2);

    //  核心修正：注入 Logic 层正在监听的 "status_bits" 键
    out.status["status_bits"] = raw_val;

    // 保留旧的字典键以兼容其他可能的业务逻辑
    out.status[prefix + "raw"]            = raw_val;
    out.status[prefix + "mains_abnormal"] = (bits01[0] == '1');
    out.status[prefix + "battery_low"]    = (bits01[1] == '1');
    out.status[prefix + "bypass"]         = (bits01[2] == '1');
    out.status[prefix + "fault"]          = (bits01[3] == '1');
    out.status[prefix + "standby"]        = (bits01[4] == '1');
    out.status[prefix + "testing"]        = (bits01[5] == '1');
}

/* =======================
 * Q1
 * ======================= */

bool Ups232AsciiProto::parseQ1(const std::vector<std::string>& t, DeviceData& out) {
    double vin=0, vin_last=0, vout=0, fin=0, cell=0, temp=0;
    int load=0;

    if (!parseDoubleToken(t[0], vin)) return false;
    parseDoubleToken(t[1], vin_last);
    parseDoubleToken(t[2], vout);
    parseIntToken   (t[3], load);
    parseDoubleToken(t[4], fin);
    parseDoubleToken(t[5], cell);
    parseDoubleToken(t[6], temp);

    out.num["input.v"]        = vin;
    out.num["input.last.v"]   = vin_last;
    out.num["output.v"]       = vout;
    out.num["load.pct"]       = load;
    out.num["input.freq.hz"]  = fin;
    out.num["battery.cell.v"] = cell;
    out.num["battery.12v.v"]  = cell * 6.0;
    out.num["temp.c"]         = temp;

    fillBits8(t[7], out, "ups.");

    return true;
}

/* =======================
 * WA
 * ======================= */

bool Ups232AsciiProto::parseWA(const std::vector<std::string>& t, DeviceData& out) {
    if (t.size() != 13) return false;

    double w_r=0,w_s=0,w_t=0, va_r=0,va_s=0,va_t=0;
    double w_total=0, va_total=0;
    double a_r=0,a_s=0,a_t=0;
    int load=0;

    parseDoubleToken(t[0], w_r);
    parseDoubleToken(t[1], w_s);
    parseDoubleToken(t[2], w_t);
    parseDoubleToken(t[3], va_r);
    parseDoubleToken(t[4], va_s);
    parseDoubleToken(t[5], va_t);
    parseDoubleToken(t[6], w_total);
    parseDoubleToken(t[7], va_total);
    parseDoubleToken(t[8], a_r);
    parseDoubleToken(t[9], a_s);
    parseDoubleToken(t[10],a_t);
    parseIntToken   (t[11],load);

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
    out.num["load.pct"]            = load;

    fillBits8(t[12], out, "ups.");

    return true;
}

/* =======================
 * Q6
 * ======================= */

bool Ups232AsciiProto::parseQ6(const std::vector<std::string>& t, DeviceData& out) {
    // 你的实际机型：Q6 token = 20
    // 0..15: 常规数值
    // 16: KB (两字符)
    // 17: fault hex
    // 18: warning hex
    // 19: YO (两字符)
    if (t.size() < 20) return false;

    auto D = [&](size_t i, const char* k){
        const std::string key(k);
        if (isDashDouble(t[i])) { out.status[key + ".valid"] = 0; return; }

        double v;
        if (parseDoubleToken(t[i], v)) {
            out.status[key + ".valid"] = 1;
            out.num[k] = v;
        }
    };

    auto I = [&](size_t i, const char* k){
        const std::string key(k);
        if (isDashInt(t[i])) { out.status[key + ".valid"] = 0; return; }

        int v;
        if (parseIntToken(t[i], v)) {
            out.status[key + ".valid"] = 1;
            out.value[k] = v;
        }
    };

    // 浮点
    D(0,"input.v.r");  D(1,"input.v.s");  D(2,"input.v.t");
    D(3,"input.freq.hz");

    D(4,"output.v.r");
    // 5,6 可能是 ---.-（单相机型缺省），parseDoubleToken 会失败 => 不写入 out.num
    D(5,"output.v.s");
    D(6,"output.v.t");

    D(7,"output.freq.hz");

    // 电流：8 是 "010"，9/10 是 "---"（缺省）
    D(8,"output.i.r");
    D(9,"output.i.s");
    D(10,"output.i.t");

    D(11,"battery.v.pos");
    D(12,"battery.v.neg"); // 这里通常是 ---.-

    D(13,"temp.c");

    // 整数
    I(14,"battery.remain.sec");
    I(15,"battery.capacity");

    // KB：t[16] = "31" => K=3, B=1
    if (t[16].size() >= 2) {
        char K = t[16][0];
        char B = t[16][1];
        if (K >= '0' && K <= '9') out.value["system.mode"] = K - '0';
        if (B >= '0' && B <= '9') out.value["battery.test.state"] = B - '0';
    }

    // fault / warning hex
    try {
        uint32_t f_code_raw = (uint32_t)std::stoul(t[17], nullptr, 16);
        uint32_t w_bits     = (uint32_t)std::stoul(t[18], nullptr, 16);

        // 核心修正：拆解 4 个故障容器 (每个容器 8 位，存储 1 个故障码)
        out.value["ups_fault_code_1"] = (int32_t)((f_code_raw >> 24) & 0xFF);
        out.value["ups_fault_code_2"] = (int32_t)((f_code_raw >> 16) & 0xFF);
        out.value["ups_fault_code_3"] = (int32_t)((f_code_raw >> 8)  & 0xFF);
        out.value["ups_fault_code_4"] = (int32_t)((f_code_raw >> 0)  & 0xFF);

        // 保留基础值以兼容可能存在的 "fault_code_nonzero" 判断逻辑
        out.value["ups_fault_code"] = (int32_t)f_code_raw;
        out.status["warning_bits"]  = w_bits;
        out.status["warning.bits"]  = w_bits;
    } catch (...) {
        return false;
    }

    // YO：t[19] = "11"
    if (t[19].size() >= 2) {
        out.status["transformer.y"] = (t[19][0] == '1');
        out.status["lcd.phase.v"]   = (t[19][1] == '1');
    }

    return true;
}
