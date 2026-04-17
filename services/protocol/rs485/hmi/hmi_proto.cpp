#include "hmi_proto.h"

#include "logger.h"
#include "../modbus/modbus_crc.h"   // 按你项目实际 include 路径调整


HMIProto::HMIProto(uint8_t slave_id) : addr_(slave_id) {
    // ✅ 不再使用 compat
}

void HMIProto::appendCrc(std::vector<uint8_t>& out) {
    uint16_t crc = modbusCRC::calc(out);
    out.push_back(uint8_t(crc & 0xFF));        // CRC_LO
    out.push_back(uint8_t((crc >> 8) & 0xFF)); // CRC_HI
}

bool HMIProto::handleRequest(const std::vector<uint8_t>& req, std::vector<uint8_t>& resp) {
    resp.clear();

    if (req.size() < 8) return false;
    // if (req[1] == 0x04)
    // {
    //     LOG_COMM_HEX("[HMI][RX][REQ]", req.data(), req.size());
    // }
    if (req[1] == 0x0F || req[1] == 0x05)
    {
        LOG_COMM_HEX("[HMI][RX][REQ]", req.data(), req.size());
    }

    const uint8_t ra   = req[0];
    const uint8_t func = req[1];

    if (ra != addr_) return false; // 从站 ID 过滤

    // CRC 错：吞掉不回
    if (!modbusCRC::verify(req)) return true;

    const uint16_t start    = be16(&req[2]);
    const uint16_t v_or_qty = be16(&req[4]);

    // printf( "func 0x%04X, start 0x%04X\n",func, start);
    switch (func) {
        case 0x01:
        case 0x02:
            if (v_or_qty == 0) return true;
            return handleReadBits(func, start, v_or_qty, resp);

        case 0x03:
        case 0x04:
            if (v_or_qty == 0) return true;
            return handleReadRegs(func, start, v_or_qty, resp);

        case 0x05:
            return handleWriteSingleCoil(req, start, v_or_qty, resp);

        case 0x06:
            return handleWriteSingleReg(req, start, v_or_qty, resp);

        case 0x0F:
            return handleWriteMultiCoils(req, start, v_or_qty, resp);

        case 0x10:
            return handleWriteMultiRegs(req, start, v_or_qty, resp);

        default:
            return true; // 吞掉
    }
}
#include "getTime.h"

bool HMIProto::handleReadBits(uint8_t func, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp) {
    const uint16_t byte_count = (qty + 7) / 8;

    resp.clear();
    resp.reserve(3 + byte_count + 2);
    resp.push_back(addr_);
    resp.push_back(func);
    resp.push_back(uint8_t(byte_count));
    for (uint16_t i = 0; i < byte_count; ++i) resp.push_back(0x00);

    for (uint16_t i = 0; i < qty; ++i) {
        const uint16_t addr = uint16_t(start + i);
        bool v = false;

        if (func == 0x01) {
            // Coils: bool_rw 0x0050..0x006F
            uint16_t out = 0;
            if (table_.mapCoilAddr(addr, out)) {
                table_.readBoolRw(addr, v); // ✅ addr 就是屏幕地址
            }
        } else {
            // Discrete Inputs: bool_read 0x0100..0x2FFF
            uint16_t out = 0;
            if (table_.mapDiscreteAddr(addr, out)) {
                table_.readBoolRead(addr, v); // ✅ addr 就是屏幕地址
            }
        }

        if (v) {
            const uint16_t byte_i = i / 8;
            const uint16_t bit_i  = i % 8;
            resp[3 + byte_i] |= uint8_t(1u << bit_i);
        }
    }

    // ===== DEBUG: 打印回包数据区（不含CRC）=====
    if (func == 0x02) { // 只关心离散输入
        // resp: [addr][func][byte_count][data...]
        std::string hex;
        char buf[8];
        for (uint16_t i = 0; i < byte_count && (3 + i) < resp.size(); ++i) {
            snprintf(buf, sizeof(buf), "%02X ", resp[3 + i]);
            hex += buf;
        }
        // LOGD("[HMI_PROTO][DI_READ] start=0x%04X qty=%u bytes=%u data=%s",
             // start, qty, byte_count, hex.c_str());
    }
    //0207
    // if (func == 0x02 && start == 0x0115) {
    //     uint64_t now = nowMs(); /* nowMs/steady_clock */
    //     LOGD("[AC_LAT][T4_TX] now=%llu start=0x%04X qty=%u bytes=%u data=%02X %02X %02X %02X %02X",
    //          (unsigned long long)now, start, qty, byte_count,
    //          resp.size() > 3 ? resp[3] : 0,
    //          resp.size() > 4 ? resp[4] : 0,
    //          resp.size() > 5 ? resp[5] : 0,
    //          resp.size() > 6 ? resp[6] : 0,
    //          resp.size() > 7 ? resp[7] : 0);
    // }
    // LOG_COMM_HEX("[HMI][TX][RESP]", resp.data(), resp.size());
    appendCrc(resp);

    return true;
}
bool HMIProto::handleReadRegs(uint8_t func, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp) {
    const uint16_t byte_count = qty * 2;

    resp.clear();
    resp.reserve(3 + byte_count + 2);
    resp.push_back(addr_);
    resp.push_back(func);
    resp.push_back(uint8_t(byte_count));
    if (start ==0x4007)//>= HMI_INT_RD_START && start <= HMI_INT_RD_END)
    {
        // printf("read reg 0x4018\n");
    }
    for (uint16_t i = 0; i < qty; ++i) {
        const uint16_t addr = uint16_t(start + i);
        if (addr ==0x4018)
        {
            // printf("read reg 0x4018\n");
        }
        uint16_t v = 0;

        if (func == 0x03) {
            // Holding: int_rw 0x0070..0x008F
            uint16_t out = 0;
            if (table_.mapHoldingAddr(addr, out)) {
                table_.readIntRw(addr, v); // ✅ addr 就是屏幕地址
            }
        } else {
            // Input Registers: int_read 0x3000..0x4FFF
            uint16_t out = 0;
            if (table_.mapInputRegAddr(addr, out)) {
                table_.readIntRead(addr, v); // ✅ addr 就是屏幕地址
            }
        }

        // if (addr == 0x142D || addr == 0x142E || addr == 0x142F || addr == 0x1430) {
        //     printf("[HMI][READ_REGS] func=0x%02X start=0x%04X qty=%u addr=0x%04X v=%u\n",
        //            func, start, qty, addr, v);
        // }
        resp.push_back(uint8_t((v >> 8) & 0xFF));
        resp.push_back(uint8_t(v & 0xFF));
    }

    if (start == 0x4125)
    {
        LOG_COMM_HEX("[HMI][TX][RESP]", resp.data(), resp.size());
    }
    appendCrc(resp);
    return true;
}

bool HMIProto::handleWriteSingleCoil(const std::vector<uint8_t>& req, uint16_t addr, uint16_t value, std::vector<uint8_t>& resp) {
    // FC05 写单线圈：bool_rw
    uint16_t out = 0;
    if (!table_.mapCoilAddr(addr, out)) return true; // 越界吞掉

    const bool v = (value == 0xFF00);
    table_.writeBoolRw(addr, v); // ✅ addr=屏幕地址

    if (on_write_) {
        HmiWriteEvent ev;
        ev.addr = addr;          // ✅ 屏幕地址
        ev.is_bool = true;
        ev.value_u16 = v ? 1 : 0;
        ev.is_multi = false;
        on_write_(ev);
    }

    // 回显：响应=请求前 6 字节 + CRC
    resp.assign(req.begin(), req.begin() + 6);
    appendCrc(resp);
    return true;
}

bool HMIProto::handleWriteSingleReg(const std::vector<uint8_t>& req, uint16_t addr, uint16_t value, std::vector<uint8_t>& resp) {
    // FC06 写单保持寄存器：int_rw
    uint16_t out = 0;
    if (!table_.mapHoldingAddr(addr, out)) return true;

    table_.writeIntRw(addr, value); // ✅ addr=屏幕地址

    if (on_write_) {
        HmiWriteEvent ev;
        ev.addr = addr;          // ✅ 屏幕地址
        ev.is_bool = false;
        ev.value_u16 = value;
        ev.is_multi = false;
        on_write_(ev);
    }

    resp.assign(req.begin(), req.begin() + 6);
    appendCrc(resp);
    return true;
}

bool HMIProto::handleWriteMultiCoils(const std::vector<uint8_t>& req, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp) {
    // FC0F：addr func start qty bytecount data... crc
    if (req.size() < 9) return true;
    const uint8_t bytecount = req[6];
    const size_t need = size_t(7) + bytecount + 2;
    if (req.size() < need) return true;

    for (uint16_t i = 0; i < qty; ++i) {
        const uint16_t addr = uint16_t(start + i);

        const uint16_t byte_i = i / 8;
        const uint16_t bit_i  = i % 8;
        if (byte_i >= bytecount) break;

        const bool v = ((req[7 + byte_i] >> bit_i) & 0x01) != 0;

        uint16_t out = 0;
        if (table_.mapCoilAddr(addr, out)) {
            table_.writeBoolRw(addr, v);

            if (on_write_) {
                HmiWriteEvent ev;
                ev.addr = addr;  // ✅ 屏幕地址
                ev.is_bool = true;
                ev.value_u16 = v ? 1 : 0;
                ev.is_multi = true;
                on_write_(ev);
            }
        }
    }

    // 响应：addr func start qty + CRC
    resp.clear();
    resp.push_back(addr_);
    resp.push_back(0x0F);
    resp.push_back(uint8_t((start >> 8) & 0xFF));
    resp.push_back(uint8_t(start & 0xFF));
    resp.push_back(uint8_t((qty >> 8) & 0xFF));
    resp.push_back(uint8_t(qty & 0xFF));
    appendCrc(resp);
    return true;
}

bool HMIProto::handleWriteMultiRegs(const std::vector<uint8_t>& req, uint16_t start, uint16_t qty, std::vector<uint8_t>& resp) {
    // FC10：addr func start qty bytecount data... crc
    if (req.size() < 9) return true;
    const uint8_t bytecount = req[6];
    const size_t need = size_t(7) + bytecount + 2;
    if (req.size() < need) return true;

    const uint16_t reg_n = uint16_t(bytecount / 2);
    const uint16_t n = (qty < reg_n) ? qty : reg_n;

    for (uint16_t i = 0; i < n; ++i) {
        const uint16_t addr = uint16_t(start + i);

        const uint8_t hi = req[7 + i * 2];
        const uint8_t lo = req[7 + i * 2 + 1];
        const uint16_t v = (uint16_t(hi) << 8) | uint16_t(lo);

        uint16_t out = 0;
        if (table_.mapHoldingAddr(addr, out)) {
            table_.writeIntRw(addr, v);

            if (on_write_) {
                HmiWriteEvent ev;
                ev.addr = addr;  // ✅ 屏幕地址
                ev.is_bool = false;
                ev.value_u16 = v;
                ev.is_multi = true;
                on_write_(ev);
            }
        }
    }

    // 响应：addr func start qty + CRC
    resp.clear();
    resp.push_back(addr_);
    resp.push_back(0x10);
    resp.push_back(uint8_t((start >> 8) & 0xFF));
    resp.push_back(uint8_t(start & 0xFF));
    resp.push_back(uint8_t((qty >> 8) & 0xFF));
    resp.push_back(uint8_t(qty & 0xFF));
    appendCrc(resp);
    return true;
}
