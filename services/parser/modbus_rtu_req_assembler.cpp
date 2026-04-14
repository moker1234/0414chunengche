//
// Created by lxy on 2026/2/3.
//

#include "modbus_rtu_req_assembler.h"
#include "../../services/protocol/modbus/modbus_crc.h"  // 按你实际路径

void ModbusRtuReqAssembler::onBytes(int idx, const std::vector<uint8_t>& bytes) {
    auto& b = bufs_[idx];
    for (auto v : bytes) b.push_back(v);
}

int ModbusRtuReqAssembler::expectedLen(const std::deque<uint8_t>& buf) {
    if (buf.size() < 2) return -1;
    const uint8_t func = buf[1];

    // 01/02/03/04/05/06 固定 8 字节请求
    if (func == 0x01 || func == 0x02 || func == 0x03 || func == 0x04 ||
        func == 0x05 || func == 0x06) {
        return 8;
        }

    // 0F/10: 至少要有 7 字节才能读 bytecount
    if (func == 0x0F || func == 0x10) {
        if (buf.size() < 7) return -1;
        const uint8_t bytecount = buf[6];
        return 7 + bytecount + 2;
    }

    // 其他功能码：你说不处理异常，可以直接认为未知，交给上层丢弃
    return -2;
}

bool ModbusRtuReqAssembler::tryGetFrame(int idx, std::vector<uint8_t>& frame) {
    frame.clear();
    auto it = bufs_.find(idx);
    if (it == bufs_.end()) return false;

    auto& b = it->second;

    while (true) {
        if (b.size() < 4) return false; // addr+func+crc(2) 最小保护

        int exp = expectedLen(b);
        if (exp == -1) return false; // 还不够判断
        if (exp == -2) {             // 未知功能码：丢 1 字节 resync
            b.pop_front();
            continue;
        }

        if ((int)b.size() < exp) return false;

        // 拿 exp 字节组成一帧
        frame.resize(exp);
        for (int i = 0; i < exp; ++i) {
            frame[i] = b.front();
            b.pop_front();
        }

        // CRC 校验失败：丢弃该帧首字节进行重同步（把剩余字节塞回前面）
        if (!modbusCRC::verify(frame)) {
            // resync：把 frame[1..] 放回队头（等价于“滑窗移1字节”）
            for (int i = exp - 1; i >= 1; --i) b.push_front(frame[i]);
            frame.clear();
            continue;
        }

        return true; // 得到一帧合法 Modbus RTU 请求
    }
}
