//
// Created by lxy on 2026/2/3.
//

#ifndef ENERGYSTORAGE_MODBUS_RTU_REQ_ASSEMBLER_H
#define ENERGYSTORAGE_MODBUS_RTU_REQ_ASSEMBLER_H


#pragma once
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

class ModbusRtuReqAssembler {
public:
    void onBytes(int idx, const std::vector<uint8_t>& bytes);
    bool tryGetFrame(int idx, std::vector<uint8_t>& frame);

private:
    static int expectedLen(const std::deque<uint8_t>& buf);
    std::unordered_map<int, std::deque<uint8_t>> bufs_;
};



#endif //ENERGYSTORAGE_MODBUS_RTU_REQ_ASSEMBLER_H