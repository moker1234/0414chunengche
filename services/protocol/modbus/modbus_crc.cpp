//
// Created by forlinx on 2025/12/31.
//
/* 解释整个文件的作用
 * 该文件实现了Modbus协议的CRC16计算函数。
 * 包括计算CRC16值和验证CRC16值的函数。
 */
#include "modbus_crc.h"


#include <cstdint>
#include <vector>

#include "./modbus_crc.h"

#include <cstdio>

// #define  DEBUG_CRC

/*
 * @brief 计算 Modbus CRC16
 * @param data 数据指针
 * @param len 数据长度
 * @return uint16_t CRC16 值
 */
uint16_t modbusCRC::calc(const uint8_t* data, uint16_t len)
{
#ifdef DEBUG_CRC
    printf("[MODBUS][CRC] data =");
    for (uint16_t i = 0; i < len; ++i)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
#endif
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t modbusCRC::calc(const std::vector<uint8_t>& data)
{
    if (data.empty()) return 0;
    return calc(data.data(), data.size());
}

bool modbusCRC::verify(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 4)
    {
        return false; // 最少 addr + func + crc16
    }

    uint16_t recv_crc =
        frame[frame.size() - 2] |
        (frame[frame.size() - 1] << 8);

    uint16_t calc_crc = calc(frame.data(),
                             frame.size() - 2);

    return recv_crc == calc_crc;
}
