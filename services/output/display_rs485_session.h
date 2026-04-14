//
// Created by lxy on 2026/1/12.
//
/**
 * 写屏会话
 *  - 管理与显示设备的 RS485 通信
 *  - 处理写屏数据映射
 */

#pragma once

#include "../aggregator/data_aggregator.h"
#include "../display/display_data_map.h"
#include "../device/device_types.h"

#include <deque>
#include <vector>
#include <functional>

class DisplayRs485Session {
public:
    using SendSerialFn =
        std::function<void(dev::LinkType, int, const std::vector<uint8_t>&)>;

    explicit DisplayRs485Session(int rs485_index);

    void setSendSerial(SendSerialFn fn);

    // Aggregator → Display
    void onSnapshot(const agg::SystemSnapshot& snap);

    // 串口 RX → Display
    void onSerialRx(const dev::SerialRxPacket& rx);

    // 定时驱动
    void onTick();

private:
    enum class State {
        Idle,
        WaitingResp,
    };

    // ---- 协议 ----
    std::vector<uint8_t> buildWriteFrame(const display::WordBlock& blk);
    std::vector<uint8_t> buildReadFrame(uint16_t addr, uint8_t words);

    bool tryParseRx();

    static uint16_t crc16(const uint8_t* data, size_t len);
    static void pushU16(std::vector<uint8_t>& buf, uint16_t v);

private:
    int rs485_index_;
    State state_{State::Idle};

    display::DisplayDataMap data_map_;

    std::deque<display::WordBlock> tx_queue_;
    std::vector<uint8_t> rx_buf_;

    SendSerialFn send_serial_;
};
