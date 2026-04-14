//
// Created by lxy on 2026/1/12.
//
/**
 * 构建写屏数据帧
 */

#include "../output/display_rs485_session.h"
#include <cstdio>

static constexpr uint16_t FRAME_HEAD = 0xAA55;
static constexpr uint8_t  CMD_WRITE  = 0x82;
static constexpr uint8_t  CMD_READ   = 0x83;

DisplayRs485Session::DisplayRs485Session(int rs485_index)
    : rs485_index_(rs485_index) {}

void DisplayRs485Session::setSendSerial(SendSerialFn fn) {
    send_serial_ = std::move(fn);
}

void DisplayRs485Session::onSnapshot(const agg::SystemSnapshot& snap) {
    auto blocks = data_map_.buildWriteBlocks(snap);
    for (auto& b : blocks) {
        tx_queue_.push_back(b);
    }
}

void DisplayRs485Session::onTick() {
    if (state_ != State::Idle) return;
    if (tx_queue_.empty()) return;
    if (!send_serial_) return;

    display::WordBlock blk = tx_queue_.front();
    tx_queue_.pop_front();

    auto frame = buildWriteFrame(blk);
    send_serial_(dev::LinkType::RS485, rs485_index_, frame);

    state_ = State::WaitingResp;
}

void DisplayRs485Session::onSerialRx(const dev::SerialRxPacket& rx) {
    if (rx.type != dev::LinkType::RS485) return;
    if (rx.serial_index != rs485_index_) return;

    rx_buf_.insert(rx_buf_.end(), rx.bytes.begin(), rx.bytes.end());
    tryParseRx();
}

bool DisplayRs485Session::tryParseRx() {
    if (rx_buf_.size() < 6) return false;

    uint16_t head = (rx_buf_[0] << 8) | rx_buf_[1];
    if (head != FRAME_HEAD) {
        rx_buf_.erase(rx_buf_.begin());
        return false;
    }

    uint8_t len = rx_buf_[2];
    size_t frame_len = 2 + 1 + len + 2;
    if (rx_buf_.size() < frame_len) return false;

    uint16_t crc_rx =
        (rx_buf_[frame_len - 2] << 8) |
         rx_buf_[frame_len - 1];

    uint16_t crc_calc =
        crc16(rx_buf_.data(), frame_len - 2);

    if (crc_rx != crc_calc) {
        rx_buf_.clear();
        state_ = State::Idle;
        return false;
    }

    uint8_t cmd = rx_buf_[3];
    if (cmd == CMD_READ) {
        // TODO：屏幕变量变化 → 系统事件（后续）
    }

    rx_buf_.erase(rx_buf_.begin(),
                  rx_buf_.begin() + frame_len);

    state_ = State::Idle;
    return true;
}

std::vector<uint8_t>
DisplayRs485Session::buildWriteFrame(const display::WordBlock& blk) {
    std::vector<uint8_t> buf;
    pushU16(buf, FRAME_HEAD);

    uint8_t len = 1 + 2 + blk.data.size() * 2;
    buf.push_back(len);
    buf.push_back(CMD_WRITE);
    pushU16(buf, blk.start_addr);

    for (auto w : blk.data) {
        pushU16(buf, w);
    }

    uint16_t crc = crc16(buf.data(), buf.size());
    pushU16(buf, crc);
    return buf;
}

std::vector<uint8_t>
DisplayRs485Session::buildReadFrame(uint16_t addr, uint8_t words) {
    std::vector<uint8_t> buf;
    pushU16(buf, FRAME_HEAD);
    buf.push_back(1 + 2 + 1);
    buf.push_back(CMD_READ);
    pushU16(buf, addr);
    buf.push_back(words);

    uint16_t crc = crc16(buf.data(), buf.size());
    pushU16(buf, crc);
    return buf;
}

void DisplayRs485Session::pushU16(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(v >> 8);
    buf.push_back(v & 0xFF);
}

uint16_t DisplayRs485Session::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}
