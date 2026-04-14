// services/parser/protocol_parser_thread.h
//
// Created by lxy on 2026/1/19.
//

#ifndef ENERGYSTORAGE_PROTOCOL_PARSER_THREAD_H
#define ENERGYSTORAGE_PROTOCOL_PARSER_THREAD_H

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <unordered_map>
#include <string>

#include "../../services/device/device_base.h"
#include "parser_message.h"
#include "request_context.h"
#include "hmi/hmi_proto.h"
#include "../protocol/can/bms/bms_thread/bms_queue.h"


#define protocol_parser_thread_LOG 0

namespace parser {


//  P1：发送模式（Normal：不允许 pending 重发；Resend：允许并刷新 pending）
enum class PollSendMode {
    Normal,
};

class ProtocolParserThread {
public:
    using OnParsedFn = std::function<void(const ParserMessage&)>;
    using SendSerialFn =
        std::function<void(dev::LinkType /*type*/, int /*index*/, const std::vector<uint8_t>& /*bytes*/)>;

    ProtocolParserThread();
    ~ProtocolParserThread();

    void start();
    void stop();

    // Driver → Parser
    void pushSerial(const dev::SerialRxPacket& rx);
    void pushCan(const dev::CanRxPacket& rx);

    // Parser → App
    void setOnParsed(OnParsedFn cb);

    // Parser → Driver（真正发送串口字节）
    void setSendSerial(SendSerialFn fn);

    // 为了bms线程异步解析
    void setBmsQueue(int bms_can_index, proto::bms::BmsQueue* q);

    // Scheduler → Parser：执行一次轮询发送（决策在 Scheduler）
    // 新模型下 sendPoll 只负责“发”，不再负责 pending/timeout 健康链路
    // timeout_ms 参数继续保留，作为上层传入的断连窗口兼容参数，但本函数不再注册 pending
    bool sendPoll(dev::LinkType type,
                  int index,
                  const std::string& device_name,
                  uint32_t timeout_ms,
                  PollSendMode mode = PollSendMode::Normal);

    HMIProto* getHmiProto(int rs485_index);
private:
    void threadLoop();
    static uint64_t nowMs();

private:
    std::atomic<bool> running_{false};
    std::thread th_;

    std::mutex mtx_;
    std::deque<dev::SerialRxPacket> serial_q_;
    std::deque<dev::CanRxPacket>    can_q_;

    OnParsedFn on_parsed_;
    SendSerialFn send_serial_;

    // 兼容保留：pending 容器不再参与设备健康状态判定
    std::mutex pending_mtx_;

    // 兼容保留：不再定时驱动 timeout 事实
    uint64_t last_timeout_check_ms_{0};
    uint32_t timeout_check_interval_ms_{100};
};

} // namespace parser

#endif // ENERGYSTORAGE_PROTOCOL_PARSER_THREAD_H
