// services/parser/protocol_parser_thread.cpp
//
// Created by lxy on 2026/1/19.
//
/*
 * 协议解析线程实现
 *
 * 职责：
 *  - RX 拼帧 + 协议解析
 *  - 轮询发送动作由 DeviceScheduler 通过 sendPoll(...) 触发
 *  - 新模型下，设备健康状态不再由 parser 的 pending/timeout 驱动
 *
 * 说明：
 *  - sendPoll 仍负责把查询字节发到底层
 *  - 成功解析后仍正常上报 DeviceData
 *  - 旧 pending/timeout 逻辑仅作兼容保留，不再参与健康状态判定
 */

#include "protocol_parser_thread.h"

#include <chrono>

#include "frame_assembler.h"
#include "protocol_router.h"
#include "logger.h"
#include "modbus_rtu_req_assembler.h"
#include "232ups_ascii/ascii_frame_assembler.h"

static parser::AsciiFrameAssembler g_ascii;
using namespace parser;

static FrameAssembler g_frame;
static ModbusRtuReqAssembler g_hmi_rtu;
static ProtocolRouter g_router;

ProtocolParserThread::ProtocolParserThread() = default;

ProtocolParserThread::~ProtocolParserThread()
{
    stop();
}

uint64_t ProtocolParserThread::nowMs()
{
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

void ProtocolParserThread::setOnParsed(OnParsedFn cb)
{
    on_parsed_ = std::move(cb);
}

void ProtocolParserThread::setSendSerial(SendSerialFn fn)
{
    send_serial_ = std::move(fn);
}

void ProtocolParserThread::setBmsQueue(int bms_can_index, proto::bms::BmsQueue* q)
{
    g_router.setBmsQueue(bms_can_index, q);
}

void ProtocolParserThread::start()
{
    if (running_.exchange(true)) return;
    th_ = std::thread(&ProtocolParserThread::threadLoop, this);
    LOG_SYS_I("PARSER thread started");
}

void ProtocolParserThread::stop()
{
    if (!running_.exchange(false)) return;
    if (th_.joinable()) th_.join();
    LOG_SYS_I("PARSER thread stopped");
}

void ProtocolParserThread::pushSerial(const dev::SerialRxPacket& rx)
{
    std::lock_guard<std::mutex> lk(mtx_);
    serial_q_.push_back(rx);
}

void ProtocolParserThread::pushCan(const dev::CanRxPacket& rx)
{
    std::lock_guard<std::mutex> lk(mtx_);
    can_q_.push_back(rx);
}


/**
 * @brief Scheduler 触发一次轮询发送
 *
 * 当前模型下：
 * - 发送节奏由 Scheduler 决定
 * - Parser 只负责把查询字节发到底层
 * - 设备健康状态只由成功 DeviceData + Scheduler aging 判定
 * - 不再维护 pending / resend / retry 状态机
 */
bool ProtocolParserThread::sendPoll(dev::LinkType type,
                                    int index,
                                    const std::string& device_name,
                                    uint32_t timeout_ms,
                                    PollSendMode mode)
{
    (void)device_name;
    (void)timeout_ms;
    (void)mode;

    if (!send_serial_) return false;

    std::vector<uint8_t> req;

    // ===== RS485 =====
    if (type == dev::LinkType::RS485)
    {
        req = g_router.buildQuery(index);

        if (req.empty()) return false;

        send_serial_(type, index, req);
        return true;
    }

    // ===== RS232 =====
    if (type == dev::LinkType::RS232)
    {
        req = g_router.buildQueryRs232(index);

        if (req.empty()) return false;

        send_serial_(type, index, req);
        return true;
    }

    return false;
}

void ProtocolParserThread::threadLoop()
{
    while (running_)
    {


        dev::SerialRxPacket srx{};
        dev::CanRxPacket    crx{};
        bool has_serial = false;
        bool has_can    = false;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!can_q_.empty()) {
                crx = std::move(can_q_.front());
                can_q_.pop_front();
                has_can = true;
            } else if (!serial_q_.empty()) {
                srx = std::move(serial_q_.front());
                serial_q_.pop_front();
                has_serial = true;
            }
        }

        if (!has_serial && !has_can)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        // ===== CAN：直接 parse（不需要拼帧）=====
        if (has_can) {
            DeviceData d;
            const auto r = g_router.parseCan(crx.can_index, crx.frame, d);

            if (r == proto::CanParseResult::Parsed) {
                ParserMessage msg;
                msg.type = ParsedType::DeviceData;
                msg.link_type = dev::LinkType::CAN;
                msg.link_index = crx.can_index;
                msg.device_data = std::move(d);
                msg.device_name = msg.device_data.device_name;
                msg.rx_ts_ms = nowMs();

                if (on_parsed_) on_parsed_(msg);
            } else if (r == proto::CanParseResult::Consumed) {
                // ✅ BMS async：已入队，不当作错误，不发 ParseError
            } else {
#if protocol_parser_thread_LOG
                LOG_THROTTLE_MS("can_parse_fail", 500, LOG_COMM_W,
                                "RX CAN#%d parse=FAIL id=0x%08X dlc=%d r=%d",
                                crx.can_index,
                                (crx.frame.can_id & CAN_EFF_MASK),
                                crx.frame.can_dlc,
                                (int)r);
#endif
                if (on_parsed_) {
                    ParserMessage err;
                    err.type = ParsedType::ParseError;
                    err.link_type = dev::LinkType::CAN;
                    err.link_index = crx.can_index;
                    err.error_code = -1;
                    err.error_text = "CAN parse failed";
                    err.rx_ts_ms = nowMs();
                    on_parsed_(err);
                }
            }

            continue;
        }


        // ===== RS485 / RS232：拼帧 + parse =====
        if (srx.type == dev::LinkType::RS485 || srx.type == dev::LinkType::RS232)
        {
            // chunk 级别（默认 TRACE，可用来查分片）
            if (!srx.bytes.empty())
            {
#if protocol_parser_thread_LOG
                LOG_COMM_T("RX_BYTES %s#%d chunk_len=%zu",
                           (srx.type==dev::LinkType::RS485)?"RS485":"RS232",
                           srx.serial_index,
                           srx.bytes.size());
#endif
            }

            std::vector<uint8_t> frame;

            if (srx.type == dev::LinkType::RS485)
            {
                const bool is_slave_port = g_router.isRs485SlavePort(srx.serial_index);

                // ===== 从站口（HMI）：用 Modbus RTU 请求拼帧器 + 吞掉 =====
                if (is_slave_port)
                {
                    // 用专用拼帧器（你现在叫 g_hmi_rtu，未来也可以换成更通用名字）
                    g_hmi_rtu.onBytes(srx.serial_index, srx.bytes);

                    while (g_hmi_rtu.tryGetFrame(srx.serial_index, frame))
                    {
#if protocol_parser_thread_LOG
                        LOG_COMM_D("RX RS485#%d(SLAVE) len=%zu", srx.serial_index, frame.size());
                        LOG_COMM_HEX("RX", frame.data(), frame.size());
#endif
                        std::vector<uint8_t> tx;
                        const bool handled = g_router.handleRs485Slave(srx.serial_index, frame, tx);

                        // 只要这是“从站口”，策略就是：不管 handled true/false 都吞掉
                        // handled=false 只表示“协议没认出来/不是本从站地址”等，但仍不要走 parseRs485，避免刷屏
                        if (handled && !tx.empty() && send_serial_)
                        {
#if protocol_parser_thread_LOG
                            LOG_COMM_D("TX RS485#%d(SLAVE) len=%zu", srx.serial_index, tx.size());
                            LOG_COMM_HEX("TX", tx.data(), tx.size());
#endif
                            send_serial_(dev::LinkType::RS485, srx.serial_index, tx);
                        }
                    }

                    // ✅ 从站口处理完本 chunk 就结束本次循环（不走下面 parseRs485）
                    continue;
                }

                // ===== 其它 RS485 口：维持原拆帧 + parseRs485 =====
                g_frame.onBytes(srx.serial_index, srx.bytes);

                while (g_frame.tryGetFrame(srx.serial_index, frame))
                {
#if protocol_parser_thread_LOG
                    LOG_COMM_D("RX RS485#%d len=%zu", srx.serial_index, frame.size());
                    LOG_COMM_HEX("RX", frame.data(), frame.size());
#endif

                    DeviceData d;
                    const bool ok = g_router.parseRs485(srx.serial_index, frame, d);

                    if (ok)
                    {
                        ParserMessage msg;
                        msg.type = ParsedType::DeviceData;
                        msg.link_type = srx.type;
                        msg.link_index = srx.serial_index;
                        msg.device_data = std::move(d);

                        msg.device_name = msg.device_data.device_name;
                        // 0207
                        msg.rx_ts_ms = nowMs();
                        if (on_parsed_) on_parsed_(msg);

                        // clearPending(srx.type, srx.serial_index);
                    }
                    else
                    {
#if protocol_parser_thread_LOG
                        LOG_THROTTLE_MS("rs485_parse_fail", 500, LOG_COMM_W,
                                        "RX RS485#%d parse=FAIL len=%zu", srx.serial_index, frame.size());
#endif

                        if (on_parsed_)
                        {
                            ParserMessage err;
                            err.type = ParsedType::ParseError;
                            err.link_type = srx.type;
                            err.link_index = srx.serial_index;
                            err.error_code = -1;
                            err.error_text = "RS485 parse failed";
                            on_parsed_(err);
                        }
                    }
                }
            }
            else if (srx.type == dev::LinkType::RS232)
            {
                g_ascii.onBytes(srx.serial_index, srx.bytes);

                while (g_ascii.tryGetFrame(srx.serial_index, frame))
                {
                    // 统一 COMM RX
#if protocol_parser_thread_LOG
                    LOG_COMM_D("RX RS232#%d len=%zu", srx.serial_index, frame.size());
                    LOG_COMM_HEX("RX", frame.data(), frame.size());
#endif
                    DeviceData d;
                    const bool ok = g_router.parseRs232(srx.serial_index, frame, d);

                    if (ok)
                    {
                        ParserMessage msg;
                        msg.type = ParsedType::DeviceData;
                        msg.link_type = srx.type;
                        msg.link_index = srx.serial_index;
                        msg.device_data = std::move(d);

                        msg.device_name = msg.device_data.device_name;

                        if (on_parsed_) on_parsed_(msg);

                        // clearPending(srx.type, srx.serial_index);
                    }
                    else
                    {
#if protocol_parser_thread_LOG
                        // UPS/RS232 更容易刷屏：必须限频
                        LOG_THROTTLE_MS("rs232_parse_fail_ups", 500, LOG_COMM_W,
                                        "RX RS232#%d dev=UPS parse=FAIL len=%zu", srx.serial_index, frame.size());
#endif
                        if (on_parsed_)
                        {
                            ParserMessage err;
                            err.type = ParsedType::ParseError;
                            err.link_type = srx.type;
                            err.link_index = srx.serial_index;
                            err.error_code = -1;
                            err.error_text = "RS232 parse failed";
                            on_parsed_(err);
                        }
                    }
                }
            }
        }
    }
}


HMIProto* ProtocolParserThread::getHmiProto(int rs485_index) {
    // Router 判断这个口是否是 HMI
    if (!g_router.isRs485SlavePort(rs485_index)) return nullptr;

    // 通过 proto_ 取出并 cast（这里在 parser 子系统内部做，App 不需要知道）
    return g_router.getHmiProto(rs485_index);
}
