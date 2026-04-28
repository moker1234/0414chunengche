// services/control/command_dispatcher.cpp
//
// 工业级控制：命令下发器实现
// Created by lxy on 2026/02/24.
//

#include "command_dispatcher.h"

#include "../driver/driver_manager.h"
#include "../parser/protocol_parser_thread.h"
#include "../utils/logger/logger.h"
#include "../utils/time/getTime.h"

namespace control {

CommandDispatcher::CommandDispatcher(DriverManager& drv, parser::ProtocolParserThread& parser)
    : drv_(drv), parser_(parser)
{
}

void CommandDispatcher::dispatch(const std::vector<Command>& cmds)
{
    last_dispatch_ts_ = nowMs();

    for (const auto& c : cmds) {
        dispatchOne_(c);
    }
}

void CommandDispatcher::dispatchOne_(const Command& c)
{
    switch (c.type)
    {
    case Command::Type::SendCan:
    {
        const auto& cc = c.can;
        if (cc.can_index < 0) return;
        (void)drv_.sendCan(cc.can_index, cc.frame);
        break;
    }
    case Command::Type::SendSerialRaw:
    {
        const auto& sc = c.serial_raw;
        if (sc.index < 0) return;
        (void)drv_.sendSerial(sc.link_type, sc.index, sc.bytes);
        break;
    }
    case Command::Type::SendPoll:
    {
        const auto& pc = c.poll;
        if (pc.index < 0) return;

        // 走 Parser 的 sendPoll，复用 pending/timeout
        (void)parser_.sendPoll(
            pc.link_type,
            pc.index,
            pc.device_name,
            pc.timeout_ms,
            pc.mode
        );
        break;
    }
    case Command::Type::WriteDo:
        {
            const auto& dc = c.write_do;

            auto it = last_do_cache_.find(dc.channel_id);
            if (it != last_do_cache_.end() && it->second == dc.value) {
                // 值没变，跳过（减少抖动与重复写）
                return;
            }

            const bool ok = drv_.writeDo(dc.channel_id, dc.value);
            if (!ok) {
                LOG_THROTTLE_MS("do_write_fail", 1000, LOGWARN,
                                "[CTRL][DO] write fail channel=%d value=%d",
                                dc.channel_id, dc.value ? 1 : 0);
                return;
            }

            last_do_cache_[dc.channel_id] = dc.value;

            LOG_THROTTLE_MS("do_write_ok", 1000, LOGINFO,
                            "[CTRL][DO] write ok channel=%d value=%d",
                            dc.channel_id, dc.value ? 1 : 0);
            break;
        }
    default:
        break;
    }
}

} // namespace control