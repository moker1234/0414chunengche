// services/control/command_dispatcher.cpp
//
// 工业级控制：命令下发器实现
// Created by ChatGPT on 2026/02/24.
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
        // 你当前仓库的 GPIO 部分还没形成“多 DO 通道管理器”，
        // 因此这里先做“合并缓存骨架”，实际写 GPIO 的映射会在后续改动里补齐。
        const auto& dc = c.write_do;

        auto it = last_do_cache_.find(dc.channel_id);
        if (it != last_do_cache_.end() && it->second == dc.value) {
            // 值没变，跳过（减少抖动与重复写）
            return;
        }
        last_do_cache_[dc.channel_id] = dc.value;

        // TODO: 把 channel_id -> 具体 GPIODriver 映射接上
        // 例如：gpio_mgr_.setDo(channel_id, value);

        LOG_THROTTLE_MS("do_write_stub", 2000, LOGWARN,
                        "[CTRL][DO] write stub channel=%d value=%d (TODO map to GPIODriver)",
                        dc.channel_id, dc.value ? 1 : 0);
        break;
    }
    case Command::Type::SetHmiRw:
    {
        // TODO: 需要把 HMIProto/HmiAddressTable 暴露一个 setIntRw(addr,val)
        // 目前先留空实现
        break;
    }
    default:
        break;
    }
}

} // namespace control