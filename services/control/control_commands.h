// services/control/control_commands.h
//
// 工业级控制：统一命令定义（LogicEngine 输出 -> CommandDispatcher 下发）
// Created by lxy on 2026/02/24.
//

#ifndef ENERGYSTORAGE_CONTROL_COMMANDS_H
#define ENERGYSTORAGE_CONTROL_COMMANDS_H

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <linux/can.h>

#include "../device/device_types.h"              // dev::LinkType
#include "../parser/protocol_parser_thread.h"    // parser::PollSendMode

namespace control {

    /**
     * @brief 命令：发送 CAN 帧
     */
    struct SendCanCmd {
        int can_index{-1};
        can_frame frame{};

        /*
         * TX 镜像。
         *
         * 用途：
         *   LogicEngine 生成 PCU 发送帧时，同时构造一个 DeviceData 镜像。
         *   ControlLoop 在真正 dispatch SendCan 之后，把这个镜像回调给 AppManager。
         *
         * 注意：
         *   这个镜像不是设备 RX，不能用于在线判断；
         *   Aggregator 第三批已经保证 PCU_x_CTRL 会合并到 PCU_x.pcu.ctrl。
         */
        bool has_tx_mirror{false};
        DeviceData tx_mirror;
    };

    /**
     * @brief 命令：串口原始字节发送（不走 pending）
     * 用途：对 HMI 从站口回包由 Parser 已处理；一般控制命令建议走 sendPoll/专用写命令接口。
     */
    struct SendSerialRawCmd {
        dev::LinkType link_type{dev::LinkType::RS485}; // RS485/RS232
        int index{-1};
        std::vector<uint8_t> bytes;
    };

    /**
     * @brief 命令：触发一次“轮询发送”（复用 Parser pending/timeout/retry 机制）
     * 注意：这是工业项目最稳的 RS485/RS232 主站发送方式之一。
     */
    struct SendPollCmd {
        dev::LinkType link_type{dev::LinkType::RS485};
        int index{-1};
        std::string device_name;
        uint32_t timeout_ms{200};
        parser::PollSendMode mode{parser::PollSendMode::Normal};
    };

    /**
     * @brief 命令：写 DO（开关量输出）
     * 你当前仓库的 GPIODriver 只有 setValue(bool)，因此这里用 channel_id + value，
     * 具体 channel_id -> GPIO 实例映射建议在 CommandDispatcher 内实现。
     */
    struct WriteDoCmd {
        int channel_id{-1}; // 逻辑 DO 通道号（映射到某个 gpio line）
        bool value{false};
    };

    /**
     * @brief 统一 Command
     */
    struct Command {
        enum class Type : uint8_t {
            SendCan = 0,
            SendSerialRaw,
            SendPoll,
            WriteDo,
        };

        Type type{Type::SendCan};

        SendCanCmd can;
        SendSerialRawCmd serial_raw;
        SendPollCmd poll;
        WriteDoCmd write_do;
    };

} // namespace control

#endif // ENERGYSTORAGE_CONTROL_COMMANDS_H