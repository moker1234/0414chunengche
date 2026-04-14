// services/protocol/j1939/opensae_adapter.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

    // 直接使用 OpenSAE 自己的枚举定义，避免重复 typedef 冲突
#include "Open_SAE_J1939.h"   // 内部会 include Enum_Send_Status.h 定义 ENUM_J1939_STATUS_CODES

    /*
     * OpenSAE 的 Hardware.h 里，CAN_Read_Message/CAN_Send_Message 没有通道参数。
     * 我们通过 active_can_index 让 OpenSAE 在“当前正在处理的通道”上收发。
     */

    /** 设置当前 OpenSAE 操作的 CAN 通道索引（0/1/2...） */
    void opensae_set_active_can_index(int can_index);

    /** 获取当前 OpenSAE 操作的 CAN 通道索引 */
    int opensae_get_active_can_index(void);

    /** 从你的 CanThread RX 回调里调用：把帧推入 adapter 的 RX 队列 */
    void opensae_push_rx_frame(int can_index, uint32_t can_id, uint8_t dlc, const uint8_t* data);

    /** 可选：清空某通道队列（解绑/重启时用） */
    void opensae_clear_rx_queue(int can_index);

    /*
     * 下面这些符号是 OpenSAE 里会引用到的（Hardware/Hardware.h 声明）。
     * 由本 adapter 提供实现，从而完成“收发闭环”。
     *
     * 注意：你不要再编译 OpenSAE 自带的 Hardware/CAN_Transmit_Receive.c，
     * 否则会重复定义。
     */

    /** 发送 8 bytes（OpenSAE 基本都用 8） */
    ENUM_J1939_STATUS_CODES CAN_Send_Message(uint32_t ID, uint8_t data[]);

    /** 发送 PGN request（DLC=3） */
    ENUM_J1939_STATUS_CODES CAN_Send_Request(uint32_t ID, uint8_t PGN[]);

    /** 读取一帧（若无新帧返回 false） */
    bool CAN_Read_Message(uint32_t* ID, uint8_t data[]);

    /** delay ms */
    void CAN_Delay(uint8_t milliseconds);

    /** OpenSAE 注册回调（我们存起来，但闭环验证不强依赖） */
    void CAN_Set_Callback_Functions(
        void (*Callback_Function_Send_)(uint32_t, uint8_t, uint8_t[]),
        void (*Callback_Function_Read_)(uint32_t*, uint8_t[], bool*),
        void (*Callback_Function_Traffic_)(uint32_t, uint8_t, uint8_t[], bool),
        void (*Callback_Function_Delay_ms_)(uint8_t)
    );

#ifdef __cplusplus
}
#endif
