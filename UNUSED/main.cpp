//
// Created by forlinx on 2025/12/17.
//
#include <unistd.h>
#include <vector>
#include <string.h>
#include <stdio.h>

#include "driver_manager.h"

int main() {
    auto& driverMgr = DriverManager::instance();

    driverMgr.init();
    driverMgr.start();

    /* ===== 测试串口发送 ===== */
    const char* hello = "Hello World\r\n";
    std::vector<uint8_t> serial_data(
        hello,
        hello + strlen(hello)
    );

    /* ===== 测试 CAN ===== */
    can_frame frame{};
    frame.can_id  = 0x123;
    frame.can_dlc = 2;
    frame.data[0] = 0x11;
    frame.data[1] = 0x22;

    printf("[MAIN] start test loop\n");

    /* ========= 获取串口 RX 消息队列 ========= */
    auto& serialRxQ = driverMgr.get_serialRxQueue();
    while (1) {
        /* 串口通过 SerialThread 发送 */
        driverMgr
            .getSerialThread()
            .send(serial_data);

        /* CAN 通过 CanThread 发送 */
        driverMgr
            .getCanThread1()
            .send(frame);

        printf("[MAIN] send serial + can\n");

        /* === 3. 处理串口 RX 事件（非阻塞示例） === */
        SerialEvent ev;   ///////////////////////////////////////////////////////
        while (serialRxQ.tryPop(ev))
        {
            switch (ev.type) {
            case SerialEventType::RX_DATA:
                printf("-----------------------[APP][SERIAL RX][%s] ",
                       ev.port.c_str());
                fwrite(ev.data.data(), 1, ev.data.size(), stdout);
                printf("\n");
                break;

            case SerialEventType::RX_ERROR:
                printf("[APP][SERIAL RX ERROR][%s] errno=%d\n",
                       ev.port.c_str(), ev.error);
                break;

            case SerialEventType::TX_ERROR:
                printf("[APP][SERIAL TX ERROR][%s] errno=%d\n",
                       ev.port.c_str(), ev.error);
                break;

            default:
                break;
            }
        }

        frame.data[1]++;
        sleep(1);
    }

    driverMgr.stop();
    return 0;



}
