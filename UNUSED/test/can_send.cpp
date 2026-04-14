//
// Created by forlinx on 2025/12/28.
//
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstdio>

#include "driver_manager.h"
#include "app_manager.h"

static std::atomic<bool> g_stop{false};

static void onSignal(int) {
    g_stop = true;
}

int main() {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    printf("[MAIN] CAN Thread TX test start\n");

    /* ===== AppManager 仅作为占位（CanThread 需要） ===== */
    AppManager app;

    /* ===== DriverManager ===== */
    DriverManager driverMgr(app);

    driverMgr.init();
    driverMgr.start();

    /* ===== 构造测试 CAN 帧 ===== */
    can_frame frame{};
    frame.can_id  = 0x123;   // 标准帧 ID
    frame.can_dlc = 2;
    frame.data[0] = 0x11;
    frame.data[1] = 0x22;

    printf("[MAIN] Enter send loop (Ctrl+C to exit)\n");

    while (!g_stop.load()) {
        driverMgr
            .getCanThread1()
            .send(frame);

        printf("[MAIN] CAN TX id=0x%X data=%02X %02X\n",
               frame.can_id,
               frame.data[0],
               frame.data[1]);

        frame.data[1]++;   // 简单变化，方便抓包观察
        sleep(1);
    }

    printf("[MAIN] stopping...\n");
    driverMgr.stop();

    return 0;
}

