// main.cpp
#include "app_manager.h"
#include <csignal>
#include <atomic>

static std::atomic<bool> g_stop{false};

static void onSignal(int) { g_stop = true; }

int main() {
    std::signal(SIGINT, onSignal); // 注册 Ctrl+C 信号处理函数
    std::signal(SIGTERM, onSignal); // 注册终止信号处理函数

    AppManager app;
    if (!app.init()) {
        LOGINFO("[MAIN] app.init failed");
        return 1;
    }
    LOGINFO("[MAIN] app.init done");


    LOGINFO("[MAIN] calling app.start...");
    app.start();
    LOGINFO("[MAIN] app.start done");


    app.post(Event{Event::Type::CmdStart});

    LOGINFO("[MAIN] before main loop / pause");
    while (!g_stop.load()) {
        app.pumpOnce();     // 主线程“抽水”：取事件 -> 分发
    }

    app.stop();
    LOGINFO("[MAIN] exit");
    return 0;
}
