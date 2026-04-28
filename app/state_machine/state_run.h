//
// Created by lxy on 2025/12/25.
//

#ifndef ENERGYSTORAGE_STATE_RUN_H
#define ENERGYSTORAGE_STATE_RUN_H
// state_run.h
#pragma once
#include <memory>
#include "state_base.h"

class StateRun : public StateBase {
public:
    StateRun();
    const char* name() const override { return "Run"; }
    void onEnter(AppManager& app) override {
        LOGINFO("[STATE][Run] enter");
        // 这里可以通知 Scheduler 进入运行模式（如果你有模式概念）
        // app.scheduler().setMode(DeviceScheduler::Mode::Run);
        (void)app;
    }
    void onEvent(AppManager& app, const Event& e) override;
    void onExit(AppManager& app) override {
        (void)app;
        LOGINFO("[STATE][Run] exit");
    }

};

#endif //ENERGYSTORAGE_STATE_RUN_H