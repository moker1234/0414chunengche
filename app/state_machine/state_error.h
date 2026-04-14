//
// Created by forlinx on 2025/12/25.
//

#ifndef ENERGYSTORAGE_STATE_ERROR_H
#define ENERGYSTORAGE_STATE_ERROR_H
// state_error.h
#pragma once
#include "state_base.h"
#include <string>

class StateError : public StateBase {
public:
    StateError(int code, std::string msg) : code_(code), msg_(std::move(msg)) {}
    const char* name() const override { return "Error"; }

    void onEnter(AppManager& app) override {
        LOGINFO("[STATE][Error] enter code=%d msg=%s\n", code_, msg_.c_str());
        // 可通知 Scheduler 降级/停轮询
        // app.scheduler().setMode(DeviceScheduler::Mode::Safe);
        (void)app;
    }

    void onEvent(AppManager& app, const Event& e) override;

private:
    int code_{0};
    std::string msg_;
};

#endif //ENERGYSTORAGE_STATE_ERROR_H