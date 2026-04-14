//
// Created by forlinx on 2025/12/25.
//

#ifndef ENERGYSTORAGE_STATE_IDLE_H
#define ENERGYSTORAGE_STATE_IDLE_H

#pragma once
#include "state_base.h"

class StateIdle : public StateBase {
public:
    const char* name() const override { return "Idle"; }
    void onEnter(AppManager& app) override {
        (void)app;
        LOGINFO("[STATE][Idle] enter");
    }

    void onEvent(AppManager& app, const Event& e) override;
};

#endif //ENERGYSTORAGE_STATE_IDLE_H