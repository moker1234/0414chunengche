//
// Created by forlinx on 2025/12/17.
//



// state_idle.cpp
#include "state_idle.h"
#include "../app_manager.h"
#include "state_run.h"
#include "state_error.h"

inline void StateIdle::onEvent(AppManager& app, const Event& e) {
    switch (e.type) {
    case Event::Type::Boot:
        // Boot 后可以做自检、发 Tick、或等待 CmdStart
        // app.transitionTo(std::make_unique<StateRun>());
            return;
    case Event::Type::CmdStart:
        app.transitionTo(std::make_unique<StateRun>());
            return;
    case Event::Type::Error:
        app.transitionTo(std::make_unique<StateError>(e.code, e.text));
            return;
    default:
            return;
    }
}
