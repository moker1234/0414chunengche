//
// Created by forlinx on 2025/12/17.
//


// state_error.cpp
#include "state_error.h"
#include "../app_manager.h"
#include "state_idle.h"
#include "state_run.h"

// void StateError::onEnter(AppManager&) {
//     // 记录日志/告警上报（也可以由 AppManager 统一处理）
// }

inline void StateError::onEvent(AppManager& app, const Event& e) {
    switch (e.type) {
    case Event::Type::CmdStop:
        app.transitionTo(std::make_unique<StateIdle>());
        return;
    default:
        return;
    }
}
