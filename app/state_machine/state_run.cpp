//
// Created by forlinx on 2025/12/17.
//

// state_run.cpp
#include "state_run.h"
#include "../app_manager.h"
#include "state_idle.h"
#include "state_error.h"


StateRun::StateRun() {
    LOGINFO("[STATE][Run] ctor this=%p", this);
}

inline void StateRun::onEvent(AppManager& app, const Event& e) {
    switch (e.type) {
    case Event::Type::CmdStop:
        app.transitionTo(std::make_unique<StateIdle>());
        return;

    case Event::Type::CanDown:
        app.transitionTo(std::make_unique<StateError>(-1, "CAN down"));
        return;
    case Event::Type::SerialDown:
        // LOGWARN("[STATE][Run] Serial link down idx=%d (ignored, wait policy)",
                // e.link_index);
        return;

    case Event::Type::SerialUp:
        // LOGINFO("[STATE][Run] Serial link recovered idx=%d",
                // e.link_index);
        return;

    case Event::Type::Error:
        app.transitionTo(std::make_unique<StateError>(e.code, e.text));
        return;

    default:
        return;
    }
}

