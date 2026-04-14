//
// Created by forlinx on 2025/12/17.
//

#ifndef ENERGYSTORAGE_STATE_BASE_H
#define ENERGYSTORAGE_STATE_BASE_H
// state_base.h
#pragma once
#include "logger.h"
#include <string>

struct Event;
class AppManager;

class StateBase {
public:
    virtual ~StateBase() = default;

    virtual const char* name() const = 0;

    virtual void onEnter(AppManager& app) {(void)app;}  // 进入状态时调用
    virtual void onExit(AppManager& app) {(void)app;}  // 退出状态时调用
    virtual void onEvent(AppManager& app, const Event& e) = 0; // 处理事件
};

#endif //ENERGYSTORAGE_STATE_BASE_H