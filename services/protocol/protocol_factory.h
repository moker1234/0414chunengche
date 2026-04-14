//
// Created by forlinx on 2025/12/31.
//
/*
 * 协议工厂定义
 */
/* 解释整个文件的作用
 * 该文件实现了协议工厂的定义，包括构造函数和创建协议函数。
 * 构造函数用于初始化协议工厂，创建协议函数用于根据设备类型和地址创建对应的协议实例。
 */
#ifndef ENERGYSTORAGE_PROTOCOL_FACTORY_H
#define ENERGYSTORAGE_PROTOCOL_FACTORY_H


#pragma once
#include <memory>
#include "protocol_base.h"

enum class DeviceType {
    TEMP_HUMI,
    AIR_CONDITIONER,
    GAS_SENSOR,
    UPS
};

class ProtocolFactory {
public:
    static std::unique_ptr<ProtocolBase>
    create(DeviceType type, uint8_t addr = 1);
};


#endif //ENERGYSTORAGE_PROTOCOL_FACTORY_H