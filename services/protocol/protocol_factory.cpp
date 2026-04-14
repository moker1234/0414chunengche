//
// Created by forlinx on 2025/12/31.
//
/*
 * 协议工厂实现
 */
/* 解释整个文件的作用
 * 该文件实现了协议工厂的定义，包括构造函数和创建协议函数。
 * 构造函数用于初始化协议工厂，创建协议函数用于根据设备类型和地址创建对应的协议实例。
 */
#include "protocol_factory.h"

#include "protocol_factory.h"
// #include "rs232/ups_proto.h"

std::unique_ptr<ProtocolBase>
ProtocolFactory::create(DeviceType type, uint8_t addr) {
    (void)addr;
    switch (type) {
    // case DeviceType::TEMP_HUMI:
    //     return std::make_unique<TempHumiProto>(addr);
    // case DeviceType::UPS:
    //     return std::make_unique<UPSProto>();
    default:
        return nullptr;
    }
}
