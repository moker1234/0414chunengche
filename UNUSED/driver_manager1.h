//
// Created by forlinx on 2025/12/18.
//

#ifndef ENERGYSTORAGE_DRIVER_MANAGER_H
#define ENERGYSTORAGE_DRIVER_MANAGER_H
#include "can_driver.h"
#include <memory>

#include "can_thread.h"
#include "serial/serial_driver_base.h"
#include "serial//serial_thread.h"
#include "serial_event.h"
#include "../utils/queue/msg_queue.h"

class DriverManager
{
public:
    static DriverManager& instance() {
        static DriverManager inst;
        return inst;
    }

    void init();   // 初始化所有硬件
    void start();  // 启动线程或轮询
    void stop();   // 停止线程

    // 对外提供接口
    CanDriver& getCan1() { return *can1_; }
    // CanDriver& getCan2() { return *can2_; }
    CanThread& getCanThread1() { return *canThread1_; }
    SerialDriverBase& getSerialDriver() { return *serialDrv_; }
    SerialThread& getSerialThread() { return *serialThread_; }
    MsgQueue<SerialEvent>& get_serialRxQueue() { return serial_rx_queue_; }

private:
    DriverManager() = default;
    ~DriverManager() = default;

    void serial_init(const std::string& name);

    std::unique_ptr<CanDriver> can1_;
    std::unique_ptr<CanThread> canThread1_;
    // std::unique_ptr<CanDriver> can2_;
    std::unique_ptr<SerialDriverBase> serialDrv_;
    std::unique_ptr<SerialThread>     serialThread_;
    MsgQueue<SerialEvent> serial_rx_queue_;
};

#endif //ENERGYSTORAGE_DRIVER_MANAGER_H