//
// Created by lxy on 2025/12/18.
// Modified by lxy on 2026/01/12
//

#ifndef ENERGYSTORAGE_DRIVER_MANAGER_H
#define ENERGYSTORAGE_DRIVER_MANAGER_H
#pragma once
#include <memory>
#include <vector>
#include <linux/can.h>

#include "can_driver.h"
#include "can_thread.h"
#include "../serial/serial_driver_base.h"
#include "../serial/serial_thread.h"

#include "../services/parser/protocol_parser_thread.h"
#include "../serial/serial_event.h"
#include "../../utils/queue/msg_queue.h"
#include "../app/event.h"


#include "gpio/di.h"
#include "gpio/do.h"
#include "adc/ai.h"
#include "io/io_thread.h"

class AppManager;
class SerialThread;
class SerialDriverBase;
class CanThread;
class CanDriver;


class DriverManager {
public:
    DriverManager(AppManager& app,
                  parser::ProtocolParserThread& parser);

    void init();
    void start();
    void stop();

    bool sendCan(int can_index, const can_frame& f);
    bool sendSerial(dev::LinkType type,
                    int idx,
                    const std::vector<uint8_t>& bytes);


    void setIoSampleCallback(IoThread::SampleCallback cb);
    bool writeDo(int channel_id, bool value);

    // timeout
    void onSerialLinkTimeout(int idx);
    void onSerialLinkRecover(int idx);


private:
    AppManager& app_;
    parser::ProtocolParserThread& parser_;

    std::vector<std::unique_ptr<CanDriver>> can_drivers_;
    std::vector<std::unique_ptr<CanThread>> can_threads_;

    std::vector<std::unique_ptr<SerialDriverBase>> serialDrvs_;
    std::vector<std::unique_ptr<SerialThread>> serialThreads_;

    std::vector<std::unique_ptr<SerialDriverBase>> rs232Drvs_;
    std::vector<std::unique_ptr<SerialThread>>     rs232Threads_;
    MsgQueue<SerialEvent> serial_rx_queue_;


    // ===== IO managers =====
    std::unique_ptr<DigitalInputManager> di_mgr_;
    std::unique_ptr<DigitalOutputManager> do_mgr_;
    std::unique_ptr<AiDriver> ai_drv_;
    std::unique_ptr<IoThread> io_thread_;
    IoThread::SampleCallback io_sample_cb_;

};


#endif // ENERGYSTORAGE_DRIVER_MANAGER_H
