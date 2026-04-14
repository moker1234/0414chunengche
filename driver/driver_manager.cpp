#include "driver_manager.h"
#include "logger.h"
#include "rs232_driver.h"
#include "rs485_driver.h"
#include "config_loader.h"
/* 解释整个文件的作用
 * 该文件实现了驱动管理类DriverManager的定义，包括构造函数、初始化函数和CAN线程回调函数。
 * 构造函数用于初始化应用管理器和协议解析线程，初始化函数用于初始化CAN驱动和串口驱动，CAN线程回调函数用于处理CAN接收数据。
 */
DriverManager::DriverManager(AppManager& app,
                             parser::ProtocolParserThread& parser)
    : app_(app), parser_(parser)
{
}

void DriverManager::init()
{
    // ===== CAN =====
    // can1_ = std::make_unique<CanDriver>("can0");
    // can1_->init();
    //
    // canThread1_ = std::make_unique<CanThread>(*can1_, app_, "can0_thread");
    // canThread1_->setRxCallback(
    //     [this](int idx, const can_frame& f)
    //     {
    //         dev::CanRxPacket rx{};
    //         rx.can_index = idx;
    //         rx.frame = f;
    //         parser_.pushCan(rx);
    //     }
    // );

    // ===== 读取 io_map.json：生成 RS485 / RS232 端口列表 =====
    // 你需要在本 cpp 顶部 include：
    // #include "utils/config/config_loader.h"
    // 以及确保链接了 nlohmann/json
    IoMapConfig io{};
    std::string io_err;
    if (!ConfigLoader::loadIoMap("/home/zlg/userdata//config/io_map.json", io, io_err)) {
        LOGERR("[DRV] load /home/zlg/userdata/config/io_map.json failed: %s (fallback to default ports)",
               io_err.c_str());

        io.can = {
            {"can0", "can0", 125000, true},
            {"can1", "can1", 125000, true},
            {"can2", "can2", 500000, true}
        };
        // fallback：保持你当前工程的默认口位/波特率
        io.rs485 = {
            {"rs485_1", "/dev/ttyRS485-1",  9600,   true},
            {"rs485_2", "/dev/ttyRS485-2",  9600,   true},
            {"rs485_3", "/dev/ttyRS485-3",  9600,   true},
            {"rs485_4", "/dev/ttyRS485-4", 115200,  true} // HMI
        };
        io.rs232 = {
            {"rs232_1", "/dev/ttyRS232-1", 2400, true}     // UPS
        };
    }

    // ===== CAN（从 io_map.json 导入）=====
    {
        // 按 io.can 数组顺序：can_index = 0..N-1
        can_drivers_.clear();
        can_threads_.clear();
        can_drivers_.resize(io.can.size());
        can_threads_.resize(io.can.size());

        for (size_t i = 0; i < io.can.size(); ++i) {
            const auto& c = io.can[i];
            if (!c.enable) {
                LOG_SYS_I("[DRV][CAN] skip idx=%zu if=%s (disabled)", i, c.ifname.c_str());
                continue;
            }

            auto drv = std::make_unique<CanDriver>(c.ifname);
            if (!drv->init()) {
                LOGERR("[DRV][CAN] init failed idx=%zu if=%s", i, c.ifname.c_str());
                continue;
            }

            auto th = std::make_unique<CanThread>(*drv, app_, (int)i, c.ifname + "_thread");

            // 你现在已经能看到 CAN 打印：在 CanThread::handleRead() 内会 print
            // 这里额外把 rx 送进 parser（保持你现有架构）
            th->setRxCallback([this](int idx, const can_frame& f) {
                dev::CanRxPacket rx{};
                rx.can_index = idx;
                rx.frame = f;
                parser_.pushCan(rx);

                // 使用总的解析架构，而不走 J1939Manager 的“闭环”
                // if (j1939_mgr_) {
                //     j1939_mgr_->onCanFrame(idx, f);
                // }
            });

            can_drivers_[i] = std::move(drv);
            can_threads_[i] = std::move(th);

            LOG_SYS_I("[DRV][CAN] add idx=%zu if=%s", i, c.ifname.c_str());
        }
    }

    // ===== RS485 =====
    {
        SerialConfig cfg{};
        cfg.data_bits  = 8;
        cfg.parity     = 'N';
        cfg.stop_bits  = 1;
        cfg.non_block  = true;

        // 按 io_map.json 数组顺序创建，serial_index = 0..N-1 连续
        for (size_t i = 0; i < io.rs485.size(); ++i)
        {
            const auto& p = io.rs485[i];
            if (!p.enable) continue;

            cfg.baudrate = p.baudrate;

            auto drv = std::make_unique<Rs485Driver>(p.device, -1);
            if (!drv->init(cfg))
            {
                LOGERR("[DRV] serial init failed %s", p.device.c_str());
                continue; // 不创建 thread
            }

            const std::string th_name =
                !p.name.empty() ? p.name : ("rs485_" + std::to_string(i));

            auto th = std::make_unique<SerialThread>(
                (int)i,
                *drv,
                th_name,
                serial_rx_queue_
            );

            // timeout callback
            th->setLinkTimeoutCallback(
                [this](const SerialThread::LinkTimeoutInfo& info)
                {
                    this->onSerialLinkTimeout(info.serial_index);
                }
            );
            th->setLinkRecoverCallback(
                [this](const SerialThread::LinkTimeoutInfo& info)
                {
                    this->onSerialLinkRecover(info.serial_index);
                }
            );

            th->setRxCallback(
                [this](int idx, const std::vector<uint8_t>& bytes)
                {
                    dev::SerialRxPacket rx{};
                    rx.type = dev::LinkType::RS485;
                    rx.serial_index = idx;
                    rx.bytes = bytes;
                    parser_.pushSerial(rx);
                }
            );

            serialDrvs_.push_back(std::move(drv));
            serialThreads_.push_back(std::move(th));

            LOG_SYS_I("[DRV] RS485 add idx=%zu name=%s dev=%s baud=%d",
                      i, th_name.c_str(), p.device.c_str(), p.baudrate);
        }
    }

    // ===== RS232 (UPS) =====
    {
        SerialConfig cfg232{};
        cfg232.data_bits  = 8;
        cfg232.parity     = 'N';
        cfg232.stop_bits  = 1;
        cfg232.non_block  = true;

        // 同样按 io_map.json 数组顺序创建，rs232 serial_index = 0..N-1
        for (size_t i = 0; i < io.rs232.size(); ++i)
        {
            const auto& p = io.rs232[i];
            if (!p.enable) continue;

            cfg232.baudrate = p.baudrate;

            auto drv = std::make_unique<Rs232Driver>(p.device);
            if (!drv->init(cfg232))
            {
                LOGERR("[DRV] rs232 init failed %s", p.device.c_str());
                continue;
            }

            const std::string th_name =
                !p.name.empty() ? p.name : ("rs232_" + std::to_string(i));

            auto th = std::make_unique<SerialThread>(
                (int)i,
                *drv,
                th_name,
                serial_rx_queue_
            );

            // 如果你也希望 RS232 有断链检测，可在这里补 setLinkTimeoutCallback / setLinkRecoverCallback

            th->setRxCallback(
                [this](int idx, const std::vector<uint8_t>& bytes)
                {
                    dev::SerialRxPacket rx{};
                    rx.type = dev::LinkType::RS232;
                    rx.serial_index = idx;
                    rx.bytes = bytes;
                    parser_.pushSerial(rx);
                }
            );

            rs232Drvs_.push_back(std::move(drv));
            rs232Threads_.push_back(std::move(th));

            LOG_SYS_I("[DRV] RS232 add idx=%zu name=%s dev=%s baud=%d",
                      i, th_name.c_str(), p.device.c_str(), p.baudrate);
        }
    }
}


void DriverManager::start()
{
    for (auto& th : can_threads_) if (th) th->start();
    for (auto& th : serialThreads_) if (th) th->start();
    for (auto& th : rs232Threads_) if (th) th->start();
}

void DriverManager::stop()
{
    for (auto& th : can_threads_) if (th) th->stop();
    for (auto& th : serialThreads_) if (th) th->stop();
    for (auto& th : rs232Threads_) if (th) th->stop();
}

bool DriverManager::sendCan(int can_index, const can_frame& f) {
    if (can_index < 0) return false;
    if ((size_t)can_index >= can_threads_.size()) return false;
    auto& th = can_threads_[can_index];
    if (!th) return false;
    th->send(f);
    return true;
}

bool DriverManager::sendSerial(dev::LinkType type,
                               int idx,
                               const std::vector<uint8_t>& bytes)
{
    if (idx < 0) return false;

    if (type == dev::LinkType::RS485)
    {
        if (idx >= (int)serialThreads_.size()) return false;
        serialThreads_[idx]->send(idx, bytes);
        return true;
    }

    if (type == dev::LinkType::RS232)
    {
        if (idx >= (int)rs232Threads_.size()) return false;
        rs232Threads_[idx]->send(idx, bytes);
        return true;
    }

    return false;
}

void DriverManager::onSerialLinkTimeout(int idx) {
    // LOGWARN("[DRV][L1] serial link timeout idx=%d", idx);

    Event e;
    e.type = Event::Type::SerialDown;
    e.link_index = idx;
    app_.post(e);
}

void DriverManager::onSerialLinkRecover(int idx) {
    // LOGINFO("[DRV][L1] serial link recover idx=%d", idx);

    Event e;
    e.type = Event::Type::SerialUp;
    e.link_index = idx;
    app_.post(e);
}
