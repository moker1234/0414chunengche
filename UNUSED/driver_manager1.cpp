//
// Created by forlinx on 2025/12/18.
//

#include "driver_manager.h"


void DriverManager::init() {
    can1_ = std::make_unique<CanDriver>("can0");
    if (!can1_->init()) {
        printf("[DRV][ERROR] can0 init failed\n");
        can1_.reset();
        return;
    }
    canThread1_ = std::make_unique<CanThread>(*can1_, "can0_Thread");

    /* ================= SERIAL TEST ================= */
    SerialConfig cfg{};
    cfg.baudrate  = 115200;
    cfg.data_bits = 8;
    cfg.parity    = 'N';
    cfg.stop_bits = 1;
    cfg.non_block = true;

    serialDrv_ = std::make_unique<SerialDriverBase>("/dev/ttyS4");
    if (!serialDrv_->init(cfg)) {
        printf("[DRV][ERROR] ttyS4 init failed\n");
        serialDrv_.reset();
    } else {
        serialThread_ = std::make_unique<SerialThread>(
            *serialDrv_,
            "/dev/ttyS4",
            serial_rx_queue_
        );
        printf("[DRV] ttyS4 serial init ok\n");
    }

}

void DriverManager::start() {
    printf("[DRV] DriverManager start\n");
    canThread1_->start();
    if (serialThread_) {
        serialThread_->start();
    }
}

void DriverManager::stop() {
    printf("[DRV] DriverManager stop\n");

    /* === 先停线程 === */
    if (serialThread_) serialThread_->stop();
    if (canThread1_)   canThread1_->stop();

    /* === 再关 fd === */
    if (serialDrv_) serialDrv_->close();
    if (can1_)      can1_->close();

    canThread1_.reset();
    can1_.reset();
    serialThread_.reset();
    serialDrv_.reset();
}
