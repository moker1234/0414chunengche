//
// Created by forlinx on 2025/12/23.
//
#include  "driver_manager.h"

#ifndef ENERGYSTORAGE_MODULE_TEST_H
#define ENERGYSTORAGE_MODULE_TEST_H


class module_test
{
    public:
    module_test();
    ~module_test();

    void serial_init();

    void serial_send();

    void serial_receive();


private:
    DriverManager* driverMgr;
    std::vector<uint8_t> test_str_[5];
    int serial_send_cnt_;
};


#endif //ENERGYSTORAGE_MODULE_TEST_H