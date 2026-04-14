// //
// // Created by forlinx on 2025/12/23.
// //
//
#include "module_test.h"

#include <cstring>

#include  "driver_manager.h"
#include "serial_event.h"

// module_test::module_test()
// {
//     auto& driverMgr = DriverManager(*this);
//
//     driverMgr.init();
//     driverMgr.start();
//
// }
//
// void module_test::serial_init()
// {
//
//     for (int i = 0; i < std::size(test_str_); i++)
//     {
//         const char* hello = ("Hello World "+std::to_string(i)+"\r\n").c_str();
//         test_str_[i] = std::vector<uint8_t>(hello, hello + strlen(hello));
//     }
//         serial_send_cnt_ = 0;
//
// }
//
// void module_test::serial_send() {
//     driverMgr->getSerialThread().send(test_str_[serial_send_cnt_]);
//     serial_send_cnt_++;
// }
//
// void module_test::serial_receive()
// {
//     auto& serialRxQ = driverMgr->get_serialRxQueue();
//         SerialEvent ev;
//     while (serialRxQ.tryPop(ev))
//     {
//         switch (ev.type) {
//         case SerialEventType::RX_DATA:
//             printf("-----------------------[APP][SERIAL RX][%s] ",
//                    ev.port.c_str());
//             fwrite(ev.data.data(), 1, ev.data.size(), stdout);
//             printf("\n");
//             break;
//
//         case SerialEventType::RX_ERROR:
//             printf("[APP][SERIAL RX ERROR][%s] errno=%d\n",
//                    ev.port.c_str(), ev.error);
//             break;
//
//         case SerialEventType::TX_ERROR:
//             printf("[APP][SERIAL TX ERROR][%s] errno=%d\n",
//                    ev.port.c_str(), ev.error);
//             break;
//
//         default:
//             break;
//         }
//     }
// }
