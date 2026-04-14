// third_party/openSAE_j1939/iso11783_stub.c
#include <stdint.h>
#include <stdbool.h>

// 这些函数在 OpenSAE 的 Listen_For_Messages.c / Request.c 里会被引用。
// 你当前项目不需要 ISO11783（ISOBUS），先用 stub 让链接通过。
// 后续如果需要 ISOBUS，把 OPENSAE_ENABLE_ISO11783=ON 并编译 ISO_11783 真实实现即可。

void ISO_11783_Read_Response_Request_Auxiliary_Estimated_Flow(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    (void)ID; (void)dlc; (void)data;
}
void ISO_11783_Read_Response_Request_General_Purpose_Valve_Estimated_Flow(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    (void)ID; (void)dlc; (void)data;
}
void ISO_11783_Read_Response_Request_Auxiliary_Valve_Measured_Position(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    (void)ID; (void)dlc; (void)data;
}
void ISO_11783_Read_Auxiliary_Valve_Command(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    (void)ID; (void)dlc; (void)data;
}
void ISO_11783_Read_General_Purpose_Valve_Command(uint32_t ID, uint8_t dlc, uint8_t data[]) {
    (void)ID; (void)dlc; (void)data;
}

// Request.c 里会多次引用这些 Response_Request_*
// 这里全部做空实现即可
void ISO_11783_Response_Request_Auxiliary_Valve_Estimated_Flow(uint8_t dest) { (void)dest; }
void ISO_11783_Response_Request_General_Purpose_Valve_Estimated_Flow(uint8_t dest) { (void)dest; }
void ISO_11783_Response_Request_Auxiliary_Valve_Measured_Position(uint8_t dest) { (void)dest; }
