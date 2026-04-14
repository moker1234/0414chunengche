//
// Created by lxy on 2026/3/13.
//

#ifndef ENERGYSTORAGE_BMS_LOGIC_ADAPTER_OUTFACTOR_H
#define ENERGYSTORAGE_BMS_LOGIC_ADAPTER_OUTFACTOR_H


inline struct ST2_Factor {
    uint8_t soc = 10;
    uint8_t soh = 10;
    uint8_t pack_v = 10;
    uint8_t pack_i = 1;
} st2_factor;






#endif //ENERGYSTORAGE_BMS_LOGIC_ADAPTER_OUTFACTOR_H