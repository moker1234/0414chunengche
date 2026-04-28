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


inline struct ST5_Factor {
    uint16_t st5_max_ucell = 1000;
    uint16_t st5_min_ucell = 1000;
    uint16_t st5_avg_ucell = 1000;
} st5_factor;

inline struct EE_Factor {
    uint16_t EE_single_chg_energy = 10;
} ee_factor;

inline struct CL_Factor {
    uint16_t CL_pulse_discharge_limit_a = 10;
    uint16_t CL_pulse_charge_limit_a = 10;
    uint16_t CL_follow_charge_limit_a = 10;
} cl_factor;




#endif //ENERGYSTORAGE_BMS_LOGIC_ADAPTER_OUTFACTOR_H