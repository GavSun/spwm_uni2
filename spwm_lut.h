#ifndef SPWM_LUT
    #define SPWM_LUT

    #include <math.h>
    #include <stdio.h>
    #include "pico/stdlib.h"
    
    uint32_t spwm_unipolar_arrays(uint8_t signal_freq, uint16_t mf, double ma, 
                            uint32_t* p_h1_high, uint32_t* p_h2_high,
                            uint32_t* h1_sync, uint32_t* h2_sync);
#endif