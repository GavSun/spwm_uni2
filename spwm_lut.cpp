#include "spwm_lut.h"

//Constants used in this function
#define HIGH true
#define LOW false
#define PI 3.141592654f
#define T_STEP 1.0e-8f      //Time increment = 10ns
#define scaling_factor 1000000  //Multiplication factor used for amplitudes of the carrier and signal waves.

/**
 * @brief Fills two arrays with ON & OFF durations of SPWM signals for a H bridge inverter. 
 * 
 * @param signal_freq   Signal frequency. 
 * Use 50 for 50Hz or 60 for 60Hz.
 * 
 * @param mf    Freq modulation index.
 * It determines the frequency of trangular carrier wave = mf x signal_freq.
 * 
 * @param ma    Amplitude modulation index.
 * It is used to scale the amplitude of sinusoidal signal w.r.t carrier signal. Must be less than 1.0 always. 0.8 mostly.
 * 
 * @param p_h1_high Pointer to 1st array. 
 * Here the ON and OFF durations for left side of H bridge are stored. The ON & OFF period are stored 
 * alternatively. First entry is always of ON duration. The lookup table must have size = (2 * mf).
 * 
 * @param p_h2_high Pointer to 2nd array.
 * Here the ON and OFF durations for right side of H Bridge are stored. The ON & OFF period are stored 
 * alternatively. First entry is always of ON duration. The lookup table must have size = (2 * mf).
 * 
 * @param h1_sync   Pointer to store synchronization value for 1st table.
 * It is used to start the timer / counter / PIO etc. peripherals for SPWM generation in a synchronised way.
 * 
 * @param h2_sync   Pointer to store synchronisation value for 2nd table.
 * It is used to start the timer / counter / PIO etc. peripherals for SPWM generation in a synchronised way.
 * 
 * @returns signal_duration Actual duration of main signal.
 * 
 * @note
 * First array is for the left side H-bridge and 2nd array is for driving right side of H_bridge.
 * However, the Hi & LO switches of H-bridge cannot be driven directly using these spwm values, 
 * as it needs complementary outputs. Same needs to acheived afterwards using these tables. 
 * 
 * The array length is always assumed = mf * 2. (e.g. if mf=256 then array size = ( 256 * 2 ) = 512)
 * 
 * SPWM value calculation Method:
 * The sine wave & triangular wave are computed at each T_STEP increment. (e.g. 10 nSec)
 * Then the amplitude of the sinusoidal signal (unity amplitude X ma) is compared with 
 * that of triangular wave (having unity amplitude).
 * 
 * Whenever the Sinusoidal signal goes above the triangular signal, the ON period starts.
 * Similarly whenever the Sinusoidal signal goes below the triangular wave, the OFF period starts.
 *  
 * Then the durations of the ON & OFF periods are calculated as below:
 * 
 * ON duration = Time when sine wave goes above tri wave - Time (previous) when sine wave goes below tri wave
 * 
 * OFF duration = Time when sine wave goes below tri wave - Time (previous) when sine wave goes above tri wave
 * 
 * Both the lookup tables hold total (mf * 2) values :
 * - 256 entries for ON durations (i.e. if mf = 256).
 * - 256 entries for OFF durations (i.e. if mf = 256).
 * 
 * For faster execution, mostely integer calculations are used. The floating point calculations are used only 
 * for evealuationg the sin() function. 
 */
uint32_t spwm_unipolar_arrays( uint8_t signal_freq, uint16_t mf, double ma,
                            uint32_t* p_h1_high, uint32_t* p_h2_high,
                            uint32_t* h1_sync, uint32_t* h2_sync ){

    /// Duration for one full cycle of triangular carrier wave where ach count = 1 T_STEP.
    uint32_t carrier_duration_quarter = (uint32_t)(1.0f/(T_STEP * (double)(signal_freq * mf * 4 )));
    uint32_t carrier_duration_half = (2 * carrier_duration_quarter); 
    uint32_t carrier_duration_3_quarter = (3 * carrier_duration_quarter);
    uint32_t carrier_duration = (4 * carrier_duration_quarter);
   
    /// Slope of triangular carrier wave which ramps from 1v to -1V and back to 1V.
    /// Slope = deta_Y / delta_X = ( 0-1 ) / ( 0 - Quarter carrier wave duration )
    ///       = 1 / Quarter carrier wave duration
    ///       = 1 / ( quarter duration count * T_STEP )
    /// For reducing the speed of calculation in subsequent steps, the formula arrived is as-
    /// Slope = ( scaling_factor * T_STEP ) / ( quarter duration count * T_STEP )
    ///       = scaling_factor  / quarter duration count
    int32_t carrier_slope = scaling_factor / carrier_duration_quarter; 
    int32_t carrier_amplitude = 0;

    ///Duration for one full cycle of signal (i.e. signal where each count = 1 T_STEP.
    uint32_t signal_duration = (carrier_duration * mf);
    uint32_t signal_duration_quarter = (signal_duration / 4);
    
    /// omega = w = 2 * PI * signal_frequency 
    ///  = 2 * PI * ( 1 / ( signal_duration count * T_STEP ) )
    /// The T_STEP is also to be mutipilied in numerator to speed up the calculations in subsequent steps
    /// w = ( 2 * PI * T_STEP ) / ( signal_duration count * T_STEP )
    /// After removing T_STEP, finally -
    /// w = ( 2 * PI ) / signal_duration
    double omega = (2.0f * PI) / (double)signal_duration;

    /// The ma is used to scale the signal sine wave. In addition the sine wave itself is 
    /// required to scaled up using 'scaling_factor'. Therfore -
    /// ma_scaled = ma * scaling_factor.
    uint32_t ma_scaled = (ma * scaling_factor); 
    
    /// Main sinusoidal signal
    int32_t s1_amplitude = 0; 
    bool s1_sync_captured = false;
    uint32_t h1_high_val_old = 0;

    /// Complemetary sinusoidal signal (inverse of main signal)
    int32_t s2_amplitude = 0;
    bool s2_sync_captured = false;
    uint32_t h2_high_val_old = 0;

    /// Holds the running time of signal wave (max count = one complete cycle). 1 count = 1 T_STEP
    uint32_t time_counter = 0;
    /// Hold the running time of triangular carrier wave (max count = one complete cycle). 1 count = 1 T_STEP
    uint32_t tri_time_counter = 0;

    /// Pointer for accessing the array elements
    uint32_t *p_h1_ref1, *p_h1_ref2, *p_h1_ref3, *p_h1_ref4, *p_h2_ref1, *p_h2_ref2, *p_h2_ref3, *p_h2_ref4;

    //Initialising the pointer to different locations of the arrays
    //Pointer to the elements of first array (First half of H Bridge)
    p_h1_ref1 = p_h1_high;              //pointer to first element of +ve half of sine wave s1 [0]
    p_h1_ref2 = p_h1_high + (mf-2);     //pointer to last-1 element of +ve half of sine wave s1 [254]
    p_h1_ref3 = p_h1_high + mf;         //pointer to first element of -ve half of sine wave s1 [256]
    p_h1_ref4 = p_h1_high + ((mf*2)-2); //pointer to last-1 element of -ve half of sine wave s1 [510]
    
    //Pointer to the elements of Second Array (Second half of H Bridge)
    p_h2_ref1 = p_h2_high;              //pointer to first element of +ve half of sine wave s2 [0]
    p_h2_ref2 = p_h2_high + (mf-2);     //pointer to last-1 element of +ve half of sine wave s2 [254]
    p_h2_ref3 = p_h2_high + mf;         //pointer to first element of -ve half of sine wave s2 [256]
    p_h2_ref4 = p_h2_high + ((mf*2)-2); //pointer to last-1 element of -ve half of sine wave s1 [510]

    uint16_t array_mid_point = (mf-1);
    uint16_t array_end_point = (2 * mf) - 1;

    uint16_t n = 0;                     // carrier wave cycle counter                 
    uint32_t result = 0;
    uint16_t max_cycle_counts = (mf/4); // Max limit for carrier wave cycle counts in one quarter of a sine wave
    
    //run one complete tri_wave carrier through its 4 quarters
    while (n < max_cycle_counts) { 
        //printf("N:%3d\n", n);
        
        //-------------------------------------------------------
        //Calculations during first quarter of the carrier wave
        //-------------------------------------------------------
        
        //Find the Sinewave amplitude at next known time (Here on 1 quarter end of carrier wave)
        // = sin( w * t )
        // = sin( ( 2.pi().Fs ) . ( T_COUNT.T_STEP ) )
        // = sin( (2.pi() / ( T_fs.T_STEP ) ) . ( T_COUNT.T_STEP ) )
        // Eliminating T_STEP from numerator & Denominator
        // = sin( ( 2.pi() / T_fs ) . T_COUNT )
        // omega is actualy pre-calulated this way
        // = sin(w . T_COUNT)
        s1_amplitude = (int32_t)(ma_scaled * sin( omega * ((n * carrier_duration) + carrier_duration_quarter) ) );
        
        // Estimates the time when carrier wave will reach to sin wave amplitude as calculated above
        // for carrier wave 1V = 1,000,000 counts (i.e. = scaling_factor)
        // at t= Tx the tri_time_counter = (Vinitial - Vslope@Tx) = s1_amplitude
        // Vinitial - ( Slope . Tx) = s1_amplitude
        // Vinitial - ( Slope . T_count . T_STEP) = s1_amplitude
        // After Rearranging for T_count (note T_ count is tri_time_counter)
        // T_Count  = Vinitial - s1_amplitude / (Slope . T_STEP)
        //          = Vinitial - s1_amplitude / ( (1 / Quarter_duration . T_STEP) . T_STEP )
        // After removing T_STEP from denominator
        //  T_Count  = Vinitial - s1_amplitude / ( (1 / Quarter_duration ) )
        // Divide numerator by scaling_factor as voltages are scaledup with the scaling_factor
        // T_Count  = Vinitial - s1_amplitude / (scaling_factor . (1 / Quarter_duration ) )
        // The calculation in denominator = slope and it is already implemented. Therefore finally
        // T_Count  = Vinitial - s1_amplitude / carrier slope
        // with Vinitial = 1v = 1000000 = scaling factor, final equation is:
        tri_time_counter = (scaling_factor - s1_amplitude) / carrier_slope ;
        
        // Futher calculations are performed only between this estinated time and when the
        // signal amplitude goes above carrier wave. 
        // (In this quarterof tri wave carrier only s1 needs comparison s2 can be ignored)
        // Advance the scan time so that unnecessary calculations are avoided.
        time_counter = (n * carrier_duration) + tri_time_counter;
                        
        //printf("1Q: G:%8d", tri_time_counter);
        while(tri_time_counter < carrier_duration_quarter) {
            //Get sine wave signal (s1) amplitude at this time 
            s1_amplitude = ma_scaled * sin( omega * time_counter );
            //Get carrier wave amplitude at same time instance
            //In first quarter the carrier wave the amplitude = Vinitial - ( Time *slope)
            //For carrier wave amplitude Vinitial = 1V = full scale = scaling_factor. Therefore -
            carrier_amplitude = scaling_factor - (carrier_slope * tri_time_counter);
            //Compare signal & carrier amplitudes
            if(s1_amplitude >= carrier_amplitude){ 
                //printf(" - T:%8d = %8d\n", tri_time_counter, (tri_time_counter - guess) ); 
                if(!s1_sync_captured){
                    *h1_sync = time_counter;    //Capture the sync count for 1st sine wave
                    //printf("n:%3d T1:%8d", n, tri_time_counter); 

                    //Set for next iteration
                    s1_sync_captured = true;
                    h1_high_val_old = time_counter;
                    time_counter = time_counter + (carrier_duration_quarter - tri_time_counter);
                    tri_time_counter = carrier_duration_quarter;
                }else{
                    //printf("n:%3d T1:%8d", n, tri_time_counter);  
                    result = time_counter - h1_high_val_old;

                    *p_h1_ref1 = result;  //S1 Array - Original location
                    p_h1_ref1++;
                    *p_h1_ref2 = result;  //S1 Array - Mirror location
                    p_h1_ref2--;
                    *p_h2_ref3 = result;  //S2 Array - Copy of S1 original location
                    p_h2_ref3++;
                    *p_h2_ref4 = result;  //S2 Array - Copy of S1 mirror location
                    p_h2_ref4--;
                    
                    //Setup for next quarter of tri wave carrier
                    h1_high_val_old = time_counter;
                    time_counter = time_counter + (carrier_duration_quarter - tri_time_counter);
                    tri_time_counter = carrier_duration_quarter;
                }
            }else{
                //Preparation for next loop iteration, advance time by T_STEP (i.e 10ns at 100MHz clk)
                time_counter++;
                tri_time_counter++;
            }
        } // while(time_counter < signal_duration_quarter)
        //printf(" Target: c:%8d, T:%8d\n", tri_time_counter, time_counter);

        //-------------------------------------------------------
        //Calculations during second quarter of the carrier wave
        //------------------------------------------------------- 
        //Advance carrier wave by minimising the time going into avaoidable calculations.
        s2_amplitude = -1 * (ma_scaled * sin( omega * ((n * carrier_duration) + carrier_duration_quarter))); 
        tri_time_counter = ((scaling_factor - s2_amplitude) / carrier_slope) ;
        time_counter = (n * carrier_duration) + tri_time_counter;
        //printf("2Q: G:%8d", tri_time_counter);

        while(tri_time_counter < carrier_duration_half){
            s2_amplitude = -1 * (ma_scaled * sin( omega * time_counter ));        
            carrier_amplitude = scaling_factor - (carrier_slope * tri_time_counter);
            if(s2_amplitude >= carrier_amplitude){ 
                //printf(" - T:%8d = %8d\n", tri_time_counter, (tri_time_counter - guess) );   
                if(!s2_sync_captured){
                    *h2_sync = time_counter;    //Capture the sync count for second sine wave
                    //printf(" T2:%8d", tri_time_counter);

                    result = *h1_sync + *h2_sync;
                    *(p_h1_high + array_mid_point) = result; //array1[255] : end of +ve halfcycle and start of -ve halfcycle
                    *(p_h1_high + array_end_point) = result; //array1[511] : last location. end of full cycle.
                    *(p_h2_high + array_mid_point) = result; //array2[255]: end of +ve halfcycle and start of -ve halfcycle
                    *(p_h2_high + array_end_point) = result; //array2[511] : last location. end of full cycle.
                    
                    //Setup for next iteration
                    s2_sync_captured = true;
                    h2_high_val_old = time_counter;
                    time_counter = time_counter + (carrier_duration_half - tri_time_counter);
                    tri_time_counter = carrier_duration_half;
                }else{
                    //printf(" T2:%8d", tri_time_counter);
                    result = time_counter - h2_high_val_old;

                    *p_h2_ref1 = result;  //S2 Array - Original location
                    p_h2_ref1++;
                    *p_h2_ref2 = result;  //S2 Array - Mirror location
                    p_h2_ref2--;
                    *p_h1_ref3 = result;  //S1 Array - Copy of S2 original location
                    p_h1_ref3++;
                    *p_h1_ref4 = result;  //S1 Array - Copy of S2 mirror location
                    p_h1_ref4--;
                    
                    //Setup for next iteration
                    h2_high_val_old = time_counter;
                    time_counter = time_counter + (carrier_duration_half - tri_time_counter);
                    tri_time_counter = carrier_duration_half;
                }
            }else{
                //Preparation for next loop iteration
                time_counter++;
                tri_time_counter++;
            }
        } //while(time_counter < signal_duration_half)
        //printf(" Target: c:%8d, T:%8d\n", tri_time_counter, time_counter);
        
        //-------------------------------------------------------
        //Calculations during third quarter of the carrier wave 
        //-------------------------------------------------------
        //Advance carrier wave by minimising the time going into avaoidable calculations.
        s2_amplitude = -1 * (ma_scaled * sin( omega * ((n * carrier_duration) + carrier_duration_3_quarter))); //time_counter
        tri_time_counter = ((scaling_factor + s2_amplitude) / carrier_slope) + carrier_duration_half;
        time_counter = (n * carrier_duration) + tri_time_counter;
        //printf("3Q: G:%8d", tri_time_counter);

        while(tri_time_counter < carrier_duration_3_quarter){
            s2_amplitude = -1 * (ma_scaled * sin( omega * time_counter ));
            carrier_amplitude = (-1 * scaling_factor) + ( carrier_slope * (tri_time_counter - carrier_duration_half));
            if(carrier_amplitude >= s2_amplitude){
                //printf(" - T:%8d = %8d\n", tri_time_counter, (tri_time_counter - guess) ); 
                result = time_counter - h2_high_val_old;

                *p_h2_ref1 = result;  //S2 Array - Original location
                p_h2_ref1++;
                *p_h2_ref2 = result;  //S2 Array - Mirror location
                p_h2_ref2--;
                *p_h1_ref3 = result;  //S1 Array - Copy of S2 original location
                p_h1_ref3++;
                *p_h1_ref4 = result;  //S1 Array - Copy of S2 mirror location
                p_h1_ref4--;
                
                //Preparation for next loop iteration
                h2_high_val_old = time_counter;
                time_counter = time_counter + (carrier_duration_3_quarter - tri_time_counter);
                tri_time_counter = carrier_duration_3_quarter;
            }else{
                //Preparation for next loop iteration
                time_counter++;
                tri_time_counter++;
            }
        }//while(tri_time_counter < carrier_duration_3_quarter)
        //printf(" Target: c:%8d, T:%8d\n", tri_time_counter, time_counter);

        //-------------------------------------------------------
        //Calculations during fourth quarter of the carrier wave 
        //-------------------------------------------------------
        //Advance carrier wave by minimising the time going into avaoidable calculations.
        s1_amplitude = ma_scaled * sin( omega * ((n * carrier_duration) + carrier_duration_3_quarter)); //time_counter
        tri_time_counter = ((scaling_factor + s1_amplitude) / carrier_slope) + carrier_duration_half;
        time_counter = (n * carrier_duration) + tri_time_counter;
        //printf("4Q: G:%8d", tri_time_counter);

        while(tri_time_counter < carrier_duration){
            s1_amplitude = ma_scaled * sin( omega * time_counter );
            carrier_amplitude = (-1 * scaling_factor) + ( carrier_slope * (tri_time_counter - carrier_duration_half));
            if( carrier_amplitude >= s1_amplitude ){  
                //printf(" - T:%8d = %8d\n", tri_time_counter, (tri_time_counter - guess) ); 
                result = time_counter - h1_high_val_old;

                *p_h1_ref1 = result;  //S1 Array - Original location
                p_h1_ref1++;
                *p_h1_ref2 = result;  //S1 Array - Mirror location
                p_h1_ref2--;
                *p_h2_ref3 = result;  //S2 Array - Copy of S1 original location
                p_h2_ref3++;
                *p_h2_ref4 = result;  //S2 Array - Copy of S1 mirror location
                p_h2_ref4--;
                
                //Set for next iteration
                h1_high_val_old = time_counter;
                time_counter = time_counter + (carrier_duration - tri_time_counter);
                tri_time_counter = carrier_duration;
                
            }else{
                //Preparation for next loop iteration
                time_counter++;
                tri_time_counter++;
            }
        }//while(tri_time_counter < carrier_duration)
        //printf(" Target: c:%8d, T:%8d\n", tri_time_counter, time_counter);
        
        //printf("Cycle:%3d of %3d complete\n", n, max_cycle_counts);
        n++;    //run next carrier wave (0 to (mf *2)-1)
    }//while (time_counter < signal_duration_quarter)    
    //printf("Cycle:%3d of %3d complete\n", n, max_cycle_counts);
    
    //------------------------------------
    //The above computations were for the first 90 degree of the sine wave s1 & s2.
    //For 90 to 180 deg the Symmtry of sine wave was used and values were copied.
    //Similary for 180 to 270 degree the symmetry used and values computed for s2 were copied.
    //Same operations performed for computing all the array elements of s2.
    //
    //Total [mf/4] cycles of tri wave carriers ( = 64 if mf = 256 ) are travelled.
    //And during this (mf/4) ON durations ( = 64 if mf = 256 ) and 
    //[(mf/40) - 1] OFF durations ( = 63 if mf = 256 ) were computed and stored. 
    //
    //The code below computes the last remaining OFF duration for both the sine waves & stores. 
    //(i.e. the element number 127 in array range from 0 to 511 if mf=256)
    //Note: Multiplication by 2 is for using the symmetry of wave at 90 deg
    result = 2 * (signal_duration_quarter - h1_high_val_old );
    *p_h1_ref1 = result;    //Array S1 element 127
    *p_h2_ref3 = result;    //copy same into S2 Array element number 383
    
    //Note: Multiplication by 2 is for using the symmetry of wave at 90 deg
    result = 2 * (signal_duration_quarter - h2_high_val_old );
    *p_h2_ref1 = result;    //Array S1 element 127
    *p_h1_ref3 = result;    ///copy same into S1 Array element number 383

    return(signal_duration);
}//void spwm_unipolar_arrays()    

uint32_t SPWM_array[128] = {
//-0----1-----0-----1-----0-----1-----0-----1---    
1944, 3945, 3830, 4021, 3753, 4098, 3676, 4174, 
3600, 4251, 3524, 4327, 3448, 4403, 3372, 4478, 
3297, 4554, 3222, 4628, 3147, 4703, 3073, 4777, 
2999, 4850, 2926, 4923, 2854, 4995, 2782, 5067, 
2710, 5138, 2640, 5208, 2570, 5277, 2501, 5346, 
2433, 5413, 2366, 5480, 2300, 5546, 2234, 5610, 
2170, 5674, 2107, 5737, 2045, 5798, 1984, 5859, 
1924, 5918, 1865, 5976, 1808, 6033, 1751, 6089, 
1697, 6143, 1643, 6196, 1591, 6247, 1540, 6297, 
1491, 6346, 1443, 6393, 1396, 6439, 1351, 6483, 
1308, 6526, 1266, 6567, 1226, 6606, 1187, 6644, 
1150, 6680, 1115, 6714, 1081, 6747, 1049, 6778, 
1019, 6808,  991, 6835,  964, 6861,  939, 6885, 
 916, 6907,  894, 6928,  875, 6946,  857, 6963, 
 841, 6978,  827, 6991,  815, 7003,  805, 7012, 
 796, 7020,  790, 7025,  785, 7029,  782, 7031}; //781    

