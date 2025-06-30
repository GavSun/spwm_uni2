# Generates Unipolar SPWM signals on RP2350 rpPico2 board

A H_Bridge single phase inverter uses 4 power semiconductor switches to generate AC output. The AC wave form is generated using high frequency Sinusoidal Pulse Width Modulated (SPWM) signals to drive these 4 switches of the H-Bridge.

These 4 switches are arranged in 2 half bridge configuration. Each half bridge needs 2 signals to drive the 2 switches in it. And these signals must be complementary so that only one switch is ON at any point of time.

These objectives are meet by the 2 files : 1) spwm_lut.cpp and 2) Main.cpp

### spwm_lut.cpp
- Fills two arrays with the ON & OFF durations of SPWM signals. The execution time for this function is less that 2.9ms.
 - The first array is for driving the one side of H-bridge and 2nd array is for driving the remaining side of H_bridge.
 - The arrays are filled with values for one complete cycle of the main signal (i.e. 50Hz or 60Hz).
 - Both the arrays hold total (mf * 2) values. e.g. If mf = 256, then each arrays will have:
    - 256 entries for the ON durations, and
    - 256 entries for the OFF durations.
- These ON & OFF duration values are stored sequentially in the arrays. The first value from the array is the duration carrying a logic HIGH output. The next value from the array is the duration carrying a logic LOW output. Same pattern is repeated for the entire array elements. i.e. There is no special storage for the logic level applicable to each value in the array.

Input Parameters required by spwm_lut.cpp:
- uint8_t signal_freq: Signal Frequency. Use 50 for 50Hz or 60 for 60Hz.

- uint16_t mf: Freq modulation index. It determines the frequency of trangular carrier wave = mf x signal_freq

- double ma: Amplitude modulation index. It is used to scale the amplitude of sinusoidal signal w.r.t carrier signal. Must be less than 1.0 always. 0.8 mostly.

- uint32_t* p_h1_high : Pointer to 1st lookup table. Here the ON and OFF durations for left side of H bridge are stored. The ON & OFF period are stored alternatively. First entry is always of ON duration. The lookup table must have size = (2 * mf).

- uint32_t* p_h2_high : pointer to 2nd lookup table. Here the ON and OFF durations for right side of H Bridge are stored. The ON & OFF period are stored alternatively. First entry is always of ON duration. The lookup table must have size = (2 * mf).

- uint32_t* h1_sync : Pointer to store synchronization value for 1st table. It is used to start the timer / counter / PIO etc. peripherals for SPWM generation in a synchronised way.

- uint32_t** h2_sync* : Pointer to store synchronisation value for 2nd table. It is used to start the timer / counter / PIO etc. peripherals for SPWM generation in a synchronised way.

This function returns 
- uint32_t duration_signal_freq : the duration of the exact signal_freq decided by this fuction.

### SPWM value calculation Method::
- This function internaly generates two sine waves which are 180* out of phase with each other (both at amplitude = 0.8 x ma) and a trangular wave (unity amplitude signal).
- The amplitudes of these 2 sine waves & triangular wave are computed at each T_STEP increment. (e.g. 10 nSec)
- The sine wave amplitudes are compared independently with trangular wave:
  - Whenever the Sinusoidal signal goes above the triangular signal, the ON period starts.
  - Similarly whenever the Sinusoidal signal goes below the triangular wave, the OFF period starts.
  - Then the durations of the ON & OFF periods are calculated as below:
  - ON duration = Time when sine wave goes above tri wave - Time (previous) when sine wave goes below tri wave
  - OFF duration = Time when sine wave goes below tri wave - Time (previous) when sine wave goes above tri wave
- These ON & OFF durations are stored in the respective arrays.

### main.cpp 
- First calls spwm_lut.cpp which in turn fills 2 arrays with SPWM values for one complete cycle of main signal.
- The the main.cpp corrects the values received in those two arrays for adding DEADTIME and for componsating the execution delays which gets added while loading those values into the peripherals (i.e. PIO)
- Therafter, it uses PIO and generates SPWM signals on GPIO pins of RP2350 RP PICO2 board.

### IMPORTANT NOTE:
The RP2350 gpio pins cannot drive H_bridge switches directly. MOSFET/IFBT gate drivers must be used on these pins to drive the respective switches.
