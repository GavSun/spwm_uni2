//includes from system
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/uart.h"

//includes from this project
#include "spwm_lut.h"

//Our assembly program
#include "spwm_uni.pio.h"

// This example uses the default led pin
// Change this by defining HELLO_PIO_LED_PIN to use a different gpio
#if !defined HELLO_PIO_LED_PIN && defined PICO_DEFAULT_LED_PIN
#define HELLO_PIO_LED_PIN PICO_DEFAULT_LED_PIN
#endif

// Check the pin is compatible with the platform
#if HELLO_PIO_LED_PIN >= NUM_BANK0_GPIOS
#error Attempting to use a pin>=32 on a platform that does not support it
#endif

//MCU RP2350 GPIO pins on PICO2 used for full bridge drive
#define PICO2_PIN_GP14 14   //H1_HIGH
#define PICO2_PIN_GP15 15   //H1_LOW
#define PICO2_PIN_GP17 17   //H2_HIGH
#define PICO2_PIN_GP16 16   //H2_LOW
#define SYNC_OUT_50HZ 18   //50Hz Sync out //PICO2_PIN_GP22

// UART defines -By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

//Buffer size in bits= (1<<size_bits) = 2^11 = 2048 bytes
//Used while configuring the DMA for PIO data transfer
#define BUFFER_SIZE_BITS 11

//Buffer Size in Bytes
#define BUFFER_SIZE 2048    //number of bytes in an array of uint32_t type with size 512 (512 * 4)

//Arrays for holding the sinusodal waveform values
uint32_t __attribute__ ((aligned(BUFFER_SIZE))) spwm_h1_high_table[BUFFER_SIZE/4]; //Array size = 512
uint32_t __attribute__ ((aligned(BUFFER_SIZE))) spwm_h2_high_table[BUFFER_SIZE/4]; //Array size = 512

uint32_t h1_sync_count, h2_sync_count;
//uint32_t  __attribute__ ((aligned(8))) flash_led_50hz[2] = { 1000000, 1000000}; //, 1000000, 1000000 };

//-----------------------------------------------
//      Constants used in this Application
//-----------------------------------------------
// Configurable constants before compilation
//#define TRIWAVE_DURATION 78.12e-6f
#define SIGNAL_FREQ 50
#define MOD_INDEX_MA 0.8f
#define MOD_INDEX_MF 256

#define DEAD_TIME 50            //The DEAD_TIME to ensure HI & LO side switches do not switch simultaniously
#define DEADTIME_COMPENSATION 2 //This delay is added by the instructions in the PIO program while adding DEADTIME.
#define IE_DELAY_COMPENSATION 3 //This delay is added by the instructions in the PIO program while creating SPWM pulses.

// Auto calculations
#define NET_DEADTIME_COUNT (DEAD_TIME-DEADTIME_COMPENSATION)
//#define SYNCOUT_HALF_DURATION (((uint32_t(TRIWAVE_DURATION * 1.0e+8f) * MOD_INDEX_MF)/2)-DEADTIME_COMPENSATION)
//-----------------------------------------------

void configure_dma_for_pio(PIO pio_spwm, uint sm_spwm, int dc, dma_channel_config *dc_cfg, uint32_t *data_array){
    
    /*
    //Shall the PIO & SM be disabled?
    pio_sm_set_enabled(pio, sm, false);

    //Shall the FIFO be cleared?
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    */
   
    //The default configuration for the DMA channel is received in dc_cfg. Modify it
    channel_config_set_transfer_data_size(dc_cfg, DMA_SIZE_32); //32-bit transfers
    channel_config_set_read_increment(dc_cfg, true);            //Inc read address (data array)
    channel_config_set_write_increment(dc_cfg, false);          //Don't inc write address (PIO FIFO)
    
    //The third param is 'size_bits'. It holds the ring_buffer size (in bytes) in a different way.
    //e.g. if address wrap-up is to take place for 512 values of 32 bits each, 
    //then ring buffer will have 512 x 4 = 2048 bytes. 
    //And as 2^11 = 2048,  therefore the size_bits = 11 0r 0x0B;
    channel_config_set_ring(dc_cfg, false, BUFFER_SIZE_BITS);    //2^11 = 2048 bytes = 512 words of 4 bytes each              
    channel_config_set_dreq(dc_cfg, pio_get_dreq(pio_spwm, sm_spwm, true));   //Pace to PIO DREQ
    
    dma_channel_configure(
        dc,                         // Channel to be configured
        dc_cfg,                    // The configuration just created
        &pio_spwm->txf[sm_spwm],    // Write address (PIO FIFO)
        data_array,                 // Adress of 1st value in SPWM lookup table
        -1,                         // -1 = 0xFFFF FFFF for TRANS_COUNT register of DMA
                                    // Bits 31:28 sets the mode, Bits 27:0 are the transfer_count
                                    // Set bits 31:28 = 0xf to let DMA channel Run endlessly.
                                    // In this mode the transfer_count never decrements
        true                        // Start immediately.
    );
    
    //channel_config_set_enable (&dc_cfg0, true);    //DMA Enabled by Default
    
    /*
    //Was the PIO & SM was disabled for configuring the DMA? then enable again.
    pio_sm_set_enabled(pio, sm, true);
    */
}


int main()
{
    stdio_init_all();

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    sleep_ms(10000);

    //Send out a string, with CR/LF conversions
    //uart_puts(UART_ID, " Hello, UART!\n");
    
    //-----------------------------------------------------------------------------------
    
    printf("Lookup table computation starts ----- \n");
    
    uint64_t start_time, end_time;
    
    //Added only to get idea about how much time it takes to calulate the array elements.
    start_time = time_us_64();
    
    //Compute SPWM lookup table values
    uint32_t signal_duration = spwm_unipolar_arrays(SIGNAL_FREQ, MOD_INDEX_MF, MOD_INDEX_MA, spwm_h1_high_table, spwm_h2_high_table, 
                        &h1_sync_count, &h2_sync_count);
    
    end_time = time_us_64();
    printf("execution time: %llu\n", end_time - start_time);
    
    //The values in the lookup tables must be adjusted for DEADTIME & execution delays.
    //DEAD_TIME is the time when both the switches must be off before one of them switches ON.
    //Execution delay is the delay introduced by the assembly instructions in PIO program.
    printf("%3d SPWM1: %5d", -1, h1_sync_count);
    h1_sync_count = h1_sync_count - DEAD_TIME - IE_DELAY_COMPENSATION;
    printf(" : %5d", h1_sync_count);
    
    printf(" SPWM2: %5d", h2_sync_count);
    h2_sync_count = h2_sync_count - DEAD_TIME - IE_DELAY_COMPENSATION;
    printf(" : %5d\n", h2_sync_count);

    for(uint16_t i=0; i<(MOD_INDEX_MF*2); i++){
        printf("%3d SPWM1: %5d", i, spwm_h1_high_table[i]);
        spwm_h1_high_table[i] = spwm_h1_high_table[i] - DEAD_TIME - IE_DELAY_COMPENSATION;
        printf(" : %5d",spwm_h1_high_table[i]);
        printf(" SPWM2: %5d", spwm_h2_high_table[i]);
        spwm_h2_high_table[i] = spwm_h2_high_table[i] - DEAD_TIME - IE_DELAY_COMPENSATION;
        printf(" : %5d\n",spwm_h2_high_table[i]);
    }
    
    uint32_t sync_out_half_duration = ((signal_duration/2)-DEADTIME_COMPENSATION);

    printf("Lookup table computation complete....\n");
    
    //-----------------------------------------------------------------------------------
    
    printf("Preparing to start SPWM switching. Setting up PIO....\n");

    PIO pio;
    uint sm[3];         //Using all 4 SM from same PIO
    uint offset[3];     //all 4 SM have independent programs to run
    float clkdiv = 1.5f;  // (fsys / 1.5) = 150Mhz / 1.5 = 100MHz clk to PIO     
    //Each PIO instruction takes One clk or 1/ 100MHZ = 10ns for execution

    //Array to hold dma channels and their configurations
    int dma_ch0, dma_ch1, dma_ch2, dma_ch3;
    dma_channel_config dma_cfg0, dma_cfg1, dma_cfg2, dma_cfg3;

    //----------------------------------------------------------------
    //Setting up the PIO and the first SM in it for H1 half bridge (h1_hi & h1_lo output)
    // Find a free pio and state machine
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&spwm_h1_program, &pio, &sm[0], &offset[0], PICO2_PIN_GP14, 2, true); 
    if(!success) {printf("NO PIO or SM for H1 halfbridge..\n");}
    hard_assert(success);
    // Configure pio to run the assembley program, and start it, using the helper function in .pio file.
    spwm_h1_program_init(pio, sm[0], offset[0], clkdiv, PICO2_PIN_GP14, 2); 
    //The pio and SM ready but not enabled yet.
    printf("SPWM Output on pico2 board -> H1_hi:GP%d H1_lo:GP%d\n", PICO2_PIN_GP14, PICO2_PIN_GP15); 
    
    //Load the "DEAD_TIME' into ISR PIO,  it will be inserted while changing the hi & lo side swiches
    pio_sm_clear_fifos (pio, sm[0]);    //Clear TX & RX FIFO
    pio_sm_put (pio, sm[0], NET_DEADTIME_COUNT);    //Put a 'NET_DEADTIME_COUNT' into TX FIFO
    pio_sm_exec(pio, sm[0], pio_encode_pull(false, false)); // Pull the count into OSR
    pio_sm_exec(pio, sm[0], pio_encode_out(pio_isr, 32));   // Copy OSR contents into ISR
    pio_sm_put (pio, sm[0], h1_sync_count);    //Put synchronization count into TX_FIFO , it is required at startup.
    //The pio and SM ready but not enabled yet.

    // Now get and set a DMA channel to assit PIO, panic() if there are none
    dma_ch0 = dma_claim_unused_channel(true);
    dma_cfg0 = dma_channel_get_default_config(dma_ch0);
    configure_dma_for_pio(pio, sm[0], dma_ch0, &dma_cfg0, &spwm_h1_high_table[0]);
    printf("1st DMA assigned to PIO:SM[0]....\n");
    
    //--------------------------------------------
    //setting up second SM in same PIO for H2 half bridge (h2_hi & H2_lo output)
    success = pio_claim_free_sm_and_add_program_for_gpio_range(&spwm_h2_program, &pio, &sm[1], &offset[1], PICO2_PIN_GP16, 2, false);
    if(!success) {printf("NO PIO or SM for H2 halfbridge..\n");}
    hard_assert(success);
    printf("SPWM Output on pico2 board -> H2_hi:GP%d H2_lo:GP%d\n", PICO2_PIN_GP16, PICO2_PIN_GP17);
    spwm_h2_program_init(pio, sm[1], offset[1], clkdiv, PICO2_PIN_GP16, 2);

    //Load the "DEAD_TIME'into ISR, to be inserted while changing the state of hi & lo side swiches
    pio_sm_clear_fifos (pio, sm[1]);    //Clear TX & RX FIFO
    pio_sm_put (pio, sm[1], NET_DEADTIME_COUNT);    //Put a 'NET_DEADTIME_COUNT' into TX FIFO
    pio_sm_exec(pio, sm[1], pio_encode_pull(false, false)); // Pull the count into OSR
    pio_sm_exec(pio, sm[1], pio_encode_out(pio_isr, 32));   // Copy OSR contents into ISR
    pio_sm_put (pio, sm[1], h2_sync_count);    //Put synchronization count into TX_FIFO , it is required at startup.
    //The pio and SM ready but not enabled yet.

    // Now get and set a DMA channel to assit PIO, panic() if there are none
    dma_ch1 = dma_claim_unused_channel(true);
    dma_cfg1 = dma_channel_get_default_config(dma_ch1);
    configure_dma_for_pio(pio, sm[1], dma_ch1, &dma_cfg1, &spwm_h2_high_table[0]);
    printf("2nd DMA assigned to PIO:SM[1]..\n");
    
    //--------------------------------------------
    //setting up 3rd SM for 50HZ SYNC_OUT
    success = pio_claim_free_sm_and_add_program_for_gpio_range(&sync_out_program, &pio, &sm[2], &offset[2], SYNC_OUT_50HZ, 1, false);
    
    if(!success) {printf("NO PIO or SM for LED flashing..\n");}
    hard_assert(success);
    printf("50Hz SYNC_OUT on PICO-2: GP %d\n", SYNC_OUT_50HZ);
    sync_out_program_init(pio, sm[2], offset[2], clkdiv, SYNC_OUT_50HZ, 1);
    //The pio and SM ready but not enabled yet.
    
    //Load the "Duration' required for producing 50HC SYNC_OUT
    pio_sm_clear_fifos (pio, sm[2]);    //Clear TX & RX FIFO
    pio_sm_put (pio, sm[2], sync_out_half_duration);    //Put a 'Half Duration of SYNC_OUT' into TX FIFO
    pio_sm_exec(pio, sm[2], pio_encode_pull(false, false)); // Pull the 'Duration' into OSR
    pio_sm_exec(pio, sm[2], pio_encode_out(pio_isr, 32));   // Copy OSR contents into ISR
    //Now PIO and SM can be eanbled to run the assembly program.
    
    printf("PIO & SM2 started. No DMA required here...\n");

    //------------------------------------------------------------------------

    //Noew start all the SM in same PIO synchronusly.
    pio_enable_sm_mask_in_sync( pio, ( (1 << sm[0]) | (1 << sm[1]) | (1 << sm[2]) ) );  
    
    // press a key to exit. 
    //while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
    while (true) {
        //pio_sm_put_blocking(pio, sm, 100000000); //OFF period
        sleep_ms(100);
    }

    //Time to stop DMA PIO SM etc and free the resources.

    //Disable DMA
    channel_config_set_enable (&dma_cfg0, false);
    channel_config_set_enable (&dma_cfg1, false);
    printf("DMA Disabled ....\n");

    //Disable SM in PIO being used.
    //pio_sm_set_enabled(pio, sm[0], false);
    //pio_sm_set_enabled(pio, sm[0], false);
    pio_enable_sm_mask_in_sync(pio, ( (0 << sm[0]) | (0 << sm[1]) | (0 << sm[2]) )); // | (0 << sm[3])
    printf("PIO Disabled ....\n");

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&spwm_h1_program, pio, sm[0], offset[0]);
    pio_remove_program_and_unclaim_sm(&spwm_h2_program, pio, sm[1], offset[1]);
    pio_remove_program_and_unclaim_sm(&sync_out_program, pio, sm[2], offset[2]);
}//main()
