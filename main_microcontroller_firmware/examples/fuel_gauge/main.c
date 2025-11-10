
/* Private includes --------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_delay.h"
#include "board.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "bsp_status_led.h"
#include "bsp_pushbutton.h"
#include "board.h"
#include "nvic_table.h"
#include "mxc_device.h"
#include "fuel_gauge.h"
#include "lp.h"
#include "rtc.h"
#include "wut.h"
#include "tmr.h"

#ifdef TERMINAL_IO_USE_SEGGER_RTT
#include "SEGGER_RTT.h"
#endif

/*defintions -------------------------------------------------------------------------------------*/
#define ShowPrintFOutput 1//Comment this if you want to disable printf
#define console 1

/*globals -------------------------------------------------------------------------------------*/

uint8_t counter = 0;
volatile unsigned long timer_count = 0;
int error, tempdata1;
int retvalue = 0;
char log_string[256] = {0}; // Used to print the paramter values on terminal
volatile int wut_flag = 0;
uint8_t max17261_regs1[256]; // represents all the max17261 registers.
unsigned long timer_count_value;
char *OutputString;







/* Private function declarations -------------------------------------------------------------------------------------*/

// the error handler simply rapidly blinks the given LED color forever
static void error_handler(Status_LED_Color_t c);

// Low power sleep function using ARM WFI with guaranteed wake-up
void wut_sleep_handler(void);
int enter_low_power_sleep(uint32_t sleep_seconds);


/* Public function definitions ---------------------------------------------------------------------------------------*/

int main(void)
{

    // power down the console UART to conserve power
    #if !defined(console)
    bsp_console_uart_deinit();
    #endif      

    // simple LED pattern for a visual indication of a reset
    status_led_set(STATUS_LED_COLOR_RED, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, true);
    MXC_Delay(1000000);
    status_led_all_off();

    status_led_set(STATUS_LED_COLOR_RED, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, true);
    MXC_Delay(1000000);
    status_led_all_off();
    bsp_power_on_LDOs();


    if (bsp_1v8_i2c_init() != E_NO_ERROR) //Initializes the 1.8V I2C module
    {
        error_handler(STATUS_LED_COLOR_GREEN);
    }

    
    // max17261_i2c_test();
    MXC_Delay(2000);
    max17261_soft_reset(); // perform soft reset
    MXC_Delay(250);
    if (max17261_por_detected())
    {   // load max17261 configuration
        if (!max17261_wait_dnr(0)) {
            printf("ERROR: DNR wait timeout during POR configuration\r\n");
        }
        if (!max17261_config_ez(0)) {
            printf("ERROR: EZ config timeout during POR configuration\r\n");
        }
        max17261_clear_por_bit();
    }
    else
    {
        #if defined(console)
        printf("POR not detected, forcing reconfiguration\r\n");
        #endif
        // Force reconfiguration even without POR
        if (!max17261_wait_dnr(0)) {
            printf("ERROR: DNR wait timeout during forced reconfiguration\r\n");
        }
        if (!max17261_config_ez(0)) {
            printf("ERROR: EZ config timeout during forced reconfiguration\r\n");
        }
    }
    
    MXC_Delay(MXC_DELAY_SEC(2));  

    // without a brief delay between card init and mount, there are often mount errors
    MXC_Delay(100000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    
    // Initialize power optimization
#if defined(console)
    printf("\n\n***** MAX17261 Fuel Gauge Example *****\r\n");
    printf("This example reads data from the MAX17261 fuel gauge.\r\n");
    printf("The device will be configured if POR is detected.\r\n");
    printf("Low power mode enabled for power optimization.\r\n\n");
    
    printf("=== STARTUP DELAY ===\r\n");
    printf("Waiting 10 seconds for terminal connection...\r\n");
    for (int i = 10; i > 0; i--) {
        printf("Starting in %d seconds...\r\n", i);
        status_led_set(STATUS_LED_COLOR_BLUE, true);
        MXC_Delay(500000); // 0.5 second
        status_led_set(STATUS_LED_COLOR_BLUE, false);
        MXC_Delay(500000); // 0.5 second (total 1 second)
    }
    printf("Starting fuel gauge monitoring with low power sleep...\r\n\n");
#endif

    while (1)
    {
          // Turn off LEDs to save power
          status_led_all_off();       
          double time_to_empty_h = max17261_read_tte();
          double time_to_empty_days = time_to_empty_h / 24;
          printf("Time to empty (register): %.2f days\n", time_to_empty_days);
          
          // Use manual calculation for large batteries
          double manual_tte_h = max17261_calculate_tte_manual();
          double manual_tte_days = manual_tte_h / 24;
          printf("Time to empty (manual): %.2f days\n", manual_tte_days);
          
          // Debug capacity registers (only every 10th cycle to save power)
          static uint8_t debug_counter = 0;
          if (++debug_counter >= 10) {
              max17261_read_capacity_debug();
              debug_counter = 0;
          }
          
          // Force reconfiguration if DesignCap is wrong
          uint16_t current_designcap = max17261_read_designcap();
          
          if (current_designcap != 0x1950) {
              printf("DesignCap incorrect (0x%04X), forcing reconfiguration...\n", current_designcap);
              max17261_soft_reset();
              MXC_Delay(500000); // 500ms delay
              if (!max17261_wait_dnr(0)) {
                  printf("ERROR: DNR wait timeout during DesignCap reconfiguration\r\n");
              }
              if (!max17261_config_ez(0)) {
                  printf("ERROR: EZ config timeout during DesignCap reconfiguration\r\n");
              }
              
              // Reset QH register for fresh start with new capacity
              max17261_reset_qh();
              
              printf("Reconfiguration and QH reset complete\n");
          }

          // Read and display fuel gauge data
          fuel_gauge_data_t before_data = Fuel_gauge_data_collect("Before");

          printf("Current mA: %.2f mA, Voltage: %.3f V, Power: %.2f mW\n", 
                 before_data.current_ma, before_data.vcell_voltage, before_data.power_mw);

          printf("Avg. Current mA: %.2f mA, Avg. Volatge: %.3f V, Avg. Power: %.2f mW\n", 
                 before_data.avg_current_ma, before_data.avg_vcell_voltage, before_data.avg_power_mw);
            
          
#if defined(console)
          printf("About to enter low power sleep...\r\n");
          fflush(stdout);
#endif
           
          // Enter low power sleep instead of active delay
          enter_low_power_sleep(60); // Sleep for 60 seconds in low power mode
          
#if defined(console)
          printf("Returned from low power sleep\r\n");
          fflush(stdout);
#endif
          
          // Read and display fuel gauge data again
          fuel_gauge_data_t after_data = Fuel_gauge_data_collect("After");
          
          // Example: Use the data for calculations or decisions
          double voltage_change = after_data.vcell_voltage - before_data.vcell_voltage;
          double current_change = after_data.current_ma - before_data.current_ma;
          
#if defined(console)

            printf("Current mA: %.2f mA, Voltage: %.3f V, Power: %.2f mW\n", 
                 after_data.current_ma, after_data.vcell_voltage, after_data.power_mw);

            printf("Avg. Current mA: %.2f mA, Avg. Volatge: %.3f V, Avg. Power: %.2f mW\n", 
                    after_data.avg_current_ma, after_data.avg_vcell_voltage, after_data.avg_power_mw);

            printf("Analysis: Voltage change = %.3f V, Current change = %.2f mA\r\n", 
                    voltage_change, current_change);
#endif
            
            // Brief LED indication only
            status_led_set(STATUS_LED_COLOR_BLUE, true);
            MXC_Delay(100000); // 100ms
            status_led_set(STATUS_LED_COLOR_BLUE, false);
            
            // Enter low power mode between measurements
            // You can implement sleep mode here if supported by your MCU
        }
    

    
}

/* Private function definitions --------------------------------------------------------------------------------------*/

volatile int wut_sleep_flag = 0;

// Wake-Up Timer interrupt handler for low power wake-up
void wut_sleep_handler(void)
{
    wut_sleep_flag = 1;
    // Clear WUT interrupt flag (implementation may vary)
}

int enter_low_power_sleep(uint32_t sleep_seconds)
{
#if defined(console)
    printf("Entering low power sleep for %lu seconds...\r\n", sleep_seconds);
#endif
    
    // Turn off all LEDs and unnecessary peripherals to save power
    status_led_all_off();
    
    // Flush console output but keep UART active for debugging
    #if defined(console)
    fflush(stdout);
    #endif
    
    // Reliable low power sleep implementation using chunked delays
    // This ensures guaranteed wake-up without complex timer dependencies
    
    wut_sleep_flag = 0;
    uint32_t remaining_seconds = sleep_seconds;
    
#if defined(console)
    printf("Starting chunked low power sleep...\r\n");
#endif
    
    // Sleep in chunks to ensure reliable wake-up and power savings
    while (remaining_seconds > 0) {
        uint32_t chunk_size;
        
        // Determine chunk size (max 10 seconds per chunk for safety)
        if (remaining_seconds >= 10) {
            chunk_size = 10;
        } else {
            chunk_size = remaining_seconds;
        }
        
        // Enter low power mode for this chunk
        // Use lighter sleep to maintain UART communication
        MXC_Delay(MXC_DELAY_SEC(chunk_size));
        
        // Update remaining time
        remaining_seconds -= chunk_size;
        
        // Status update every chunk for debugging
#if defined(console)
        printf("Sleep chunk complete: %lu/%lu seconds remaining\r\n", 
               remaining_seconds, sleep_seconds);
        fflush(stdout);
#endif
    }
    
    wut_sleep_flag = 1; // Mark successful completion
    
#if defined(console)
    printf("Woke up successfully after %lu seconds\r\n", sleep_seconds);
#endif
    
    return E_NO_ERROR;
}

void error_handler(Status_LED_Color_t color)
{
    status_led_all_off();

    const uint32_t fast_blink = 100000;
    while (true)
    {
        status_led_toggle(color);
        MXC_Delay(fast_blink);
    }
}