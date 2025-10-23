
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
#include"fuel_gauge.h"

#ifdef TERMINAL_IO_USE_SEGGER_RTT
#include "SEGGER_RTT.h"
#endif

/*defintions -------------------------------------------------------------------------------------*/
#define console //Comment this if you want to disable printf


/*globals -------------------------------------------------------------------------------------*/

uint8_t counter = 0;
volatile unsigned long timer_count = 0;
int error, tempdata1;
int retvalue = 0;
char log_string[256] = {0}; // Used to print the paramter values on terminal
uint8_t max17261_regs1[256]; // represents all the max17261 registers.
unsigned long timer_count_value;
char *OutputString;







/* Private function declarations -------------------------------------------------------------------------------------*/

// the error handler simply rapidly blinks the given LED color forever
static void error_handler(Status_LED_Color_t c);


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
        max17261_wait_dnr();
        max17261_config_ez();
        max17261_clear_por_bit();
    }
    else
    {
        #if defined(console)
        printf("POR not detected\r\n");
        #endif
        
        error_handler(STATUS_LED_COLOR_RED);
    }
    
    MXC_Delay(MXC_DELAY_SEC(2));  

    // without a brief delay between card init and mount, there are often mount errors
    MXC_Delay(100000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);

    while (1)
    {
          status_led_set(STATUS_LED_COLOR_GREEN, false);
          status_led_set(STATUS_LED_COLOR_BLUE, false);           
          memset(&log_string, 0, sizeof(log_string));     // initialize log_string to all zeroes
          Fuel_gauge_data_collect_before(&log_string[0]); // read fuel gauge registers and make string to log
            
#if defined(console)
            printf("log_string = %s", &log_string[0]);  // print log string to debug console
#endif
           
            MXC_Delay(5000000);//Waiting so we can see some difference between fuel gauge readings


            memset(&log_string, 0, sizeof(log_string));    // initialize log_string to all zeroes
            Fuel_gauge_data_collect_after(&log_string[0]); // read fuel gauge registers and make string to log
            printf("log_string = %s", &log_string[0]);  // print log string to debug console
            

#if defined(console)
             printf("log_string = %s", &log_string[0]);  // print log string to debug console
#endif
            
            status_led_set(STATUS_LED_COLOR_BLUE, true);
        }

        MXC_Delay(500000);
    

    
}

/* Private function definitions --------------------------------------------------------------------------------------*/

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