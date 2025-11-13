/**
 * @file main.c
 * @brief BME688 Environmental Sensor Example
 * 
 * This example demonstrates reading temperature, humidity, pressure, and gas resistance
 * from the BME688 environmental sensor using I2C communication at 1.8V.
 * 
 * Features:
 * - BME688 initialization and configuration
 * - Periodic sensor readings with low power sleep
 * - Temperature, humidity, pressure, and gas measurements
 * - Error handling and status indication via LEDs
 * - Console output for debugging and monitoring
 */

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
#include "environmental_sensor.h"
#include "lp.h"
#include "rtc.h"
#include "wut.h"
#include "tmr.h"

#ifdef TERMINAL_IO_USE_SEGGER_RTT
#include "SEGGER_RTT.h"
#endif

/* Definitions -------------------------------------------------------------------------------------*/
#define ShowPrintFOutput 1 // Comment this if you want to disable printf

/* Globals -------------------------------------------------------------------------------------*/
uint8_t counter = 0;
volatile unsigned long timer_count = 0;
int error, tempdata1;
int retvalue = 0;
char log_string[256] = {0}; // Used to print the parameter values on terminal
volatile int wut_flag = 0;
unsigned long timer_count_value;
char *OutputString;

/* Private function declarations -------------------------------------------------------------------------------------*/

// The error handler simply rapidly blinks the given LED color forever
static void error_handler(Status_LED_Color_t c);

// Removed sleep functions for simplified testing

// BME688 specific functions
void display_sensor_data(bme688_data_t *data, const char *label);
void analyze_air_quality(float gas_resistance);

/* Public function definitions ---------------------------------------------------------------------------------------*/

int main(void)
{
    int result;

    // Power down the console UART to conserve power (optional)
    // bsp_console_uart_deinit(); // Uncomment if you want to disable console      

    // Simple LED pattern for a visual indication of a reset
    status_led_set(STATUS_LED_COLOR_RED, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, true);
    MXC_Delay(1000000);
    status_led_all_off();

    printf("\n\n***** BME688 Environmental Sensor Example *****\r\n");
    printf("This example reads environmental data from the BME688 sensor.\r\n");
    printf("Measurements include: Temperature, Humidity, Pressure, and Gas Resistance.\r\n");
    printf("Continuous monitoring with 1-second intervals.\r\n\n");
    
    // Power on LDOs for sensor operation
    bsp_power_on_LDOs();

    printf("Initializing I2C (1v8) ... \n");

    // Initialize 1.8V I2C module for BME688 communication
    if (bsp_1v8_i2c_init() != E_NO_ERROR) {
        printf("ERROR: I2C (1v8) initialization failed\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    printf("[SUCCESS] --> I2C (1v8) initialized\n");

    // Brief delay for I2C stabilization
    MXC_Delay(2000);

    // Initialize BME688 sensor
    result = bme688_init();
    if (result != E_NO_ERROR) {
        printf("ERROR: BME688 initialization failed with code: %d\r\n", result);
        error_handler(STATUS_LED_COLOR_RED);
    }
    printf("[SUCCESS] --> BME688 initialized\n");

    // Brief delay after initialization
    MXC_Delay(100000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);    
    
    printf("=== STARTUP DELAY ===\r\n");
    printf("Waiting 10 seconds for terminal connection...\r\n");
    for (int i = 10; i > 0; i--) {
        printf("Starting in %d seconds...\r\n", i);
        status_led_set(STATUS_LED_COLOR_BLUE, true);
        MXC_Delay(500000); // 0.5 second
        status_led_set(STATUS_LED_COLOR_BLUE, false);
        MXC_Delay(500000); // 0.5 second (total 1 second)
    }
    printf("Starting continuous environmental monitoring...\r\n\n");

    // Main measurement loop with power-efficient single readings
    uint32_t reading_count = 0;
    
    while (1) {
        reading_count++;
        
        // Turn off LEDs to save power
        status_led_all_off();

        printf("=== Reading #%lu ===\r\n", reading_count);
        
        // Single sensor reading (BME688 will automatically return to sleep after measurement)
        bme688_data_t sensor_data = bme688_read_all_data("Power-Efficient Reading");
        
        if (sensor_data.valid_data) {
            // Display comprehensive sensor data
            display_sensor_data(&sensor_data, "Current Reading");
            
            // Analyze air quality based on gas resistance
            analyze_air_quality(sensor_data.gas_resistance_ohm);
            
            // Convert pressure to different units for display
            float pressure_psi = sensor_data.pressure_pa * 0.000145038f; // Convert Pa to PSI
            float pressure_atm = sensor_data.pressure_pa / 101325.0f; // Convert Pa to ATM
            float pressure_hpa = sensor_data.pressure_pa / 100.0f; // Convert Pa to hPa
            
            // Calculate sea level pressure for weather monitoring (Ithaca, NY is ~400 ft elevation)
            float elevation_m = 122.0f; // Ithaca elevation in meters
            float sea_level_hpa = pressure_hpa * powf(1.0f + (0.0065f * elevation_m / 288.15f), 5.255f);
            
            printf("=== Summary ===\r\n");
            printf("Temperature: %.2f°C (%.1f°F)\r\n", sensor_data.temperature_c, sensor_data.temperature_c * 9.0f/5.0f + 32.0f);
            printf("Pressure: %.1f hPa (%.3f PSI, %.4f ATM)\r\n", pressure_hpa, pressure_psi, pressure_atm);
            printf("Sea Level Pressure: %.1f hPa (for weather comparison)\r\n", sea_level_hpa);
            printf("Humidity: %.1f%%RH\r\n", sensor_data.humidity_percent);
            printf("Gas Resistance: %.0f Ohms\r\n", sensor_data.gas_resistance_ohm);
            printf("Gas Status: Valid=%s, Heat Stable=%s\r\n", 
                   sensor_data.gas_valid ? "Yes" : "No", 
                   sensor_data.heat_stable ? "Yes" : "No");
            printf("Location: Ithaca, NY (~400 ft elevation)\r\n");
            printf("===============\r\n\n");
            
            // Brief success indication
            status_led_set(STATUS_LED_COLOR_GREEN, true);
            MXC_Delay(200000); // 200ms
            status_led_set(STATUS_LED_COLOR_GREEN, false);
            
        } else {
            printf("ERROR: Failed to read valid sensor data\r\n");
            // Error indication
            status_led_set(STATUS_LED_COLOR_RED, true);
            MXC_Delay(500000); // 500ms
            status_led_set(STATUS_LED_COLOR_RED, false);
        }

        // Power-efficient sleep between measurements
        printf("BME688 in sleep mode - minimal power consumption\r\n");
        printf("Next reading in 10 seconds...\r\n\n");
        fflush(stdout);
        
        // Turn off all LEDs for maximum power savings during sleep
        status_led_all_off();
        
        // 10 second delay between measurements for power efficiency
        // During this time, BME688 is in sleep mode consuming <1µA
        MXC_Delay(MXC_DELAY_SEC(10));
        
        // Brief activity indication before next reading
        status_led_set(STATUS_LED_COLOR_BLUE, true);
        MXC_Delay(100000); // 100ms
        status_led_set(STATUS_LED_COLOR_BLUE, false);
    }
}

/* Private function definitions --------------------------------------------------------------------------------------*/

void display_sensor_data(bme688_data_t *data, const char *label)
{
    printf("=== %s ===\r\n", label);
    printf("Temperature: %.2f C\r\n", data->temperature_c);
    printf("Pressure: %.3f PSI (%.4f ATM)\r\n", data->pressure_pa * 0.000145038f, data->pressure_pa / 101325.0f);
    printf("Humidity: %.2f %%RH\r\n", data->humidity_percent);
    printf("Gas Resistance: %.0f Ohms (Range: %d)\r\n", data->gas_resistance_ohm, data->gas_range);
    printf("Gas Valid: %s, Heat Stable: %s\r\n", 
           data->gas_valid ? "Yes" : "No", 
           data->heat_stable ? "Yes" : "No");
    
    // Raw data for debugging
    printf("Raw Data - Temp: 0x%06lX, Press: 0x%06lX, Hum: 0x%04lX, Gas: 0x%04lX\r\n",
           data->temperature_raw, data->pressure_raw, data->humidity_raw, data->gas_resistance_raw);
    printf("===============================\r\n");
}

void analyze_air_quality(float gas_resistance)
{
    printf("=== Air Quality Analysis ===\r\n");
    
    // Basic air quality assessment based on gas resistance
    // Note: These thresholds are examples and should be calibrated for specific applications
    if (gas_resistance > 50000) {
        printf("Air Quality: EXCELLENT (Clean air)\r\n");
    } else if (gas_resistance > 20000) {
        printf("Air Quality: GOOD (Minor pollutants)\r\n");
    } else if (gas_resistance > 10000) {
        printf("Air Quality: MODERATE (Noticeable pollutants)\r\n");
    } else if (gas_resistance > 5000) {
        printf("Air Quality: POOR (Significant pollutants)\r\n");
    } else {
        printf("Air Quality: VERY POOR (High pollution)\r\n");
    }
    
    printf("Gas Resistance: %.0f Ohms\r\n", gas_resistance);
    printf("Note: Calibrate thresholds for your specific environment\r\n");
    printf("=============================\r\n");
}

// Sleep functions removed for simplified testing

void error_handler(Status_LED_Color_t color)
{
    status_led_all_off();

    const uint32_t fast_blink = 100000;
    while (true) {
        status_led_toggle(color);
        MXC_Delay(fast_blink);
    }
}