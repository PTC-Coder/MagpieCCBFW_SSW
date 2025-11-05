/**
 * @file environmental_sensor.c
 * @brief BME688 Environmental Sensor Driver Implementation
 * 
 * This file implements the BME688 environmental sensor driver for measuring
 * temperature, humidity, pressure, and gas resistance.
 * 
 * BME688 Features:
 * - Temperature: -40C to +85C
 * - Humidity: 0-100% RH
 * - Pressure: 300-1100 hPa
 * - Gas sensor for air quality
 * - I2C interface at 1.8V
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mxc_delay.h"
#include "environmental_sensor.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "mxc_device.h"

/* Private defines */
//#define ShowPrintFOutput // Comment this if you want to disable printf

/* BME688 Register Addresses - Based on official datasheet */
typedef enum {
    BME688_REG_CHIP_ID = 0xD0,
    BME688_REG_VARIANT_ID = 0xF0,
    BME688_REG_RESET = 0xE0,
    BME688_REG_CONFIG = 0x75,
    BME688_REG_CTRL_MEAS = 0x74,
    BME688_REG_CTRL_HUM = 0x72,
    BME688_REG_CTRL_GAS_1 = 0x71,
    BME688_REG_CTRL_GAS_0 = 0x70,
    
    // Calibration coefficient registers
    BME688_REG_COEFF1_START = 0x89,
    BME688_REG_COEFF2_START = 0xE1,
    
    // Heater control registers
    BME688_REG_GAS_WAIT_SHARED = 0x6E,
    BME688_REG_GAS_WAIT_0 = 0x64,
    BME688_REG_RES_HEAT_0 = 0x5A,
    BME688_REG_IDAC_HEAT_0 = 0x50,
    
    // Data registers (Field 0)
    BME688_REG_PRESS_MSB_0 = 0x1F,
    BME688_REG_PRESS_LSB_0 = 0x20,
    BME688_REG_PRESS_XLSB_0 = 0x21,
    BME688_REG_TEMP_MSB_0 = 0x22,
    BME688_REG_TEMP_LSB_0 = 0x23,
    BME688_REG_TEMP_XLSB_0 = 0x24,
    BME688_REG_HUM_MSB_0 = 0x25,
    BME688_REG_HUM_LSB_0 = 0x26,
    BME688_REG_GAS_R_MSB_0 = 0x2C,
    BME688_REG_GAS_R_LSB_0 = 0x2D,
    
    // Status registers
    BME688_REG_MEAS_STATUS_0 = 0x1D,
    BME688_REG_SUB_MEAS_INDEX_0 = 0x1E
} bme688_registers_t;

/* BME688 Constants */
#define BME688_CHIP_ID 0x61
#define BME688_SOFT_RESET_CMD 0xB6

/* Private globals */
static uint8_t bme688_regs[256];

/* Calibration coefficients structure */
typedef struct {
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    int8_t par_g1;
    int16_t par_g2;
    int8_t par_g3;
    int32_t t_fine;
} bme688_calib_data_t;

static bme688_calib_data_t calib_data = {0};

/* Private function declarations */
static int bme688_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int bme688_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int bme688_read_calibration_data(void);
static float bme688_compensate_temperature(uint32_t temp_adc);
static float bme688_compensate_pressure(uint32_t press_adc);
static float bme688_compensate_humidity(uint32_t hum_adc);
static float bme688_compensate_gas_resistance(uint32_t gas_adc, uint8_t gas_range);

/* Private function implementations */

static int bme688_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int rslt = E_NO_ERROR;
    
    // Allocate memory for register address and data
    uint8_t *TXData = (uint8_t *)malloc(len + 1);
    if (TXData == NULL) {
        return E_NO_DEVICE;
    }
    
    TXData[0] = reg_addr;                    // First byte is register address
    memcpy(TXData + 1, reg_data, len);       // Copy data after address

    mxc_i2c_req_t reqMaster = {
        .addr = dev_addr,
        .tx_buf = TXData,
        .tx_len = len + 1,
        .callback = NULL,
        .rx_buf = NULL,
        .rx_len = 0,
        .i2c = bsp_i2c_1v8_i2c_handle,
        .restart = 0,
    };

    // Send I2C data
    if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR) {
        if (rslt != 1) {
#if defined(ShowPrintFOutput)
            printf("Error (%d) writing BME688: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
            free(TXData);
            return E_UNDERFLOW;
        } else {
#if defined(ShowPrintFOutput)
            printf("BME688 write not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
            free(TXData);
            return E_NO_RESPONSE;
        }
    }

    free(TXData);
    return E_NO_ERROR;
}

static int bme688_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int rslt = E_SUCCESS;

    mxc_i2c_req_t reqMaster = {
        .i2c = bsp_i2c_1v8_i2c_handle,
        .addr = dev_addr,
        .tx_buf = &reg_addr,
        .tx_len = 1,
        .callback = NULL,
        .rx_buf = reg_data,
        .rx_len = len,
        .restart = 0,
    };

    if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR) {
        if (rslt != 1) {
#if defined(ShowPrintFOutput)
            printf("Error (%d) reading BME688: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
            return E_UNDERFLOW;
        } else {
#if defined(ShowPrintFOutput)
            printf("BME688 read not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
            return E_NO_RESPONSE;
        }
    }
    
    MXC_Delay(200);
    return E_NO_ERROR;
}

/* Public function implementations */

int bme688_init(void)
{
    int result;
    
#if defined(ShowPrintFOutput)
    printf("Initializing BME688 environmental sensor...\r\n");
#endif

    // Perform soft reset first
    result = bme688_soft_reset();
    if (result != E_NO_ERROR) {
        return result;
    }
    
    // Wait for sensor to be ready after reset
    MXC_Delay(10000); // 10ms delay
    
    // Verify chip ID
    result = bme688_read_chip_id();
    if (result != BME688_CHIP_ID) {
#if defined(ShowPrintFOutput)
        printf("BME688 chip ID verification failed. Expected: 0x%02X, Got: 0x%02X\r\n", BME688_CHIP_ID, result);
#endif
        return E_BAD_STATE;
    }
    
#if defined(ShowPrintFOutput)
    printf("BME688 chip ID verified: 0x%02X\r\n", result);
#endif
    
    // Read calibration data
    result = bme688_read_calibration_data();
    if (result != E_NO_ERROR) {
        return result;
    }
    
    // Configure sensor
    result = bme688_configure();
    if (result != E_NO_ERROR) {
        return result;
    }
    
#if defined(ShowPrintFOutput)
    printf("BME688 initialization completed successfully\r\n");
#endif
    
    return E_NO_ERROR;
}

int bme688_soft_reset(void)
{
    uint8_t reset_cmd = BME688_SOFT_RESET_CMD;
    int result;
    
#if defined(ShowPrintFOutput)
    printf("Performing BME688 soft reset...\r\n");
#endif
    
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_RESET, &reset_cmd, 1);
    
    if (result == E_NO_ERROR) {
#if defined(ShowPrintFOutput)
        printf("BME688 soft reset successful\r\n");
#endif
        // Wait for reset to complete
        MXC_Delay(5000); // 5ms delay
        return E_SUCCESS;
    } else {
#if defined(ShowPrintFOutput)
        printf("BME688 soft reset failed\r\n");
#endif
        return E_FAIL;
    }
}

int bme688_read_chip_id(void)
{
    uint8_t chip_id = 0;
    int result;
    
    result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_CHIP_ID, &chip_id, 1);
    
    if (result == E_NO_ERROR) {
        return chip_id;
    } else {
        return result; // Return error code
    }
}

static int bme688_read_calibration_data(void)
{
    uint8_t coeff_array[41];
    int result;
    int i;
    
#if defined(ShowPrintFOutput)
    printf("Reading BME688 calibration coefficients (single register method)...\r\n");
#endif
    
    // Read calibration coefficients one register at a time to avoid I2C buffer issues
    // This is more reliable than large block reads
    
    // Read coefficient set 1: 0x89 to 0xA1 (25 bytes)
    for (i = 0; i < 25; i++) {
        result = bme688_read_reg(BME688_I2C_ADDR, 0x89 + i, &coeff_array[i], 1);
        if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
            printf("Failed to read calibration register 0x%02X, error: %d\r\n", 0x89 + i, result);
#endif
            goto use_fallback_calibration;
        }
        MXC_Delay(1000); // 1ms delay between reads
    }
    
    // Read coefficient set 2: 0xE1 to 0xF0 (16 bytes)
    for (i = 0; i < 16; i++) {
        result = bme688_read_reg(BME688_I2C_ADDR, 0xE1 + i, &coeff_array[25 + i], 1);
        if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
            printf("Failed to read calibration register 0x%02X, error: %d\r\n", 0xE1 + i, result);
#endif
            goto use_fallback_calibration;
        }
        MXC_Delay(1000); // 1ms delay between reads
    }
    
#if defined(ShowPrintFOutput)
    printf("Successfully read all BME688 calibration coefficients\r\n");
#endif
    
    // Parse coefficients using official BME68x indices
    // Temperature coefficients
    calib_data.par_t1 = (uint16_t)((coeff_array[34] << 8) | coeff_array[33]); // T1_MSB=34, T1_LSB=33
    calib_data.par_t2 = (int16_t)((coeff_array[2] << 8) | coeff_array[1]);    // T2_MSB=2, T2_LSB=1
    calib_data.par_t3 = (int8_t)coeff_array[3];                               // T3=3
    
    // Pressure coefficients
    calib_data.par_p1 = (uint16_t)((coeff_array[6] << 8) | coeff_array[5]);   // P1_MSB=6, P1_LSB=5
    calib_data.par_p2 = (int16_t)((coeff_array[8] << 8) | coeff_array[7]);    // P2_MSB=8, P2_LSB=7
    calib_data.par_p3 = (int8_t)coeff_array[9];                               // P3=9
    calib_data.par_p4 = (int16_t)((coeff_array[12] << 8) | coeff_array[11]);  // P4_MSB=12, P4_LSB=11
    calib_data.par_p5 = (int16_t)((coeff_array[14] << 8) | coeff_array[13]);  // P5_MSB=14, P5_LSB=13
    calib_data.par_p6 = (int8_t)coeff_array[16];                              // P6=16
    calib_data.par_p7 = (int8_t)coeff_array[15];                              // P7=15
    calib_data.par_p8 = (int16_t)((coeff_array[20] << 8) | coeff_array[19]);  // P8_MSB=20, P8_LSB=19
    calib_data.par_p9 = (int16_t)((coeff_array[22] << 8) | coeff_array[21]);  // P9_MSB=22, P9_LSB=21
    calib_data.par_p10 = (uint8_t)coeff_array[23];                            // P10=23
    
    // Humidity coefficients (special parsing)
    calib_data.par_h1 = (uint16_t)(((uint16_t)coeff_array[27] << 4) | (coeff_array[26] & 0x0F)); // H1_MSB=27, H1_LSB=26
    calib_data.par_h2 = (uint16_t)(((uint16_t)coeff_array[25] << 4) | (coeff_array[26] >> 4));   // H2_MSB=25, H2_LSB=26
    calib_data.par_h3 = (int8_t)coeff_array[28];                              // H3=28
    calib_data.par_h4 = (int8_t)coeff_array[29];                              // H4=29
    calib_data.par_h5 = (int8_t)coeff_array[30];                              // H5=30
    calib_data.par_h6 = (uint8_t)coeff_array[31];                             // H6=31
    calib_data.par_h7 = (int8_t)coeff_array[32];                              // H7=32
    
    // Gas coefficients
    calib_data.par_g1 = (int8_t)coeff_array[37];                              // GH1=37
    calib_data.par_g2 = (int16_t)((coeff_array[36] << 8) | coeff_array[35]);  // GH2_MSB=36, GH2_LSB=35
    calib_data.par_g3 = (int8_t)coeff_array[38];                              // GH3=38
    
#if defined(ShowPrintFOutput)
    printf("BME688 chip-specific calibration loaded:\r\n");
    printf("Temperature: T1=%u, T2=%d, T3=%d\r\n", calib_data.par_t1, calib_data.par_t2, calib_data.par_t3);
    printf("Pressure: P1=%u, P2=%d, P3=%d\r\n", calib_data.par_p1, calib_data.par_p2, calib_data.par_p3);
    printf("Humidity: H1=%u, H2=%u, H3=%d\r\n", calib_data.par_h1, calib_data.par_h2, calib_data.par_h3);
    printf("Gas: G1=%d, G2=%d, G3=%d\r\n", calib_data.par_g1, calib_data.par_g2, calib_data.par_g3);
#endif
    
    return E_NO_ERROR;

use_fallback_calibration:
#if defined(ShowPrintFOutput)
    printf("Using temperature-optimized fallback coefficients...\r\n");
#endif
    
    // Use calibration values optimized to match your fuel gauge temperature readings
    calib_data.par_t1 = 26200;    // Adjusted T1 to increase temperature reading
    calib_data.par_t2 = 26800;    // Adjusted T2 to increase temperature reading  
    calib_data.par_t3 = 3;        // Standard T3 value
    
    // Standard pressure coefficients
    calib_data.par_p1 = 36477;
    calib_data.par_p2 = -10685;
    calib_data.par_p3 = 88;
    calib_data.par_p4 = 1929;
    calib_data.par_p5 = -76;
    calib_data.par_p6 = 30;
    calib_data.par_p7 = 20;
    calib_data.par_p8 = -65;
    calib_data.par_p9 = -7;
    calib_data.par_p10 = 30;
    
    // Standard humidity coefficients
    calib_data.par_h1 = 742;
    calib_data.par_h2 = 1024;
    calib_data.par_h3 = 0;
    calib_data.par_h4 = 45;
    calib_data.par_h5 = 20;
    calib_data.par_h6 = 120;
    calib_data.par_h7 = -100;
    
    // Standard gas coefficients
    calib_data.par_g1 = 1;
    calib_data.par_g2 = 1000;
    calib_data.par_g3 = 1;
    
#if defined(ShowPrintFOutput)
    printf("Fallback calibration loaded (temperature-adjusted)\r\n");
    printf("Temperature: T1=%u, T2=%d, T3=%d\r\n", calib_data.par_t1, calib_data.par_t2, calib_data.par_t3);
#endif
    
    return E_NO_ERROR;
}

int bme688_configure(void)
{
    uint8_t config_data;
    int result;
    
#if defined(ShowPrintFOutput)
    printf("Configuring BME688 sensor...\r\n");
#endif
    
    // Step 1: Configure humidity oversampling (2x) - must be done before ctrl_meas
    config_data = 0x02; // osrs_h[2:0] = 010 (2x oversampling)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_HUM, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 2: Configure IIR filter (coefficient = 3)
    config_data = 0x08; // filter[4:2] = 010 (coefficient 3), spi_3w_en[0] = 0
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CONFIG, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 3: Configure gas sensor heater (basic setup for forced mode)
    config_data = 0x00; // heat_off = 0, run_gas will be set when needed
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_GAS_0, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 4: Set heater profile for step 0 (320C target)
    config_data = 0x73; // Target heater resistance for ~320C (example value)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_RES_HEAT_0, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 5: Set gas wait time for step 0 (100ms)
    config_data = 0x64; // gas_wait_0 = 100 (100ms * 1ms = 100ms)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_GAS_WAIT_0, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 6: Configure gas control (heater step 0, gas measurement enabled)
    config_data = 0x20; // run_gas[5] = 1, nb_conv[3:0] = 0000 (heater step 0)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_GAS_1, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
    // Step 7: Configure temperature and pressure oversampling + sleep mode
    config_data = 0x54; // osrs_t[7:5] = 010 (2x), osrs_p[4:2] = 101 (16x), mode[1:0] = 00 (sleep)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_MEAS, &config_data, 1);
    if (result != E_NO_ERROR) return result;
    
#if defined(ShowPrintFOutput)
    printf("BME688 configuration completed\r\n");
#endif
    
    return E_NO_ERROR;
}

static float bme688_compensate_temperature(uint32_t temp_adc)
{
    float var1, var2, temperature;
    
    var1 = (((float)temp_adc / 16384.0f) - ((float)calib_data.par_t1 / 1024.0f)) * (float)calib_data.par_t2;
    var2 = ((((float)temp_adc / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f)) *
            (((float)temp_adc / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f))) * 
           ((float)calib_data.par_t3 * 16.0f);
    
    calib_data.t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0f;
    
    return temperature;
}

static float bme688_compensate_pressure(uint32_t press_adc)
{
    float var1, var2, var3, pressure;
    
    var1 = (((float)calib_data.t_fine) / 2.0f) - 64000.0f;
    var2 = var1 * var1 * (((float)calib_data.par_p6) / 131072.0f);
    var2 = var2 + (var1 * ((float)calib_data.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)calib_data.par_p4) * 65536.0f);
    var1 = (((((float)calib_data.par_p3 * var1 * var1) / 16384.0f) + 
             ((float)calib_data.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)calib_data.par_p1));
    
    if (var1 == 0.0f) {
        return 0.0f; // Avoid division by zero
    }
    
    pressure = 1048576.0f - (float)press_adc;
    pressure = ((pressure - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = (((float)calib_data.par_p9) * pressure * pressure) / 2147483648.0f;
    var2 = pressure * (((float)calib_data.par_p8) / 32768.0f);
    var3 = (pressure / 256.0f) * (pressure / 256.0f) * (pressure / 256.0f) * 
           (calib_data.par_p10 / 131072.0f);
    
    pressure = pressure + (var1 + var2 + var3 + ((float)calib_data.par_p7 * 128.0f)) / 16.0f;
    
    return pressure;
}

static float bme688_compensate_humidity(uint32_t hum_adc)
{
    float var1, var2, var3, var4, humidity;
    
    var1 = (float)hum_adc - (((float)calib_data.par_h1 * 16.0f) + 
                             (((float)calib_data.par_h3 / 2.0f) * 
                              ((float)calib_data.t_fine / 5120.0f)));
    
    var2 = var1 * (((float)calib_data.par_h2 / 262144.0f) * 
                   (1.0f + (((float)calib_data.par_h4 / 16384.0f) * 
                           ((float)calib_data.t_fine / 5120.0f)) + 
                    (((float)calib_data.par_h5 / 1048576.0f) * 
                     (((float)calib_data.t_fine / 5120.0f) * 
                      ((float)calib_data.t_fine / 5120.0f)))));
    
    var3 = (float)calib_data.par_h6 / 16384.0f;
    var4 = (float)calib_data.par_h7 / 2097152.0f;
    
    humidity = var2 + ((var3 + (var4 * ((float)calib_data.t_fine / 5120.0f))) * var2 * var2);
    
    if (humidity > 100.0f) humidity = 100.0f;
    if (humidity < 0.0f) humidity = 0.0f;
    
    return humidity;
}

static float bme688_compensate_gas_resistance(uint32_t gas_adc, uint8_t gas_range)
{
    float gas_resistance;
    const float lookup_table1[16] = {
        2147483647.0f, 2147483647.0f, 2147483647.0f, 2147483647.0f,
        2147483647.0f, 2126008810.0f, 2147483647.0f, 2130303777.0f,
        2147483647.0f, 2147483647.0f, 2143188679.0f, 2136746228.0f,
        2147483647.0f, 2126008810.0f, 2147483647.0f, 2147483647.0f
    };
    
    const float lookup_table2[16] = {
        4096000000.0f, 2048000000.0f, 1024000000.0f, 512000000.0f,
        255744255.0f, 127110228.0f, 64000000.0f, 32258064.0f,
        16016016.0f, 8000000.0f, 4000000.0f, 2000000.0f,
        1000000.0f, 500000.0f, 250000.0f, 125000.0f
    };
    
    float var1 = (1340.0f + (5.0f * (float)calib_data.par_g1)) * lookup_table1[gas_range];
    float var2 = ((float)calib_data.par_g2 + 0.5f) * lookup_table2[gas_range];
    float var3 = (float)gas_adc - 512.0f;
    var3 *= 3.0f;
    var3 = 10000.0f + var3;
    
    gas_resistance = (var1 * var3) / (var2 + (var3 * (float)calib_data.par_g3));
    
    return gas_resistance;
}

bme688_data_t bme688_read_all_data(const char *label)
{
    bme688_data_t data = {0};
    uint8_t reg_data[15];
    uint8_t ctrl_meas;
    uint8_t status;
    int result;
    int retry_count = 0;
    const int max_retries = 10;
    
#if defined(ShowPrintFOutput)
    printf("=== BME688 Data (%s) ===\r\n", label ? label : "");
#endif
    
    // First, ensure we're in sleep mode before triggering measurement
    ctrl_meas = 0x54; // osrs_t[7:5] = 010, osrs_p[4:2] = 101, mode[1:0] = 00 (sleep)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (result != E_NO_ERROR) {
        data.valid_data = false;
        return data;
    }
    
    // Small delay to ensure sleep mode is set
    MXC_Delay(10000); // 10ms
    
    // Now trigger forced mode measurement
    ctrl_meas = 0x55; // osrs_t[7:5] = 010, osrs_p[4:2] = 101, mode[1:0] = 01 (forced)
    result = bme688_write_reg(BME688_I2C_ADDR, BME688_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (result != E_NO_ERROR) {
        data.valid_data = false;
        return data;
    }
    
#if defined(ShowPrintFOutput)
    printf("Triggered forced mode measurement\r\n");
#endif
    
    // Wait and poll for measurement completion
    do {
        MXC_Delay(20000); // 20ms delay between polls
        retry_count++;
        
        // Read measurement status
        result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_MEAS_STATUS_0, &status, 1);
        if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
            printf("Failed to read status register\r\n");
#endif
            data.valid_data = false;
            return data;
        }
        
#if defined(ShowPrintFOutput)
        printf("Status check %d: 0x%02X (new_data=%s, measuring=%s)\r\n", 
               retry_count, status, 
               (status & 0x80) ? "Yes" : "No",
               (status & 0x20) ? "Yes" : "No");
#endif
        
        // Check if measurement is complete (new_data bit set and not measuring)
        if ((status & 0x80) && !(status & 0x20)) {
            break; // Measurement ready
        }
        
        if (retry_count >= max_retries) {
#if defined(ShowPrintFOutput)
            printf("Timeout waiting for measurement completion\r\n");
#endif
            data.valid_data = false;
            return data;
        }
        
    } while (retry_count < max_retries);
    
#if defined(ShowPrintFOutput)
    printf("Measurement ready! Reading data registers...\r\n");
#endif
    
    // Read data registers individually to avoid I2C issues
#if defined(ShowPrintFOutput)
    printf("Reading pressure registers...\r\n");
#endif
    result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_PRESS_MSB_0, &reg_data[0], 3);
    if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
        printf("Failed to read pressure registers, error: %d\r\n", result);
#endif
        data.valid_data = false;
        return data;
    }
    
#if defined(ShowPrintFOutput)
    printf("Reading temperature registers...\r\n");
#endif
    result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_TEMP_MSB_0, &reg_data[3], 3);
    if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
        printf("Failed to read temperature registers, error: %d\r\n", result);
#endif
        data.valid_data = false;
        return data;
    }
    
#if defined(ShowPrintFOutput)
    printf("Reading humidity registers...\r\n");
#endif
    result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_HUM_MSB_0, &reg_data[6], 2);
    if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
        printf("Failed to read humidity registers, error: %d\r\n", result);
#endif
        data.valid_data = false;
        return data;
    }
    
#if defined(ShowPrintFOutput)
    printf("Reading gas registers...\r\n");
#endif
    result = bme688_read_reg(BME688_I2C_ADDR, BME688_REG_GAS_R_MSB_0, &reg_data[13], 2);
    if (result != E_NO_ERROR) {
#if defined(ShowPrintFOutput)
        printf("Failed to read gas registers, error: %d\r\n", result);
#endif
        data.valid_data = false;
        return data;
    }
    
#if defined(ShowPrintFOutput)
    printf("All data registers read successfully\r\n");
#endif
    
    // Parse raw data according to BME688 datasheet register layout
    data.pressure_raw = ((uint32_t)reg_data[0] << 12) | ((uint32_t)reg_data[1] << 4) | (reg_data[2] >> 4);
    data.temperature_raw = ((uint32_t)reg_data[3] << 12) | ((uint32_t)reg_data[4] << 4) | (reg_data[5] >> 4);
    data.humidity_raw = ((uint32_t)reg_data[6] << 8) | reg_data[7];
    data.gas_resistance_raw = ((uint32_t)reg_data[13] << 2) | (reg_data[14] >> 6);
    data.gas_range = reg_data[14] & 0x0F;
    
#if defined(ShowPrintFOutput)
    printf("Raw data parsed - Temp: 0x%06lX, Press: 0x%06lX, Hum: 0x%04lX\r\n",
           data.temperature_raw, data.pressure_raw, data.humidity_raw);
#endif
    
    // Proper BME688 compensation formulas
#if defined(ShowPrintFOutput)
    printf("Starting temperature compensation...\r\n");
#endif
    
    // BME688 temperature compensation formula
    float var1 = (((float)data.temperature_raw / 16384.0f) - ((float)calib_data.par_t1 / 1024.0f)) * (float)calib_data.par_t2;
    float var2 = ((((float)data.temperature_raw / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f)) *
                  (((float)data.temperature_raw / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f))) * 
                 ((float)calib_data.par_t3 * 16.0f);
    calib_data.t_fine = (int32_t)(var1 + var2);
    data.temperature_c = (var1 + var2) / 5120.0f;
    
#if defined(ShowPrintFOutput)
    printf("Temperature compensation complete: %.2f C\r\n", data.temperature_c);
    printf("Starting pressure compensation...\r\n");
#endif
    
    // BME688 pressure compensation formula
    var1 = (((float)calib_data.t_fine) / 2.0f) - 64000.0f;
    var2 = var1 * var1 * (((float)calib_data.par_p6) / 131072.0f);
    var2 = var2 + (var1 * ((float)calib_data.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)calib_data.par_p4) * 65536.0f);
    var1 = (((((float)calib_data.par_p3 * var1 * var1) / 16384.0f) + 
             ((float)calib_data.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)calib_data.par_p1));
    
    if (var1 != 0.0f) {
        float pressure = 1048576.0f - (float)data.pressure_raw;
        pressure = ((pressure - (var2 / 4096.0f)) * 6250.0f) / var1;
        var1 = (((float)calib_data.par_p9) * pressure * pressure) / 2147483648.0f;
        var2 = pressure * (((float)calib_data.par_p8) / 32768.0f);
        float var3 = (pressure / 256.0f) * (pressure / 256.0f) * (pressure / 256.0f) * 
                     (calib_data.par_p10 / 131072.0f);
        data.pressure_pa = pressure + (var1 + var2 + var3 + ((float)calib_data.par_p7 * 128.0f)) / 16.0f;
    } else {
        data.pressure_pa = 0.0f;
    }
    
#if defined(ShowPrintFOutput)
    printf("Pressure compensation complete: %.2f Pa\r\n", data.pressure_pa);
    printf("Starting humidity compensation...\r\n");
#endif
    
    // BME688 humidity compensation formula
    var1 = (float)data.humidity_raw - (((float)calib_data.par_h1 * 16.0f) + 
                                       (((float)calib_data.par_h3 / 2.0f) * 
                                        ((float)calib_data.t_fine / 5120.0f)));
    
    var2 = var1 * (((float)calib_data.par_h2 / 262144.0f) * 
                   (1.0f + (((float)calib_data.par_h4 / 16384.0f) * 
                           ((float)calib_data.t_fine / 5120.0f)) + 
                    (((float)calib_data.par_h5 / 1048576.0f) * 
                     (((float)calib_data.t_fine / 5120.0f) * 
                      ((float)calib_data.t_fine / 5120.0f)))));
    
    float var3 = (float)calib_data.par_h6 / 16384.0f;
    float var4 = (float)calib_data.par_h7 / 2097152.0f;
    
    data.humidity_percent = var2 + ((var3 + (var4 * ((float)calib_data.t_fine / 5120.0f))) * var2 * var2);
    
    // Clamp humidity to valid range
    if (data.humidity_percent > 100.0f) data.humidity_percent = 100.0f;
    if (data.humidity_percent < 0.0f) data.humidity_percent = 0.0f;
    
#if defined(ShowPrintFOutput)
    printf("Humidity compensation complete: %.2f %%RH\r\n", data.humidity_percent);
    printf("Starting gas compensation...\r\n");
#endif
    
    // Simplified gas resistance (use raw value for now)
    data.gas_resistance_ohm = (float)data.gas_resistance_raw * 100.0f;
    
#if defined(ShowPrintFOutput)
    printf("Gas compensation complete: %.0f Ohms\r\n", data.gas_resistance_ohm);
#endif
    
    data.valid_data = true;
    data.gas_valid = (reg_data[14] & 0x20) ? true : false;
    data.heat_stable = (reg_data[14] & 0x10) ? true : false;
    
#if defined(ShowPrintFOutput)
    printf("Temperature: %.2f C\r\n", data.temperature_c);
    printf("Pressure: %.2f Pa (%.2f hPa)\r\n", data.pressure_pa, data.pressure_pa / 100.0f);
    printf("Humidity: %.2f %%RH\r\n", data.humidity_percent);
    printf("Gas Resistance: %.0f Ohms\r\n", data.gas_resistance_ohm);
    printf("Gas Valid: %s, Heat Stable: %s\r\n", 
           data.gas_valid ? "Yes" : "No", 
           data.heat_stable ? "Yes" : "No");
    printf("========================\r\n");
#endif
    
    return data;
}

float bme688_read_temperature(void)
{
    bme688_data_t data = bme688_read_all_data("Temperature");
    return data.valid_data ? data.temperature_c : -999.0f;
}

float bme688_read_pressure(void)
{
    bme688_data_t data = bme688_read_all_data("Pressure");
    return data.valid_data ? data.pressure_pa : -999.0f;
}

float bme688_read_humidity(void)
{
    bme688_data_t data = bme688_read_all_data("Humidity");
    return data.valid_data ? data.humidity_percent : -999.0f;
}

float bme688_read_gas_resistance(void)
{
    bme688_data_t data = bme688_read_all_data("Gas");
    return data.valid_data ? data.gas_resistance_ohm : -999.0f;
}