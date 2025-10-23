/**
 * @file    max17261.h
 * @brief   A software module for controlling the max17261 fuel gauge is represented here.
 * @details This mddule is responsible for initializing, configuring and reading the fueal gauge.
            
 *
 * This module requires:
 * - shared I2C bus on the 1v8 domain using 7 bit address 0x36u
 */

#ifndef MAX17261_H_
#define MAX17261_H_

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>



/**
 * @brief    max17261_soft_reset(). Function to "soft reset" the MAX17261 device completely.
 * @retval   'E_SUCCESS` if successful, else a negative error code.
 ****************************************************************************/
int max17261_soft_reset(void);

/**
 * @brief    max17261_por_detected(). Function to check if a power-on-reset
 *           has occurred with the max17261 device.
 * @return   true on success, false on failure
 ****************************************************************************/
bool max17261_por_detected(void);

/**
 * @brief    max17261_clear_por_bit(). Function to clear a power-on-reset flag that
 *           has occurred on the max17261 device.
 ****************************************************************************/
void max17261_clear_por_bit(void);

/**
 * @brief    max17261_wait_dnr(). Function to wait for the MAX1726x to complete
 *           its startup operations. See "MAX1726x Software Implementation Guide" for details
 ****************************************************************************/
void max17261_wait_dnr(void);



/**
 * @brief   max17261_read_repsoc(). This function gets the state of charge (SOC) as
 *          described in Software Implementation guide for the MAX17261. It gives how much
 *          charge is left on the battery.
 * @return the "HiByte" of RepSOC register for %1 resolution
 ****************************************************************************/
uint8_t max17261_read_repsoc(void);

/**
 * @brief    max17261_read_repcap(). This function gets the RepCap register data
 *           as described in Software Implementation guide. It gives how many milli amp
 *           hours is left on the battery.
 * @return   the 2-byte register data as a uint16_t.
 ****************************************************************************/
uint16_t max17261_read_repcap(void);
/**
 * @brief    max17261_read_tte(). This function the TTE register data
 *           as described in Software Implementation guide. It gives how much milli amp
 *           hours is left on the battery. This function is currently not used or tested.
 * @param[out]  *hrs. Hours
 * @param[out]  *mins. Minutes
 * @param[out]  *secs. Seconds
 ****************************************************************************/
void max17261_read_tte(uint8_t *hrs, uint8_t *mins, uint8_t *secs);



/**
 * @brief    max17261_config_ez(). This function performs EZ configuraiton
 *           as described in Software Implementation guide.
 * @return void
 ****************************************************************************/
void max17261_config_ez(void);

/**
 * @brief    Fuel_gauge_data_collect. This function reads the fuel gauge registers and outputs to OutputString Variable. 
             The idea is to use this right after SD-card write 
 * @param[in] unsigned long timer_count_value. This is a time stamp in seconds.
 * @param[out]  char * OutputString.  This is the output string of fuel gauge readings.
 *
 ****************************************************************************/
void Fuel_gauge_data_collect_after(char * OutputString);

/**
 * @brief    Fuel_gauge_data_collect. This function reads the fuel gauge registers and outputs to OutputString Variable
             The idea is to use this right before SD-card write
 * @param[in] unsigned long timer_count_value. This is a time stamp in seconds.
 * @param[out]  char * OutputString.  This is the output string of fuel gauge readings.
 *
 ****************************************************************************/
void Fuel_gauge_data_collect_before(char * OutputString);


#endif /* MAX17261_H_ */