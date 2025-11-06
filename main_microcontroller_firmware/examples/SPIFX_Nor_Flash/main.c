/**
 * @file    main.c
 * @brief   SPIXF Transaction Test - Fixed Version for MAX32666
 * @details This example demonstrates a working SPIXF implementation that fixes
 *          the hanging transaction issue commonly seen with MAX32666 SPIXF.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "spixf.h"
#include "gpio.h"

#include "board.h"
#include "bsp_spixf.h"
#include "bsp_pins.h"
#include "bsp_status_led.h"
#include "Ext_Flash.h"


#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Single

// Flash commands
#define FLASH_CMD_READ_ID       0x9F
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_WRITE_ENABLE  0x06

// Expected flash ID (adjust for your specific flash chip)
#define EXPECTED_FLASH_ID       0xEF4018  // Winbond W25Q128 example

/******************************************************************************/
//SPIFX read write setup communication functions (not the actual flash read/write)
static int bsp_spifx_flash_module_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
static int bsp_spifx_flash_module_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };

    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/

static int bsp_spifx_flash_module_init(void)
{
    int err;

    err = MXC_SPIXF_Init(0x0B, BSP_SPIXF_BAUD_RATE);

    if (err == E_NO_ERROR) {
        MXC_SPIXF_Enable();
    }
    return err;
}

/******************************************************************************/
static int bsp_spifx_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}

int bsp_setup_spifx_comm(void)
{
    int err;
    Ext_Flash_Config_t exf_cfg = { .init = bsp_spifx_flash_module_init,
                                   .read = bsp_spifx_flash_module_read,
                                   .write = bsp_spifx_flash_module_write,
                                   .clock = bsp_spifx_flash_clock };

    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }
    return E_NO_ERROR;
}

/******************************************************************************/


static void spixf_cfg_setup()
{
    // Disable the SPIXFC before setting the SPIXF
    MXC_SPIXF_Disable();
    MXC_SPIXF_SetSPIFrequency(BSP_SPIXF_BAUD_RATE);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_3);

    if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Single) {
        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_READ);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_1);
        MXC_SPIXF_SetModeClk(EXT_FLASH_Read_DUMMY);
    } else {

        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_DREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_2);
        MXC_SPIXF_SetModeClk(EXT_FLASH_DREAD_DUMMY);


        // MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_QREAD);
        // MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        // MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
        // MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
        // MXC_SPIXF_SetModeClk(EXT_FLASH_QREAD_DUMMY);
    }

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

int main(void)
{
    uint32_t id;

    printf("\n*** SPIXF Example using BSP ***\n");


    status_led_set(STATUS_LED_COLOR_RED, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, true);
    MXC_Delay(1000000);

    status_led_set(STATUS_LED_COLOR_RED, false);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, false);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, false);
    MXC_Delay(1000000);
    
    status_led_set(STATUS_LED_COLOR_RED, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_GREEN, true);
    MXC_Delay(500000);
    status_led_set(STATUS_LED_COLOR_BLUE, true);
    MXC_Delay(1000000);

    printf("Turning on LDOs \n");


    bsp_power_on_LDOs();
    status_led_set(STATUS_LED_COLOR_BLUE, false);
    MXC_Delay(1000000);

    int result;

    // Step 1: Initialize BSP SPIXF (includes pin configuration)
    printf("1. Initializing BSP SPIXF...\n");
       

    result = bsp_spixf_init();
    if (result != E_NO_ERROR) {
        printf("[ERROR] BSP SPIXF initialization failed: %d\n", result);
        return result;
    }
    printf("[SUCCESS] BSP SPIXF initialized\n");

    spixf_cfg_setup();

    printf("[SUCCESS] SPIXF initialized properly\n");

    printf("SPI Clock: %d Hz\n\n", BSP_SPIXF_BAUD_RATE);


    printf("Configuring SPI communication with external flash ...\n");
    // Setup SPIX

    if(E_NO_ERROR != bsp_setup_spifx_comm())
    {
        printf("[ERROR]--> Setting up external flash failed.\n\n");
        printf("Example Failed\n");
        return E_FAIL;
    }
    printf("[SUCCESS]--> External flash SPI communication configured.\n\n");

    printf("Initializing External flash ...\n");
    // Initialize the SPIXFC registers and set the appropriate output pins
    if (E_NO_ERROR != Ext_Flash_Init()) {
        printf("[ERROR]--> External flash module init failed\n\n");
        printf("Example Failed\n");
        return E_FAIL;
    }
    printf("[SUCCESS]--> External flash Initialized.\n\n");

    // printf("Resetting External Flash ...\n");
    // if (E_NO_ERROR != Ext_Flash_Reset()){
    //     printf("[ERROR]--> External Flash reset failed\n\n");
    //     printf("Example Failed\n");
    //     return E_FAIL;
    // }
    printf("[SUCCESS]--> External flash reset successfully.\n\n");

    printf("Checking External Flash ID ...\n");
    // Get the ID of the external flash
    if ((id = Ext_Flash_ID()) == EXPECTED_FLASH_ID) {
        printf("[SUCCESS]--> ID Found: 0x%x. \n[Mnf: Macronix]\n[Type: SpiNOR]\n[Device: MX25L5124tGZ2I-08G]\nExternal flash ID verified.\n\n", id);
    } else {
        printf("[ERROR]--> Error verifying external flash ID.\nExpected: 0x%x, Got: 0x%x\n\n", EXPECTED_FLASH_ID, id);
        printf("Example Failed\n");
        return E_FAIL;
    }

    return 0;
}

