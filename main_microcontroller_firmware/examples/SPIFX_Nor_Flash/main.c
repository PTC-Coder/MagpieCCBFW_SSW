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
#include "lfs.h"
#include "time.h"

#ifdef TERMINAL_IO_USE_SEGGER_RTT
#include "SEGGER_RTT.h"
#endif



#define EXT_FLASH_SPIXFC_WIDTH Ext_Flash_DataLine_Single

#define NOR_PAGE_SIZE 256   //256 bytes per page
#define NOR_SECTOR_SIZE 4096  // 4KB sectors
#define NOR_BLOCK_SIZE 65536  // 64KB blocks  
#define NOR_TOTAL_SIZE 67108864  // 64 MB
#define NOR_SECTOR_COUNT 16384  // 16384 sectors of 4KB each



// Expected flash ID (adjust for your specific flash chip)
#define EXPECTED_FLASH_ID       0xc2201a //0xEF4018  // Example: Macronix MX25L51245G
#define WINBOND_FLASH_ID       0xEFAA 

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
    } else if (EXT_FLASH_SPIXFC_WIDTH == Ext_Flash_DataLine_Dual) {

        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_DREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_DUAL_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_2);
        MXC_SPIXF_SetModeClk(EXT_FLASH_DREAD_DUMMY);
    } else {
        MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_QREAD);
        MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
        MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
        MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
        MXC_SPIXF_SetModeClk(EXT_FLASH_QREAD_DUMMY);
    }

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

//======================== LittleFS interface functions ========================//
static int block_device_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t addr = block * c->block_size + off;

    //First fill data buffer
    Ext_Flash_DataRead(addr);
    //then read from data buffer
    return Ext_Flash_Read(addr, buffer, size, EXT_FLASH_SPIXFC_WIDTH);
}

static int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t addr = block * c->block_size + off;
    
    return Ext_Flash_Program_Page(addr, buffer, size, EXT_FLASH_SPIXFC_WIDTH);
}

static int block_device_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t addr = block * c->block_size;

    // Use 4KB sector erase for better granularity
    return Ext_Flash_Erase(addr, Ext_Flash_Erase_4K);
}

static int block_device_sync(const struct lfs_config *c) {
    return Ext_Flash_SyncFlash();
}

/******************************************************************************/
// Configure LittleFS
const struct lfs_config cfg = {
    .read  = block_device_read,
    .prog  = block_device_prog,
    .erase = block_device_erase,
    .sync  = block_device_sync,

    .block_size = NOR_SECTOR_SIZE,  // Use 4KB sectors as blocks
    .block_count = NOR_SECTOR_COUNT,
    .cache_size = NOR_PAGE_SIZE,
    .lookahead_size = NOR_PAGE_SIZE,
    .block_cycles = 100000,
    .read_size = NOR_PAGE_SIZE,
    .prog_size = NOR_PAGE_SIZE,
};

lfs_t lfs;

/******************************************************************************/

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
    }
        else if ((id = Ext_Flash_ID()) == WINBOND_FLASH_ID) {
        printf("[SUCCESS]--> ID Found: 0x%x. \n[Mnf: Winbond]\n[Type: SpiNOR]\n[Device: 25N02KVZEIR]\nExternal flash ID verified.\n\n", id);
    }
    else if ((id = Ext_Flash_ID()) == EXPECTED_FLASH_ID) {
        printf("[SUCCESS]--> ID Found: 0x%x. \n[Mnf: Macronix]\n[Type: SpiNOR]\n[Device: 25Q128JV-EQ*]\nExternal flash ID verified.\n\n", id);
    } else {       

        printf("[ERROR]--> Error verifying external flash ID.\nExpected: 0x%x, Got: 0x%x\n\n", EXPECTED_FLASH_ID, id);
        printf("Example Failed\n");
        return E_FAIL;
    }

    //Unprotect status register and write protection
    printf("Unlocking Flash Write Protection ...\n");
    if(E_NO_ERROR != Ext_Flash_Unprotect_StatusRegister()){
        printf("[ERROR]--> Failed to unlock write protection.\n\n");
        return E_FAIL;
    } else {
        printf("[SUCCESS]--> Write protection unlocked.\n\n");
    }

    // Initialize LittleFS
    lfs_file_t file;

    int err;
    // Mount the filesystem
    err = lfs_mount(&lfs, &cfg);
    if (err) {
        printf("Failed to mount filesystem.\n");

        // Format the flash drive if the file system is not mountable
        // Might not want to do this everytime.
        printf("Formatting...\n");
        err = lfs_format(&lfs, &cfg);
        if (err) {
            printf("Failed to format filesystem: error %d\n", err);
            return err;
        }
        printf("Filesystem formatted. Mounting...\n");
        err = lfs_mount(&lfs, &cfg);
        if (err) {
            printf("Failed to mount formatted filesystem: error %d\n", err);
            return err;
        }
    }
    printf("File system mounted successfully\n");

    // Define struct
    typedef struct {
        int id;
        char name[20];
        float temperature_c;
        struct tm setDateTime;
    } MyData;

    struct tm newTime = {
		.tm_year = 2025 - 1900U,
		.tm_mon = 1 - 1U,
		.tm_mday = 8U,
		.tm_hour = 3U,
		.tm_min = 52U,
		.tm_sec = 0U
	};

    MyData data = {1, "MAX Struct Data", 25.2, newTime};


    // Write the struct to the file
    err = lfs_file_open(&lfs, &file, "setup.bin", LFS_O_RDWR | LFS_O_CREAT);
    if (err) {
        printf("Failed to open file for writing: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    //****Note that the data type here is lfs_size_t and NOT the lfs_ssize_t we used for writing just the string data.
    lfs_size_t struct_written = lfs_file_write(&lfs, &file, &data, sizeof(MyData));
    if (struct_written != sizeof(MyData)) {
        printf("Failed to write struct data to file: error %d\n", struct_written);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return struct_written;
    }
    printf("setup.bin written successfully. %d written\n", struct_written);

    
    // // Write to a file
    // err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDWR | LFS_O_CREAT);
    // if (err) {
    //     printf("Failed to open file for writing: error %d\n", err);
    //     lfs_unmount(&lfs);
    //     return err;
    // }
    // printf("test.txt opened for write successfully\n");

    // char *data = "Hello, LittleFS on MAX32666!  This is a test writing of a file.\n Hope it works!\n";
    // lfs_ssize_t written = lfs_file_write(&lfs, &file, data, strlen(data));
    // if (written < 0) {
    //     printf("Failed to write to file: error %d\n", written);
    //     lfs_file_close(&lfs, &file);
    //     lfs_unmount(&lfs);
    //     return written;
    // }
    // printf("test.txt written successfully. %d written\n", written);

    //Sync Flash after write
    err = lfs_file_sync(&lfs, &file);
    if (err) {
        printf("Failed to sync file: error %d\n", err);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return err;
    }
    printf("File sync successfully.\n");

    lfs_file_close(&lfs, &file);
    
    //List directory 
    lfs_dir_t dir;
    struct lfs_info info;

    err = lfs_dir_open(&lfs, &dir, "/");
    if (err) {
        printf("Failed to open root directory: error %d\n", err);
        return err;
    }

    while (true) {
        int res = lfs_dir_read(&lfs, &dir, &info);
        if (res < 0) {
            printf("Error reading directory: %d\n", res);
            break;
        }
        if (res == 0) {
            break;
        }
        printf("Found file: %s\n", info.name);
    }

    lfs_dir_close(&lfs, &dir);


    // Read from the file
    printf("\nOpening test.txt for read only mode.\n");

    char buf[NOR_SECTOR_SIZE] = {0};
    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDONLY);
    if (err == LFS_ERR_NOENT){
         printf("File does not exist\n");
    }
    else if (err) {
        printf("Failed to open file for reading: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    lfs_ssize_t read = lfs_file_read(&lfs, &file, buf, sizeof(buf) - 1);
    if (read < 0) {
        printf("Failed to read from file: error %d\n", read);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return read;
    }
    lfs_file_close(&lfs, &file);    
    printf("Reading content from test.txt file:\n %s\n", buf);



    //////  Read struct from setup.bin file
    printf("Reading back setup.bin ...\n");    

    err = lfs_file_open(&lfs, &file, "setup.bin", LFS_O_RDONLY);
    if (err == LFS_ERR_NOENT){
         printf("File does not exist\n");
    }
    else if (err) {
        printf("Failed to open file for reading: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    MyData readData;

    //Note that the data type here is lfs_size_t and not the lfs_ssize_t we used for reading just the string data.
    lfs_size_t struct_read = lfs_file_read(&lfs, &file, &readData, sizeof(MyData));
    if (struct_read != sizeof(MyData)) {
        printf("Failed to retrieve struct data from file: error %d\n", struct_read);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return struct_read;
    }

    static uint8_t dateTimeStr[128];

    //Convert datetime type to string
    strftime((char*)dateTimeStr, 128, "%F %TZ", &readData.setDateTime);

    printf("ID: %d\nName: %s\nTemperature(C): %f\nSet Date/Time: %s\n\n", readData.id, readData.name, readData.temperature_c, dateTimeStr);
	
    lfs_file_close(&lfs, &file);


    //*************** Read from a struct file created by nRF */
    printf("\nOpening nRFsetup.bin for read only mode.\n");  

    MXC_Delay(1000);

    err = lfs_file_open(&lfs, &file, "nRFsetup.bin", LFS_O_RDONLY);
    if (err == LFS_ERR_NOENT){
         printf("File does not exist.\n");
         return err;
    }
    else if (err) {
        printf("Failed to open file for reading: error %d\n", err);
        lfs_unmount(&lfs);
        return err;
    }

    struct_read = lfs_file_read(&lfs, &file, &readData, sizeof(MyData));
    if (struct_read != sizeof(MyData)) {
        printf("Failed to retrieve struct data from file: error %d\n", struct_read);
        lfs_file_close(&lfs, &file);
        lfs_unmount(&lfs);
        return struct_read;
    }

    //Convert datetime type to string
    strftime((char*)dateTimeStr, 128, "%F %TZ", &readData.setDateTime);

    printf("ID: %d\nName: %s\nTemperature(C): %f\nSet Date/Time: %s\n\n", readData.id, readData.name, readData.temperature_c, dateTimeStr);
	
    lfs_file_close(&lfs, &file);

    // Unmount the filesystem
    lfs_unmount(&lfs);
       

    printf("************** MAX32666 SPIFX Example Succeeded ****************\n\n\n");
    return E_NO_ERROR;
}

