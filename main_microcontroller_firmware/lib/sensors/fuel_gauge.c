/**
 * Description:
 * This file represents a software module for controlling the fuel gauge.
 * A MAX17261 is used in this application.
 *
 * 1)The driver helps us to write and read different parametrs from the fuel gauge.
   2)Fuel gauge parameters -- customer defined for Option 1 EZ Config.
	 See App Note: "MAX1726x Software Implementation Guide"
	 https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
	 https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
 *
 **/

#include <stdio.h>
#include <stdint.h>
#include "mxc_delay.h"
#include "max17261.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include <stdlib.h>
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "mxc_device.h"
#include "bsp_status_led.h"
#include "bsp_pushbutton.h"

/* Private defines ----------------------------------------------------------------------------------------------------*/
#define console //Comment this if you want to disable printf
#define MAX17261_I2C_ADDR 0x36u // Fuel Gauge I2C 7 bit address

//Time LSB values please refere to https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
#define tte_hr 1.6				// Bits 15:10 unit = 1.6 hours
#define tte_min 1.5;			// Bits 9:4 unit = 1.5 minutes
#define tte_sec 5.625;			// Bits 3:0 unit = 5.625 seconds

#define current_reg_resolution 781.25 //This is for >2000 mAh capacity please refere to page 23 of the https://www.analog.com/media/en/technical-documentation/data-sheets/max17261.pdf



/* Private MAX17261 specific globals ----------------------------------------------------------------------------------------------*/
uint8_t max17261_regs[256]; // represents all the max17261 registers.


/* Private enumerations ----------------------------------------------------------------------------------------------*/
/**
 * @brief masks to control different parameters of fuel gauge
 *
 */
enum masks
{
	ModelCfg = 0x8000, // Since charge voltage is less than 4.275 volts
	DesignCap = 0x0578,
	IChgTerm = 0x01C0,
	VEmpty = 0x9661,
	POR_BIT = 0x0002,
	DNR_BIT = 0x0001,
};

/**
 * @brief  Fuel gauge parameter register addresses
 *
 */
typedef enum uint8_t
{
	// ModelGauge M5 EZ configuration registers
	DesignCap_addr = 0x18u,
	VEmpty_addr = 0x3Au,
	ModelCfg_addr = 0xDBu,
	IChgTerm_addr = 0x1Eu,
	Config1_addr = 0x1Du,
	Config2_addr = 0xBBu,

	// ModelGauge m5 Register memory map
	FullCapNom_addr = 0x23u,
	FullCapRep_addr = 0x10u,
	FullSocThr_addr = 0x13u,
	FullCap_addr = 0x35u,

	LearnCfg_addr = 0x28u,
	FilterCfg_addr = 0x29u,
	RelaxCfg_addr = 0x2Au,
	MiscCfg_addr = 0x2Bu,
	QRTable00_addr = 0x12u,
	QRTable10_addr = 0x22u,
	QRTable20_addr = 0x32u,
	QRTable30_addr = 0x42u,
	RComp0_addr = 0x38u,
	TempCo_addr = 0x39u,

	// Fuel gauge parameter other can be found in
	// https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
	FStat_addr = 0x3Du,
	HibCFG_addr = 0xBAu,
	Soft_Reset_addr = 0x60u,
	DevName_addr = 0x21u,
	Status_addr = 0x00u,

	// Fuel Gauge parameters to read/log
	VCell_addr = 0x09u,
	AvgVCell_addr = 0x19u,
	Temp_addr = 0x08u,
	AvgTA_addr = 0x16u,
	Current_addr = 0x0Au,
	AvgCurrent_addr = 0x0Bu,
	TTF_addr = 0x20u,
	RepCap_addr = 0x05u,
	VFRemCap_addr = 0x4Au,
	MixCap_addr = 0x0Fu,
	QResidual_addr = 0x0Cu,
	REPSOC_addr = 0x06u,
	AvSOC_addr = 0x0Eu,
	Timer_addr = 0x3Eu,
	TimerH_addr = 0xBEu,
	QH_addr = 0x4Du,
	AvCap_addr = 0x1Fu,
	MixSOC_addr = 0x0Du,
	TTE_addr = 0x11u,
} max17261_registers_addr_t;

/* Private function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief   max17261_write_reg. Generic function to read MAX17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of MAX17261.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start writing to.
 * @param[in]  *reg_data. Array of uint8_t data to write to the I2C slave.
 * @param[out]  len. Number of uint16_t registers to write.
 *  @retval      'E_NO_ERROR` if successful, else a negative error code.
 ****************************************************************************/

int max17261_write_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief   max17261_read_reg. Generic function to read max17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of energy harvester.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start reading from.
 * @param[in]  *reg_data. Array of uint8_t data to read from the I2C slave.
 * @param[out]  len. Number of uint16_t registers to read.
 * @retval      'E_NO_ERROR` if successful, else a negative error code.
 ****************************************************************************/
int max17261_read_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief   max17261_write_verify_reg. Generic function to read max17261 registers.
 * @param[out]  dev_addr. The 1-byte slave address of energy harvester.
 * @param[out]  reg_addr. The 1-byte address of the register on the I2C slave to start reading from.
 * @param[in]  *reg_data. Array of uint8_t data to read from the I2C slave.
 * @param[out]  num_of_byts. Number of bytes to read.
 * @return      true on success, false on failure
 ****************************************************************************/
bool max17261_write_verify_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t num_of_bytes);

/* max17261 function definitions --------------------------------------------------------------------------------------*/

int max17261_write_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len)
{

	int rslt = E_NO_ERROR;
	// Allocate memory for register address and data
	uint8_t *TXData = (uint8_t *)malloc(sizeof(reg_data) + sizeof(reg_addr));
	memcpy(TXData, &reg_addr, 1);	   // First sends the register address
	memcpy(TXData + 1, reg_data, len); // Then sends the data

	mxc_i2c_req_t reqMaster =
		{
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
	if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR)
	{
		// Communication error
		if (rslt != 1)
		{
#if defined(console)
			printf("Error (%d) writing data: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
			return E_UNDERFLOW;
		}
		// Message not acknowledged
		else
		{
#if defined(console)
			printf("Write was not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
			return E_NO_RESPONSE;
		}
	}

	free(TXData);

	return E_NO_ERROR;
}

int max17261_read_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t len)
{

	int rslt = E_SUCCESS;

	mxc_i2c_req_t reqMaster =
		{
			.i2c = bsp_i2c_1v8_i2c_handle,
			.addr = dev_addr,
			.tx_buf = &reg_addr,
			.tx_len = 1,
			.callback = NULL,
			.rx_buf = reg_data,
			.rx_len = len,
			.restart = 0,

		};

	if ((rslt = MXC_I2C_MasterTransaction(&reqMaster)) != E_NO_ERROR)
	{

		// Communication error
		if (rslt != 1)
		{
#if defined(console)
			printf("Error (%d) reading data: Device = 0x%X; Register = 0x%X\n", rslt, dev_addr, reg_addr);
#endif
			return E_UNDERFLOW;
		}
		// Message not acknowledged
		else
		{
#if defined(console)
			printf("Read was not acknowledged: Device = 0x%X; Register = 0x%X\n", dev_addr, reg_addr);
#endif
			return E_NO_RESPONSE;
		}
	}
	MXC_Delay(200);
	return E_NO_ERROR;
}

bool max17261_write_verify_reg(uint8_t dev_addr, max17261_registers_addr_t reg_addr, uint8_t *reg_data, uint16_t num_of_bytes)
{
	bool is_verified = false;
	int i = 0;
	uint8_t read_data[256];
	while (!is_verified)
	{
		max17261_write_reg(dev_addr, reg_addr, reg_data, num_of_bytes);
		// delay 3ms with timer 1
		MXC_Delay(3000);
		max17261_read_reg(dev_addr, reg_addr, &read_data[0], num_of_bytes);
#if defined(console)
		printf("write_and_verify reg_data = ");
#endif
		for (i = 0; i < num_of_bytes; i++)
		{
#if defined(console)
			printf("%02X", *(reg_data + i));
#endif
			if (read_data[i] != *(reg_data + i))
			{
				is_verified = false;
				return is_verified;
			}
		}
#if defined(console)
		printf("\n");
#endif
		is_verified = true;
	}
	return is_verified;
}

int max17261_soft_reset(void)
{
	// The procedure for a MAX17261 reset is below:
	// Write 0x000F to 0x60.
	int result;
	max17261_regs[0] = 0x0F;
	max17261_regs[1] = 0x00;
	uint8_t message[] = {max17261_regs[0], max17261_regs[1]};
	result = max17261_write_reg(MAX17261_I2C_ADDR, Soft_Reset_addr, &message, 2);
	if (result == E_NO_ERROR)
	{
#if defined(console)
		printf("Soft reset succesful\r\n");
#endif
		return E_SUCCESS;
	}
	else
	{
#if defined(console)
		printf("Soft reset failed\r\n");
#endif
		return E_FAIL;
	}
}

bool max17261_por_detected(void)
{
	uint16_t status_register = 0;
	max17261_read_reg(MAX17261_I2C_ADDR, Status_addr, &max17261_regs[0x00], 2); // read status register
	status_register = (max17261_regs[1] << 8) + max17261_regs[0];				// make a 16-bit integer representing status register
#if defined(console)
	printf("status_register = %04x\n", status_register);
#endif
	if ((status_register & 0x0002) > 0) // POR bit is in the 2nd position (bit 1 position of Status register)
	{
#if defined(console)
		printf("MAX17261 POR detected\n");
#endif
		return true;
	}
#if defined(console)
	printf("MAX17261 POR not detected\n");
#endif
	return false;
}

void max17261_clear_por_bit(void)
{
	max17261_read_reg(MAX17261_I2C_ADDR, Status_addr, &max17261_regs[0x00], 2);	 // read POR bit again
	max17261_regs[0] = max17261_regs[0] & 0xFD;									 // LSB -- Set POR bit to 0. Bit position 1
	max17261_write_reg(MAX17261_I2C_ADDR, Status_addr, &max17261_regs[0x00], 2); // clear POR bit
}

void max17261_wait_dnr(void)
{
	uint16_t FStat_register = 0;
	// max17261_write_reg(MAX17261_I2C_ADDR,FStat_addr,&max17261_regs[0x00], 2);
	max17261_read_reg(MAX17261_I2C_ADDR, FStat_addr, &max17261_regs[0x00], 2);
	FStat_register = (max17261_regs[1] << 8) + max17261_regs[0];

	while ((FStat_register & 0x0001) == 0x0001)
	{

		// 11 ms delay
		MXC_Delay(11000);
		// max17261_write_reg(MAX17261_I2C_ADDR,FStat_addr,&max17261_regs[0x00], 2);
		max17261_read_reg(MAX17261_I2C_ADDR, FStat_addr, &max17261_regs[0x00], 2);
		FStat_register = (max17261_regs[1] << 8) + max17261_regs[0];
	}
}

void max17261_config_ez(void)
{
	uint16_t tempdata;
	/// Store original HibCFG value, read in HibCfg; prepare to load model
	uint8_t hibcfg_reg[2];
	max17261_read_reg(MAX17261_I2C_ADDR, HibCFG_addr, &hibcfg_reg[0x00], 2); // read hibcfg register
																			 /// Exit Hibernate Mode step
	max17261_regs[0] = 0x90;
	max17261_regs[1] = 0x00;
	max17261_write_reg(MAX17261_I2C_ADDR, Soft_Reset_addr, &max17261_regs[0x00], 2); // Soft wakeup (Step 1)
	max17261_regs[0] = 0x00;

	max17261_write_reg(MAX17261_I2C_ADDR, HibCFG_addr, &max17261_regs[0x00], 2);	 // Exit hibernate mode Step 2
	max17261_write_reg(MAX17261_I2C_ADDR, Soft_Reset_addr, &max17261_regs[0x00], 2); // Exit hibernate mode Step 3

	/// OPTION 1 EZ Config (No INI file is needed)
	// load DesignCap
	tempdata = DesignCap;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, DesignCap_addr, &max17261_regs[0x00], 2);

	// load IChgTerm
	tempdata = IChgTerm;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, IChgTerm_addr, &max17261_regs[0x00], 2);

	// load VEmpty
	tempdata = VEmpty;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, VEmpty_addr, &max17261_regs[0x00], 2);

	// load ModelCfg
	tempdata = ModelCfg;
	max17261_regs[0] = tempdata & 0x00FF;
	max17261_regs[1] = tempdata >> 8;
	max17261_write_reg(MAX17261_I2C_ADDR, ModelCfg_addr, &max17261_regs[0x00], 2);

	// Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
	max17261_read_reg(MAX17261_I2C_ADDR, ModelCfg_addr, &max17261_regs[0x00], 2); // read status register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];

	while ((tempdata & 0x8000) == 0x8000)
	{
		MXC_Delay(11000);															  // 11 ms delay // delay 11ms
		max17261_read_reg(MAX17261_I2C_ADDR, ModelCfg_addr, &max17261_regs[0x00], 2); // read ModelCfg register
		MXC_Delay(2000);
		tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
		MXC_Delay(2000);
	}

	// Restore Original HibCFG value
	MXC_Delay(2000);
	max17261_write_reg(MAX17261_I2C_ADDR, HibCFG_addr, &hibcfg_reg[0x00], 2);
	MXC_Delay(2000);
}

uint8_t max17261_read_repsoc(void)
{
	max17261_read_reg(MAX17261_I2C_ADDR, REPSOC_addr, &max17261_regs[0x00], 2); // Read RepSOC
	return (max17261_regs[1]);													// The RepSOC "HiByte" can be directly displayed to the user for 1% resolution.
}

uint16_t max17261_read_repcap(void)
{
	uint16_t tempdata = 0;
	max17261_read_reg(MAX17261_I2C_ADDR, 0x05, &max17261_regs[0x00], 2); // Read RepCap
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	return (tempdata);
}

void max17261_read_tte(uint8_t *hrs, uint8_t *mins, uint8_t *secs)
{
	// This function gets the "Time to Empty" (TTE) value. TTE is in memory location 0x11.
	// The LSB of the TTE register is 5.625 seconds.
	uint16_t tte_register = 0;

	double tte_hrs = 0;	 // Bits 15:10 unit = 1.6 hours
	double tte_mins = 0; // Bits 9:4 unit = 1.5 minutes
	double tte_secs = 0; // Bits 3:0 unit = 5.625 seconds

	double secs_from_hrs = 0.0;
	double secs_from_mins = 0.0;
	double secs_from_secs = 0.0;
	double total_seconds = 0.0;

	max17261_read_reg(MAX17261_I2C_ADDR, TTE_addr, &max17261_regs[0x00], 2); // Read Time To Empty
	tte_register = (((uint16_t)max17261_regs[0x01]) << 8) + max17261_regs[0x00];

	
	tte_hrs = (double)(max17261_regs[1] >> 2) * (double)tte_hr;
	secs_from_hrs = tte_hrs * 3600;
	
	tte_mins = (double)(tte_register & 0x03F0) * (double)tte_min;
	secs_from_mins = tte_mins * 60;
	
	tte_secs = (double)(max17261_regs[0] & 0x0F) * (double)tte_sec;
	secs_from_secs = tte_secs;
	total_seconds = round(secs_from_hrs + secs_from_mins + secs_from_secs); // add all seconds up

	*hrs = (uint8_t)(total_seconds / 3600);
	*mins = (uint8_t)((total_seconds - (3600 * (*hrs))) / 60);
	*secs = (uint8_t)(total_seconds - (3600 * (*hrs)) - ((*mins) * 60));
}

void Fuel_gauge_data_collect_after(char *OutputString)
{
	// 		// local variables
	uint16_t tempdata = 0;
	int16_t stempdata = 0;
	char tempstring[20] = {0};

	// Instance name for logging
	sprintf(tempstring, "%s,", "After");
	strcat(OutputString, (const char *)&tempstring);

	// Read Current (in mA)
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, Current_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	stempdata = (int16_t)tempdata; // this is going to be signed number, so convert to 16-bit int
#if defined(console)
	printf("Current register = %f\r\n", (double)stempdata * (double)current_reg_resolution / 1000);
#endif
	sprintf(tempstring, "%f,", (double)stempdata * (double)current_reg_resolution / 1000);
	strcat(OutputString, (const char *)&tempstring);

	// Read AvgCurrent
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, AvgCurrent_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	stempdata = (int16_t)tempdata; // this is going to be signed number, so convert to 16-bit int
#if defined(console)
	printf("AvgCurrent register = %f\r\n", (double)stempdata * (double)current_reg_resolution / 1000);
#endif
	sprintf(tempstring, "%f,", (double)stempdata * (double)current_reg_resolution / 1000);
	strcat(OutputString, (const char *)&tempstring);

	// Read QH
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, QH_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
#if defined(console)
	printf("QH register(coulomb count) = %f\r\n", (double)tempdata * (double)0.25); // We have 20mOhm sense resistor so the LSB is 0.25
#endif
	sprintf(tempstring, "%f,", (double)tempdata * (double)0.25);
	strcat(OutputString, (const char *)&tempstring);
	strcat(OutputString, "\n");
}

/**
 * @brief    Fuel_gauge_data_collect. This function reads the fuel gauge registers and outputs to OutputString Variable
 * @param[in] None
 * @param[out]  char * OutputString.  This is the output string of fuel gauge readings.
 *
 ****************************************************************************/
void Fuel_gauge_data_collect_before(char *OutputString)
{
	// 		// local variables
	uint16_t tempdata = 0;
	int16_t stempdata = 0;
	char tempstring[20] = {0};

	// Instance name for logging
	sprintf(tempstring, "%s,", "Before");
	strcat(OutputString, (const char *)&tempstring);

	// Read Current (in mA)
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, Current_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	stempdata = (int16_t)tempdata; // this is going to be signed number, so convert to 16-bit int
#if defined(console)
	printf("Current register = %f\r\n", (double)stempdata * (double)current_reg_resolution / 1000);
#endif
	sprintf(tempstring, "%f,", (double)stempdata * (double)current_reg_resolution / 1000);
	strcat(OutputString, (const char *)&tempstring);

	// Read AvgCurrent
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, AvgCurrent_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
	stempdata = (int16_t)tempdata; // this is going to be signed number, so convert to 16-bit int
#if defined(console)
	printf("AvgCurrent register = %f\r\n", (double)stempdata * (double)current_reg_resolution / 1000);
#endif
	sprintf(tempstring, "%f,", (double)stempdata * (double)current_reg_resolution / 1000);
	strcat(OutputString, (const char *)&tempstring);

	// Read QH
	tempdata = 0;
	memset(tempstring, 0, 20);
	max17261_read_reg(MAX17261_I2C_ADDR, QH_addr, &max17261_regs[0x00], 2); // Read register
	tempdata = (max17261_regs[1] << 8) + max17261_regs[0];
#if defined(console)
	printf("QH register(coulomb count) = %f\r\n", (double)tempdata * (double)0.25); // We have 20mOhm sense resistor so the LSB is 0.25
#endif
	sprintf(tempstring, "%f,", (double)tempdata * (double)0.25);
	strcat(OutputString, (const char *)&tempstring);
	strcat(OutputString, "\n");
}