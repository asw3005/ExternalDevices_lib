/*
 * si5338.c source file.
 *
 * Created on: Sep 1, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "si5338.h"

/* To write the configuration of the chip you need to use Skyworks ClockBuilderPro. */

/* External variables. */
extern I2C_HandleTypeDef hi2c1;

/* Private variables. */
static I2C_HandleTypeDef* SI5338I2C = &hi2c1;

/* Register pregenerated values by ClockBuilderPro. */
static SI5338_RegData_t const RegStore[SI5338_NUM_REGS_MAX] = {

		{  0,0x00,0x00},
		{  1,0x00,0x00},
		{  2,0x00,0x00},
		{  3,0x00,0x00},
		{  4,0x00,0x00},
		{  5,0x00,0x00},
		{  6,0x08,0x1D},
		{  7,0x00,0x00},
		{  8,0x70,0x00},
		{  9,0x0F,0x00},
		{ 10,0x00,0x00},
		{ 11,0x00,0x00},
		{ 12,0x00,0x00},
		{ 13,0x00,0x00},
		{ 14,0x00,0x00},
		{ 15,0x00,0x00},
		{ 16,0x00,0x00},
		{ 17,0x00,0x00},
		{ 18,0x00,0x00},
		{ 19,0x00,0x00},
		{ 20,0x00,0x00},
		{ 21,0x00,0x00},
		{ 22,0x00,0x00},
		{ 23,0x00,0x00},
		{ 24,0x00,0x00},
		{ 25,0x00,0x00},
		{ 26,0x00,0x00},
		{ 27,0x70,0x80},
		{ 28,0x16,0xFF},
		{ 29,0x90,0xFF},
		{ 30,0xB0,0xFF},
		{ 31,0xC0,0xFF},
		{ 32,0xE3,0xFF},
		{ 33,0xE3,0xFF},
		{ 34,0xE3,0xFF},
		{ 35,0x00,0xFF},
		{ 36,0x01,0x1F},
		{ 37,0x00,0x1F},
		{ 38,0x00,0x1F},
		{ 39,0x00,0x1F},
		{ 40,0x77,0xFF},
		{ 41,0x0C,0x7F},
		{ 42,0x23,0x3F},
		{ 43,0x00,0x00},
		{ 44,0x00,0x00},
		{ 45,0x00,0xFF},
		{ 46,0x00,0xFF},
		{ 47,0x14,0x3F},
		{ 48,0x39,0xFF},
		{ 49,0x00,0xFF},
		{ 50,0xC4,0xFF},
		{ 51,0x07,0xFF},
		{ 52,0x10,0xFF},
		{ 53,0x00,0xFF},
		{ 54,0x13,0xFF},
		{ 55,0x00,0xFF},
		{ 56,0x00,0xFF},
		{ 57,0x00,0xFF},
		{ 58,0x00,0xFF},
		{ 59,0x01,0xFF},
		{ 60,0x00,0xFF},
		{ 61,0x00,0xFF},
		{ 62,0x00,0x3F},
		{ 63,0x10,0xFF},
		{ 64,0x00,0xFF},
		{ 65,0x00,0xFF},
		{ 66,0x00,0xFF},
		{ 67,0x00,0xFF},
		{ 68,0x00,0xFF},
		{ 69,0x00,0xFF},
		{ 70,0x00,0xFF},
		{ 71,0x00,0xFF},
		{ 72,0x00,0xFF},
		{ 73,0x00,0x3F},
		{ 74,0x10,0xFF},
		{ 75,0x00,0xFF},
		{ 76,0x00,0xFF},
		{ 77,0x00,0xFF},
		{ 78,0x00,0xFF},
		{ 79,0x00,0xFF},
		{ 80,0x00,0xFF},
		{ 81,0x00,0xFF},
		{ 82,0x00,0xFF},
		{ 83,0x00,0xFF},
		{ 84,0x00,0x3F},
		{ 85,0x10,0xFF},
		{ 86,0x00,0xFF},
		{ 87,0x00,0xFF},
		{ 88,0x00,0xFF},
		{ 89,0x00,0xFF},
		{ 90,0x00,0xFF},
		{ 91,0x00,0xFF},
		{ 92,0x00,0xFF},
		{ 93,0x00,0xFF},
		{ 94,0x00,0xFF},
		{ 95,0x00,0x3F},
		{ 96,0x10,0x00},
		{ 97,0x66,0xFF},
		{ 98,0x30,0xFF},
		{ 99,0x08,0xFF},
		{100,0x00,0xFF},
		{101,0x00,0xFF},
		{102,0x00,0xFF},
		{103,0x05,0xFF},
		{104,0x00,0xFF},
		{105,0x00,0xFF},
		{106,0x80,0xBF},
		{107,0x00,0xFF},
		{108,0x00,0xFF},
		{109,0x00,0xFF},
		{110,0xC0,0xFF},
		{111,0x00,0xFF},
		{112,0x00,0xFF},
		{113,0x00,0xFF},
		{114,0x40,0xFF},
		{115,0x00,0xFF},
		{116,0x80,0xFF},
		{117,0x00,0xFF},
		{118,0x40,0xFF},
		{119,0x00,0xFF},
		{120,0x00,0xFF},
		{121,0x00,0xFF},
		{122,0x40,0xFF},
		{123,0x00,0xFF},
		{124,0x00,0xFF},
		{125,0x00,0xFF},
		{126,0x00,0xFF},
		{127,0x00,0xFF},
		{128,0x00,0xFF},
		{129,0x00,0x0F},
		{130,0x00,0x0F},
		{131,0x00,0xFF},
		{132,0x00,0xFF},
		{133,0x00,0xFF},
		{134,0x00,0xFF},
		{135,0x00,0xFF},
		{136,0x00,0xFF},
		{137,0x00,0xFF},
		{138,0x00,0xFF},
		{139,0x00,0xFF},
		{140,0x00,0xFF},
		{141,0x00,0xFF},
		{142,0x00,0xFF},
		{143,0x00,0xFF},
		{144,0x00,0xFF},
		{145,0x00,0x00},
		{146,0xFF,0x00},
		{147,0x00,0x00},
		{148,0x00,0x00},
		{149,0x00,0x00},
		{150,0x00,0x00},
		{151,0x00,0x00},
		{152,0x00,0xFF},
		{153,0x00,0xFF},
		{154,0x00,0xFF},
		{155,0x00,0xFF},
		{156,0x00,0xFF},
		{157,0x00,0xFF},
		{158,0x00,0x0F},
		{159,0x00,0x0F},
		{160,0x00,0xFF},
		{161,0x00,0xFF},
		{162,0x00,0xFF},
		{163,0x00,0xFF},
		{164,0x00,0xFF},
		{165,0x00,0xFF},
		{166,0x00,0xFF},
		{167,0x00,0xFF},
		{168,0x00,0xFF},
		{169,0x00,0xFF},
		{170,0x00,0xFF},
		{171,0x00,0xFF},
		{172,0x00,0xFF},
		{173,0x00,0xFF},
		{174,0x00,0xFF},
		{175,0x00,0xFF},
		{176,0x00,0xFF},
		{177,0x00,0xFF},
		{178,0x00,0xFF},
		{179,0x00,0xFF},
		{180,0x00,0xFF},
		{181,0x00,0x0F},
		{182,0x00,0xFF},
		{183,0x00,0xFF},
		{184,0x00,0xFF},
		{185,0x00,0xFF},
		{186,0x00,0xFF},
		{187,0x00,0xFF},
		{188,0x00,0xFF},
		{189,0x00,0xFF},
		{190,0x00,0xFF},
		{191,0x00,0xFF},
		{192,0x00,0xFF},
		{193,0x00,0xFF},
		{194,0x00,0xFF},
		{195,0x00,0xFF},
		{196,0x00,0xFF},
		{197,0x00,0xFF},
		{198,0x00,0xFF},
		{199,0x00,0xFF},
		{200,0x00,0xFF},
		{201,0x00,0xFF},
		{202,0x00,0xFF},
		{203,0x00,0x0F},
		{204,0x00,0xFF},
		{205,0x00,0xFF},
		{206,0x00,0xFF},
		{207,0x00,0xFF},
		{208,0x00,0xFF},
		{209,0x00,0xFF},
		{210,0x00,0xFF},
		{211,0x00,0xFF},
		{212,0x00,0xFF},
		{213,0x00,0xFF},
		{214,0x00,0xFF},
		{215,0x00,0xFF},
		{216,0x00,0xFF},
		{217,0x00,0xFF},
		{218,0x00,0x00},
		{219,0x00,0x00},
		{220,0x00,0x00},
		{221,0x0D,0x00},
		{222,0x00,0x00},
		{223,0x00,0x00},
		{224,0xF4,0x00},
		{225,0xF0,0x00},
		{226,0x00,0x00},
		{227,0x00,0x00},
		{228,0x00,0x00},
		{229,0x00,0x00},
		{230,0x0E,0x0F},
		{231,0x00,0x00},
		{232,0x00,0x00},
		{233,0x00,0x00},
		{234,0x00,0x00},
		{235,0x00,0x00},
		{236,0x00,0x00},
		{237,0x00,0x00},
		{238,0x14,0x00},
		{239,0x00,0x00},
		{240,0x00,0x00},
		{242,0x02,0x02},
		{243,0xF0,0x00},
		{244,0x00,0x00},
		{245,0x00,0x00},
		{247,0x00,0x00},
		{248,0x00,0x00},
		{249,0xA8,0x00},
		{250,0x00,0x00},
		{251,0x84,0x00},
		{252,0x00,0x00},
		{253,0x00,0x00},
		{254,0x00,0x00},
		{255, 1, 0xFF}, // set page bit to 1
		{  0,0x00,0x00},
		{  1,0x00,0x00},
		{  2,0x00,0x00},
		{  3,0x00,0x00},
		{  4,0x00,0x00},
		{  5,0x00,0x00},
		{  6,0x00,0x00},
		{  7,0x00,0x00},
		{  8,0x00,0x00},
		{  9,0x00,0x00},
		{ 10,0x00,0x00},
		{ 11,0x00,0x00},
		{ 12,0x00,0x00},
		{ 13,0x00,0x00},
		{ 14,0x00,0x00},
		{ 15,0x00,0x00},
		{ 16,0x00,0x00},
		{ 17,0x01,0x00},
		{ 18,0x00,0x00},
		{ 19,0x00,0x00},
		{ 20,0x90,0x00},
		{ 21,0x31,0x00},
		{ 22,0x00,0x00},
		{ 23,0x00,0x00},
		{ 24,0x01,0x00},
		{ 25,0x00,0x00},
		{ 26,0x00,0x00},
		{ 27,0x00,0x00},
		{ 28,0x00,0x00},
		{ 29,0x00,0x00},
		{ 30,0x00,0x00},
		{ 31,0x00,0xFF},
		{ 32,0x00,0xFF},
		{ 33,0x01,0xFF},
		{ 34,0x00,0xFF},
		{ 35,0x00,0xFF},
		{ 36,0x90,0xFF},
		{ 37,0x31,0xFF},
		{ 38,0x00,0xFF},
		{ 39,0x00,0xFF},
		{ 40,0x01,0xFF},
		{ 41,0x00,0xFF},
		{ 42,0x00,0xFF},
		{ 43,0x00,0x0F},
		{ 44,0x00,0x00},
		{ 45,0x00,0x00},
		{ 46,0x00,0x00},
		{ 47,0x00,0xFF},
		{ 48,0x00,0xFF},
		{ 49,0x01,0xFF},
		{ 50,0x00,0xFF},
		{ 51,0x00,0xFF},
		{ 52,0x90,0xFF},
		{ 53,0x31,0xFF},
		{ 54,0x00,0xFF},
		{ 55,0x00,0xFF},
		{ 56,0x01,0xFF},
		{ 57,0x00,0xFF},
		{ 58,0x00,0xFF},
		{ 59,0x00,0x0F},
		{ 60,0x00,0x00},
		{ 61,0x00,0x00},
		{ 62,0x00,0x00},
		{ 63,0x00,0xFF},
		{ 64,0x00,0xFF},
		{ 65,0x01,0xFF},
		{ 66,0x00,0xFF},
		{ 67,0x00,0xFF},
		{ 68,0x90,0xFF},
		{ 69,0x31,0xFF},
		{ 70,0x00,0xFF},
		{ 71,0x00,0xFF},
		{ 72,0x01,0xFF},
		{ 73,0x00,0xFF},
		{ 74,0x00,0xFF},
		{ 75,0x00,0x0F},
		{ 76,0x00,0x00},
		{ 77,0x00,0x00},
		{ 78,0x00,0x00},
		{ 79,0x00,0xFF},
		{ 80,0x00,0xFF},
		{ 81,0x00,0xFF},
		{ 82,0x00,0xFF},
		{ 83,0x00,0xFF},
		{ 84,0x90,0xFF},
		{ 85,0x31,0xFF},
		{ 86,0x00,0xFF},
		{ 87,0x00,0xFF},
		{ 88,0x01,0xFF},
		{ 89,0x00,0xFF},
		{ 90,0x00,0xFF},
		{ 91,0x00,0x0F},
		{ 92,0x00,0x00},
		{ 93,0x00,0x00},
		{ 94,0x00,0x00},
		{255, 0, 0xFF} }; // set page bit to 0

/* Private function prototypes. */
static void SI5338_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);
static void SI5338_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);

/* Init general struct. */
static SI5338_GStr_t si5338 = {
		/* Set default address. */
		.addr = SI5338_ADDRESS0 << 1,
		.delay_fp = HAL_Delay,
		.i2c_rx_fp = SI5338_I2CRxData,
		.i2c_tx_fp = SI5338_I2CTxData
};


/*
 * @brief
 */
void SI5338_Init(void) {

	uint8_t CurrentVal, WriteVal;
	uint8_t Val, Val0;

	/* Check if the address is actual and set it to the proper value. */
	if (SI5338_GetDevCfg()->BasePartNumber != SI5338_BASE_PART_NUMBER) {
		si5338.addr = SI5338_ADDRESS1 << 1;
		if (SI5338_GetDevCfg()->BasePartNumber == SI5338_BASE_PART_NUMBER) {
			__NOP();
		}
	}

	SI5338_GetDevRevID();
	/* Disable outputs. */
	SI5338_WriteReg(230, 0x1F);
	/* Pause PLL lock status update. */
	SI5338_WriteReg(241, 0xE5);

	/* Write a new configuration according to ClockBuilder generated table. */
	for (uint8_t StrSel = 0; StrSel < 2; StrSel++) {
		for (uint16_t i = 0; i < SI5338_NUM_REGS_MAX; i++) {
			/* Ignore registers with masks of 0x00. */
			if(RegStore[i].RegMask != 0x00) {
				WriteVal = RegStore[i].RegVal;
				if(RegStore[i].RegMask == 0xFF) {
					/* Do a regular I2C write to the register at addr with the desired data value. */
					//WriteVal = RegStore[i].RegVal;
					si5338.i2c_tx_fp(RegStore[i].RegAddr, &WriteVal, 1);
					si5338.delay_fp(5);
				} else {
					/* Do a read-modify-write using I2C and bit-wise operations get the current value from the device at the
					register located at addr. */
					si5338.i2c_rx_fp(RegStore[i].RegAddr, &CurrentVal, 1);
					si5338.delay_fp(5);
					/* Clear the bits that are allowed to be accessed in the current value of the register. */
					/* Combine the cleared values to get the new value to write to the desired register. */
					WriteVal = (WriteVal | (CurrentVal & (~RegStore[i].RegMask)));
					si5338.i2c_tx_fp(RegStore[i].RegAddr, &WriteVal, 1);
					si5338.delay_fp(5);
				}
			}
		}
	}


	/* Select page one. */
	SI5338_WriteReg(255, 0);
	/* Validate input clocks. */
	SI5338_ReadReg(218);
	/* Disable FCAL values update. */
	Val = SI5338_ReadReg(49);
	SI5338_WriteReg(49, Val & 0x7F);
	/* Initiate locking of PLL. */
	SI5338_WriteReg(246, 0x02);
	HAL_Delay(25);
	/* Restart PLL lock status update. */
	SI5338_WriteReg(241, 0x65);
	/* Confirm PLL lock status. */
	SI5338_ReadReg(218);
	/* Copy FCAL values to active registers. */
	Val = SI5338_ReadReg(237);
	Val &= 0x03;
	Val0 = SI5338_ReadReg(47);
	Val0 &= 0xFC;
	SI5338_WriteReg(47, Val | Val0);

	Val = SI5338_ReadReg(236);
	SI5338_WriteReg(46, Val);

	Val = SI5338_ReadReg(235);
	SI5338_WriteReg(45, Val);

	Val = SI5338_ReadReg(47);
	Val &= 0x03;
	SI5338_WriteReg(47, Val | 0x14);

	/* Set PLL to use FCAL values. */
	Val = SI5338_ReadReg(49);
	SI5338_WriteReg(49, Val | 0x80);
	/* Enable  output CLK0A. */
	SI5338_WriteReg(230, 0x0E);
	__NOP();


}


/*
 * @brief Get device revision ID.
 *
 * @retval DevRevID : 0 - Rev A,
 * 					  1 - Rev B.
 */
uint8_t SI5338_GetDevRevID(void) {

	static uint8_t DevRevID;
	si5338.i2c_rx_fp(SI5338_DEV_REV_ID, &DevRevID, 1);
	si5338.delay_fp(1);

	return DevRevID & 0x07;
}

/*
 * @brief Get device configuration.
 *
 * @retval BasePartNumber 	: Represent the last two digits of the base part number: "38" for Si5338.
 * @retval DevGrade 		: Represent the device grade: 1 through 24 = A through Z.
 * @retval NVMCodeNumber 	: Represents 17 bit NVM code assigned by Skyworks Solutions (00000 through 99999).
 */
SI5338_ReadyDevCfg_t* SI5338_GetDevCfg(void) {

	SI5338_DevCfg_t DevConfig;
	static SI5338_ReadyDevCfg_t ReadyDevConfig;

	si5338.i2c_rx_fp(SI5338_DEV_START_CFG, &DevConfig.DevCfg2, 4);
	si5338.delay_fp(10);

	ReadyDevConfig.BasePartNumber = DevConfig.DEV_CFG2_5_0;
	ReadyDevConfig.DevGrade = DevConfig.DEV_CFG3_7_3;
	ReadyDevConfig.NVMCodeNumber = ((uint32_t)DevConfig.DEV_CFG3_0 << 16) | ((uint32_t)DevConfig.DevCfg4 << 8) | (uint32_t)DevConfig.DevCfg5;

	return &ReadyDevConfig;
}

/*
 * @brief
 *
 * @retval
 *
 */
uint8_t SI5338_ReadReg(uint8_t address) {

	static uint8_t RegVal;
	si5338.i2c_rx_fp(address, &RegVal, 1);
	si5338.delay_fp(1);

	return RegVal;
}

/*
 * @brief
 *
 * @retval
 *
 */
void SI5338_WriteReg(uint8_t address, uint8_t value) {

	static uint8_t RegVal;

	RegVal = value;

	si5338.i2c_tx_fp(address, &RegVal, 1);
	si5338.delay_fp(1);
}

/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
static void SI5338_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Read(SI5338I2C, si5338.addr, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

/*
 * @brief Transmit data to the chip.
 */
static void SI5338_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Write(SI5338I2C, si5338.addr, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}




