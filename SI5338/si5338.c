/*
 * si5338.c source file.
 *
 * Created on: Sep 1, 2023
 * Author: Supervisor
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "si5338.h"

/* To write the configuration of the chip you need to use Skyworks ClockBuilderPro. */

/* External variables. */
extern I2C_HandleTypeDef hi2c1;

/* Private variables. */
static I2C_HandleTypeDef* SI5338I2C = &hi2c1;
static SI5338_GStr_t si5338 = {
		.delay_fp = HAL_Delay,
		.i2c_rx_fp = SI5338_I2CRxData,
		.i2c_tx_fp = SI5338_I2CTxData
};

/* Private function prototypes. */


/*
 * @brief
 */
void SI5338_Init(void) {



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


























/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
void SI5338_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Read(SI5338I2C, SI5338_CURRENT_ADDR, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

/*
 * @brief Transmit data to the chip.
 */
void SI5338x_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Write(SI5338I2C, SI5338_CURRENT_ADDR, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

