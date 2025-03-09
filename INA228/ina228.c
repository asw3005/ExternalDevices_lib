/*
 * ina228.c  source file.
 *
 *  Created on: Jan 23, 2025
 *  Author: user asw3005
 */

#include "ina228.h"
#include <stm32f1xx_hal.h>



/* External variables. */
extern I2C_HandleTypeDef hi2c1;

/* Private variables. */
static I2C_HandleTypeDef* INA228_I2C = &hi2c1;

/* Private function prototypes. */
static void INA228_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);
static void INA228_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);

/* Init general struct. */
static INA228_GStr_t ina228_inst = {
		.DevAddress = INA228_CURRENT_ADDR,
		.delay_fp = HAL_Delay,
		.i2c_rx_fp = INA228_I2CRxData,
		.i2c_tx_fp = INA228_I2CTxData
};












/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
static void INA228_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Read(INA228_I2C, ina228_inst.DevAddress, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

/*
 * @brief Transmit data to the chip.
 */
static void INA228_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Write(INA228_I2C, ina228_inst.DevAddress, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

