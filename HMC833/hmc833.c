/*
 * hmc833.c source file.
 *
 * Created on: Sep 7, 2023
 * Author: asw3005
 */
#include "stm32f1xx_hal.h"
#include "hmc833.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;


/* Private function prototypes. */
static void HMC833_SpiRxData(uint8_t *pData, uint8_t Size);
static void HMC833_SpiTxData(uint8_t *pData, uint8_t Size);

/* Private variables. */
static SPI_HandleTypeDef* HMC833Spi = &hspi1;
static HMC833_GStr_t hmc833 = {
		.delay_fp = HAL_Delay,
		.spi_rx_fp = HMC833_SpiRxData,
		.spi_tx_fp = HMC833_SpiTxData
};






























/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
static void HMC833_SpiRxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Receive(HMC833Spi, pData, Size, 25);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Transmit data to the chip.
 */
static void HMC833_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(HMC833Spi, pData, Size, 25);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
}
