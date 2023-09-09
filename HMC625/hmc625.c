/*
 * hmc625.c source file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "hmc625.h"

/* External variables. */
/* Private variables. */
/* Private function prototypes. */
static void HMC625_LatchData(void);
#ifdef SERIAL_MODE
static void HMC625_ClockData(void);
static void HMC625_ESpiTxData(uint8_t* pData, uint8_t Size);
#endif  /* SERIAL_MODE */
#ifdef PARALLEL_MODE
static void HMC625_ParalTxData(uint8_t* pData, uint8_t Size);
#endif /* PARALLEL_MODE */

/* Global function struct init. */
static HMC625_GStr_t hmc625 = {
		.delay_fp = HAL_Delay,
		#ifdef SERIAL_MODE
		.tx_fp = HMC625_ESpiTxData
		#endif  /* SERIAL_MODE */
		#ifdef PARALLEL_MODE
		.tx_fp = HMC625_ParalTxData
		#endif  /* PARALLEL_MODE */
};

/*
 * @brief Initialization the chip.
 */
void HMC625_Init(void) {

	HAL_GPIO_WritePin(HMC625_DATA_PORT, HMC625_DATA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC625_CLK_PORT, HMC625_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC625_LE_PORT, HMC625_LE_PIN, GPIO_PIN_RESET);

	/* Parallel or serial mode. */
}

/*
 * @brief Set gain, where Gain is 0 to 63 (-31.5dB - 0dB), step 0.5dB.
 */
void HMC625_SetGain(uint8_t Gain) {

	uint8_t GainData;

	GainData = Gain;
	hmc625.tx_fp(&GainData, 1);
}

/* Hardware dependent functions. */

/*
 * @brief Select parallel or serial mode.
 *
 * @param PSSelect : 0 - parallel mode selected,
 * 					 1 - serial mode selected.
 */
void HMC625_PSSelect(uint8_t PSSelect) {

	if (PSSelect) { HAL_GPIO_WritePin(HMC625_PS_PORT, HMC625_PS_PIN, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(HMC625_PS_PORT, HMC625_PS_PIN, GPIO_PIN_RESET); }

}

/*
 * @brief Enable latching data from register to output.
 */
static void HMC625_LatchData(void) {

	HAL_GPIO_WritePin(HMC625_LE_PORT, HMC625_LE_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(HMC625_LE_PORT, HMC625_LE_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

#ifdef SERIAL_MODE
/*
 * @brief Clocking data.
 */
static void HMC625_ClockData(void) {

	HAL_GPIO_WritePin(HMC625_CLK_PORT, HMC625_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(HMC625_CLK_PORT, HMC625_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Transmit data to the chip (MSB first).
 */
static void HMC625_ESpiTxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;

	/* Transmitting the data to the shift register. */
	TxData = *pData;
	TxData <<= 2;
	for (uint8_t i = HMC625_DATA_NUMBER; i > 0; i--) {
		/* MSB first. */
		PinState = (TxData & HMC625_DATA_MASK) ? (PinState = 1) : (PinState = 0);
		HAL_GPIO_WritePin(HMC625_DATA_PORT, HMC625_DATA_PIN, PinState);
		/* Clocking data. */
		HMC625_ClockData();
		/* Getting next bit.*/
		TxData <<= 1;
	}
	/* Latch the data. */
	HMC625_LatchData();
}
#endif  /* SERIAL_MODE */

#ifdef PARALLEL_MODE
/*
 * @brief Parallel transmit data to the chip.
 */
static void HMC625_ParalTxData(uint8_t* pData, uint8_t Size) {

	uint8_t PinState;

	/* Transmitting the data to the shift register. */
	PinState = (*pData & 0x01) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D0_PORT, HMC625_D0_PIN, PinState);
	PinState  = (*pData & 0x02) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D1_PORT, HMC625_D1_PIN, PinState);
	PinState  = (*pData & 0x04) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D2_PORT, HMC625_D2_PIN, PinState);
	PinState  = (*pData & 0x08) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D3_PORT, HMC625_D3_PIN, PinState);
	PinState  = (*pData & 0x10) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D4_PORT, HMC625_D4_PIN, PinState);
	PinState  = (*pData & 0x20) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(HMC625_D5_PORT, HMC625_D5_PIN, PinState);
	/* Latch the data. */
	HMC625_LatchData();
}
#endif /* PARALLEL_MODE */


