/*
 * x130.c source file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "x130.h"

/* External variables. */
/* Private variables. */
/* Private function prototypes. */
static void X130_LatchData(void);
#ifdef X130_SERIAL_MODE
static void X130_ClockData(void);
static void X130_ESpiTxData(uint8_t* pData, uint8_t Size);
#endif  /* X130_SERIAL_MODE */
#ifdef X130_PARALLEL_MODE
static void X130_ParalTxData(uint8_t* pData, uint8_t Size);
#endif /* X130_PARALLEL_MODE */

/* Global function struct init. */
static X130_GStr_t x130 = {
		.delay_fp = HAL_Delay,
		#ifdef X130_SERIAL_MODE
		.tx_fp = X130_ESpiTxData
		#endif  /* X130_SERIAL_MODE */
		#ifdef X130_PARALLEL_MODE
		.tx_fp = X130_ParalTxData
		#endif  /* X130_PARALLEL_MODE */
};

/*
 * @brief Initialization the chip.
 */
void X130_Init(void) {

	HAL_GPIO_WritePin(X130_DATA_PORT, X130_DATA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X130_CLK_PORT, X130_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X130_LE_PORT, X130_LE_PIN, GPIO_PIN_RESET);

	/* Parallel or serial mode. */
}

/*
 * @brief Set gain, where Gain is 0 to 127 (-31.5dB - 0dB), step 0.25dB.
 */
void X130_SetGain(uint8_t Gain) {

	uint8_t GainData;

	GainData = Gain;
	x130.tx_fp(&GainData, 1);
}

/* Hardware dependent functions. */

/*
 * @brief Select parallel or serial mode.
 *
 * @param PSSelect : 0 - parallel mode selected,
 * 					 1 - serial mode selected.
 */
void X130_PSSelect(uint8_t PSSelect) {

	if (PSSelect) { HAL_GPIO_WritePin(X130_PS_PORT, X130_PS_PIN, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(X130_PS_PORT, X130_PS_PIN, GPIO_PIN_RESET); }

}

/*
 * @brief Enable latching data from register to output.
 */
static void X130_LatchData(void) {

	HAL_GPIO_WritePin(X130_LE_PORT, X130_LE_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(X130_LE_PORT, X130_LE_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

#ifdef X130_SERIAL_MODE
/*
 * @brief Clocking data.
 */
static void X130_ClockData(void) {

	HAL_GPIO_WritePin(X130_CLK_PORT, X130_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(X130_CLK_PORT, X130_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Transmit data to the chip (LSB first).
 */
static void X130_ESpiTxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;

	/* Transmitting the data to the shift register. */
	TxData = *pData & X130_BIT7_DATA_MASK;
	for (uint8_t i = X130_DATA_NUMBER; i > 0; i--) {
		/* LSB first. */
		PinState = (TxData & X130_DATA_MASK) ? (PinState = 1) : (PinState = 0);
		HAL_GPIO_WritePin(X130_DATA_PORT, X130_DATA_PIN, PinState);
		/* Clocking data. */
		X130_ClockData();
		/* Getting next bit.*/
		TxData >>= 1;
	}
	/* Latch the data. */
	X130_LatchData();
}
#endif  /* X130_SERIAL_MODE */

#ifdef X130_PARALLEL_MODE
/*
 * @brief Parallel transmit data to the chip.
 */
static void X130_ParalTxData(uint8_t* pData, uint8_t Size) {

	uint8_t PinState;

	/* Transmitting the data to the shift register. */
	PinState = (*pData & 0x01) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D0_PORT, X130_D0_PIN, PinState);
	PinState  = (*pData & 0x02) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D1_PORT, X130_D1_PIN, PinState);
	PinState  = (*pData & 0x04) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D2_PORT, X130_D2_PIN, PinState);
	PinState  = (*pData & 0x08) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D3_PORT, X130_D3_PIN, PinState);
	PinState  = (*pData & 0x10) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D4_PORT, X130_D4_PIN, PinState);
	PinState  = (*pData & 0x20) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D5_PORT, X130_D5_PIN, PinState);
	PinState  = (*pData & 0x40) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X130_D6_PORT, X130_D6_PIN, PinState);
	/* Latch the data. */
	X130_LatchData();
}
#endif /* X130_PARALLEL_MODE */

#ifdef X130_USE_EXT_AMP
/*
 * @brief X110 external operational amplifier control.
 *
 * @param ExtAMPCtrl : 0 - normal operation,
 * 					   1 - shut down.
 */
void X110_ExtAMPControl(uint8_t ExtAMPCtrl) {

	if (ExtAMPCtrl) { HAL_GPIO_WritePin(X130_EXT_AMP_PORT, X130_EXT_AMP_PIN, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(X130_EXT_AMP_PORT, X130_EXT_AMP_PIN, GPIO_PIN_RESET); }

}
#endif /* X130_USE_EXT_AMP */
