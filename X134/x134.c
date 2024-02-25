/*
 * X134.c source file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "X134.h"

/* External variables. */
/* Private variables. */
/* Private function prototypes. */
#ifdef X134_SERIAL_MODE
static void X134_ClockData(void);
static void X134_ESpiTxData(uint8_t* pData, uint8_t Size);
#endif  /* X134_SERIAL_MODE */
#ifdef X134_PARALLEL_MODE
static void X134_ParalTxData(uint8_t* pData, uint8_t Size);
#endif /* X134_PARALLEL_MODE */

/* Global function struct init. */
static X134_GStr_t X134 = {
		.delay_fp = HAL_Delay,
		#ifdef X134_SERIAL_MODE
		.tx_fp = X134_ESpiTxData
		#endif  /* X134_SERIAL_MODE */
		#ifdef X134_PARALLEL_MODE
		.tx_fp = X134_ParalTxData
		#endif  /* X134_PARALLEL_MODE */
};

/*
 * @brief Initialization the chip.
 */
void X134_Init(void) {

	#ifdef X134_SERIAL_MODE
	HAL_GPIO_WritePin(X134_DATA_PORT, X134_DATA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X134_CLK_PORT, X134_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(X134_LE_PORT, X134_LE_PIN, GPIO_PIN_RESET);
	#endif  /* X134_SERIAL_MODE */

	/* Parallel or serial mode. */
}

/*
 * @brief Set gain, where Gain is 0 to 63 (0dB or -31.5dB), step 0.5dB.
 */
void X134_SetGain(uint8_t Gain) {

	static uint8_t GainData;

	GainData = Gain;
	X134.tx_fp(&GainData, 1);
}

#ifdef X134_SERIAL_MODE
/* Hardware dependent functions. */

/*
 * @brief Select parallel or serial mode.
 *
 * @param PSSelect : 0 - parallel mode selected,
 * 					 1 - serial mode selected.
 */
void X134_PSSelect(uint8_t PSSelect) {

	if (PSSelect) { HAL_GPIO_WritePin(X134_PS_PORT, X134_PS_PIN, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(X134_PS_PORT, X134_PS_PIN, GPIO_PIN_RESET); }

}

/*
 * @brief Enable latching data from register to output.
 */
static void X134_LatchData(void) {

	HAL_GPIO_WritePin(X134_LE_PORT, X134_LE_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(X134_LE_PORT, X134_LE_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Clocking data.
 */
static void X134_ClockData(void) {

	HAL_GPIO_WritePin(X134_CLK_PORT, X134_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(X134_CLK_PORT, X134_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Transmit data to the chip (LSB first).
 */
static void X134_ESpiTxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;

	/* Transmitting the data to the shift register. */
	TxData = *pData & X134_BIT7_DATA_MASK;
	for (uint8_t i = X134_DATA_NUMBER; i > 0; i--) {
		/* LSB first. */
		PinState = (TxData & X134_DATA_MASK) ? (PinState = 1) : (PinState = 0);
		HAL_GPIO_WritePin(X134_DATA_PORT, X134_DATA_PIN, PinState);
		/* Clocking data. */
		X134_ClockData();
		/* Getting next bit.*/
		TxData >>= 1;
	}
	/* Latch the data. */
	X134_LatchData();
}
#endif  /* X134_SERIAL_MODE */

#ifdef X134_PARALLEL_MODE
/*
 * @brief Parallel transmit data to the chip.
 */
static void X134_ParalTxData(uint8_t* pData, uint8_t Size) {

	uint8_t PinState;

	/* Transmitting the data to the shift register. */
	PinState = (*pData & 0x01) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D0_PORT, X134_D0_PIN, PinState);
	PinState  = (*pData & 0x02) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D1_PORT, X134_D1_PIN, PinState);
	PinState  = (*pData & 0x04) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D2_PORT, X134_D2_PIN, PinState);
	PinState  = (*pData & 0x08) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D3_PORT, X134_D3_PIN, PinState);
	PinState  = (*pData & 0x10) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D4_PORT, X134_D4_PIN, PinState);
	PinState  = (*pData & 0x20) ? (PinState = 1) : (PinState = 0);
	HAL_GPIO_WritePin(X134_D5_PORT, X134_D5_PIN, PinState);
	__NOP();
}
#endif /* X134_PARALLEL_MODE */

#ifdef X134_USE_EXT_AMP
/*
 * @brief X110 external operational amplifier control.
 *
 * @param ExtAMPCtrl : 0 - normal operation,
 * 					   1 - shut down.
 */
void X110_ExtAMPControl(uint8_t ExtAMPCtrl) {

	if (ExtAMPCtrl) { HAL_GPIO_WritePin(X134_EXT_AMP_PORT, X134_EXT_AMP_PIN, GPIO_PIN_SET); }
	else { HAL_GPIO_WritePin(X134_EXT_AMP_PORT, X134_EXT_AMP_PIN, GPIO_PIN_RESET); }

}
#endif /* X134_USE_EXT_AMP */
