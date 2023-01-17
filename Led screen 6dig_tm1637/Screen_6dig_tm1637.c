/*
 * @brief Driver for the 6-digit screen based on the TM1637 led driver.
 *	Created 08.16.22 by asw3005
 **/

#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#include "Screen_6dig_tm1637.h"
#include "tm1637.h"

/* Interpret actual grid numbers to logical. */
static const uint8_t GridNumber[6] = {
	3, 4, 5, 0, 1, 2
};

/* Numbers of the screen. */
static const uint8_t ScreenNumbers[10][7] = {
	/* 0 */
	{ 1, 1, 1, 1, 1, 1, 0 }, 
	/* 1 */
	{ 0, 1, 1, 0, 0, 0, 0 },
	/* 2 */
	{ 1, 1, 0, 1, 1, 0, 1 },
	/* 3 */
	{ 1, 1, 1, 1, 0, 0, 1 },
	/* 4 */
	{ 0, 1, 1, 0, 0, 1, 1 },
	/* 5 */
	{ 1, 0, 1, 1, 0, 1, 1 },
	/* 6 */
	{ 1, 0, 1, 1, 1, 1, 1 },
	/* 7 */
	{ 1, 1, 1, 0, 0, 0, 0 },
	/* 8 */
	{ 1, 1, 1, 1, 1, 1, 1 },
	/* 9 */
	{ 1, 1, 1, 1, 0, 1, 1 }
};

/* Special characters. */
static const uint8_t SpecialSymbol[3][7] = {
	/* - */
	{ 0, 0, 0, 0, 0, 0, 1 },
	/* Celsius degree. */
	{ 1, 1, 0, 0, 0, 1, 1 },
	/* C */
	{ 1, 0, 0, 1, 1, 1, 0 }
};

/* Private variables.*/
static 	TM1637_GInst_t tm1637_inst = {

		.delay = HAL_Delay,
		.tx_data = TM1637_WriteTo,
		.rx_data = TM1637_ReadFrom
};

/* Private function prototypes. */

/*
	@brief Screen control.

	@param brightness : May be from 0 to 6 (min and max respectively).
*/
void TM1637_ScreenCtrl(uint8_t state, uint8_t brightness) {

	TM1637_DisplayCtrl(&tm1637_inst, state, brightness);
}

/*
	@brief Send a number to the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
	@param number : Number from 0 to 9.
*/
void TM1637_SendNumber(uint8_t grid, uint8_t number) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1637_WriteSeg(GridNumber[grid], i, ScreenNumbers[number][i]);
	}
	TM1637_WriteRAM(&tm1637_inst);
}

/*
	@brief Clear the number's grid on the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
*/
void TM1637_ClearNumber(uint8_t grid) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1637_WriteSeg(GridNumber[grid], i, 0);
	}
	TM1637_WriteRAM(&tm1637_inst);
}

/*
	@brief Turn on/off a dot on the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
	@param state : 0 means nonactive, 1 is active.
*/
void TM1637_TurnDot(uint8_t grid, uint8_t state) {
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_WriteSeg(GridNumber[grid], 7, state);
		TM1637_WriteRAM(&tm1637_inst);
	}
}

/*
	@brief Draw a minus on the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
*/
void TM1637_DrawMinus(uint8_t grid) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1637_WriteSeg(GridNumber[grid], i, SpecialSymbol[0][i]);
	}
	TM1637_WriteRAM(&tm1637_inst);
}

/*
	@brief Draw a celcius degree sign on the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
*/
void TM1637_DrawCelsius(uint8_t grid) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1637_WriteSeg(GridNumber[grid], i, SpecialSymbol[2][i]);
	}
	TM1637_WriteRAM(&tm1637_inst);
}

/*
	@brief Draw a celcius degree sign on the screen.

	@param grid : Can be next values - 0, 1, 2, 3, 4, 5.
*/
void TM1637_DrawDegree(uint8_t grid) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1637_WriteSeg(GridNumber[grid], i, SpecialSymbol[1][i]);
	}
	TM1637_WriteRAM(&tm1637_inst);
}

/*
	@brief Screen test.
*/
void TM1637_ScreenTest(void) {
	/* Turn on all segments one by one. */
	for (uint8_t j = 0; j < 6; j++) {
		for (uint8_t i = 0; i < 8; i++) {
			TM1637_WriteSeg(GridNumber[j], i, 1);
			TM1637_WriteRAM(&tm1637_inst);
			HAL_Delay(50);
		}

	}
	/* Turn off all segments one by one. */
	for (uint8_t j = 0; j < 6; j++) {
		for (uint8_t i = 0; i < 8; i++) {
			TM1637_WriteSeg(GridNumber[j], i, 0);
			TM1637_WriteRAM(&tm1637_inst);
			HAL_Delay(50);
		}
	}

	for (uint8_t j = 0; j < 6; j++) {
		for (uint8_t i = 0; i < 7; i++) {
			TM1637_WriteSeg(GridNumber[j], i, ScreenNumbers[j][i]);
		}
		TM1637_WriteRAM(&tm1637_inst);
		HAL_Delay(250);

	}
	/* Turn off all segments one by one. */
	for (uint8_t j = 0; j < 6; j++) {
		TM1637_ClearNumber(j);
		HAL_Delay(250);
	}

	/* Turn on/off the dots on the screen. */
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_TurnDot(i, 1);
		HAL_Delay(100);
	}

	/* Clear the dots. */
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_TurnDot(i, 0);
		HAL_Delay(100);
	}

	/* Send all numbers to the screen. */
	for (uint8_t grid = 0; grid < 6; grid++) {
		for (uint8_t number = 0; number < 10; number++) {
			TM1637_SendNumber(grid, number);
			HAL_Delay(250);
		}
		TM1637_ClearNumber(grid);
	}
	/* Turn on the dots. */
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_TurnDot(i, 1);
		HAL_Delay(100);
	}

	/* Send all numbers to the screen. */
	for (uint8_t number = 0; number < 10; number++) {
		for (uint8_t grid = 0; grid < 6; grid++) {
			TM1637_SendNumber(grid, number);
		}
		HAL_Delay(250);
	}
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_ClearNumber(i);
	}

	/* Clear the dots. */
	for (uint8_t i = 0; i < 6; i++) {
		TM1637_TurnDot(i, 0);
		HAL_Delay(100);
	}

	TM1637_DrawMinus(3);
	TM1637_DrawCelsius(2);
	TM1637_DrawDegree(1);
	HAL_Delay(3000);
	TM1637_ClearNumber(3);
	TM1637_ClearNumber(2);
	TM1637_ClearNumber(1);
}

