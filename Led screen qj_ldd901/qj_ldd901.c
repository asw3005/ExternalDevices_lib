/*
 * @brief Driver for the 2-digit screen based on the QJ_LDD901 led driver.
 *	Created 08.16.22 by asw3005
 **/

#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#include "qj_ldd901.h"
#include "tm1618.h"

/* Private variables.*/

/* Grid and segment location on the PCB. */
static const uint8_t SegPlace[20] = {
	/* GEED0 */
	/* TIMER.	W.F. */
		2,		12,
	/* GEED1 */
	/* LO. RUN. DEFROST. HI. */
		0,  1,	   2,	 13,
	/* GEED2 */
	/* A.  B. C. D. E.  F.  G. */
	   11, 4, 0, 1, 13, 12, 2	    
	/* GEED3 */
	/* A. B. C. D. E. F. G. */
	/*11, 4, 0, 1, 13, 12, 2	 */    
};

/* Interpret actual grid numbers to logical. */
static const uint8_t GridNumber[4] = {
	2, 3, 1, 0
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

/* General struct instance. */
static 	TM1618_GInst_t tm1618_inst = {

		.delay = HAL_Delay,
		.tx_data = TM1618_WriteTo,
		.rx_data = TM1618_ReadFrom
};

/* Private function prototypes. */

/*
	@brief Initialize the screen.
*/
void QJ_LDD901_ScreenInit(void) {

	TM1618_GridSegCfg(&tm1618_inst, TM1618_4grid_8SEGMENT);
	TM1618_PwrCtrl(&tm1618_inst, TM1618_DISPLAY_ON, TM1618_PULSE_WIDTH_13_16);
}

/*
	@brief Send a number to the screen.

	@param grid : Can be next values - 0, 1.
	@param number : Number from 0 to 9.
*/
void QJ_LDD901_SendNumber(uint8_t grid, uint8_t number) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1618_WriteSeg(GridNumber[grid], SegPlace[6 + i], ScreenNumbers[number][i]);
	}
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief Clear the number's grid on the screen.

	@param grid : Can be next values - 0, 1.
*/
void QJ_LDD901_ClearNumber(uint8_t grid) {
	for (uint8_t i = 0; i < 7; i++) {
		TM1618_WriteSeg(GridNumber[grid], SegPlace[6 + i], 0);
	}
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off TIMER led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_TIMERLedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID2], SegPlace[0], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off W.F. led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_WFLedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID2], SegPlace[1], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off LO led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_LOLedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID3], SegPlace[2], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off HI led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_HILedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID3], SegPlace[5], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off RUN led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_RUNLedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID3], SegPlace[3], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief On/off DEFROST led.

	@param state : If 0, the ctrl piece is deactivated. If 1 this one is activated.
*/
void QJ_LDD901_DEFROSTLedCtrl(uint8_t state) {
	TM1618_WriteSeg(GridNumber[GRID3], SegPlace[4], state);
	TM1618_WriteRAM(&tm1618_inst);
}

/*
	@brief Screen test.
*/
void QJ_LDD901_ScreenTest(void) {
	/* Turn on all segments one by one. */
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 0; j < 7; j++) {
			TM1618_WriteSeg(GridNumber[i], SegPlace[6 + j], 1);
			TM1618_WriteRAM(&tm1618_inst);
			HAL_Delay(100);
		}
	}
	HAL_Delay(500);
	/* Turn off all segments one by one. */
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t j = 7; j > 0; j--) {
			TM1618_WriteSeg(GridNumber[i], SegPlace[6 + j - 1], 0);
			TM1618_WriteRAM(&tm1618_inst);
			HAL_Delay(100);
		}
	}
	HAL_Delay(500);
	/* Send number to the screen. */
	for (uint8_t i = 0; i < 2; i++) {
		for (uint8_t j = 0; j < 10; j++) {
			QJ_LDD901_SendNumber(i, j);
			HAL_Delay(250);
		}
	}

	QJ_LDD901_ClearNumber(0);
	HAL_Delay(500);
	QJ_LDD901_ClearNumber(1);
	HAL_Delay(500);
}

