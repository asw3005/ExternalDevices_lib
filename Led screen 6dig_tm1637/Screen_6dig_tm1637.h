/*
 * @brief SCREEN_TM1637's header file.
	Created 08.16.22 by asw3005
 *
 **/

#ifndef SCREEN_TM1637_H_
#define SCREEN_TM1637_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */

#include "stm32f429xx.h"

/* Public function prototypes. */
void TM1637_ScreenCtrl(uint8_t state, uint8_t pulse_width);
void TM1637_SendNumber(uint8_t grid, uint8_t number);
void TM1637_ClearNumber(uint8_t grid);
void TM1637_TurnDot(uint8_t grid, uint8_t state);
void TM1637_DrawMinus(uint8_t grid);
void TM1637_DrawCelsius(uint8_t grid);
void TM1637_DrawDegree(uint8_t grid);
void TM1637_ScreenTest(void);

#endif /* SCREEN_TM1637_H_ */
