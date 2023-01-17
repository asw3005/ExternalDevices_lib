/*
 * @brief SCREEN_QJ_LDD901's header file.
	Created 08.16.22 by asw3005
 *
 **/

#ifndef QJ_LDD901_H_
#define QJ_LDD901_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */

#include "stm32f429xx.h"

#define GRID2 3
#define GRID3 2

/* Public function prototypes. */
void QJ_LDD901_ScreenInit(void);

void QJ_LDD901_SendNumber(uint8_t grid, uint8_t number);
void QJ_LDD901_ClearNumber(uint8_t grid);
void QJ_LDD901_TIMERLedCtrl(uint8_t state);
void QJ_LDD901_WFLedCtrl(uint8_t state);
void QJ_LDD901_LOLedCtrl(uint8_t state);
void QJ_LDD901_HILedCtrl(uint8_t state);
void QJ_LDD901_RUNLedCtrl(uint8_t state);
void QJ_LDD901_DEFROSTLedCtrl(uint8_t state);


void QJ_LDD901_ScreenTest(void);

#endif /* QJ_LDD901_H_ */
