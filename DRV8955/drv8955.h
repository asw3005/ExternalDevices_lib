/*
 * @brief Common header for DRV8955.
 * Created 02.12.26 by asw3005. 
 *
 **/

#ifndef DRV8955_H_
#define DRV8955_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* Method control select. */

/* Input control select. */

/* Comment this string to use ONE SOLID OUTPUT mode. */
//#define ONE_OUT_CTRL
#ifndef ONE_OUT_CTRL
/* Comment this string to use TWO SEPARATE OUTPUTS mode. */
#define TWO_OUT_CTRL
#ifndef TWO_OUT_CTRL
/* This mode uses FOUR SEPARATE OUTPUTS. */
#define FOUR_OUT_CTRL
#endif
#endif


/* GPIO configuration. */
#define IN4_PIN					GPIO_PIN_7
#define IN4_GPIO_Port			GPIOB
#ifndef ONE_OUT_CTRL
#define IN2_PIN					GPIO_PIN_5
#define IN2_GPIO_Port			GPIOB
#endif
#ifdef FOUR_OUT_CTRL
#define IN3_PIN					GPIO_PIN_5
#define IN3_GPIO_Port			GPIOB
#define IN1_PIN					GPIO_PIN_5
#define IN1_GPIO_Port			GPIOB
#endif


#define nSLEEP_PIN				GPIO_PIN_9
#define nSLEEP_GPIO_Port		GPIOB
#define nFAULT_PIN				GPIO_PIN_2
#define nFAULT_GPIO_Port		GPIOD



/*
 * @brief Low power sleep mode.
 *
 **/
typedef enum {
	
	DRV8955_SLEEP,
	DRV8955_EN
	
} DRV8955_SLEEPMODE_t;


/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {


} DRV8955_GInst_t;


/* Public function prototypes. */
void DRV8955_Init(void);
void DRV8955_IN1Ctrl(uint8_t state);
void DRV8955_IN2Ctrl(uint8_t state);
void DRV8955_IN3Ctrl(uint8_t state);
void DRV8955_IN4Ctrl(uint8_t state);
void DRV8955_SleepCtrl(uint8_t state);

void DRV8955_IN2PWMCtrl(uint8_t state);

#endif /* DRV8955_H_ */








