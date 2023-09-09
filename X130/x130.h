/*
 * x130.h header file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */

#ifndef X130_H_
#define X130_H_

#include "stm32f1xx.h"

/* Select SPI or Parallel mode (if serial mode is undefined the parallel mode selected). */
#define X130_SERIAL_MODE
#define X130_USE_EXT_AMP

/* Define automatically parallel mode when serial mode is'n defined. */
#ifndef X130_SERIAL_MODE
#define X130_PARALLEL_MODE
#endif

/* Pin definition S/P mode select. */
#define X130_PS_PIN 		GPIO_PIN_0
#define X130_PS_PORT 		GPIOA
#ifdef X130_SERIAL_MODE
/* Pin definitions serial port. */
#define X130_LE_PIN 		GPIO_PIN_0
#define X130_CLK_PIN 		GPIO_PIN_1
#define X130_DATA_PIN 		GPIO_PIN_2
#define X130_LE_PORT 		GPIOA
#define X130_CLK_PORT 		GPIOA
#define X130_DATA_PORT 		GPIOA
/* Significant data bits. */
#define X130_DATA_NUMBER	8
#define X130_DATA_MASK		0x01
#define X130_BIT7_DATA_MASK 0x7F
#endif  /* SERIAL_MODE */
#ifdef X130_PARALLEL_MODE
/* Pin definitions parallel port. */
#define X130_D0_PIN 		GPIO_PIN_0
#define X130_D1_PIN 		GPIO_PIN_1
#define X130_D2_PIN 		GPIO_PIN_2
#define X130_D3_PIN 		GPIO_PIN_0
#define X130_D4_PIN 		GPIO_PIN_1
#define X130_D5_PIN 		GPIO_PIN_2
#define X130_D6_PIN 		GPIO_PIN_2
#define X130_D0_PORT 		GPIOA
#define X130_D1_PORT 		GPIOA
#define X130_D2_PORT 		GPIOA
#define X130_D3_PORT 		GPIOA
#define X130_D4_PORT 		GPIOA
#define X130_D5_PORT 		GPIOA
#define X130_D6_PORT 		GPIOA
#endif /* PARALLEL_MODE */
#ifdef X130_USE_EXT_AMP
#define X130_EXT_AMP_PIN	GPIO_PIN_0
#define X130_EXT_AMP_PORT 	GPIOA
#endif /* X130_USE_EXT_AMP */

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);

/*
 * @brief General struct.
 */
typedef struct {

	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr tx_fp;

} X130_GStr_t;

/* Public function prototypes. */
void X130_Init(void);
void X130_SetGain(uint8_t Gain);
void X130_PSSelect(uint8_t PSSelect);
void X110_ExtAMPControl(uint8_t ExtAMPCtrl);

#endif /* X130_H_ */
