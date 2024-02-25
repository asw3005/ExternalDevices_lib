/*
 * X134.h header file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */

#ifndef X134_H_
#define X134_H_

#include "stm32f4xx.h"

/* Select SPI or Parallel mode (if serial mode is undefined the parallel mode selected). */
//#define X134_SERIAL_MODE
#define X134_USE_EXT_AMP

/* Define automatically parallel mode when serial mode is'n defined. */
#ifndef X134_SERIAL_MODE
#define X134_PARALLEL_MODE
#endif

#ifdef X134_SERIAL_MODE
/* Pin definition S/P mode select. */
#define X134_PS_PIN 		GPIO_PIN_0
#define X134_PS_PORT 		GPIOA
/* Pin definitions serial port. */
#define X134_LE_PIN 		GPIO_PIN_0
#define X134_CLK_PIN 		GPIO_PIN_1
#define X134_DATA_PIN 		GPIO_PIN_2
#define X134_LE_PORT 		GPIOA
#define X134_CLK_PORT 		GPIOA
#define X134_DATA_PORT 		GPIOA
/* Significant data bits. */
#define X134_DATA_NUMBER	8
#define X134_DATA_MASK		0x01
#define X134_BIT7_DATA_MASK 0x7F
#endif  /* SERIAL_MODE */
#ifdef X134_PARALLEL_MODE
/* Pin definitions parallel port. */
#define X134_D0_PIN 		GPIO_PIN_1
#define X134_D1_PIN 		GPIO_PIN_2
#define X134_D2_PIN 		GPIO_PIN_3
#define X134_D3_PIN 		GPIO_PIN_4
#define X134_D4_PIN 		GPIO_PIN_5
#define X134_D5_PIN 		GPIO_PIN_6
#define X134_D0_PORT 		GPIOE
#define X134_D1_PORT 		GPIOE
#define X134_D2_PORT 		GPIOE
#define X134_D3_PORT 		GPIOE
#define X134_D4_PORT 		GPIOE
#define X134_D5_PORT 		GPIOE
#endif /* PARALLEL_MODE */
#ifdef X134_USE_EXT_AMP
#define X134_EXT_AMP_PIN	GPIO_PIN_13
#define X134_EXT_AMP_PORT 	GPIOC
#endif /* X134_USE_EXT_AMP */

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

} X134_GStr_t;

/* Public function prototypes. */
void X134_Init(void);
void X134_SetGain(uint8_t Gain);
void X134_PSSelect(uint8_t PSSelect);
void X110_ExtAMPControl(uint8_t ExtAMPCtrl);

#endif /* X134_H_ */
