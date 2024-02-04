/*
 * hmc625.h header file.
 *
 * Created on: Sep 6, 2023
 * Author: asw3005
 */

#ifndef HMC625_H_
#define HMC625_H_

#include "stm32f4xx.h"

/* Select SPI or Parallel mode (if serial mode is undefined the parallel mode selected). */
#define SERIAL_MODE

/* Define automatically parallel mode when serial mode is'n defined. */
#ifndef SERIAL_MODE
#define PARALLEL_MODE
#endif

/* Pin definition S/P mode select. */
#define HMC625_PS_PIN 		GPIO_PIN_0
#define HMC625_PS_PORT 		GPIOA
#ifdef SERIAL_MODE
/* Pin definitions serial port. */
#define HMC625_LE_PIN 		GPIO_PIN_0
#define HMC625_CLK_PIN 		GPIO_PIN_10
#define HMC625_DATA_PIN 	GPIO_PIN_12
#define HMC625_LE_PORT 		GPIOD
#define HMC625_CLK_PORT 	GPIOC
#define HMC625_DATA_PORT 	GPIOC
/* Significant data bits. */
#define HMC625_DATA_NUMBER	6
#define HMC625_DATA_MASK	0x80
#endif  /* SERIAL_MODE */
#ifdef PARALLEL_MODE
/* Pin definitions parallel port. */
#define HMC625_D0_PIN 		GPIO_PIN_0
#define HMC625_D1_PIN 		GPIO_PIN_1
#define HMC625_D2_PIN 		GPIO_PIN_2
#define HMC625_D3_PIN 		GPIO_PIN_0
#define HMC625_D4_PIN 		GPIO_PIN_1
#define HMC625_D5_PIN 		GPIO_PIN_2
#define HMC625_D0_PORT 		GPIOA
#define HMC625_D1_PORT 		GPIOA
#define HMC625_D2_PORT 		GPIOA
#define HMC625_D3_PORT 		GPIOA
#define HMC625_D4_PORT 		GPIOA
#define HMC625_D5_PORT 		GPIOA
#endif /* PARALLEL_MODE */

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*hmc625tx_fptr)(uint8_t* pData, uint8_t Size);

/*
 * @brief General struct.
 */
typedef struct {

	/* Function pointers. */
	delay_fptr delay_fp;
	hmc625tx_fptr tx_fp;

} HMC625_GStr_t;

/* Public function prototypes. */
void HMC625_Init(void);
void HMC625_SetGain(uint8_t Gain);
void HMC625_PSSelect(uint8_t PSSelect);

#endif /* HMC625_H_ */
