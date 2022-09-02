/*
 *	@brief AM2301 header.
 *	Created 08.24.2020 by asw3005.
 *
 **/

#ifndef AM2301_H_				  
#define AM2301_H_

#include "stm32f429xx.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/* Hardware reading the data with timer in input capture mode. */
//#define HARDWARE_SENSOR_READ
/* Convertion test for below zero temperature. */
//#define BELOW_ZERO_TEST

/* You need to adjust this value on your system. */
#define PLATPHORM_DELAY_CONST_TICK_PER_uS 7

/* Select data port and pin. */
#define GPIO_DATA_PORT	GPIOF
#define GPIO_DATA_PIN	GPIO_PIN_6

/*
 * @brief AM2301 constants' list.
 *
 **/
typedef enum {
	/* Temperature and humidity. */
	AM2301_TMAX					= 800,
	AM2301_TMIN					= -400,
	AM2301_HMAX					= 1000,
	/* Amount of received data. */
	AM2301_RX_POL_PRESENCE		= 2,
	AM2301_RX_HARD_PRESENCE		= 3,
	AM2301_RX_DATA				= 40,
	AM2301_GAP_BYTE				= 1,
	/* Time slot constants. */
	#ifdef HARDWARE_SENSOR_READ
	AM2301_RESPONSE_TIME_MAX	= 175,
	AM2301_RESPONSE_TIME_MIN	= 140,
	AM2301_ZERO_BIT_MAX			= 90,
	AM2301_ZERO_BIT_MIX			= 80,
	AM2301_ONE_BIT_MAX			= 145,
	AM2301_ONE_BIT_MIN			= 125,
	AM2301_TERMINATION_MAX		= 95,
	AM2301_TERMINATION_MIN		= 80,
	#endif /* HARDWARE_SENSOR_READ */
	AM2301_DATA_CHECK_ERROR		= -50,
	/* Timeout. */
	AM2301_TIMEOUT				= 2500

} AM2301_CONST_LIST_t;

/*
 *	@brief Delay function typedef pointer.
 *
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx function typedef pointer.
 *
 *	@param *buffer : Transmitting/receiving data buffers.
 *	@param size : Amount bytes of data.
 *
 **/
typedef void* (*rx_data_fptr)(void);

/*
	@brief Data buffer types.
*/
#ifdef HARDWARE_SENSOR_READ
typedef struct {
	uint8_t CntRxData;
	uint8_t RAWData[AM2301_RX_DATA + AM2301_RX_HARD_PRESENCE + AM2301_GAP_BYTE];
} AM2301_RxDataBuff_t;
#endif /* HARDWARE_SENSOR_READ */

/*
	@brief Converted data of enviroment.
*/
typedef struct __attribute__((aligned(1), packed)) {

	uint8_t Parity;
	union {
		uint8_t data[4];
		struct {
			int16_t Temperature;
			uint16_t Humidity;
		};
	};

} AM2301_ConvertedData_t;

/* Public function prototypes. */ 

uint8_t AM2301_InitPerif(void);
AM2301_ConvertedData_t* AM2301_GetData(void);

#endif /* AM2301_H_ */
