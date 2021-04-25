/*
 *	@brief LTS6660 driver.
 *	Created 21.09.2020
 *	LTS6660_H_
 *
 **/

//#pragma once
#ifndef LTS6660_H_				  
#define LTS6660_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

#define NUMBER_OF_DIGITS 6

/*
 * @brief Position of symbols on the screen.
 *
 **/
typedef enum
{
	LTS6660_MIN_POSITION = 0,
	LTS6660_POSITION_ONE = 1,
	LTS6660_POSITION_TWO,
	LTS6660_POSITION_THREE,
	LTS6660_POSITION_FOUR,
	LTS6660_POSITION_FIVE,
	LTS6660_POSITION_SIX,
	LTS6660_POSITION_SEVEN,
	LTS6660_POSITION_EIGHT,
	LTS6660_POSOTION_NINE,
	LTS6660_MAX_POSITION = NUMBER_OF_DIGITS - 1,
	LTS6660_POSITION_MAX_THREEDIG = 2,
	LTS6660_POSITION_MAX_FOURDIG = 3
	
} LTS6660_Position;

/*
 * @brief Blink mask.
 *
 **/
typedef enum 
{
	LTS6660_DISPLAY_TIME,
	LTS6660_SET_HOURS,
	LTS6660_SET_MINUTES,
	
	LTS6660_SET_DAY,
	LTS6660_SET_DATE,
	LTS6660_SET_MONTH,
	LTS6660_SET_YEAR,
	
} LTS6660_Mask;

typedef struct
{
	uint8_t data[6];
	
} LTS6660_DataBuffer;



/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 * @brief Remote control for the latch pin.
 *
 * @param pull_up : Zero is pull down, more than zero is pull up.
 *
 **/
typedef void(*latch_remote_fptr)(uint8_t pull_up);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef void(*tx_data_fptr)(uint8_t *buffer, uint8_t size);

typedef struct
{
	LTS6660_DataBuffer data_buffer;
	delay_fptr delay;
	latch_remote_fptr latch_pin_fptr;
	tx_data_fptr transmit_data_fptr;
	
} LTS6660_GInstance_t;

/*
 * @brief Public function prototype.
 *
 **/
void LTS6660_ClrScreen(LTS6660_GInstance_t *device);
void LTS6660_Send_Number(LTS6660_GInstance_t *device, LTS6660_Position position, uint8_t number);
void LTS6660_Send_TimeDate(LTS6660_GInstance_t *device, const uint8_t hours_date, const uint8_t minutes_month, const uint8_t seconds_year, const LTS6660_Mask blinking);
void LTS6660_Send_TemperatureC(LTS6660_GInstance_t *device, float number);
void LTS6660_MenuScreen(LTS6660_GInstance_t *device, uint8_t state);
#endif /* LTS6660_H_ */
