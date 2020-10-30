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
	LTS6660_MAX_POSITION = NUMBER_OF_DIGITS,
	LTS6660_POSITION_MAX_THREEDIG = 2,
	LTS6660_POSITION_MAX_FOURDIG = 3
	
} LTS6660_Position;

/*
 * @brief Massive of number.
 *
 **/
typedef enum
{
	LTS6660_NUMBER_ZERO = 0,
	LTS6660_NUMBER_ONE,
	LTS6660_NUMBER_TWO,
	LTS6660_NUMBER_THREE,
	LTS6660_NUMBER_FOUR,
	LTS6660_NUMBER_FIVE,
	LTS6660_NUMBER_SIX,
	LTS6660_NUMBER_SEVEN,
	LTS6660_NUMBER_EIGHT,
	LTS6660_NUMBER_NINE
	
} LTS6660_Numbers;

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
	
} LTS6660_GInstance_typedef;

/*
 * @brief Public function prototype.
 *
 **/
void LTS6660_ClrScreen(LTS6660_GInstance_typedef *device);
void LTS6660_Set_Number(LTS6660_GInstance_typedef *device, LTS6660_Numbers number, LTS6660_Position position);
void LTS6660_DisplayTc(LTS6660_GInstance_typedef *device, float number);
#endif /* LTS6660_H_ */
