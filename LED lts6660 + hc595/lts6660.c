/*
 *	@brief LTS6660 driver.
 *	Created 21.09.2020
 *	LTS6660.c
 *
 **/

/*
 * @brief Include files.
 *
 **/
#include "stm32f1xx_hal.h"
#include "lts6660.h"

/*
 * @brief Character's constant.
 *
 **/
static const uint8_t LTS6660_CHARACTERS[] = { 
		
	0x12,	// '0' Active LOW, active high level is 0xED	
	0xF3,	// '1' Active LOW, active high level is 0x0C	
	0x26,	// '2' Active LOW, active high level is 0xD9	
	0xA2,	// '3' Active LOW, active high level is 0x5D	
	0xC3,	// '4' Active LOW, active high level is 0x3C	
	0x8A,	// '5' Active LOW, active high level is 0x75	
	0x0A,	// '6' Active LOW, active high level is 0xF5	
	0xF2,	// '7' Active LOW, active high level is 0x0D	
	0x02,	// '8' Active LOW, active high level is 0xFD	
	0x82,	// '9' Active LOW, active high level is 0x7D	
	0xFD,	// '.' Active LOW, active high level is 0x02
	0xEF,	// '-' Active LOW, active high level is 0x10
	0xC6,	// '°' Active LOW, active high level is 0x39
	0x1E,	// 'C' Active LOW, active high level is 0xE1 
	0x4E,	// 'F' Active LOW, active high level is 0xB1
	0xFF	// ' ' Active LOW, active high level is 0x00 //15
};

static const uint8_t LTS6660_ERROR_CODE[] = "Error";	//5

/*
 * @brief 
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_ClrScreen(LTS6660_GInstance_typedef *device)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		device->data_buffer.data[i] = LTS6660_CHARACTERS[15];
	}
	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
	device->delay(1);
	device->latch_pin_fptr(0);
}

/*
 * @brief Sends one number to screen depending on the position.
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_Set_Number(LTS6660_GInstance_typedef *device, LTS6660_Numbers number, LTS6660_Position position)
{
	if (position < LTS6660_MIN_POSITION || position > LTS6660_MAX_POSITION)
	{
		return; //position error
	}
	device->data_buffer.data[LTS6660_MAX_POSITION - position] = LTS6660_CHARACTERS[number];
	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
	device->delay(1);
	device->latch_pin_fptr(0);
}

/*
 * @brief Display temperature in the Celsius degree.
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_DisplayTc(LTS6660_GInstance_typedef *device, float number)
{
	uint16_t tmpNumber = (uint16_t)(number * 10);

	for(uint8_t i = 1 ; i < 4 && (LTS6660_POSITION_TWO + LTS6660_POSITION_MAX_THREEDIG) <= LTS6660_MAX_POSITION ; i++) {				
		
		if (i == 2) device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10] & 0xFD;
		else device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10];
		tmpNumber /= 10;
	}
	
	if (number < 0) device->data_buffer.data[5] = 0xEF;
	else device->data_buffer.data[5] = 0xFF;
		
	device->data_buffer.data[1] = 0xC6;//'°'
	device->data_buffer.data[0] = 0x1E;//'C'	
	
	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
	device->delay(1);
	device->latch_pin_fptr(0);
}

/*
 * @brief Display temperature in the Fahrenheit degree.
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
//void LTS6660_DisplayTf(LTS6660_GInstance_typedef *device, float number)
//{
//	uint16_t tmpNumber = (uint16_t)(number * 10);
//
//	for (uint8_t i = 0; i < 4 && (LTS6660_POSITION_TWO + LTS6660_POSITION_MAX_FOURDIG) <= LTS6660_MAX_POSITION; i++) {				
//		
//		if (i == 1) device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10] & 0xFD;
//		else device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10];
//		tmpNumber /= 10;
//	}		
//	device->data_buffer.data[1] = 0x4E; //'F'
//
//	
//	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
//	device->delay(1);
//	device->latch_pin_fptr(0);
//}

/*
 * @brief 
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_Set_DotChar(LTS6660_GInstance_typedef *device, LTS6660_Position position)
{
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_Set_DegreeChar(LTS6660_GInstance_typedef *device, LTS6660_Position position)
{
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_Set_DashChar(LTS6660_GInstance_typedef *device, LTS6660_Position position)
{
	
}

/*
 * @brief 
 *
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_y(LTS6660_GInstance_typedef *device)
{
	
}