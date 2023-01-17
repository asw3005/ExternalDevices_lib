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

/* Constants for the menu. */
static const uint8_t LTS6660_MENU_CONST[] = {
		
		//1G  //2F					  //3				4//				  //5		  //6
		0xEF, 0xDF, 0xDE, 0xD6, 0xD2, 0xBF, 0xBB, 0xB3, 0xFE, 0xF6, 0xF2, 0xBF, 0xBB, 0xEF 
	};

static const uint8_t LTS6660_MENU_CONST1[] = {
		
		//1G  //2F	//3	  4//	//5	  //6
		0xEF, 0x0E, 0xAE, 0xAE, 0xA2, 0xEF 
	};

static const uint8_t LTS6660_ERROR_CODE[] = "Error";	//5

/* Private function prototype. */
static void LTS6660_SendData(LTS6660_GInstance_t *device);

/*
 * @brief 
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_ClrScreen(LTS6660_GInstance_t *device)
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
void LTS6660_BlinkBit(LTS6660_GInstance_t *device, LTS6660_Mask blink_mask)
{

	
	//device->data_buffer.data[LTS6660_MAX_POSITION - position] = LTS6660_CHARACTERS[number];
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
void LTS6660_Send_Number(LTS6660_GInstance_t *device, LTS6660_Position position, uint8_t number)
{
	if (position < LTS6660_MIN_POSITION || position > LTS6660_MAX_POSITION)
	{
		return; //position error
	}
	
	for (uint8_t i = 6; i != 0; i--)
	{
		device->data_buffer.data[i] = LTS6660_CHARACTERS[15];
	}
	
	device->data_buffer.data[LTS6660_MAX_POSITION - position - 2] = LTS6660_CHARACTERS[number % 10];
	number /= 10;
	device->data_buffer.data[LTS6660_MAX_POSITION - position - 1] = LTS6660_CHARACTERS[number % 10];
	number /= 10;
	device->data_buffer.data[LTS6660_MAX_POSITION - position - 0] = LTS6660_CHARACTERS[number];
	
	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
	device->delay(1);
	device->latch_pin_fptr(0);
}

/*
 * @brief Sends time or date.
 * 
 * @param *device : Instance of LTS6660_GInstance_typedef struct.
 *
 **/
void LTS6660_Send_TimeDate(LTS6660_GInstance_t *device, const uint8_t hours_date, const uint8_t minutes_month, const uint8_t seconds_year, const LTS6660_Mask blinking)
{
	static uint8_t isBlink = 0;

	/*Blink mode.*/
	if (blinking) {
		if (isBlink) isBlink = 0; 
		else isBlink = 1;
	} else isBlink = 0;
	
	/*Set hours or date.*/	
	if (blinking == LTS6660_DISPLAY_TIME) {
		device->data_buffer.data[LTS6660_MAX_POSITION - 1] = LTS6660_CHARACTERS[hours_date % 10] & 0xFD;
	}
	else if (blinking == LTS6660_SET_HOURS || blinking == LTS6660_SET_DATE) {
		device->data_buffer.data[LTS6660_MAX_POSITION - 1] = LTS6660_CHARACTERS[hours_date % 10] & 0xFD;
	}
		else {			
			device->data_buffer.data[LTS6660_MAX_POSITION - 1] = LTS6660_CHARACTERS[hours_date % 10];
		}
	device->data_buffer.data[LTS6660_MAX_POSITION - 0] = LTS6660_CHARACTERS[(hours_date / 10) % 10];
	
	

	/*Set minutes or month.*/
	if (blinking== LTS6660_DISPLAY_TIME) {
		device->data_buffer.data[LTS6660_MAX_POSITION - 3] = LTS6660_CHARACTERS[minutes_month % 10] & 0xFD;
	}
	else if (blinking == LTS6660_SET_MINUTES || blinking == LTS6660_SET_MONTH) {
			device->data_buffer.data[LTS6660_MAX_POSITION - 3] = LTS6660_CHARACTERS[minutes_month % 10] & 0xFD;
		}
		else {			
			device->data_buffer.data[LTS6660_MAX_POSITION - 3] = LTS6660_CHARACTERS[(minutes_month % 10)];
		}	
	device->data_buffer.data[LTS6660_MAX_POSITION - 2] = LTS6660_CHARACTERS[(minutes_month / 10) % 10];
	
	
	/*Set seconds or year.*/
	if (blinking == LTS6660_DISPLAY_TIME) {
		device->data_buffer.data[LTS6660_MAX_POSITION - 5] = LTS6660_CHARACTERS[seconds_year % 10];
	}
	else if (blinking == LTS6660_SET_YEAR) {
		device->data_buffer.data[LTS6660_MAX_POSITION - 5] = LTS6660_CHARACTERS[seconds_year % 10] & 0xFD;
	} else {
		device->data_buffer.data[LTS6660_MAX_POSITION - 5] = LTS6660_CHARACTERS[seconds_year % 10];
	}
	device->data_buffer.data[LTS6660_MAX_POSITION - 4] = LTS6660_CHARACTERS[(seconds_year / 10) % 10];
	
	
	//Tens of hours and leading zero.
	if(!((hours_date / 10) % 10))
	{
		device->data_buffer.data[LTS6660_MAX_POSITION - 0] = 0xFF;
	} 		
	
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
void LTS6660_Send_TemperatureC(LTS6660_GInstance_t *device, float number)
{
	int16_t tmpNumber = (int16_t)(number * 10);
	int16_t converted_float = tmpNumber;
	
	/*If number is negative we will make it to positive.*/
	if (tmpNumber < 0) tmpNumber = -tmpNumber;

	/*write number to buffer.*/
	for(uint8_t i = 1 ; i < 4 && (LTS6660_POSITION_TWO + LTS6660_POSITION_MAX_THREEDIG) <= LTS6660_MAX_POSITION ; i++) {				
		
		if (i == 2) device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10] & 0xFD;
		else device->data_buffer.data[LTS6660_POSITION_TWO + i - 1] = LTS6660_CHARACTERS[tmpNumber % 10];
		tmpNumber /= 10;
	}	
	
	/**/
	if (converted_float >= 100) {
		device->data_buffer.data[5] = 0xFF; // ' '
	} else if (converted_float >= 0 && converted_float < 100) {
		device->data_buffer.data[5] = 0xFF; // ' '
		device->data_buffer.data[4] = 0xFF;  // ' '		
	} else if (converted_float < 0 && converted_float > -100) {
		device->data_buffer.data[5] = 0xFF;  // ' '
		device->data_buffer.data[4] = 0xEF;   // '-' 		
	} else {
		device->data_buffer.data[5] = 0xEF;    // '-'
	}

	/*Celcius symbols.*/
	device->data_buffer.data[1] = 0xC6;//'°'
	device->data_buffer.data[0] = 0x1E;//'C'	
	
	device->transmit_data_fptr(device->data_buffer.data, NUMBER_OF_DIGITS);
	device->delay(1);
	device->latch_pin_fptr(0);
}

/*
 * @brief Menu entering/exiting screen.
 *
 **/
void LTS6660_MenuScreen(LTS6660_GInstance_t *device,  uint8_t state)
{
	/* Cleare buffer. */
	for (uint8_t i = 0; i < 6; i++)	{
		device->data_buffer.data[i] = 0xFF;
	}
	
	if (state) {
		
		device->data_buffer.data[5] = LTS6660_MENU_CONST1[0];
		device->data_buffer.data[0] = LTS6660_MENU_CONST1[5];
		LTS6660_SendData(device);
		device->delay(300);
		
		device->data_buffer.data[4] = LTS6660_MENU_CONST1[1];
		device->data_buffer.data[1] = LTS6660_MENU_CONST1[4];
		LTS6660_SendData(device);
		device->delay(300);
		
		device->data_buffer.data[3] = LTS6660_MENU_CONST1[2];
		device->data_buffer.data[2] = LTS6660_MENU_CONST1[3];
		LTS6660_SendData(device);
		device->delay(300);
		
	} else {
		
		
		
	}
}

/*
 * @brief Send buffered data to the screen.
 *
 **/
static void LTS6660_SendData(LTS6660_GInstance_t *device)
{
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

