/*
 *	@brief PSTB404 display driver.
 *	Created 24.09.2020
 *	PSTB404.c
 *
 **/

/*
 * @brief Include files.
 *
 **/
#include "stm32f1xx_hal.h"
#include "pstb404.h"

/*
 * @brief Private function prototype
 * 
 **/
static void PSTB404_CRCCalc(PSTB404__GInstance_typedef *device);


/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 *
 **/
void PSTB404_ClrScreen(PSTB404__GInstance_typedef *device)
{
	PSTB404_ClrRegion(device, PSTB404_LEFT_UP_WINDOW);
	PSTB404_ClrRegion(device, PSTB404_RIGHT_UP_WINDOW);
	PSTB404_ClrRegion(device, PSTB404_LEFT_DOWN_WINDOW);
	PSTB404_ClrRegion(device, PSTB404_RIGHT_DOWN_WINDOW);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 *
 **/
void PSTB404_ClrRegion(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region)
{
	uint16_t sum = 0;
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.SelectorBlock = screen_region;
	device->data_buffer.MessageBlock[0] = ' ';
	device->data_buffer.MessageBlock[1] = ' ';
	device->data_buffer.MessageBlock[2] = ' ';
	device->data_buffer.MessageBlock[3] = ' ';
	device->data_buffer.MessageBlock[4] = ' ';
	device->data_buffer.MessageBlock[5] = ' ';
	device->data_buffer.MessageBlock[6] = ' ';
	device->data_buffer.MessageBlock[7] = ' ';
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;

	///Calculation of CRC.
	PSTB404_CRCCalc(device);
	
	//device->receive_data_fptr(&device->data_buffer.Echo, 1);
	device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
	device->delay(150);	
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 * @param dot_mode : If one, colon is flashing.
 * @param lead_zero : If 0, leading zero is throw away.
 *
 **/
void PSTB404_SendTime(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t dot_mode, uint8_t lead_zero, uint8_t timeS, uint8_t timeM, uint8_t timeH)
{
	uint16_t sum = 0;
	static uint8_t data[8];
	
	//PSTB404_ClrScreenPart(device, screen_region);
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;	
	
	///Writes message to buffer.
	device->data_buffer.SelectorBlock = screen_region;
	//Tens of hours and leading zero.
	if (!lead_zero)
	{
		if ((timeH / 10) == 0) data[0] = ' ';
		else data[0] = (timeH / 10) + 48;
	} else data[0] = (timeH / 10) + 48;	
	//Units of hours.
	data[1] = (timeH % 10) + 48;
	
	//Semicolon and their flashing. 0xBA
	if (dot_mode)
	{
		if (data[2] == ':') 
		{
			data[2] = ' ';
			data[5] = ' ';
		} else {
			data[2] = ':';
			data[5] = ':';
		}
	} else { 
		data[2] = ':'; 
		data[5] = ':';
	}
	//Tens and units of minutes.
	data[3] = (timeM / 10) + 48;
	data[4] = (timeM % 10) + 48;
	//Tens and units of seconds.
	data[6] = (timeS / 10) + 48;
	data[7] = (timeS % 10) + 48;
	
	for (uint8_t i = 0; i < 8; i++)
	{
		device->data_buffer.MessageBlock[i] = data[i];
	}
	///Calculation of CRC.
	PSTB404_CRCCalc(device);	
	//Send data.
	device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
	device->delay(150);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 * @param dot_mode : If one, colon is flashing.
 * @param lead_zero : If 0, leading zero is throw away.
 *
 **/
void PSTB404_SendDate(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region,uint8_t date, uint8_t month, uint8_t year)
{
	uint16_t sum = 0;
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;	
	
	///Writes message to buffer.
	device->data_buffer.SelectorBlock = screen_region;
	//Tens of hours.
	if((date / 10) == 0) device->data_buffer.MessageBlock[0] = ' ';
	else device->data_buffer.MessageBlock[0] = (date / 10) + 48;	
	//Units of hours.
	device->data_buffer.MessageBlock[1] = (date % 10) + 48;
	
	//Semicolon and their flashing. 0xBA
	device->data_buffer.MessageBlock[2] = '.'; 
	device->data_buffer.MessageBlock[5] = '.';

	//Tens and units of minutes.
	device->data_buffer.MessageBlock[3] = (month / 10) + 48;
	device->data_buffer.MessageBlock[4] = (month % 10) + 48;
	//Tens and units of seconds.
	device->data_buffer.MessageBlock[6] = (year / 10) + 48;
	device->data_buffer.MessageBlock[7] = (year % 10) + 48;
	
	///Calculation of CRC.
	PSTB404_CRCCalc(device);	
	//Send data.
	device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
	device->delay(150);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 * @param float4_1 : Float value, two integer, one fraction. 
 *
 **/
void PSTB404_SendTemperatureFloat4_1(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, float float4_1)
{
	uint16_t sum = 0;
	uint16_t tmpNumber = 0;
	
	if (float4_1 < -40) float4_1 = -40.0f;
	else if (float4_1 > 80) float4_1 = 80.0f;
	tmpNumber = (uint16_t)(float4_1 * 10);
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;	
	
	///Writes value to buffer.
	device->data_buffer.SelectorBlock = screen_region;				
		
	device->data_buffer.MessageBlock[0] = ' ';
	device->data_buffer.MessageBlock[7] = ' ';
	
	device->data_buffer.MessageBlock[4] = tmpNumber % 10 + 48;
	tmpNumber /= 10;
	device->data_buffer.MessageBlock[3] = '.';
	device->data_buffer.MessageBlock[2] = tmpNumber % 10 + 48;
	tmpNumber /= 10;

	device->data_buffer.MessageBlock[1] = tmpNumber % 10 + 48;
	tmpNumber /= 10;
	device->data_buffer.MessageBlock[5] = 34;	//'°'
	device->data_buffer.MessageBlock[6] = 'C';	

	//Minus simbol and leading zero transformation.
	if (float4_1 < 10 && float4_1 >= 0)
	{
		device->data_buffer.MessageBlock[1] = ' ';
	}
	else if (float4_1 < 0 && float4_1 > -10)
	{
		device->data_buffer.MessageBlock[1] = '-';		
	}
	else if (float4_1 <= -10)
	{
		device->data_buffer.MessageBlock[0] = '-';
	}

	///Calculation of CRC.
	PSTB404_CRCCalc(device);

	//Transmits data from the buffer.
	//device->receive_data_fptr(&device->data_buffer.Echo, 1);
	device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
	device->delay(150);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 * @param *message : Pointer to the set of chars.
 *
 **/
void PSTB404_SendMessage(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t *message)
{
	uint16_t sum = 0;
	
	//PSTB404_ClrScreenPart(device, screen_region);
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;	
	
	//Clear buffer.
	for(uint8_t i = 0; i < 8 ; i++)
	{
		device->data_buffer.MessageBlock[i] = ' ';
	}
	
	///Writes message to buffer.
	device->data_buffer.SelectorBlock = screen_region;		
	for (uint8_t i = 0; i < 8; i++)
	{
		device->data_buffer.MessageBlock[i] = *message;
		message++;
	}	

	///Calculation of CRC.
	PSTB404_CRCCalc(device);
	
	//device->receive_data_fptr(&device->data_buffer.Echo, 1);
	device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
	device->delay(150);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 * @param screen_region : It is the screen region from the PSTB404_Command enum.
 *
 **/
void PSTB404_CharTest(PSTB404__GInstance_typedef *device)
{
	uint16_t sum = 0;
	uint8_t j = 0;
	
	PSTB404_ClrScreen(device);
	
	device->data_buffer.StartBlock = PSTB404_START_BLOCK;
	device->data_buffer.EndBlock = PSTB404_END_BLOCK;	
	
	///Writes message to buffer.			
		
	for (size_t i = 0x20; i <= 0xAF; i++)
	{		
		device->data_buffer.MessageBlock[j++] = i;	
		if (j == 8) 
		{
			j = 0;
			for (uint8_t k = 0; k < 4; k++)
			{				
				device->data_buffer.SelectorBlock = PSTB404_LEFT_UP_WINDOW + k;
				///Calculation of CRC.
				PSTB404_CRCCalc(device);
				//Transmits data from the buffer.
				device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
				device->delay(150);		
				sum = 0;
			}

		}
	}
	
	for (size_t i = 0xE0; i <= 0xEF; i++)
	{
		device->data_buffer.MessageBlock[j++] = i;	
		if (j == 8) 
		{
			j = 0;
			for (uint8_t k = 0; k < 4; k++)
			{	
				device->data_buffer.SelectorBlock = PSTB404_LEFT_UP_WINDOW + k;
				///Calculation of CRC.
				PSTB404_CRCCalc(device);
				//Transmits data from the buffer.
				device->transmit_data_fptr(&device->data_buffer.StartBlock, 12);
				device->delay(150);		
				sum = 0;
			 }
		}	
	}
}

/*
 * @brief 
 * 
 * @param *device : Instance of PSTB404__GInstance_typedef struct.
 *
 **/
static void PSTB404_CRCCalc(PSTB404__GInstance_typedef *device)
{
	device->data_buffer.MessageCRC = 0;
	device->data_buffer.MessageCRC += device->data_buffer.SelectorBlock;	
	for (uint8_t i = 0; i < 8; i++)
	{
		device->data_buffer.MessageCRC += device->data_buffer.MessageBlock[i];		
	}
	device->data_buffer.MessageCRC =   256 - (device->data_buffer.MessageCRC % 256);
}

