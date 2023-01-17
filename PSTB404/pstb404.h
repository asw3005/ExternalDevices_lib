/*
 *	@brief PSTB404_ driver.
 *	Created 24.09.2020
 *	PSTB404__H_
 *
 **/

//#pragma once
#ifndef PSTB404_H_				  
#define PSTB404_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief Massive of number.
 *
 **/
typedef enum
{
	//Starting and ending blocks of transmiting data.
	PSTB404_START_BLOCK = 0x00,
	PSTB404_END_BLOCK = 0xFF,
	//The posotion of data on the display.
	PSTB404_LEFT_UP_WINDOW = 0x01,
	PSTB404_RIGHT_UP_WINDOW,
	PSTB404_LEFT_DOWN_WINDOW,
	PSTB404_RIGHT_DOWN_WINDOW,
	//Uart communication speed.
	PSTB404_BAUD_RATE = 2400
	
} PSTB404_Command;

/*
 * @brief Display's txrx buffer of data. 
 *
 **/
typedef struct /*__attribute__((aligned(1), packed))*/
{
	//The starting block, value always 0x00.
	uint8_t StartBlock;
	//The value from 0x01 to 0x05. 
	//The value 0x01 means that information write to the Left Up side of display.
	//The value 0x02 means that information write to the Right Up side of display.
	//The value 0x03 means that information write to the Left Down side of display.
	//The value 0x04 means that information write to the Right Down side of display.
	//The value 0x05 means that host writes all 32 bytes of information (four side of display). 
	uint8_t SelectorBlock;
	//The message in the ASCII format up to eight simbols.
	uint8_t MessageBlock[8];
	//The ending block, value always 0xFF.
	uint8_t EndBlock;
	//CRC is the unsigned MOD 256 sum of always transmited bytes.
	uint8_t MessageCRC;
	//It received from the display. 'Y' means right block, otherwise, 'N' means CRC did not match.
	uint8_t Echo;
	
} PSTB404_DataBuffer;

/*
 * @brief Display's txrx buffer of data. All the 32 bytes. 
 *
 **/
typedef struct /*__attribute__((aligned(1), packed))*/
{
	//The starting block, value always 0x00.
	uint8_t StartBlock;
	//The value from 0x01 to 0x05. 
	//The value 0x01 means that information write to the Left Up side of display.
	//The value 0x02 means that information write to the Right Up side of display.
	//The value 0x03 means that information write to the Left Down side of display.
	//The value 0x04 means that information write to the Right Down side of display.
	//The value 0x05 means that host writes all 32 bytes of information (four side of display). 
	uint8_t SelectorBlock;
	//The message in the ASCII format up to eight simbols.
	uint8_t MessageBlock1[8];
	uint8_t MessageBlock2[8];
	uint8_t MessageBlock3[8];
	uint8_t MessageBlock4[8];
	//The ending block, value always 0xFF.
	uint8_t EndBlock;
	//CRC is the unsigned MOD 256 sum of always transmited bytes.
	uint8_t MessageCRC;
	//It received from the display. 'Y' means right block, otherwise, 'N' means CRC did not match.
	uint8_t Echo;
	
} PSTB404_Data32Buffer;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef void(*tx_data_fptr)(uint8_t *buffer, uint8_t size);

typedef struct
{
	PSTB404_DataBuffer data_buffer;
	//PSTB404_Data32Buffer data32_buffer;
	delay_fptr delay;
	tx_data_fptr transmit_data_fptr;
	tx_data_fptr receive_data_fptr;
	
} PSTB404__GInstance_typedef;

/*
 * @brief Public function prototype.
 *
 **/
void PSTB404_ClrScreen(PSTB404__GInstance_typedef *device);
void PSTB404_ClrRegion(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region);
void PSTB404_CharTest(PSTB404__GInstance_typedef *device);
void PSTB404_Test(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region);
void PSTB404_SendTemperatureFloat4_1(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, float float4_1);
void PSTB404_SendNumberTwoUnits(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t number);
void PSTB404_SendMessage(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t *message);
void PSTB404_SendTime(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t dot_mode, uint8_t lead_zero, uint8_t timeS, uint8_t timeM, uint8_t timeH);
void PSTB404_SendDate(PSTB404__GInstance_typedef *device, PSTB404_Command screen_region, uint8_t date, uint8_t month, uint8_t year);


#endif /* PSTB404_H_ */
