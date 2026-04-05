/*
 *	@brief CD5220 customer display source.
 *	Created 04.05.2026
 *
 *	Baud rate: 9600
 *	Data byte: 8 bit
 *	Parity: none
 *	Stop bit: 1
 *
 **/

/*
 * @brief Include files.
 *
 **/
#include "stm32f4xx_hal.h"
#include "cd5220.h"

#include "stdlib.h"
#include "stdio.h"

/* Extern variables. */
extern UART_HandleTypeDef huart2;

/* Private variables. */

/* Private function prototypes. */
static void CD5220_Tx(uint8_t *pBuffer, uint8_t Size);
static void CD5220_ClrStrBuffer(void);

/* String buffers initialization. */
static CD5220_GInst_t CD5220 = {
		/* String buffer 0 preinit. */
		.LineOneTxBuff.CmdHead[0] = 0x1B,
		.LineOneTxBuff.CmdHead[1] = 0x51,
		.LineOneTxBuff.CmdHead[2] = CD5220_FIRST_STR,
		.LineOneTxBuff.LineTerm = 0x0D,
		/* String buffer 0 preinit. */
		.LineTwoTxBuff.CmdHead[0] = 0x1B,
		.LineTwoTxBuff.CmdHead[1] = 0x51,
		.LineTwoTxBuff.CmdHead[2] = CD5220_SECOND_STR,
		.LineTwoTxBuff.LineTerm = 0x0D,

		.delay = HAL_Delay,
		.tx_data_fptr = CD5220_Tx
};

/*
 * @brief Clear displayed data from the screen.
 *
 **/
void CD5220_ClrScreen(void) {
	CD5220_ClrStrBuffer();
	CD5220.tx_data_fptr(&CD5220.LineOneTxBuff.CmdHead[0], sizeof(CD5220.LineOneTxBuff));
	CD5220.tx_data_fptr(&CD5220.LineTwoTxBuff.CmdHead[0], sizeof(CD5220.LineOneTxBuff));
}

/*
 * @brief Sent time and date to the screen.
 *
 * @param ScreenLine : CD5220_FIRST_STR or CD5220_SECOND_STR.
 * @param LeadZero : if 1, discard the leading zero from hours.
 * @param Hours :
 * @param Minutes :
 * @param Seconds :
 *
 **/
void CD5220_SendTimeDate(uint8_t ScreenLine, uint8_t LeadZero, uint8_t Hours, uint8_t Minutes, uint8_t Seconds,
							uint8_t Month, uint8_t Date, uint8_t Year) {

	char StrBuff[CD5220_LINE_SIZE + 1] = { ' ' };

	if(Hours > 24) Hours = 24;
	if(Minutes > 60) Minutes = 60;
	if(Seconds > 60) Seconds = 60;
	if(Month > 12) Month = 12;
	if(Date > 31) Date = 31;
	if(Year > 99) Year = 99;

	if (LeadZero) {
	CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%2u:%02u:%02u  "
															  "%02u-%02u-20%02u",
									Hours, Minutes, Seconds, Month, Date, Year));
	} else {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%02u:%02u:%02u  "
																  "%02u-%02u-20%02u",
										Hours, Minutes, Seconds, Month, Date, Year));
	}
}

/*
 * @brief Send temperature to the screen.
 * 
 * @param ScreenLine : CD5220_FIRST_STR or CD5220_SECOND_STR.
 * @param TmpNumber :
 * @param Temperature :
 * @param Humidity :
 * @param Pressure :
 *
 **/
void CD5220_SendSTemp(uint8_t ScreenLine, char TmpNumber, float Temperature, uint8_t Humidity, uint16_t Pressure) {

	char StrBuff[CD5220_LINE_SIZE + 1] = { ' ' };

	if (Temperature >= 100.0f) { Temperature = 99.9f; }
	if (Humidity >= 99) { Humidity = 99; }
	if (Pressure >= 810) { Pressure = 810; }

	if (Temperature >= 10) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%c %3.1f\"C"
																  "  %2u%%"
																  "  %umm",
																  TmpNumber, Temperature, Humidity, Pressure));
	} else if (Temperature < 10 && Temperature >= 0) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%c  %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  TmpNumber, Temperature, Humidity, Pressure));
	} else if (Temperature <= -10) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%c%3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  TmpNumber, Temperature, Humidity, Pressure));
	} else if (Temperature < 0) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%c %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  TmpNumber, Temperature, Humidity, Pressure));
	}

	__asm__("nop"); 
}

/*
 * @brief Send temperature, humidity and pressure to the screen.
 * 
 * @param ScreenLine : CD5220_FIRST_STR or CD5220_SECOND_STR.
 * @param LeadZero : if 0, discard the leading zero from hours.
 * @param Temperature :
 * @param Humidity :
 * @param Pressure :
 *
 **/
void CD5220_SendTmpHumPress(uint8_t ScreenLine, float Temperature, uint8_t Humidity, uint16_t Pressure) {

	char StrBuff[CD5220_LINE_SIZE + 1] = { ' ' };

	if (Temperature >= 100.0f) { Temperature = 99.9f; }
	if (Humidity >= 99) { Humidity = 99; }
	if (Pressure >= 810) { Pressure = 810; }


	if (Temperature >= 10) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "  %3.1f\"C"
																  "  %2u%%"
																  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature < 10 && Temperature >= 0) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "   %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature <= -10) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, " %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature < 0) {
		CD5220_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "  %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	}

	__asm__("nop"); 
}

/*
 * @brief
 *
 * @param Line: CD5220_FIRST_STR or CD5220_SECOND_STR.
 * @param *pBuffer : pointer to the string.
 * @param Size : length of the string (must be either equal to CD5220_LINE_SIZE or less than this).
 *
 **/
void CD5220_SendString(uint8_t Line, char *pBuffer, uint8_t Size) {

	char StrBuff[CD5220_LINE_SIZE + 1] = { ' ' };

	CD5220_ClrStrBuffer();
	if (Size <= CD5220_LINE_SIZE) {
		if (Line == CD5220_FIRST_STR) {
			for (uint8_t i = 0; i < Size; i++) {
				CD5220.LineOneTxBuff.Data[i] = *pBuffer;
				pBuffer++;
			}
			CD5220.tx_data_fptr(&CD5220.LineOneTxBuff.CmdHead[0] , sizeof(CD5220.LineOneTxBuff));
		} else {
			for (uint8_t i = 0; i < Size; i++) {
				CD5220.LineTwoTxBuff.Data[i] = *pBuffer;
				pBuffer++;
			}
			CD5220.tx_data_fptr(&CD5220.LineTwoTxBuff.CmdHead[0] , sizeof(CD5220.LineTwoTxBuff));
		}
	} else {

		CD5220_SendString(Line, StrBuff, sprintf(StrBuff, "    Size is %u.    ", Size));
	}
}

/*
 * @brief
 *
 **/
void CD5220_CharTest(void) {

	uint8_t LineSelector 	= 0;
	uint8_t CharCounter 	= 0x20;

	CD5220_Lines_t CharTest = { 
		.CmdHead[0] = 0x1B,
		.CmdHead[1] = 0x51,
		.CmdHead[2] = 0x41
	};

	while(1) {
		/* First 220 characters. */
		for (uint8_t i = 0; i < 11; i++) {
			if (!LineSelector) { CharTest.CmdHead[2] = CD5220_FIRST_STR; }
			else { CharTest.CmdHead[2] = CD5220_SECOND_STR; }
			for (uint8_t j = 0; j < CD5220_LINE_SIZE; j++) {
				CharTest.Data[j] = CharCounter++;
			}
			CD5220.tx_data_fptr(&CharTest.CmdHead[0] , sizeof(CharTest));
			LineSelector = ~LineSelector;
			HAL_Delay(1000);
		}

		/* Last 4 chars */
		if (!LineSelector) { CharTest.CmdHead[2] = CD5220_FIRST_STR; }
		else { CharTest.CmdHead[2] = CD5220_SECOND_STR; }
		for (uint8_t j = 0; j < 4; j++) {
			CharTest.Data[j] = CharCounter++;
		}
		for (uint8_t j = 0; j < 16; j++) {
			CharTest.Data[j + 4] = ' ';
		}

		LineSelector = 0;
		CharCounter = 0x20;
		CD5220.tx_data_fptr(&CharTest.CmdHead[0] , sizeof(CharTest));
		HAL_Delay(1000);
	}
}

/*
 * @brief Clear data buffer for the strings.
 */
static void CD5220_ClrStrBuffer(void) {

	for (uint8_t i = 0; i < CD5220_LINE_SIZE; i++) {
		CD5220.LineOneTxBuff.Data[i] = ' ';
		CD5220.LineTwoTxBuff.Data[i] = ' ';
	}
}

/* Hardware dependent functions. */

/*
 * @brief UART data send function wrapper.
 *
 * @param pBuffer: pointer to the data buffer.
 * @param Size: buffer's size.
 *
 */
static void CD5220_Tx(uint8_t *pBuffer, uint8_t Size) {
	HAL_UART_Transmit(&huart2, pBuffer, Size, 100);
}







