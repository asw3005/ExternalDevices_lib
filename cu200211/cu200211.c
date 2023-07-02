/*
 *	@brief NORITAKE CU200211SCPB-T1G(PW-746-101) display module driver.
 *	Created 06.15.2023
 *	cu200211.c
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
#include "stm32f1xx_hal.h"
#include "cu200211.h"

#include "stdlib.h"
#include "stdio.h"

/* Extern variables. */
extern UART_HandleTypeDef huart2;

/* Private variables. */


/* Private function prototypes. */
static void CU200211_ClrStrBuffer(void);
static uint8_t CU200211_CRCCalc(uint8_t *pBuffer, uint8_t Size);
static void cu200211_uart2_transmit(uint8_t *pBuffer, uint8_t Size);

/* External interfaces initialization. */
static CU200211_GInst_t cu200211 = {
		/* First string buffer preinit. */
		.LineOneTxBuff.StartCode = CU200211_START_CODE,
		.LineOneTxBuff.ByteCnt = CU200211_LINE_MSG_SIZE,
		.LineOneTxBuff.PastCode = CU200211_PAST_CODE,
		.LineOneTxBuff.AllowProhibit = CU200211_ALLOW,
		.LineOneTxBuff.Mode = CU200211_CHAR_PRINT,
		.LineOneTxBuff.StrNumber = CU200211_FIRST_STR,
		/* Second string buffer preinit. */
		.LineTwoTxBuff.StartCode = CU200211_START_CODE,
		.LineTwoTxBuff.ByteCnt = CU200211_LINE_MSG_SIZE,
		.LineTwoTxBuff.PastCode = CU200211_PAST_CODE,
		.LineTwoTxBuff.AllowProhibit = CU200211_ALLOW,
		.LineTwoTxBuff.Mode = CU200211_CHAR_PRINT,
		.LineTwoTxBuff.StrNumber = CU200211_SECOND_STR,
		/* Arrows Tx buffer preinit. */
		.ArrowsTxBuff.StartCode = CU200211_START_CODE,
		.ArrowsTxBuff.ByteCnt = CU200211_ARROW_MSG_SIZE,
		.ArrowsTxBuff.PastCode = CU200211_PAST_CODE,
		.ArrowsTxBuff.AllowProhibit = CU200211_ALLOW,
		.ArrowsTxBuff.Mode = CU200211_CHAR_PRINT,
		.ArrowsTxBuff.StrNumber = CU200211_ARROWS,
		/* Specific char/mode buffer preinit. */
		.SpecCharModeBuff.StartCode = CU200211_START_CODE,
		.SpecCharModeBuff.PastCode = CU200211_PAST_CODE,
		.SpecCharModeBuff.AllowProhibit = CU200211_ALLOW,
		.SpecCharModeBuff.Mode = CU200211_EXTRA_CHAR_MODE,
		.SpecCharModeBuff.StrNumber = CU200211_FIRST_STR,

		.delay = HAL_Delay,
		.tx_data_fptr = cu200211_uart2_transmit
};

/*
 * @brief Clear displayed data from the screen.
 *
 **/
void CU200211_ClrScreen(void) {
	CU200211_ClrStrBuffer();
	cu200211.LineOneTxBuff.CRCByte = CU200211_CRCCalc(&cu200211.LineOneTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
	cu200211.LineTwoTxBuff.CRCByte = CU200211_CRCCalc(&cu200211.LineTwoTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
	cu200211.tx_data_fptr(&cu200211.LineOneTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
	cu200211.tx_data_fptr(&cu200211.LineTwoTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
}

/*
 * @brief Sent time and date to the screen.
 *
 * @param ScreenLine : CU200211_FIRST_STR or CU200211_SECOND_STR.
 * @param LeadZero : if 1, discard the leading zero from hours.
 * @param Hours :
 * @param Minutes :
 * @param Seconds :
 *
 **/
void CU200211_SendTimeDate(uint8_t ScreenLine, uint8_t LeadZero, uint8_t Hours, uint8_t Minutes, uint8_t Seconds,
							uint8_t Month, uint8_t Date, uint8_t Year) {

	char StrBuff[CU200211_LINE_SIZE + 1] = { ' ' };

	if (LeadZero) {
	CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%2u:%02u:%02u  "
															  "%02u-%02u-20%02u",
									Hours, Minutes, Seconds, Month, Date, Year));
	} else {
		CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "%02u:%02u:%02u  "
																  "%02u-%02u-20%02u",
										Hours, Minutes, Seconds, Month, Date, Year));
	}
}

/*
 * @brief Send temperature, humidity and pressure to the screen.
 * 
 * @param ScreenLine : CU200211_FIRST_STR or CU200211_SECOND_STR.
 * @param LeadZero : if 0, discard the leading zero from hours.
 * @param Hours :
 * @param Minutes :
 * @param Seconds :
 *
 **/
void CU200211_SendTmpHumPress(uint8_t ScreenLine, float Temperature, uint8_t Humidity, uint16_t Pressure) {

	char StrBuff[CU200211_LINE_SIZE + 1] = { ' ' };

	if (Temperature >= 100) { Temperature = 99.9; }
	if (Humidity >= 100) { Humidity = 99; }
	//if (Pressure >= 1000) { Pressure = 999; }


	if (Temperature >= 10) {
		CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "  %3.1f\"C"
																  "  %2u%%"
																  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature < 10 && Temperature >= 0) {
		CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "   %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature <= -10) {
		CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, " %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	} else if (Temperature < 0) {
		CU200211_SendString(ScreenLine, StrBuff, sprintf(StrBuff, "  %3.1f\"C"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %2u%%"
				  	  	  	  	  	  	  	  	  	  	  	  	  "  %umm",
																  Temperature, Humidity, Pressure));
	}
}

/*
 * @brief
 *
 * @param Line: CU200211_FIRST_STR or CU200211_SECOND_STR.
 * @param *pBuffer : pointer to the string.
 * @param Size : length of the string (must be either equal to CU200211_LINE_SIZE or less than this).
 *
 **/
void CU200211_SendString(uint8_t Line, char *pBuffer, uint8_t Size) {

	char StrBuff[CU200211_LINE_SIZE + 1] = { ' ' };

	CU200211_ClrStrBuffer();
	if (Size <= CU200211_LINE_SIZE) {
		if (Line == CU200211_FIRST_STR) {
			for (uint8_t i = 0; i < Size; i++) {
				cu200211.LineOneTxBuff.Data[i] = *pBuffer;
				pBuffer++;
			}
			cu200211.LineOneTxBuff.CRCByte = CU200211_CRCCalc(&cu200211.LineOneTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
			cu200211.tx_data_fptr(&cu200211.LineOneTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
		} else {
			for (uint8_t i = 0; i < Size; i++) {
				cu200211.LineTwoTxBuff.Data[i] = *pBuffer;
				pBuffer++;
			}
				cu200211.LineTwoTxBuff.CRCByte = CU200211_CRCCalc(&cu200211.LineTwoTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
				cu200211.tx_data_fptr(&cu200211.LineTwoTxBuff.StartCode, CU200211_LINE_MSG_SIZE);
		}
	} else {

		CU200211_SendString(Line, StrBuff, sprintf(StrBuff, "    Size is %u.    ", Size));
	}
}

/*
 * @brief 
 *
 **/
void CU200211_ArrowTest(void) {

		uint8_t LineSelector 	= 0;
		uint8_t CharCounter 	= 0x20;

		CU200211_Arrows_t ArrowTest = {

				.StartCode = CU200211_START_CODE,
				.ByteCnt = CU200211_ARROW_MSG_SIZE,
				.PastCode = CU200211_PAST_CODE,
				.AllowProhibit = CU200211_ALLOW,
				.Mode = CU200211_CHAR_PRINT,
				.StrNumber = CU200211_ARROWS
		};

		while(1) {
			/* First 220 characters. */
			for (uint8_t i = 0; i < 11; i++) {
				for (uint8_t j = 0; j < 20; j++) {
					//CharTest.Data[j] = CharCounter++;
				}
				ArrowTest.CRCByte = CU200211_CRCCalc(&ArrowTest.StartCode, CU200211_ARROW_MSG_SIZE);
				cu200211.tx_data_fptr(&ArrowTest.StartCode, CU200211_ARROW_MSG_SIZE);
				HAL_Delay(1000);
			}
		}
}

/*
 * @brief
 *
 **/
void CU200211_CharTest(void) {

	uint8_t LineSelector 	= 0;
	uint8_t CharCounter 	= 0x20;

	static struct {
		uint8_t StartCode;
		uint8_t ByteCnt;
		uint8_t PastCode;
		uint8_t AllowProhibit;
		uint8_t Mode;
		uint8_t StrNumber;
		uint8_t Data[20];
		uint8_t CRCByte;

	} CharTest;

	CharTest.StartCode 		= CU200211_START_CODE;
	CharTest.ByteCnt 		= CU200211_LINE_MSG_SIZE;
	CharTest.PastCode 		= CU200211_PAST_CODE;
	CharTest.AllowProhibit 	= CU200211_ALLOW;
	CharTest.Mode 			= CU200211_CHAR_PRINT;
	CharTest.StrNumber 		= CU200211_FIRST_STR;

	while(1) {
		/* First 220 characters. */
		for (uint8_t i = 0; i < 11; i++) {
			if (!LineSelector) { CharTest.StrNumber = CU200211_FIRST_STR; }
			else { CharTest.StrNumber = CU200211_SECOND_STR; }
			for (uint8_t j = 0; j < 20; j++) {
				CharTest.Data[j] = CharCounter++;
			}
			CharTest.CRCByte = CU200211_CRCCalc(&CharTest.StartCode, CU200211_LINE_MSG_SIZE);
			cu200211.tx_data_fptr(&CharTest.StartCode, CU200211_LINE_MSG_SIZE);
			LineSelector = ~LineSelector;
			HAL_Delay(1000);
		}

		/* Last 4 chars */
		if (!LineSelector) { CharTest.StrNumber = CU200211_FIRST_STR; }
		else { CharTest.StrNumber = CU200211_SECOND_STR; }
		for (uint8_t j = 0; j < 4; j++) {
			CharTest.Data[j] = CharCounter++;
		}
		for (uint8_t j = 0; j < 16; j++) {
			CharTest.Data[j + 4] = ' ';
		}

		LineSelector = 0;
		CharCounter = 0x20;
		CharTest.CRCByte = CU200211_CRCCalc(&CharTest.StartCode, CU200211_LINE_MSG_SIZE);
		cu200211.tx_data_fptr(&CharTest.StartCode, CU200211_LINE_MSG_SIZE);
		HAL_Delay(1000);
	}
}

/*
 * @brief Clear data buffer for the strings.
 */
static void CU200211_ClrStrBuffer(void) {

	for (uint8_t i = 0; i < CU200211_LINE_SIZE; i++) {
		cu200211.LineOneTxBuff.Data[i] = ' ';
		cu200211.LineTwoTxBuff.Data[i] = ' ';
	}
}

/*
 * @brief Calculate CRC of data minus CRC byte in the struct that will be send to.
 * 
 * @param *device : Instance of CU200211_GInst_t struct.
 *
 **/
static uint8_t CU200211_CRCCalc(uint8_t *pBuffer, uint8_t Size) {

	uint8_t sum = 0;

	for (uint8_t i = 0; i < Size - 1; i++) {
		sum += *pBuffer;
		pBuffer++;
	}

	return ~(sum - 1);
}

/* Hardware dependent functions. */

/*
 * @brief UART data send function wrapper.
 *
 * @param pBuffer: pointer to the data buffer.
 * @param Size: buffer's size.
 *
 */
static void cu200211_uart2_transmit(uint8_t *pBuffer, uint8_t Size) {
	HAL_UART_Transmit(&huart2, pBuffer, Size, 100);
}







