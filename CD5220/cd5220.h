/*
 *	@brief CD5220 customer display header.
 *	Created 04.05.2026
 *
 *	Baud rate: 9600
 *	Data byte: 8 bit
 *	Parity: none
 *	Stop bit: 1
 *
 **/

#ifndef CD5220_H_
#define CD5220_H_

#include "stm32f4xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif /* NULL */

/*
 * @brief PTC standart command list.
 *
 **/
typedef enum {
	/*  */
	CD5220_LINE_SIZE		= 0x14,
	CD5220_LINE_HEAD		= 0x03,
	CD5220_LINE_TERM		= 0x01,
	CD5220_FIRST_STR 		= 0x41,
	CD5220_SECOND_STR 		= 0x42,
	CD5220_FIRST_STR_SCROLL = 0x44

} CD5220_CMD_LIST_t;

/*
 * @brief Display's txrx buffer of data. 
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	/* Command header. */
	uint8_t CmdHead[3];
	/* Data. */
	uint8_t Data[CD5220_LINE_SIZE];
	/* Termination. */
	uint8_t LineTerm;
	
} CD5220_Lines_t;

/*
 *	@brief Delay function typedef.
 *
 *	@param period : time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer.
 *
 *	@param *buffer : buffer to transmit data.
 *	@param size : amount of data bytes.
 *
 **/
typedef void(*txrx_data_fptr)(uint8_t* buffer, uint8_t size);

/*
 * @brief External interfaces struct.
 */
typedef struct {

	CD5220_Lines_t LineOneTxBuff;
	CD5220_Lines_t LineTwoTxBuff;
	delay_fptr delay;
	txrx_data_fptr tx_data_fptr;
	
} CD5220_GInst_t;

/* Public function prototype. */

void CD5220_CharTest(void);
void CD5220_ClrScreen(void);
void CD5220_SendString(uint8_t Line, char *pBuffer, uint8_t Size);
void CD5220_SendTimeDate(uint8_t ScreenLine, uint8_t LeadZero, uint8_t Hours, uint8_t Minutes, uint8_t Seconds,
							uint8_t Month, uint8_t Date, uint8_t Year);
void CD5220_SendTmpHumPress(uint8_t ScreenLine, float Temperature, uint8_t Humidity, uint16_t Pressure);
void CD5220_SendSTemp(uint8_t ScreenLine, char TmpNumber, float Temperature, uint8_t Humidity, uint16_t Pressure);

#endif /* CD5220_H_ */
