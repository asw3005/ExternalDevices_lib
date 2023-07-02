/*
 *	@brief NORITAKE CU200211SCPB-T1G(PW-746-101) display module header.
 *	Created 06.15.2023
 *	cu200211.h
 *
 *	Baud rate: 9600
 *	Data byte: 8 bit
 *	Parity: none
 *	Stop bit: 1
 *
 **/

#ifndef CU200211_H_
#define CU200211_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief Command list.
 *
 **/
typedef enum {
	/*  */
	CU200211_START_CODE 		= 0x02,
	CU200211_PAST_CODE 			= 0x05,
	CU200211_LINE_SIZE			= 0x14,
	CU200211_CTRL_MSG_SIZE 		= 0x07,
	CU200211_ARROW_CTRL_SIZE	= 0x03,
	CU200211_LINE_MSG_SIZE		= CU200211_LINE_SIZE + CU200211_CTRL_MSG_SIZE,
	CU200211_ARROW_MSG_SIZE		= CU200211_ARROW_CTRL_SIZE + CU200211_CTRL_MSG_SIZE,
	/*   */
	CU200211_PROHIBIT 			= 0x00,
	CU200211_ALLOW,
	/* Mode. */
	CU200211_CHAR_CLR 			= 0x00,
	CU200211_CHAR_PRINT 		= 0x05,
	CU200211_EXTRA_CHAR_MODE 	= 0x08,
	/* Strings. */
	CU200211_FIRST_STR 			= 0x01,
	CU200211_SECOND_STR,
	CU200211_ARROWS 			= 0x04,
	/* Operating modes' masks. */
	CU200211_ENBLINK 			= 0x00,
	CU200211_DISBLINK,
	CU200211_ENINVERT 			= 0x00,
	CU200211_DISINVERT,
	CU200211_ENDOT 				= 0x00,
	CU200211_DISDOT,
	CU200211_ENCOMMA 			= 0x00,
	CU200211_DISCOMMA

} CU200211_CCMDLIST_t;

/*
 * @brief Arrows bit struct.
 */
typedef union {

	uint8_t Arrows;
	struct {
		uint8_t ARROW_0 : 1;
		uint8_t ARROW_1 : 1;
		uint8_t ARROW_2 : 1;
		uint8_t ARROW_3 : 1;
		uint8_t ARROW_4 : 1;
		uint8_t ARROW_5 : 1;
		uint8_t ARROW_6 : 1;
		uint8_t ARROW_7 : 1;
	};

} CU200211_ArrowBits_t;

/*
 * @brief Arrows bit+mode struct.
 */
typedef union {

	uint8_t ModeChar;
	struct {
		uint8_t BLINK_MODE 		: 1;
		uint8_t INVERSION_MODE 	: 1;
		uint8_t DOT_STATE		: 1;
		uint8_t COMMA_STATE		: 1;
		/* Must be zero. */
		uint8_t RESERVED		: 4;
	};

} CU200211_ModeChar_t;

/*
 * @brief Display's txrx arrows.
 * ??? Don't know how does it work yet. ???
 *
 **/
typedef struct /*__attribute__((aligned(1), packed))*/ {
	uint8_t StartCode;
	/* Amount of bytes. */
	uint8_t ByteCnt;
	uint8_t PastCode;
	uint8_t AllowProhibit;
	/* CU200211_EXTRA_CHAR_MODE */
	uint8_t Mode;
	/* Dot or comma state. */
	uint8_t StrNumber;
	/* Data bytes + CRC last byte in the line. */
	union {
		uint8_t Data[3];
		struct {
			uint8_t byte1;
			uint8_t byte2;
			/* It has only 4 significant bits. */
			CU200211_ModeChar_t ModeChar;
		};
	};
	uint8_t CRCByte;

} CU200211_SpecModeChar_t;

/*
 * @brief Display's txrx arrows.
 *
 **/
typedef struct /*__attribute__((aligned(1), packed))*/ {
	uint8_t StartCode;
	/* Amount of bytes. */
	uint8_t ByteCnt;
	uint8_t PastCode;
	uint8_t AllowProhibit;
	/* CU200211_CHAR_PRINT */
	uint8_t Mode;
	/* CU200211_ARROWS */
	uint8_t StrNumber;
	/* Data bytes + CRC last byte in the line. */
	union {
		uint8_t Data[3];
		struct {
			CU200211_ArrowBits_t ArrowsSet1;
			CU200211_ArrowBits_t ArrowsSet2;
			/* Set 3 has only 4 significant bits. */
			CU200211_ArrowBits_t ArrowsSet3;
		};
	};
	uint8_t CRCByte;

} CU200211_Arrows_t;

/*
 * @brief Display's txrx buffer of data. 
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	uint8_t StartCode;
	/* Amount of bytes. */
	uint8_t ByteCnt;
	uint8_t PastCode;
	uint8_t AllowProhibit;
	/* CU200211_CHAR_PRINT */
	uint8_t Mode;
	/* Number of string (CU200211_FIRST_STR or CU200211_SECOND_STR). */
	uint8_t StrNumber;
	/* Data bytes + CRC last byte in the line. */
	uint8_t Data[20];
	uint8_t CRCByte;
	
} CU200211_Lines_t;

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

	CU200211_Lines_t LineOneTxBuff;
	CU200211_Lines_t LineTwoTxBuff;
	CU200211_Arrows_t ArrowsTxBuff;
	CU200211_SpecModeChar_t SpecCharModeBuff;

	delay_fptr delay;
	txrx_data_fptr tx_data_fptr;
	
} CU200211_GInst_t;

/* Public function prototype. */

void CU200211_CharTest(void);
void CU200211_ClrScreen(void);
void CU200211_SendString(uint8_t Line, char *pBuffer, uint8_t Size);
void CU200211_SendTimeDate(uint8_t ScreenLine, uint8_t LeadZero, uint8_t Hours, uint8_t Minutes, uint8_t Seconds,
							uint8_t Month, uint8_t Date, uint8_t Year);
void CU200211_SendTmpHumPress(uint8_t ScreenLine, float Temperature, uint8_t Humidity, uint16_t Pressure);

#endif /* CU200211_H_ */
