/*
 * @brief TM1637's header file.
 *
 **/

#ifndef TM1637_H_
#define TM1637_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */

#include "stm32f103xb.h"

/*
 * @brief General command set.
 * 
 *
 **/
typedef enum {
	/* Data command setting. */
	TM1637_DATA_CMD					= 0x01,
	/* Display and control command setting. */
	TM1637_DISPLAY_CTRL_CMD,
	/* Address command setting. */
	TM1637_ADDR_CMD
	
} TM1637_WordCmdSet;

/*
 * @brief Display command set.
 *
 **/
typedef enum {
	
	/* Data write and read mode setting. */
	TM1637_WRITE_DATA_TO_DISPLAY	= 0x00,
	TM1637_READ_KEY_SCAN_DATA		= 0x02,
	
	/* Address adding mode setting. */
	TM1637_AUTOMATIC_ADDR_ADDING	= 0x00,
	TM1637_FIX_ADDR					= 0x01,
	
	/* Test mode setting (for internal). */
	TM1637_NORMAL_MODE				= 0x00,
	TM1637_TEST_MODE				= 0x01,
		
	/* Address command setting. */
	TM1637_C0H						= 0x00,
	TM1637_C1H,						
	TM1637_C2H,						
	TM1637_C3H,						
	TM1637_C4H,						
	TM1637_C5H,						
	
	/* Display control. */
	TM1637_PULSE_WIDTH_1_16			= 0x00,
	TM1637_PULSE_WIDTH_2_16,
	TM1637_PULSE_WIDTH_4_16,
	TM1637_PULSE_WIDTH_10_16,
	TM1637_PULSE_WIDTH_11_16,
	TM1637_PULSE_WIDTH_12_16,
	TM1637_PULSE_WIDTH_13_16,
	TM1637_PULSE_WIDTH_14_16,
	
	/* Display switch setting. */
	TM1637_DISPLAY_OFF				= 0x00,
	TM1637_DISPLAY_ON,
	
} TM1637_DataCmdSet;

/*
 * @brief Scan codes of keys.
 *
 **/
typedef enum {
	
	TM1637_KEY1_SG1K1	= 0xEF,
	TM1637_KEY2_SG1K2	= 0xF7,
	TM1637_KEY3_SG5K2	= 0xD7,
	TM1637_KEY4_SG5K1	= 0xCF,
	TM1637_KEY5_SG2K1	= 0x6F,
	TM1637_KEY6_SG2K2	= 0x77,
	TM1637_KEY7_SG6K2	= 0x57,
	TM1637_KEY8_SG6K1	= 0x4F,
	TM1637_KEY9_SG3K1	= 0xAF,
	TM1637_KEY10_SG3K2	= 0xB7,
	TM1637_KEY11_SG7K2	= 0x97,
	TM1637_KEY12_SG7K1	= 0x8F,
	TM1637_KEY13_SG4K1	= 0x2F,
	TM1637_KEY14_SG4K2	= 0x37,
	TM1637_KEY15_SG8K2	= 0x17,
	TM1637_KEY16_SG8K1	= 0x0F
	
} TM1637_KeysScanCode;

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*txrx_fptr)(uint8_t command, void *pData, uint8_t size);

/*
 * @brief Data command byte type.
 * 
 * This command is to set data write and data read. 01 and 11 are not permitted to set for B1 and B0 bits.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint8_t DataCommand;
	struct {
		/* Write data to display register or read key scan data. See the TM1637_DataCmd enum above. */
		uint8_t DATA_WR_MODE		: 2;
		/* Automatic address adding or fix address. See the TM1637_DataCmdSet enum above. */
		uint8_t ADDR_ADDING_MODE	: 1;
		/* Normal mode or test mode setting.  See the TM1637_DataCmdSet enum above. */
		uint8_t TEST_MODE_SETTING	: 1;
		/* Should be set to zero. */
		uint8_t RESERVED5_4			: 2;
		/* Command word. See the TM1637_WordCmdSet enum above. */
		uint8_t DATA_CMD_WORD		: 2;
	};
	
} TM1637_DataCmd_t;

/*
 * @brief Address command byte type.
 * 
 * The command is used to set the display register address. If the address is set as C6H or a higher one, the data will be
 * ignored until effective address is set. Once electrified, the default address is C0H.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint8_t AddressCommand;
	struct {
		/* Display register address. See the TM1637_DataCmdSet enum above. */
		uint8_t ADDRESS			: 4;
		/* Should be set to zero. */
		uint8_t RESERVED5_4		: 2;
		/* Address word. See the TM1637_WordCmdSet enum above. */
		uint8_t ADDR_CMD_WORD	: 2;
	};
	
} TM1637_AddressCmd_t;

/*
 * @brief Display command byte type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint8_t DisplayCommand;
	struct {
		/* Pulse width. */
		uint8_t PULSE_WIDTH			: 3;
		/* Display on/off. */
		uint8_t DISPLAY_ON_OFF		: 1;
		/* Should be set to zero. */
		uint8_t RESERVED5_4			: 2;
		/* Display word. See the TM1637_WordCmdSet enum above. */
		uint8_t DISPLAY_CMD_WORD	: 2;
	};
	
} TM1637_DisplayCmd_t;

/*
 * @brief Receive data struct for keys.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	
	uint8_t KeysCode[16];
	
} TM1637_KeysData_t;

/*
 * @brief RAM data bit field.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	/* Segment's number from 1 to 7. */
	uint8_t SEG1 : 1;
	uint8_t SEG2 : 1;
	uint8_t SEG3 : 1;
	uint8_t SEG4 : 1;
	uint8_t SEG5 : 1;
	uint8_t SEG6 : 1;
	uint8_t SEG7 : 1;
	uint8_t SEG8 : 1;
	
} TM1637_RamBitField_t;

/*
 * @brief Receive data struct for video RAM.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	/* Grid's number from 0 to 5. */
	uint8_t RamData[6];
	TM1637_RamBitField_t Gridx[6];	
	
} TM1637_RamData_t;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	delay_fptr delay;
	txrx_fptr tx_data;
	txrx_fptr rx_data;

} TM1637_GInst_t;

/* Public function prototypes. */



#endif /* TM1637_H_ */
