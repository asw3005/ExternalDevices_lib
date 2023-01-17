/*
 * @brief TM1618's header file.
 *	Created 08.16.22 by asw3005
 **/

#ifndef TM1618_H_
#define TM1618_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */

#include "stm32f429xx.h"

#define TM1618_STB_PORT		GPIOA
#define TM1618_CLK_PORT		GPIOF
#define TM1618_DATA_PORT	GPIOF
#define TM1618_STB_PIN		GPIO_PIN_15
#define TM1618_CLK_PIN		GPIO_PIN_6
#define TM1618_DATA_PIN		GPIO_PIN_7

/*
 * @brief General command set.
 * 
 *
 **/
typedef enum {
	/* Select grid/segment amount. */
	TM1618_DISPLAY_CFG,
	/* Data command setting. */
	TM1618_DATA_CMD,
	/* Display and control command setting. */
	TM1618_DISPLAY_CTRL_CMD,
	/* Address command setting. */
	TM1618_ADDR_CMD
	
} TM1618_WordCmdSet;

/*
 * @brief Display command set.
 *
 **/
typedef enum {
	
	/* Data write and read mode setting. */
	TM1618_WRITE_DATA_TO_DISPLAY	= 0x00,
	TM1618_READ_KEY_SCAN_DATA		= 0x02,
	
	/* Address adding mode setting. */
	TM1618_AUTOMATIC_ADDR_ADDING	= 0x00,
	TM1618_FIX_ADDR					= 0x01,
	
	/* Test mode setting (for internal). */
	TM1618_NORMAL_MODE				= 0x00,
	TM1618_TEST_MODE				= 0x01,
		
	/* Address command setting. */
	TM1618_C0H						= 0x00,
	TM1618_C1H,						
	TM1618_C2H,						
	TM1618_C3H,						
	TM1618_C4H,						
	TM1618_C5H,						
	
	/* Display control. */
	TM1618_PULSE_WIDTH_1_16			= 0x00,
	TM1618_PULSE_WIDTH_2_16,
	TM1618_PULSE_WIDTH_4_16,
	TM1618_PULSE_WIDTH_10_16,
	TM1618_PULSE_WIDTH_11_16,
	TM1618_PULSE_WIDTH_12_16,
	TM1618_PULSE_WIDTH_13_16,
	TM1618_PULSE_WIDTH_14_16,

	/* grid/segment ratio. */
	TM1618_4grid_8SEGMENT			= 0x00,
	TM1618_5grid_7SEGMENT,
	TM1618_6grid_6SEGMENT,
	TM1618_7grid_5SEGMENT,
	
	/* Display switch setting. */
	TM1618_DISPLAY_OFF				= 0x00,
	TM1618_DISPLAY_ON,
	
} TM1618_DataCmdSet;

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
		/* Write data to display register or read key scan data. See the TM1618_DataCmd enum above. */
		uint8_t DATA_WR_MODE		: 2;
		/* Automatic address adding or fix address. See the TM1618_DataCmdSet enum above. */
		uint8_t ADDR_ADDING_MODE	: 1;
		/* Normal mode or test mode setting.  See the TM1618_DataCmdSet enum above. */
		uint8_t TEST_MODE_SETTING	: 1;
		/* Should be set to zero. */
		uint8_t RESERVED5_4			: 2;
		/* Command word. See the TM1618_WordCmdSet enum above. */
		uint8_t DATA_CMD_WORD		: 2;
	};
	
} TM1618_DataCmd_t;

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
		/* Display register address. See the TM1618_DataCmdSet enum above. */
		uint8_t ADDRESS			: 4;
		/* Should be set to zero. */
		uint8_t RESERVED5_4		: 2;
		/* Address word. See the TM1618_WordCmdSet enum above. */
		uint8_t ADDR_CMD_WORD	: 2;
	};
	
} TM1618_AddressCmd_t;

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
		/* Display word. See the TM1618_WordCmdSet enum above. */
		uint8_t DISPLAY_CMD_WORD	: 2;
	};
	
} TM1618_DisplayCmd_t;

/*
 * @brief Display configuration byte type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {

	uint8_t DisplayConfig;
	struct {
		/* Select grid/segment amount. */
		uint8_t DISPLAY_CFG			: 2;
		/* Should be set to zero. */
		uint8_t RESERVED			: 4;
		/* Display word. See the TM1618_WordCmdSet enum above. */
		uint8_t DISPLAY_CFG_WORD	: 2;
	};

} TM1618_DisplayCfg_t;

/*
 * @brief Receive data struct for keys.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	/* Keys scan code byte 1. */
	union {
		uint8_t ScanCode1;
		struct {
			/* Reads like a zero. */
			uint8_t RESERVED0_B0		: 1;
			/**/
			uint8_t KS1					: 1;
			/* Reads like a zero. */
			uint8_t RESERVED0_B2_B3		: 2;
			/* Should be set to zero. */
			uint8_t KS2					: 1;
			/* Reads like a zero. */
			uint8_t RESERVED0_B5_B7		: 3;
		};
	};
	/* Keys scan code byte 2. */
	union {
		uint8_t ScanCode2;
		struct {
			/* Reads like a zero. */
			uint8_t RESERVED1_B0		: 1;
			/**/
			uint8_t KS3					: 1;
			/* Reads like a zero. */
			uint8_t RESERVED1_B_2_B3	: 2;
			/* Should be set to zero. */
			uint8_t KS4					: 1;
			/* Reads like a zero. */
			uint8_t RESERVED1_B5_B7		: 3;
		};
	};
	/* Keys scan code byte 3. */
	union {
		uint8_t ScanCode3;
		struct {
			/* Reads like a zero. */
			uint8_t RESERVED2_B0		: 1;
			/**/
			uint8_t KS5					: 1;
			/* Reads like a zero. */
			uint8_t RESERVED2_B2_B7		: 6;
		};
	};
	
} TM1618_KeysData_t;

/*
 * @brief RAM data bit field.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {

	union {
		uint8_t SegData1;
		struct {
			/* Segment's number from 1 to 7. */
			uint8_t SEG1		: 1;
			uint8_t SEG2		: 1;
			uint8_t SEG3		: 1;
			uint8_t SEG4		: 1;
			uint8_t SEG5		: 1;
			uint8_t RESERVED0	: 1;
			uint8_t RESERVED1	: 1;
			uint8_t RESERVED2	: 1;
		};
	};

	union {
		uint8_t SegData2;
		struct {
			/* Segment's number from 8 to 14. */
			uint8_t RESERVED3	: 1;
			uint8_t RESERVED4	: 1;
			uint8_t RESERVED5	: 1;
			uint8_t SEG12		: 1;
			uint8_t SEG13		: 1;
			uint8_t SEG14		: 1;
			uint8_t RESERVED6	: 1;
			uint8_t RESERVED7	: 1;
		};
	};
	
} TM1618_RamBitField_t;

/*
 * @brief Receive data struct for video RAM.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	
	TM1618_DisplayCfg_t DisplayCfg;
	TM1618_DisplayCmd_t DisplayCmd;
	TM1618_DataCmd_t DataCmd;
	TM1618_AddressCmd_t AddressCmd;
	/* Grid's number from 0 to 7. */
	TM1618_RamBitField_t Gridx[7];	
	
} TM1618_RamData_t;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	delay_fptr delay;
	txrx_fptr tx_data;
	txrx_fptr rx_data;

} TM1618_GInst_t;

/* Public function prototypes. */
void TM1618_WriteSeg(uint8_t grid, uint8_t segment, uint8_t state);
TM1618_KeysData_t TM1618_ReadKeys(TM1618_GInst_t* device);
void TM1618_PwrCtrl(TM1618_GInst_t* device, uint8_t state, uint8_t pulse_width);
void TM1618_GridSegCfg(TM1618_GInst_t* device, uint8_t grid_seg);
void TM1618_WriteRAM(TM1618_GInst_t* device);

void TM1618_WriteTo(uint8_t command, void* pData, uint8_t size);
void TM1618_ReadFrom(uint8_t command, void* pData, uint8_t size);

#endif /* TM1618_H_ */
