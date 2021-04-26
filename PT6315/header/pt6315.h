/*
 *	@brief PT6315 header.
 *	Created 04.18.21
 *	PT6315_H_
 *
 **/

#ifndef PT6315_H_				  
#define PT6315_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL ((void *)0)	
#endif //NULL

/* Invertion the LED port polarity. */
#define LED_PORT_INVERTION

/* Max and MIN values. */
#define PT6315_MIN_NUMBER_OF_SEG (uint8_t) 1
#define PT6315_MAX_NUMBER_OF_SEG (uint8_t) 24
#define PT6315_MIN_NUMBER_OF_DIG (uint8_t) 1
#define PT6315_MAX_NUMBER_OF_DIG (uint8_t) 12

/*
 * @brief Display mode setting commands. When Power is turned “ON”, the 12-digit , 16-segment modes is selected.
 *
 * The Display Mode Setting Commands determine the number of segments and grids to be used (1/4 to 1/12 duty, 
 * 16 to 24 segments). When these commands are executed, the display is forcibly turned off, the key scanning stops. 
 * A display command “ON” must be executed in order to resume display. If the same mode setting is selected, no command 
 * execution is take place, therefore, nothing happens.
 **/
typedef enum 
{
	/* Command. */
	PT6315_DISP_CMD		= 0x00,
	/* Display mode settings. */
	PT6315_4DIG_24SEG	= 0x00,
	PT6315_5DIG_23SEG	= 0x01,
	PT6315_6DIG_22SEG	= 0x02,
	PT6315_7DIG_21SEG	= 0x03,
	PT6315_8DIG_20SEG	= 0x04,
	PT6315_9DIG_19SEG	= 0x05,
	PT6315_10DIG_18SEG	= 0x06,
	PT6315_11DIG_17SEG	= 0x07,
	PT6315_12DIG_16SEG	= 0x08 //Default mode value.
	
} PT6315_DisplayModeCmd;

/*
 * @brief Display control commands.
 *
 **/
typedef enum 
{
	/* Command. */
	PT6315_CONTROL_CMD			= 0x02,
	/* Display settings. */
	PT6315_DISPLAY_OFF			= 0x00,
	PT6315_DISPLAY_ON			= 0x01,
	/* Dimming quantity settings. */
	PT6315_PULSE_WIDTH_1_16		= 0x00,
	PT6315_PULSE_WIDTH_2_16		= 0x01,
	PT6315_PULSE_WIDTH_4_16		= 0x02,
	PT6315_PULSE_WIDTH_10_16	= 0x03,
	PT6315_PULSE_WIDTH_11_16	= 0x04,
	PT6315_PULSE_WIDTH_12_16	= 0x05,
	PT6315_PULSE_WIDTH_13_16	= 0x06,
	PT6315_PULSE_WIDTH_14_16	= 0x07
	
} PT6315_DisplayControlCmd;
	
/*
 * @brief Data setting commands.
 *
 **/
typedef enum 
{
	/* Command. */
	PT6315_DATA_CMD			= 0x01,
	/* Mode settings. */
	PT6315_NORMAL_MODE		= 0x00,
	PT6315_TEST_MODE		= 0x01,
	/* Address increment mode settings (display mode). */
	PT6315_INC_ADDR			= 0x00,
	PT6315_FIX_ADDR			= 0x01,
	/* Data write and read mode settings. */
	PT6315_WRITE_DISPLAY	= 0x00,
	PT6315_WRITE_LEDPORT	= 0x01,
	PT6315_READ_KEY			= 0x02,
	PT6315_DONT_CARE		= 0x03
	
} PT6315_DataSetCmd;

/*
 * @brief Address setting commands.
 *
 **/
typedef enum 
{
	/* Command. */
	PT6315_ADDR_CMD = 0x03
		
} PT6315_AddressSetCmd;

/*
 * @brief LED display designation.
 *
 **/
typedef enum 
{
	/* LED number. */
#ifndef LED_PORT_INVERTION	
	PT6315_LED1_ON	= 0x00,
	PT6315_LED1_OFF = 0x01,
	PT6315_LED2_ON	= 0x00,
	PT6315_LED2_OFF = 0x02,
	PT6315_LED3_ON	= 0x00,
	PT6315_LED3_OFF = 0x04,
	PT6315_LED4_ON	= 0x00,
	PT6315_LED4_OFF = 0x08,
#else
	PT6315_LED1_OFF	= 0x00,
	PT6315_LED1_ON	= 0x01,
	PT6315_LED2_OFF	= 0x00,
	PT6315_LED2_ON	= 0x02,
	PT6315_LED3_OFF	= 0x00,
	PT6315_LED3_ON	= 0x04,
	PT6315_LED4_OFF	= 0x00,
	PT6315_LED4_ON	= 0x08,	
#endif
	
} PT6315_LedNumber;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 * @brief Remote control for the latch pin.
 *
 * @param pull_up : Zero is pull down, more than zero is pull up.
 *
 **/
typedef void(*latch_remote_fptr)(uint8_t pull);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit or receive data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef int8_t(*pt6315_rxtx_fptr)(uint8_t *pdata, uint16_t size);

/*
 * @brief Display mode setting commands typedef.
 *
 **/
typedef	union
{
	uint8_t DisplaySettingCmd;
	struct {
		/* Display mode settings. */
		uint8_t DISP_MODE	: 4;
		/* Not relevant. These bits are ignored. */
		uint8_t RESERVED_0	: 2;
		/* Command. */
		uint8_t CMD			: 2;
	};
		
} PT6315_DisplayModeCmd_t;

/*
 * @brief Data setting commands typedef.
 *
 **/
typedef	union
{
	uint8_t DataSettingCmd;
	struct {
		/* Data write and read mode settings. */
		uint8_t DATA_RW_MODE	: 2;
		/* Address increment mode settings. */
		uint8_t ADDR_MODE		: 1;
		/* Mode settings. */
		uint8_t SETTINGS_MODE	: 1;
		/* Not relevant. These bits are ignored. */
		uint8_t RESERVED_0		: 2;
		/* Command. */
		uint8_t CMD				: 2;
	};
		
} PT6315_DataSetCmd_t;

/*
 * @brief Display control commands typedef.
 *
 **/
typedef	union
{
	uint8_t DisplayControlCmd;
	struct {
		/* Dimming quantity settings. */
		uint8_t PULSE_WIDTH		: 3;
		/* Display settings. */
		uint8_t DISPLAY_ON_OFF	: 1;
		/* Not relevant. These bits are ignored. */
		uint8_t RESERVED_0		: 2;
		/* Command. */
		uint8_t CMD				: 2;
	};
		
} PT6315_DisplayControlCmd_t;

/*
 * @brief Address setting commands typedef.
 *
 **/
typedef	union
{
	uint8_t AddressCmd;
	struct {
		/* Address. */
		uint8_t ADDR	: 6;
		/* Command. */
		uint8_t CMD		: 2;
	};
		
} PT6315_AddressSetCmd_t;

/*
 * @brief LED display terminals typedef.
 *
 **/
typedef	union
{
	uint8_t LedControlCmd;
	struct {
		/* LED one number. */
		uint8_t LED_1		: 1;
		/* LED two number. */
		uint8_t LED_2		: 1;
		/* LED three number. */
		uint8_t LED_3		: 1;
		/* LED four number. */
		uint8_t LED_4		: 1;
		/* Not used. These bits are ignored. */
		uint8_t RESERVED_0	: 4;
	};
		
} PT6315_LedControlCmd_t;

/*
 * @brief Byte of keys type.
 *
 **/
typedef union {
	/* One of the key bytes. */
	uint8_t SGxKSx;
	struct
	{
		/* Bits state of keys. */
		uint8_t COL1_KEY1 : 1;
		uint8_t COL1_KEY2 : 1;
		uint8_t COL2_KEY1 : 1;
		uint8_t COL2_KEY2 : 1;
		uint8_t COL3_KEY1 : 1;
		uint8_t COL3_KEY2 : 1;
		uint8_t COL4_KEY1 : 1;
		uint8_t COL4_KEY2 : 1;					
	};
	
} PT6315_Key_t;

/*
 * @brief Keys state.
 *
 **/
typedef struct
{
	PT6315_Key_t SG1_4KS1_4Keys;
	PT6315_Key_t SG5_8KS5_8Keys;
	PT6315_Key_t SG9_12KS9_12Keys;
	PT6315_Key_t SG13_16KS13_16Keys;
	
} PT6315_KeysMatrix_t;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	/* Keys state. */
	PT6315_KeysMatrix_t keys;
	//Pointers for the rx, tx and delay functions.
	delay_fptr delay_fptr;
	latch_remote_fptr latch_control_fptr;
	pt6315_rxtx_fptr spi_tx_data_fptr;
	pt6315_rxtx_fptr spi_rx_data_fptr;
	
} PT6315_GInst_t;

/* Public function prototypes. */
void PT6315_SetResetRAM(PT6315_GInst_t device, uint8_t segment, uint8_t dig, uint8_t state);
void PT6315_ReadKeys(PT6315_GInst_t device);
void PT6315_LedEn(PT6315_GInst_t device, uint8_t number, uint8_t state);
void PT6315_WriteRAM(PT6315_GInst_t device);


#endif /* PT6315_H_ */