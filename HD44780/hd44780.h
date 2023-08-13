/*
 * @brief HD44780 driver.
 * Created 1.12.2020
 * 
 **/

#ifndef HD44780_H
#define HD44780_H	

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL
	
/* Hardware depending part. */

/* Define your pin polarity */
//#define HD44780_INVERTED_RS_PIN
//#define HD44780_INVERTED_RW_PIN
#define HD44780_INVERTED_E_PIN

/* Definations of the ports. */
#define HD44780_RS_PORT				GPIOD
#define HD44780_RW_PORT				GPIOD
#define HD44780_E_PORT				GPIOD

#define HD44780_DATA_PORT			GPIOA
#define HD44780_MOST4B_DATA_PORT	GPIOE
#define HD44780_LEAST4B_DATA_PORT	GPIOD
#define HD44780_BUSY_PIN_PORT		GPIOE

/*Read/write definations.*/
#define HD44780_BUSY_PIN			GPIO_PIN_10

#ifndef HD44780_INVERTED_RS_PIN
	/* Non invertion mode. */
	#define HD44780_RS_DATA_REG		GPIO_BSRR_BS13
	#define HD44780_RS_INSTR_REG	GPIO_BSRR_BR13
#else
	/* Invertion mode. */
	#define HD44780_RS_DATA_REG		GPIO_BSRR_BR13
	#define HD44780_RS_INSTR_REG	GPIO_BSRR_BS13
#endif // HD44780_INVERTED_RS_PIN

#ifndef HD44780_INVERTED_RW_PIN
	/* Non invertion mode. */
	#define HD44780_RW_READ			GPIO_BSRR_BS5
	#define HD44780_RW_WRITE		GPIO_BSRR_BR5
#else
	/* Invertion mode. */
	#define HD44780_RW_READ			GPIO_BSRR_BR5
	#define HD44780_RW_WRITE		GPIO_BSRR_BS5
#endif // HD44780_INVERTED_RW_PIN

#ifndef HD44780_INVERTED_E_PIN
	/* Non invertion mode. */
	#define HD44780_E_CLOCK_UP		GPIO_BSRR_BS7
	#define HD44780_E_CLOCK_DOWN	GPIO_BSRR_BR7
#else
	/* Invertion mode. */
	#define HD44780_E_CLOCK_UP		GPIO_BSRR_BR7
	#define HD44780_E_CLOCK_DOWN	GPIO_BSRR_BS7
#endif // HD44780_INVERTED_E_PIN

/* 
 * @brief General part.
 *
 **/
#define HD44780_BUS_TYPE_4BITS		0
#define HD44780_BUS_TYPE_8BITS		1


/*
 * @brief Display commands.
 *
 **/
typedef enum
{
	/*Cleare display, DDRAM address set to zero.*/
	HD44780_CMD_CLEAR_SCREEN		= 0x01,
	
	/*Addres set to DDRAM, address counter is zero, Also returns display from being shifted to original position.*/
	/*DDRAM contents remain unchanged.*/
	HD44780_CMD_RETURN_HOME			= 0x02,
	
	/*Sets cursor move direction and specifies display shift. These operations are performed during data write and read*/
	HD44780_CMD_INC_MODE			= 0x02 | 0x04,
	HD44780_CMD_DEC_MODE			= 0x00 | 0x04,
	HD44780_CMD_DISPLAY_SHIFT_EN	= 0x01,
	HD44780_CMD_DISPLAY_SHIFT_DIS	= 0x00,
	
	/*Sets entire display (D) on/off, cursor on/off (C), and blinking of cursor position character (B).*/
	HD44780_CMD_DISPLAY_ON			= 0x04 | 0x08,
	HD44780_CMD_DISPLAY_OFF			= 0x00 | 0x08,
	HD44780_CMD_CURSOR_ON			= 0x02,
	HD44780_CMD_CURSOR_OFF			= 0x00,
	HD44780_CMD_CURSOR_BLINKING_ON	= 0x01,
	HD44780_CMD_CURSOR_BLINKING_OFF = 0x00,
	
	/*Moves cursor and shifts display without changing DDRAM contents.*/
	HD44780_CMD_CURSOR_MOVE			= 0x00 | 0x10,
	HD44780_CMD_DISPLAY_SHIFT		= 0x08 | 0x10,
	HD44780_CMD_SHIFT_TO_LEFT		= 0x00,
	HD44780_CMD_SHIFT_TO_RIGHT		= 0x04,
	
	/*Sets interface data length (DL), number of display lines (N), and character font (F).*/
	HD44780_CMD_DATA_LENGTH_4BITS	= 0x00 | 0x20,
	HD44780_CMD_DATA_LENGTH_8BITS	= 0x10 | 0x20,
	HD44780_CMD_DISPLAY_1LINE		= 0x00,
	HD44780_CMD_DISPLAY_2LINE		= 0x08,
	HD44780_CMD_DISPLAY_FONT5_8		= 0x00,
	HD44780_CMD_DISPLAY_FONT_5_10	= 0x04,
		
	/*Sets CGRAM address. CGRAM data is sent and received after this setting.*/
	HD44780_CMD_SET_CGRAM_ADDR		= 0x40,
	
	/*Sets DDRAM address. DDRAM data is sent and received after this setting.*/
	HD44780_CMD_SET_DDRAM_ADDR		= 0x80,
		
	/*Line addresses*/
	HD44780_CMD_SET_LINE1			= (0x00 | HD44780_CMD_SET_DDRAM_ADDR),
	HD44780_CMD_SET_LINE2			= (0x40 | HD44780_CMD_SET_DDRAM_ADDR),
	HD44780_CMD_SET_LINE3			= (0x14 | HD44780_CMD_SET_DDRAM_ADDR),
	HD44780_CMD_SET_LINE4			= (0x54 | HD44780_CMD_SET_DDRAM_ADDR)
		
} HD44780_Cmd;

/*
 * @brief Cursor position.
 *
 **/
typedef enum
{
	HD44780_CURSOR_MOVE_LEFT,
	HD44780_CURSOR_MOVE_RIGHT
} HD44780_CursorMoveDirection;

/*
 * @brief Calendar language. 
 *
 **/
typedef enum
{
	LANGUAGE_EN = 0,
	LANGUAGE_RU = 1
} HD44780_CalendarLanguage;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 * @brief Tx function typedef pointer. 
 *	
 * @param bute : Just data byte. 
 *
 **/
typedef void(*tx_data_fptr)(const uint8_t byte);

/*
 *	@brief Read busy bit. 
 *
 **/
typedef void(*read_busy_fptr)(void);


/*
 * @brief Video buffer data struct.
 *
 **/
typedef struct 
{
	uint8_t Line[4][20];
	
} HD44780_DataBuffer_t;

/*
 * @brief General data struct.
 *
 **/
typedef struct 
{
	//uint8_t bus_type;
	HD44780_DataBuffer_t video_buffer;	
	delay_fptr user_delay;
	read_busy_fptr user_check_busy;
	tx_data_fptr user_write_data;	
	tx_data_fptr user_write_command;
	
} HD44780_Inst_t;

/*
 * @brief Public function prototype.
 *
 **/

void HD44780_InitScreen(HD44780_Inst_t* device, const uint8_t bus_type);
void HD44780_TestScreen(HD44780_Inst_t* device, uint8_t number_of_line, uint8_t number_of_symbols);
void HD44780_CursorMove(HD44780_Inst_t* device, HD44780_CursorMoveDirection direction, uint8_t offset);


/*Sends data to screen.*/
void HD44780_ClearScreen(HD44780_Inst_t* device);
void HD44780_ClearScreenPosition(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t size);
void HD44780_WriteChar(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t code);
void HD44780_SendMessage(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t* message, uint8_t size);
void HD44780_SendNumberF2_1(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float in_float);
void HD44780_SendTime(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t hour, uint8_t min, uint8_t sec, uint8_t delimeter);
void HD44780_SendDate(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t date, uint8_t month, uint8_t year, uint8_t year_prefix, uint8_t delimeter);
void HD44780_SendTemperature(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float temp);
void HD44780_SendHumidity(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float hum);
void HD44780_SendPressure(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float press);
void HD44780_SendDay(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t day, HD44780_CalendarLanguage language);
void HD44780_SendMonth(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t month, HD44780_CalendarLanguage language);
	

/*Hardware depending functions*/
void HD44780_WriteData(const uint8_t data);
void HD44780_WriteCommand(const uint8_t command);
void HD44780_CheckBusy(void);

#endif /* HD44780_H */
