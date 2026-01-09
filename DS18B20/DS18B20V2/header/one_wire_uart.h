/*
 *	@brief UART to one wire driver.
 *	Created 03.08.21
 *	ONE_WIRE_UART_H_
 *
 **/

//#pragma once
#ifndef ONE_WIRE_UART_H_				  
#define ONE_WIRE_UART_H_

#include "stm32f4xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

#define UART_GPIO_SPU GPIO_PIN_9

/*
 * @brief State of line  after reseting line. 
 *
 **/
typedef enum
{
	ONE_WIRE_UART_PRESENCE_IS_OK	= 1,
	ONE_WIRE_UART_DEVICE_NOT_FOUND	= 0
	
} ONE_WIRE_UART_StateOfLine;

/*
 * @brief Parasite power supply pin remote control.
 *
 **/
typedef enum
{
	ONE_WIRE_UART_STRONG_PULL_UP_EN	 = 1,
	ONE_WIRE_UART_STRONG_PULL_UP_DIS = 0
	
} ONE_WIRE_UART_StrongPullUp;

/*
 * @brief TX and RX data const.
 *
 **/
typedef enum
{
	///***INITIALISATION SEQUENCE***/
	///Baud rate for transmit data through UART.
	ONEWIRE_BAUD_RATE_RESET_TXRX			= 9600,
	///Reset pulse duration
	ONEWIRE_SEND_RESET_PULSE				= 0xF0,
	///Value of present state of 1-wire bus, his start conditon.
	ONEWIRE_READ_PRESENT_START_VALUE		= 0x10,
	///Alternative start present value. What???
	//ONEWIRE_READ_PRESENT_START_VALUE_ALT	= 0xE0,
	///Value of present state of 1-wire bus, his stop conditon.
	ONEWIRE_READ_PRESENT_STOP_VALUE			= 0x90,
	ONEWIRE_READ_PRESENT_STOP_VALUE_ALT		= 0xE0,
	
	///***COMMUNICATION SEQUENCE***/
	///Baud rate for transmit data through UART.
	ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX	= 115200,
	///Value of logic zero, start reading condition.
	ONEWIRE_READ_ZERO_START_VALUE			= 0x00,
	///Value of logic zero, stop reading condition.
	ONEWIRE_READ_ZERO_STOP_VALUE			= 0xFE,
	///Value of logic one, reading.
	ONEWIRE_READ_ONE_VALUE					= 0xFF,
	///Value of logic zero, writing.
	ONEWIRE_WRITE_ZERO_VALUE				= 0x00,
	///Value of logic one, writing.
	ONEWIRE_WRITE_ONE_VALUE					= 0xFF,	
	///	
	ONEWIRE_READ_TIME_SLOT					= 0xFF,
	
} ONEWIRE_txrx_config_value;

/*
 * @brief Init baud rate function typedef pointer.
 *
 * @param baud : Baud rate, exsample 9600, 115200.
 *
 **/
typedef void(*usart_init)(uint32_t baud);

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*onewire_delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit or receive data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef int8_t (*usart_txrx_data_fptr)(uint8_t *buffer, uint16_t size);

/*
 *	@brief Raw data from the DS18B20 temperature sensor. 
 *	
 * ///Is is data, that have been received from UART. Every bytes means one bit of data.  
 *
 **/
typedef struct
{
	///Offset for filling the buffer, must be initialised 0.
	uint8_t UART_TxRxOffset;
	uint8_t UART_TxBuffer[156];
	uint8_t UART_RxBuffer[156];
	
} ONE_WIRE_UART_RawData_t;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointer to the counter of left data.
	uint16_t *isReceiveComplete;
	///Input raw data.
	ONE_WIRE_UART_RawData_t raw_data;
	//Pointers for the rx, tx delay functions.
	usart_txrx_data_fptr uart_tx_data;
	usart_txrx_data_fptr uart_rx_data;
	onewire_delay_fptr delay;
	
} UART_1WireGInst_t;

/*
 *	@brief Public function prototypes.
 *
 **/
 void UART_1WireSPU(uint8_t pull_up);
uint8_t UART_1WireReset(UART_1WireGInst_t *device);
void UART_1WireWriteData(UART_1WireGInst_t *device, uint8_t *data, uint8_t size);
void UART_1WireReadData(UART_1WireGInst_t *device, uint8_t* data, uint8_t size);
uint8_t UART_1WireReadBit(UART_1WireGInst_t *device);


#endif /* ONE_WIRE_UART_H_ */
