/*
 *	@brief UART to one wire.
 *	Created 03.08.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "one_wire_uart.h"
#include "stdio.h"

/* Private function prototypes. */
static void UART_1WireConvByteToBit(UART_1WireGInst_t *device, uint8_t byte, uint8_t offset);
static void UART_1WireRxTxComplete(UART_1WireGInst_t *device);

/* Public functions. */

/*
 * @brief Issued the reset state on the 1 wire bus.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
uint8_t UART_1WireReset(UART_1WireGInst_t *device)
{	
	if (device == NULL || device->uart_init_baud == NULL) for (;;) { /*Device's data struct instance is not exist*/ }
	if (device->uart_rx_data == NULL || device->uart_tx_data == NULL || device->delay == NULL) 
		for (;;)
		{
			/*Device's data struct instance is not exist*/ 
		}
	
	device->uart_init_baud(ONEWIRE_BAUD_RATE_RESET_TXRX);
	device->raw_data.UART_TxBuffer[0] = ONEWIRE_SEND_RESET_PULSE;
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 1);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 1);
	///1 ms delay
	device->delay(2);
	
	if (device->raw_data.UART_RxBuffer[0] >= ONEWIRE_READ_PRESENT_START_VALUE && 
	device->raw_data.UART_RxBuffer[0] <= ONEWIRE_READ_PRESENT_STOP_VALUE_ALT)
	{
		return ONE_WIRE_UART_PRESENCE_IS_OK;
	}
	else return ONE_WIRE_UART_DEVICE_NOT_FOUND;	
}

/*
 * @brief Transmit data to the 1-Wire bus.
 * 
 * param *device : pointer to the instance of the DS2484_GeneralDataInstance_t.
 * 
 **/
void UART_1WireWriteData(UART_1WireGInst_t *device, uint8_t *data, uint8_t size)
{	
	device->raw_data.UART_TxRxOffset = 0;
	
	for (uint8_t i = size; i > 0; i--) {
		UART_1WireConvByteToBit(device, *data, device->raw_data.UART_TxRxOffset);
		data++;
	}
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	device->uart_rx_data(device->raw_data.UART_RxBuffer, device->raw_data.UART_TxRxOffset);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, device->raw_data.UART_TxRxOffset);
	device->delay(20);
} 

/*
 * @brief Receive data from the 1-Wire bus.
 * 
 * param *device : pointer to the instance of the DS2484_GeneralDataInstance_t.
 * 
 **/
void UART_1WireReadData(UART_1WireGInst_t *device, uint8_t* data, uint8_t size)
{	
	uint8_t rxByte;
	device->raw_data.UART_TxRxOffset = 0;
	
	///Write read time slots.
	for(uint8_t i = 0 ; i < size; i++)
	{
		UART_1WireConvByteToBit(device, ONEWIRE_READ_TIME_SLOT, device->raw_data.UART_TxRxOffset);
	}	
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	device->uart_rx_data(device->raw_data.UART_RxBuffer, device->raw_data.UART_TxRxOffset);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, device->raw_data.UART_TxRxOffset);
	device->delay(20);
	///Parse raw data, converting bits to byte.
	for(uint8_t bufferCounter = 0 ; bufferCounter < device->raw_data.UART_TxRxOffset; bufferCounter += 8)
	{			
		for (uint8_t bitCounter = 0; bitCounter < 8; bitCounter++)
		{
			rxByte >>= 1;
			if (device->raw_data.UART_RxBuffer[bitCounter + bufferCounter] == ONEWIRE_READ_ONE_VALUE)
			{
				rxByte |= 0x80;
			}			
		}		
		*data = rxByte;
		data++;
		rxByte = 0;
	}
} 

/*
 * @brief Receive data from the 1-Wire bus.
 * 
 * param *device : pointer to the instance of the DS2484_GeneralDataInstance_t.
 * 
 **/
uint8_t UART_1WireReadBit(UART_1WireGInst_t *device)
{	
	uint8_t rxByte;
	
	///Write read time slots.
	device->raw_data.UART_TxBuffer[0] = ONEWIRE_READ_TIME_SLOT;
	/* Transmit  one read time slot. */
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 1);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 1);
	device->delay(5);
	
	return device->raw_data.UART_RxBuffer[0];
} 


/* Private functions. */

/*
 * @brief Wait data transmition complete.
 *
 **/
static void UART_1WireRxTxComplete(UART_1WireGInst_t *device)
{
	/* 72MHz, t = 13.8nS). */
	uint32_t timeout = 1000000;
	
	while (*device->isReceiveComplete > 0)
	{
		///Wait a transmition complete.	
		timeout--;
		if (!timeout)
		{
			break;
		}
	}
}

/*
 * @brief Convertion byte to bit sequence and write it to the device buffer with offset.
 *
 * @param *device : It is the instance of general data struct.
 * @param byte : It is byte, that converting to the bit sequence.
 * @param offset : It is offset within the transmit buffer. Its size is a multiple of eight.
 *
 **/
static void UART_1WireConvByteToBit(UART_1WireGInst_t *device, uint8_t byte, uint8_t offset)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (byte & 0x01)
		{
			device->raw_data.UART_TxBuffer[i + offset] = ONEWIRE_WRITE_ONE_VALUE;			
		}
		else {
			device->raw_data.UART_TxBuffer[i + offset] = ONEWIRE_WRITE_ZERO_VALUE;
		}
		byte >>= 1;
	}
	device->raw_data.UART_TxRxOffset += 8;
}


