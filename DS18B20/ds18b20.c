/*
 *	@brief One wire mediator.
 *	Created 26.08.2020
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds18b20.h"
#include "calcCRC.h"

#pragma region PrivateFunctionPrototype


static void DS18B20_ResetDevice(DS18B20_GeneralDataInstance_typedef *device);
static void DS18B20_ReadScratchpadBytes(DS18B20_GeneralDataInstance_typedef *device, uint8_t amountOfbytes);
static void DS18B20_ConvertT(DS18B20_GeneralDataInstance_typedef *device);
static void DS18B20_ConvByteToBit(DS18B20_GeneralDataInstance_typedef *device, uint8_t byte, uint8_t offset);


#pragma endregion



#pragma region PublicFunction			  

/*
 * @brief Resets 1 wire line and checks for any devices on the bus.
 * This function must be invoked first.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
uint8_t DS18B20_ResetLine(DS18B20_GeneralDataInstance_typedef *device)
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
	device->delay(5);
	if (device->raw_data.UART_RxBuffer[0] >= ONEWIRE_READ_PRESENT_START_VALUE && 
		device->raw_data.UART_RxBuffer[0] <= ONEWIRE_READ_PRESENT_STOP_VALUE_ALT)
	{
		///Read first state.
		DS18B20_ReadScratchpadBytes(device, 9);
		device->delay(15);
		return PRESENT_IS_OK;
	}
	else return DEVICE_NOT_FOUND;	
}

/*
 * @brief Reads a unique 64–bit code.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
void DS18B20_Get_LaserRomCode(DS18B20_GeneralDataInstance_typedef *device)
{
	uint8_t rxByte;
	uint8_t *structPtr = (uint8_t *) &device->scratchpad_data.RomLaserCode;
	
	DS18B20_ResetDevice(device);
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	DS18B20_ConvByteToBit(device, ONEWIRE_READ_ROM, 0);
	///Write read time slots.
	for (uint8_t i = 0; i < 64; i++)
	{
		device->raw_data.UART_TxBuffer[8 + i] = ONEWIRE_READ_TIME_SLOT;
	}
	///Transmits commands and read time slots.
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 72);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 72);
	device->delay(15);
	///Parse raw data, converting bits to byte.
	for (uint8_t bufferCounter = 0; bufferCounter < 64; bufferCounter += 8)
	{			
		for (uint8_t bitCounter = 0; bitCounter < 8; bitCounter++)
		{
			rxByte >>= 1;
			if (device->raw_data.UART_RxBuffer[8 + bitCounter + bufferCounter] == ONEWIRE_READ_ONE_VALUE)
			{
				rxByte |= 0x80;
			}			
		}		
		*structPtr = rxByte;
		structPtr++;
		rxByte = 0;
	}
	
	device->converted_data.FamilyCode = (uint8_t)(device->scratchpad_data.RomLaserCode & 0xFF);
	device->converted_data.Serialnumber = (device->scratchpad_data.RomLaserCode & 0x00FFFFFFFFFFFF00) >> 8;	
}

/*
 * @brief Reads temperature from the sensor and converts the value according to the sensor resolution.
 * Writes the value of temperature in degrees celsius to data struct.
 *
 * @param *device : It is the instance of general data struct.
 *
 **/
void DS18B20_Get_Temperature(DS18B20_GeneralDataInstance_typedef *device, DS18B20_DataReadDepth dataDepth)
{
	//uint8_t crc8 = 0;
	
	DS18B20_ResetDevice(device);
	DS18B20_ConvertT(device);
	DS18B20_ResetDevice(device);
	DS18B20_ReadScratchpadBytes(device, dataDepth);

	//crc8  = CalculateCRC8(&device->scratchpad_data.partsTemperature.TemperatureLsb, 8);
	device->converted_data.Temperature = (device->scratchpad_data.Temperature & 0x87FF) * 0.0625f;
}

/*
 * @brief Determines the type of sensor power supply.
 * If it is logic one level, power supply mode is normal, otherwise parasite mode.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
void DS18B20_Get_PowerSupplyType(DS18B20_GeneralDataInstance_typedef *device) 
{
	DS18B20_ResetDevice(device);
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	DS18B20_ConvByteToBit(device, ONEWIRE_SKIP_ROM, 0);
	DS18B20_ConvByteToBit(device, DS18B20_READ_POWER_SUPPLY, 8);
	device->raw_data.UART_TxBuffer[16] = ONEWIRE_READ_TIME_SLOT;
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 17);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 17);
	device->delay(2);
	device->converted_data.PowerSupplyType = device->raw_data.UART_RxBuffer[16];
}

/*
 * @brief Sensor setting.
 * 
 * @param *device : It is the instance of general data struct.
 * @param tHigh : High temperature threshold.
 * @param tLow : Low temperature threshold.
 * @param resolution : Resolution of sensor's measurement.
 *
 **/
void DS18B20_Set_ThresholdAndControl(DS18B20_GeneralDataInstance_typedef *device, int8_t tHigh, int8_t tLow, 
										DS18B20_ResolutionOfMeasurement resolution)
{
	DS18B20_ResetDevice(device);
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	DS18B20_ConvByteToBit(device, ONEWIRE_SKIP_ROM, 0);
	DS18B20_ConvByteToBit(device, DS18B20_WRITE_SCRATCHPAD, 8);
	DS18B20_ConvByteToBit(device, tHigh, 16);
	DS18B20_ConvByteToBit(device, tLow, 24);
	DS18B20_ConvByteToBit(device, resolution, 32);
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 40);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 40);
	device->delay(5);
}

#pragma endregion

#pragma region PrivateFunction

/*
 * @brief Issued the reset state on the 1 wire bus.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
static void DS18B20_ResetDevice(DS18B20_GeneralDataInstance_typedef *device)
{
	device->uart_init_baud(ONEWIRE_BAUD_RATE_RESET_TXRX);
	device->raw_data.UART_TxBuffer[0] = ONEWIRE_SEND_RESET_PULSE;   //
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 1);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 1);
	device->delay(2);
}

/*
 * @brief This function transmits the converting temperature command.
 *
 * @param *device : It is the instance of general data struct.
 *
 **/
static void DS18B20_ConvertT(DS18B20_GeneralDataInstance_typedef *device)
{
	uint16_t delay = 0;	
	
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	DS18B20_ConvByteToBit(device, ONEWIRE_SKIP_ROM, 0);
	DS18B20_ConvByteToBit(device, DS18B20_CONVERT_T, 8);
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 16);
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 16);
	
	///Critical section
	if(device->converted_data.PowerSupplyType <= ONEWIRE_READ_ZERO_STOP_VALUE)
	{
		while (*device->isReceiveComplete > 0)
			{
				///Wait transmition complete.	
				__NOP();				
			}
		///Strong pull up is here.
		device->strong_pull_up(STRONG_PULL_UP_EN);
	}

	
	///Depending on  the thermometer resolution configuration bits.
	//switch(((device->scratchpad_data.ConfigurationRegister & 0x60)  >> 5))
	switch(device->scratchpad_data.ConfigurationRegister)
		{
		case MEASUREMENT_RESOLUTION_9BIT : delay = 100;
			break;
		case MEASUREMENT_RESOLUTION_10BIT :	delay = 200;
			break;
		case MEASUREMENT_RESOLUTION_11BIT : delay = 400;
			break;
		case MEASUREMENT_RESOLUTION_12BIT :	delay = 750;
			break;		
		///Default delay is MAX
		default : delay = 750;
			break;
		}
	device->delay(delay);
	if (device->converted_data.PowerSupplyType <= ONEWIRE_READ_ZERO_STOP_VALUE)
	{
		device->strong_pull_up(STRONG_PULL_UP_DIS);
	}
}

/*
 * @brief Read scratchpad memory of the DS18B20 temperature sensor.
 * 
 * @param *device : It is the instance of general data struct.
 * @param amountOfBytes : It is amount of bytes, that reading from the scratchpad memory.
 *
 **/
static void DS18B20_ReadScratchpadBytes(DS18B20_GeneralDataInstance_typedef *device, uint8_t amountOfbytes)
{
	uint8_t rxByte = 0;
	uint8_t *structPtr = &device->scratchpad_data.partsTemperature.TemperatureLsb;  
	
	if (amountOfbytes > 9) return;
	
	device->uart_init_baud(ONEWIRE_BAUD_RATE_COMMUNICATION_TXRX);
	DS18B20_ConvByteToBit(device, ONEWIRE_SKIP_ROM, 0);
	DS18B20_ConvByteToBit(device, DS18B20_READ_SCRATCHPAD, 8);
	///Write read time slots.
	for (uint8_t i = 0; i < amountOfbytes * 8; i++)
	{
		device->raw_data.UART_TxBuffer[16 + i] = ONEWIRE_READ_TIME_SLOT;
	}
	///Transmits commands and read time slots.
	device->uart_rx_data(device->raw_data.UART_RxBuffer, 16 + amountOfbytes * 8);	
	device->uart_tx_data(device->raw_data.UART_TxBuffer, 16 + amountOfbytes * 8);
	device->delay(15);
	///Parse raw data, converting bits to byte.
	for (uint8_t bufferCounter = 0; bufferCounter < amountOfbytes * 8; bufferCounter += 8)
	{			
		for (uint8_t bitCounter = 0; bitCounter < 8; bitCounter++)
		{
			rxByte >>= 1;
			if (device->raw_data.UART_RxBuffer[16 + bitCounter + bufferCounter] == ONEWIRE_READ_ONE_VALUE)
			{
				rxByte |= 0x80;
			}			
		}		
		*structPtr = rxByte;
		structPtr++;
		rxByte = 0;
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
static void DS18B20_ConvByteToBit(DS18B20_GeneralDataInstance_typedef *device, uint8_t byte, uint8_t offset)
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
}

#pragma endregion

