/*
 *	@brief One wire mediator.
 *	Created 03.08.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds18b20.h"
//#include "calcCRC.h"
#include "stdio.h"

#include "ds2484.h"
#include "one_wire_uart.h"

/* Private function prototypes. */
static void DS18B20_SendSkipMatchRom(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd);
static void DS18B20_ConvertT(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd);

/*  */
static int8_t UART_TxData(uint8_t *buffer, uint16_t size);
static int8_t UART_RxData(uint8_t *buffer, uint16_t size);

/* Public variables. */


/* Private variables. */


/*
 * @brief Resets 1 wire line and checks availability for any devices on the bus.
 * This function must be invoked first.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
uint8_t DS18B20_DeviceDetect(DS18B20_GInst_t *device)
{
	if (device == NULL) for (;;) { /*Device's data struct instance is not exist*/ }
	if (device->rx_data == NULL || device->tx_data == NULL || device->delay == NULL) 
		for (;;)
		{
			/*Device's data struct instance is not exist*/ 
		}	
	return device->reset_bus();	
}

/*
 * @brief Sets a unique 64–bit ROM code.
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_code : User known 64–bit ROM code.
 *
 **/
void DS18B20_Set_LaserRomCode(DS18B20_GInst_t *device, uint64_t rom_code)
{	
	device->converted_data.RomLaserCode = rom_code;
}

/*
 * @brief Reads a unique 64–bit code.
 * 
 * @param *device : It is the instance of general data struct.
 * @param retval : 64–bit code ROM code.
 *
 **/
uint64_t DS18B20_Get_LaserRomCode(DS18B20_GInst_t *device)
{
	uint8_t tx_command = ONEWIRE_READ_ROM;
	
	device->reset_bus();
	device->tx_data(&tx_command, 1);
	device->rx_data((uint8_t *)&device->scratchpad_data.RomLaserCode, 8);
	
	device->converted_data.RomLaserCode = device->scratchpad_data.RomLaserCode;
	return device->converted_data.RomLaserCode;
}

/*
 * @brief Reads temperature from the sensor and converts the value according to the sensor resolution.
 * Writes the value of temperature in degrees celsius to data struct.
 *
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 * @param dataDepth : This means how many bytes of data you'll read.
 *
 **/
void DS18B20_Get_Temperature(DS18B20_GInst_t *device,
	ONE_WIRE_CommandSet rom_cmd,
	DS18B20_DataReadDepth dataDepth)
{
	//uint8_t crc8 = 0;
	float temperature = 0;

	DS18B20_ConvertT(device, rom_cmd);
	DS18B20_ReadScratchpadBytes(device, rom_cmd, dataDepth);
	if ((device->scratchpad_data.Temperature & 0xF800) == 0)
		device->converted_data.Temperature = (device->scratchpad_data.Temperature & 0x87FF) * 0.0625f;
	else device->converted_data.Temperature = -((~device->scratchpad_data.Temperature + 1) * 0.0625f);
	//crc8  = CalculateCRC8(&device->scratchpad_data.partsTemperature.TemperatureLsb, 8);
}

/*
 * @brief Reads scratchpad memory of the DS18B20 temperature sensor.
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 * @param amountOfBytes : It is amount of bytes, that reading from the scratchpad memory.
 *
 **/
void DS18B20_ReadScratchpadBytes(DS18B20_GInst_t *device,
	ONE_WIRE_CommandSet rom_cmd,
	DS18B20_DataReadDepth amountOfbytes)
{
	uint8_t command = DS18B20_READ_SCRATCHPAD; 
	
	if (amountOfbytes > 9) return;
	/* Reset 1_Wire bus. */
	device->reset_bus();	
	//Select ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM command to write to the buffer.
	DS18B20_SendSkipMatchRom(device, rom_cmd);
	/* Write command. */
	device->tx_data(&command, 1);
	//device->delay(5);
	/* Read data from the sensor. */
	device->rx_data((uint8_t*)&device->scratchpad_data.Temperature, amountOfbytes);
	//device->delay(15);	
}

/*
 * @brief Saves scratchpad data (Tl, Th, Configuration register) to the internal EEPROM memory. 
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 *
 **/
void DS18B20_CopyScratchpadToEeprom(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd)
{
	uint8_t command = DS18B20_COPY_SCRATCHPAD;
	uint8_t  status = 1;
	
	/* Reset 1_Wire bus. */
	device->reset_bus();
	//Write skip or match rom command to the buffer.
	DS18B20_SendSkipMatchRom(device, rom_cmd);	
	/* Write command. */
	status = device->tx_data(&command, 1);
	///Remote the strong pull-up.
	if(device->converted_data.PowerSupplyType == 0 && status == 0) {
		device->strong_pull_up(DS18B20_STRONG_PULL_UP_EN);
	}
	//Wait 10mS for writing data to eeprom memory.
	device->delay(10);
	device->strong_pull_up(DS18B20_STRONG_PULL_UP_DIS);
}

/*
 * @brief Reads scratchpad data (Tl, Th, Configuration register) from the internal EEPROM to RAM memory. 
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 *
 **/
void DS18B20_RecallE2(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd)
{
	uint8_t command = DS18B20_RECALL_E2;
	
	/* Reset 1_Wire bus. */
	device->reset_bus();
	//Write skip or match rom command to the buffer.
	DS18B20_SendSkipMatchRom(device, rom_cmd);	
	/* Write command. */
	device->tx_data(&command, 1);
	//device->delay(5);
}

/*
 * @brief Determines the type of sensor power supply.
 * If it is logic one level, power supply mode is normal, otherwise parasite mode.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
void DS18B20_Get_PowerSupplyType(DS18B20_GInst_t *device) 
{
	uint8_t data[2] = { ONEWIRE_SKIP_ROM, DS18B20_READ_POWER_SUPPLY };
	
	/* Reset 1_Wire bus. */
	device->reset_bus();
	/* Send data. */
	device->tx_data(&data[0], 2);
	//device->delay(5);
	device->rx_data((uint8_t *)&device->converted_data.PowerSupplyType, 1);
	//device->delay(1);
}

/*
 * @brief Sets sensor setting (Th, Tl, sensor resolution).
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 * @param tHigh : High temperature threshold.
 * @param tLow : Low temperature threshold.
 * @param resolution : Resolution of sensor's measurement.
 *
 **/
void DS18B20_Set_ThresholdAndControl(DS18B20_GInst_t *device,
	ONE_WIRE_CommandSet rom_cmd, 
	int8_t tHigh,
	int8_t tLow,
	DS18B20_ResolutionOfMeasurement resolution)
{
	uint8_t data[4] = { DS18B20_WRITE_SCRATCHPAD,
						tHigh, tLow, resolution};
	/* Reset 1_Wire bus. */
	device->reset_bus();
	//Write skip or match rom command to the buffer.
	DS18B20_SendSkipMatchRom(device, rom_cmd);	
	/* Write data and command. */
	device->tx_data(&data[0], 4);
	//device->delay(20);
}

/*
 * @brief Search active devices on the bus and reads its ROM code.
 * 
 * @param *device : It is the instance of general data struct.
 *
 **/
void DS18B20_SearchDevice(DS18B20_GInst_t *device)
{

}

/* Private functions. */

/*
 * @brief This function transmits the converting temperature command.
 *
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 *
 **/
static void DS18B20_ConvertT(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd)
{
	uint16_t delay, status = 1;		
	uint8_t command = DS18B20_CONVERT_T;
	
	device->reset_bus();
	DS18B20_SendSkipMatchRom(device, rom_cmd);
	status = device->tx_data(&command, 1);
	///Remote the strong pull-up.
	if (device->converted_data.PowerSupplyType == 0 && status == 0) {
		device->strong_pull_up(DS18B20_STRONG_PULL_UP_EN);
	}		
	
	///Depending on  the thermometer resolution configuration bits.
	switch(device->scratchpad_data.ConfigurationRegister)
	{
	case DS18B20_MEASUREMENT_RESOLUTION_9BIT : delay = 100;
		break;
	case DS18B20_MEASUREMENT_RESOLUTION_10BIT :	delay = 200;
		break;
	case DS18B20_MEASUREMENT_RESOLUTION_11BIT : delay = 400;
		break;
	case DS18B20_MEASUREMENT_RESOLUTION_12BIT :	delay = 750;
		break;		
		///Default delay is MAX
		default : delay = 750;
		break;
	}
	device->delay(delay);
	device->strong_pull_up(DS18B20_STRONG_PULL_UP_DIS);
}

/*
 * @brief Selecting ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM command to write to the buffer.
 * This function select one of two command (ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM) and write 
 * it to the Tx buffer.
 * 
 * @param *device : It is the instance of general data struct.
 * @param rom_cmd : ONEWIRE_SKIP_ROM or ONEWIRE_MATCH_ROM commands for addressing the device or not.
 *
 **/
static void DS18B20_SendSkipMatchRom(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd)
{
	struct __attribute__((aligned(1), packed)) {
		uint8_t command;
		uint64_t rom_code;		
	} data;
		
	data.command = rom_cmd;
	data.rom_code = device->scratchpad_data.RomLaserCode;
	
	if (rom_cmd == ONEWIRE_SKIP_ROM) {
		device->tx_data((uint8_t *)&data, 1); 
		//device->delay(5);
	} else {
		device->tx_data((uint8_t *)&data, sizeof(data));
		//device->delay(15);
	}
}

/* Second layer functions. */

/*
 * @brief Reset 1-Wire bus.
 *
 **/
uint8_t DS18B20_1WireReset(void)
{
	return 0;
}

/*
 * @brief Strong pull-up enable/disable.
 * 
 * param state : 
 *
 **/
void DS18B20_SPU(uint8_t state)
{
	
}

/*
 * @brief Transmit data.
 *
 **/
uint8_t DS18B20_TxData(uint8_t *buffer, uint16_t size)
{
	return 0;
}

/*
 * @brief Receive data.
 *
 **/
uint8_t DS18B20_RxData(uint8_t *buffer, uint16_t size)
{
	
	return 0;
}

/*
 * @brief User delay.
 *
 **/
void DS18B20_Delay(uint32_t period)
{
	HAL_Delay(period);
}


/* Hardware dependent functions. */

/*
 * @brief UART transmit data.
 *
 **/
static int8_t UART_TxData(uint8_t *buffer, uint16_t size)
{
	return 0;
}

/*
 * @brief UART receive data.
 *
 **/
static int8_t UART_RxData(uint8_t *buffer, uint16_t size)
{
	return 0;
}


//static void printObjectData(DS18B20_GeneralDataInstance_typedef *device, uint8_t *char_buffer)
//{
//	uint8_t messageLength;
//	
//	messageLength = sprintf((char*)char_buffer,
//		"Serial number %0*X%0*X%0*X%0*X%0*X%0*X%0*X%0*Xh\r\n", 
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[7],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[6],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[5],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[4],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[3],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[2],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[1],
//		2,
//		device->converted_data.partsRomLaserCode.arrayRomLaserCode[0]);
//	HAL_UART_Transmit_IT(&huart2, char_buffer, messageLength);
//	HAL_Delay(10);	
//	
//	messageLength = sprintf((char*)char_buffer, "Th =  %0*d\r\n", 2, device->scratchpad_data.ThRegister);
//	HAL_UART_Transmit_IT(&huart2, char_buffer, messageLength);
//	HAL_Delay(10);
//	messageLength = sprintf((char*)char_buffer, "Tl =  %0*d\r\n", 2, device->scratchpad_data.TlRegister);
//	HAL_UART_Transmit_IT(&huart2, char_buffer, messageLength);
//	HAL_Delay(10);
//	messageLength = sprintf((char*)char_buffer, "Cfg =  %0*X\r\n", 2, device->scratchpad_data.ConfigurationRegister);
//	HAL_UART_Transmit_IT(&huart2, char_buffer, messageLength);
//	HAL_Delay(10);
//}
