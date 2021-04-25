/*
 *	@brief DS18B20 driver.
 *	Created 03.08.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds18b20.h"
//#include "calcCRC.h"
#include "stdio.h"

/* Define your favorite hardware interface :) */
//#define UART_COMMUNICATION
#define I2C_COMMUNICATION

/* Select your I2C interface. */
#ifdef I2C_COMMUNICATION
	#define DS2484
	//#define DS2482
#endif

/* Selector of interfaces. */
#ifdef UART_COMMUNICATION
#include "one_wire_uart.h"
/* Public variables. */
extern UART_HandleTypeDef huart1;
/* Private function prototypes. */
static void UART_ParasitePinRemote(uint8_t pull_up);
void UART_Init_Baud(uint32_t baud);
static int8_t UART_TxData(uint8_t *buffer, uint16_t size);
static int8_t UART_RxData(uint8_t *buffer, uint16_t size);
/* Private variables. */
static UART_1WireGInst_t uart_ginst = { 
		
	.strong_pull_up = UART_ParasitePinRemote,
	.isReceiveComplete = (uint16_t *) &huart1.TxXferCount,
	.uart_init_baud = UART_Init_Baud,	
	.uart_rx_data = UART_RxData,
	.uart_tx_data = UART_TxData,
	.delay = HAL_Delay
};

#else 
#ifdef I2C_COMMUNICATION
/* Public variables. */
extern I2C_HandleTypeDef hi2c1;
static int8_t I2C_TxData(uint8_t address, uint8_t *buffer, uint16_t size);
static int8_t I2C_RxData(uint8_t address, uint8_t *buffer, uint16_t size);
/* Private variables. */
#ifdef DS2484

#include "ds2484.h"
static DS2484_GInst_t ds2484_ginst = { 
		
	.isReceiveComplete = (uint16_t *) &hi2c1.XferCount,
	.delay = HAL_Delay,
	.i2c_tx_data = I2C_TxData,
	.i2c_rx_data = I2C_RxData,
};

#else
#ifdef DS2482

#include "ds2482.h"
static DS2482_GInst_t ds2482_ginst = { 
		
	.isReceiveComplete = (uint16_t *) &hi2c1.XferCount,
	.delay = HAL_Delay,
	.i2c_tx_data = I2C_TxData,
	.i2c_rx_data = I2C_RxData,
};	

#endif 
#endif
#endif
#endif


/* Private function prototypes. */
static void DS18B20_SendSkipMatchRom(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd);
static void DS18B20_ConvertT(DS18B20_GInst_t *device, ONE_WIRE_CommandSet rom_cmd);



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
uint64_t DS18B20_Get_LaserRomCode (DS18B20_GInst_t *device)
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
 * @brief User delay.
 *
 **/
void DS18B20_Delay(uint32_t period)
{
	HAL_Delay(period);
}


/*
 * @brief Reset 1-Wire bus.
 *
 **/
uint8_t DS18B20_1WireReset(void)
{	
#ifdef DS2484
	return DS2484_1WireReset(&ds2484_ginst);
#else
#ifdef DS2482
	return 0;
	//return DS2482_1WireReset(&ds2484_ginst);
#else
#ifdef UART_COMMUNICATION		 
	return UART_1WireReset(&uart_ginst);	
#endif /* DS2484 */
#endif /* DS2482 */
#endif /* UART_COMMUNICATION */
}

/*
 * @brief Strong pull-up enable/disable.
 * 
 * param state : if 1 - the strong pull-up is enable, 0 is disable. See the DS2484_CFGReg_t typedef for more detaled.
 *
 **/
void DS18B20_SPU(uint8_t state)
{
#ifdef DS2484
	DS2484_1WireSPU(&ds2484_ginst, state);
#else
#ifdef DS2482
	DS2482_1WireSPU(&ds2482_ginst, state);
#else
#ifdef UART_COMMUNICATION		 
	UART_1WireSPU(uart_ginst);	
#endif /* DS2484 */
#endif /* DS2482 */
#endif /* UART_COMMUNICATION */	
}

/*
 * @brief Transmit data.
 *
 **/
uint8_t DS18B20_TxData(uint8_t *buffer, uint16_t size)
{
#ifdef DS2484
	return DS2484_1WireWriteData(&ds2484_ginst, buffer, size);
#else
#ifdef DS2482
	return DS2482_1WireWriteData(&ds2482_ginst, buffer, size);
#else
#ifdef UART_COMMUNICATION		 
	//return UART_1WireWriteData(&uart_ginst, buffer, size);	
#endif /* DS2484 */
#endif /* DS2482 */
#endif /* UART_COMMUNICATION */		
}

/*
 * @brief Receive data.
 *
 **/
uint8_t DS18B20_RxData(uint8_t *buffer, uint16_t size)
{
#ifdef DS2484
	DS2484_1WireReadData(&ds2484_ginst, buffer, size);
	return 0;
#else
#ifdef DS2482	
	DS2482_1WireReadData(&ds2482_ginst, buffer, size);
	return 0;
#else
#ifdef UART_COMMUNICATION		 
	//return UART_1WireReadData(&uart_ginst, buffer, size);	
	return 0;
#endif /* DS2484 */
#endif /* DS2482 */
#endif /* UART_COMMUNICATION */		
}

/* Hardware dependent functions. */

#ifdef I2C_COMMUNICATION
/*
 * @brief I2C transmit data.
 *
 **/
static int8_t I2C_TxData(uint8_t address, uint8_t *buffer, uint16_t size)
{
	return HAL_I2C_Master_Transmit(&hi2c1, address, buffer, size, 10);
	//HAL_I2C_Master_Transmit_IT(&hi2c1, address, buffer, size);
	//HAL_I2C_Master_Transmit_DMA(&hi2c1, address, buffer, size);
}

/*
 * @brief I2C receive data.
 *
 **/
static int8_t I2C_RxData(uint8_t address, uint8_t *buffer, uint16_t size)
{
	return HAL_I2C_Master_Receive(&hi2c1, address, buffer, size, 10);
	//HAL_I2C_Master_Receive_IT(&hi2c1, address, buffer, size);
	//HAL_I2C_Master_Receive_DMA(&hi2c1, address, buffer, size);
}

#else
#ifdef UART_COMMUNICATION

/*
 * @brief DS18B20 parasitic power supply control.
 *
 **/
static void UART_ParasitePinRemote(uint8_t pull_up)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	if (pull_up > 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	}
	else
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	}
	
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
 * brief Reconfig UART baud rate function.
 *
 **/
void UART_Init_Baud(uint32_t baud)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = baud;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		for (;;) ;
		//Error_Handler();
	}
}

/*
 * @brief UART transmit data.
 *
 **/
static int8_t UART_TxData(uint8_t *buffer, uint16_t size)
{
	//HAL_UART_Transmit_IT(&huart1, buffer, size);
	//HAL_UART_Transmit_DMA(&huart1, buffer, size);
	return 0;
}

/*
 * @brief UART receive data.
 *
 **/
static int8_t UART_RxData(uint8_t *buffer, uint16_t size)
{
	//HAL_UART_Receive_IT(&huart1, buffer, size);
	//HAL_UART_Receive_DMA(&huart1, buffer, size);
	return 0;
}

#endif /* UART_COMMUNICATION */
#endif /* I2C_COMMUNICATION */

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
