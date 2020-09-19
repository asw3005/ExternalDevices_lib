/*
 *	@brief DS18B20 driver.
 *	Created 26.08.2020
 *	DS18B20_H_
 *
 **/

//#pragma once
#ifndef DS18B20_H_				  
#define DS18B20_H_

#include "stm32l4xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief State of line  after reseting line. 
 *
 **/
typedef enum
{
	DS18B20_PRESENT_IS_OK = 0,
	DS18B20_DEVICE_NOT_FOUND = 1
	
} DS18B20_StateOfLine;

/*
 * @briefParasite power supply pin remote control.
 *
 **/
typedef enum
{
	DS18B20_STRONG_PULL_UP_EN		= 1,
	DS18B20_STRONG_PULL_UP_DIS		= 0
	
} DS18B20_StrongPullUp;

/*
 * @brief DS18B20 data read depth.
 *
 **/
typedef enum
{
	DS18B20_ONLY_TEMPERATURE = 3,
	DS18B20_TEMPERATURE_WITH_CONFIGURATION_REGISTER = 5,
	DS18B20_FULL_SCRATCHPAD = 9
	
} DS18B20_DataReadDepth;

/*
 * @brief DS18B20 resolution.
 *
 **/
typedef enum
{
	DS18B20_MEASUREMENT_RESOLUTION_9BIT		= 0x1F,
	DS18B20_MEASUREMENT_RESOLUTION_10BIT	= 0x3F,
	DS18B20_MEASUREMENT_RESOLUTION_11BIT	= 0x5F,
	DS18B20_MEASUREMENT_RESOLUTION_12BIT	= 0x7F
	
} DS18B20_ResolutionOfMeasurement;


/*
 * @brief DS18B20 function command set and 1-Wire command set.
 *
 **/
typedef enum
{
	///***ROM COMMANDS***/
	///When a system is initially powered up, the master must identify the ROM codes of all 
	///slave devices on the bus, which allows the master to determine the number of slaves 
	///and their device types.
	ONEWIRE_SEARCH_ROM			= 0xF0,
	///This command can only be used when there is one slave on the bus. It allows the bus 
	///master to read the slave’s 64-bit ROM code without using the Search ROM procedure.
	ONEWIRE_READ_ROM			= 0x33,
	///The match ROM command followed by a 64-bit ROM code sequence allows the bus master 
	///to address a specific slave device on a multidrop or single-drop bus.
	ONEWIRE_MATCH_ROM			= 0x55,
	///The master can use this command to address all devices on the bus simultaneously 
	///without sending out any ROM code information.
	ONEWIRE_SKIP_ROM			= 0xCC,
	///The operation of this command is identical to the operation of the Search ROM 
	///command except that only slaves with a set alarm flag will respond.
	ONEWIRE_ALARM_SEARCH		= 0xEC,
	
	///***DS18B20 COMMANDS***/
	///Initiates temperature conversion.
	///DS18B20 transmits conversion status to master (not applicable for parasite-powered DS18B20s).
	///For parasite-powered DS18B20s, the master must enable a strong pullup on the 1-Wire bus during 
	///temperature conversions and copies from the scratchpad to EEPROM. No other bus activity may 
	///take place during this time.
	DS18B20_CONVERT_T			= 0x44,
	///Reads the entire scratchpad including the CRC byte.
	///DS18B20 transmits up to 9 data bytes to master.
	///The master can interrupt the transmission of data at any time by issuing a reset.
	DS18B20_READ_SCRATCHPAD		= 0xBE,
	///Writes data into scratchpad bytes 2, 3, and 4 (TH, TL, and configuration registers).
	///Master transmits 3 data bytes to DS18B20. All three bytes must be written before a reset is issued.
	DS18B20_WRITE_SCRATCHPAD	= 0x4E,
	///Copies TH, TL, and configuration register data from the scratchpad to EEPROM.
	///For parasite-powered DS18B20s, the master must enable a strong pullup on the 1-Wire bus during 
	///temperature conversions and copies from the scratchpad to EEPROM. No other bus activity may 
	///take place during this time.
	DS18B20_COPY_SCRATCHPAD		= 0x48,
	///Recalls TH, TL, and configuration register data from EEPROM to the scratchpad.
	///DS18B20 transmits recall status to master.
	DS18B20_RECALL_E2			= 0xB8,
	///Signals DS18B20 power supply mode to the master.
	///DS18B20 transmits supply status to master.
	DS18B20_READ_POWER_SUPPLY	= 0xB4
	
} DS18B20_FunctionCommandSet;

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
 * @brief Remote control for the external strong pull up.
 *
 * @param pull_up : Zero is no pull up, more than zero is pull up enable.
 *
 **/
typedef void(*parasite_power_pin_remote)(uint8_t pull_up);

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
typedef void(*usart_txrx_data_fptr)(uint8_t *buffer, uint8_t size);

typedef struct
{
	///Each DS18B20 contains a unique 64–bit code stored in ROM. 
	///The least significant 8 bits of the ROM code contain the DS18B20’s 1-Wire 
	///family code: 28h. The next 48 bits contain a unique serial number. The most 
	///significant 8 bits contain a cyclic redundancy check (CRC) byte that is 
	///calculated from the first 56 bits of the ROM code.
	union
	{
		uint64_t RomLaserCode;
		struct
		{
			uint8_t arrayRomLaserCode[8];
		} partsRomLaserCode;
	};
	///Power supply type.
	///If it is logic one level, power supply mode is normal, otherwise parasite mode.
	uint8_t PowerSupplyType;
	///Measured temperature.
	float Temperature; 
	
} DS18B20_ProcessedData_typedef;

/*
 *	@brief Convertional data from the DS18B20 temperature sensor. 
 *	
 * ///Is is data, that have been convert to the DS18B20 scratchpad format.  
 *
 **/
typedef struct
{
	///Each DS18B20 contains a unique 64–bit code stored in ROM. 
	///The least significant 8 bits of the ROM code contain the DS18B20’s 1-Wire 
	///family code: 28h. The next 48 bits contain a unique serial number. The most 
	///significant 8 bits contain a cyclic redundancy check (CRC) byte that is 
	///calculated from the first 56 bits of the ROM code.
	uint64_t RomLaserCode;
	///Temperature.
	int16_t Temperature;
	///
	uint8_t ThRegister;
	uint8_t TlRegister;	
	///The configuration register data.
	uint8_t ConfigurationRegister;
	///These bytes are reserved for internal use by the device and cannot be overwritten.
	uint8_t Reserved_0;
	uint8_t Reserved_1;
	uint8_t Reserved_2;
	///CRC = X8 + X5 + X4 + 1
	uint8_t Crc;
	
} DS18B20_ScratchpadData_typedef;

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
	
} DS18B20_RawData_typedef;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	///
	uint16_t *isReceiveComplete;
	///Converted data from the scratchpad.
	DS18B20_ProcessedData_typedef converted_data;
	///Converted scratchpad data.
	DS18B20_ScratchpadData_typedef scratchpad_data;
	///Input raw data.
	DS18B20_RawData_typedef raw_data;
	//Pointers for the rx, tx delay functions.
	parasite_power_pin_remote strong_pull_up;
	usart_init uart_init_baud;
	usart_txrx_data_fptr uart_tx_data;
	usart_txrx_data_fptr uart_rx_data;
	onewire_delay_fptr delay;
	
} DS18B20_GeneralDataInstance_typedef;

/*
 *	@brief Public function prototype.
 *
 **/

///Resets 1 wire line and checks availability for any devices on the bus.
///This function must be invoked first.
uint8_t DS18B20_DeviceDetect(DS18B20_GeneralDataInstance_typedef *device);
///Sets a unique 64–bit ROM code.
void DS18B20_Set_LaserRomCode(DS18B20_GeneralDataInstance_typedef *device, uint64_t rom_code);
///Reads a unique 64–bit code.
uint64_t  DS18B20_Get_LaserRomCode(DS18B20_GeneralDataInstance_typedef *device);
///Reads temperature from the sensor and converts the value according to the sensor resolution.
void DS18B20_Get_Temperature(DS18B20_GeneralDataInstance_typedef *device,
	DS18B20_FunctionCommandSet rom_cmd,
	DS18B20_DataReadDepth dataDepth);
///Determines the type of sensor power supply.
void DS18B20_Get_PowerSupplyType(DS18B20_GeneralDataInstance_typedef *device);
///Sets sensor setting (Th, Tl, sensor resolution).
void DS18B20_Set_ThresholdAndControl(DS18B20_GeneralDataInstance_typedef *device,
	DS18B20_FunctionCommandSet rom_cmd,
	int8_t tHigh,
	int8_t tLow,
	DS18B20_ResolutionOfMeasurement resolution);
///Reads scratchpad memory of the DS18B20 temperature sensor.
void DS18B20_ReadScratchpadBytes(DS18B20_GeneralDataInstance_typedef *device,
	DS18B20_FunctionCommandSet rom_cmd,
	uint8_t amountOfbytes);
///Saves scratchpad data (Tl, Th, Configuration register) to the internal EEPROM memory. 
void DS18B20_CopyScratchpadToEeprom(DS18B20_GeneralDataInstance_typedef *device, DS18B20_FunctionCommandSet rom_cmd);
///Reads scratchpad data (Tl, Th, Configuration register) from the internal EEPROM to RAM memory.
void DS18B20_RecallE2(DS18B20_GeneralDataInstance_typedef *device, DS18B20_FunctionCommandSet rom_cmd);

#endif /* DS18B20_H_ */