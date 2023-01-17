/* Created: 14.04.2019
 * si7021v2.h
 * 
 */	
#pragma once
#include "stm32l4xx_hal.h"

/**
 ** @brief Constants
 **/
#define SI7021_ADDR_SHIFTED             0x80    //7 bit MSB 0x76 (address) + 1 bit LSB 0x00 (read/write)
#define SI7021_ADDR						0x40	//It's not shifted address

/**
 ** @brief Internal command SI7021
 **/
static enum {
	
	SI7021_MEASURE_RHHOLD_MASTER					= 0xE5,
	SI7021_MEASURE_RHNOHOLD_MASTER					= 0xF5,
	SI7021_MEASURE_THOLD_MASTER						= 0xE3,
	SI7021_MEASURE_TNOHOLD_MASTER					= 0xF3,
	SI7021_READT_VALUE_PREVIOUS						= 0xE0,
	SI7021_SOFT_RESET_SI7021						= 0xFE,
	SI7021_WRITE_RHT_USER_REG_1						= 0xE6,
	SI7021_READ_RHT_USER_REG_1						= 0xE7,
	SI7021_WRITE_HEATER_CR							= 0x51,
	SI7021_READ_HEATER_CR							= 0x11,
	SI7021_READ_ELECTRONIC_ID_FIRST_BYTE_PART1		= 0xFA,
	SI7021_READ_ELECTRONIC_ID_FIRST_BYTE_PART2		= 0x0F,
	SI7021_READ_ELECTRONIC_ID_SECOND_BYTE_PART1		= 0xFC,
	SI7021_READ_ELECTRONIC_ID_SECOND_BYTE_PART2		= 0xC9,
	SI7021_READ_FIRMWARE_REVISION_PART1				= 0x84,
	SI7021_READ_FIRMWARE_REVISION_PART2				= 0xB8,

} SI7021_InternalCommand_enum;

/**
 ** @brief Device specific function type
 ** 
 ** @param[in] dev_addr : Device address on the I2C bus
 ** @param[in] reg_addr : Register address for device
 ** @param[in] *data : Pointer on data struct instance
 ** @param[in] len : Length of transmition/reception data
 **
 ** @return Result of API execution status
 ** @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 **/
typedef int8_t(*si7021_communication_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
typedef void(*si7021_delay_fptr)(uint32_t period);

/**
 ** @brief TempHumPress struct
 **/
typedef struct {
	union
	{
		uint64_t SerialNumber;
		struct
		{
			uint8_t SNB_0;
			uint8_t SNB_1;
			uint8_t SNB_2;
			uint8_t SNB_3;
			uint8_t SNA_0;
			uint8_t SNA_1;
			uint8_t SNA_2;
			uint8_t SNA_3;

		} ElementsSerialNumber;
	};
	uint8_t Dev_ID;
	uint8_t FirmwareRevision;
	int32_t TemperatureC;
	int32_t TemperatureF;
	uint32_t HumidityRH;
	
} SI7021_TempHumStruct_typedef;

/**
 ** @brief Configuration struct
 **/
typedef struct {
	union {
		uint8_t ControlRegister;
		struct {
			uint8_t RES0	: 1;
			uint8_t RSVD0	: 1;
			uint8_t HTRE	: 1;
			uint8_t RSVD1	: 1;
			uint8_t RSVD2	: 1;
			uint8_t RSVD3	: 1;
			uint8_t VDDS	: 1;
			uint8_t RES1	: 1;
						
		}bitsControlRegister;
	};
	
	union {
		uint8_t HeaterControlRegister;
		struct {
			uint8_t Heater	: 4;
			uint8_t RSVD	: 4;
			
		}bitsHeaterControlRegister;
	};
		
} SI7021_ConfigStruct_typedef;

/**
 ** @brief Electronic serial number struct receive
 **/
typedef struct {
	uint8_t SNA_3;
	uint8_t SNA_3_CRC;
	uint8_t SNA_2;
	uint8_t SNA_2_CRC;
	uint8_t SNA_1;
	uint8_t SNA_1_CRC;
	uint8_t SNA_0;
	uint8_t SNA_0_CRC;
	uint8_t SNB_3;
	uint8_t SNB_2;
	uint8_t SNB_3_2_CRC;
	uint8_t SNB_1;
	uint8_t SNB_0;
	uint8_t SNB_1_0_CRC;
	
} SI7021_SerialNumberStructReceive_typedef;

/**
 ** @brief Data receive parameter
 **/
typedef struct {
	uint8_t TemperatureMSB;
	uint8_t TemperatureLSB;
	uint8_t TemperatureCRC8;
	uint8_t HumidityMSB;
	uint8_t HumidityLSB;
	uint8_t HumidityCRC8;
	
} SI7021_DataReceiveStruct_typedef;

/**
 ** @brief BME280 instance struct
 **/
typedef struct {
	
	uint8_t dev_address;
	union
	{
		uint16_t dev_twin_cmd;
		struct
		{
			uint8_t dev_twin_cmd_lsb;
			uint8_t dev_twin_cmd_msb;
			
		} dev_twin_cmd_elements;
	};
	
	SI7021_ConfigStruct_typedef dev_configuration;
	SI7021_SerialNumberStructReceive_typedef dev_electronic_serial_number;
	SI7021_DataReceiveStruct_typedef dev_uncompensated_data;
	si7021_communication_fptr read_data_i2c;
	si7021_communication_fptr write_data_i2c;
	si7021_delay_fptr delay;	
	SI7021_TempHumStruct_typedef dev_compensated_data;
	
	
} SI7021_typedef;

/**
 ** @brief Public function prototype
 **/

//Get temperature
SI7021_TempHumStruct_typedef* SI7021_Get_Temp(SI7021_typedef *dev_si7021);	
//Get humidity
SI7021_TempHumStruct_typedef* SI7021_Get_Hum(SI7021_typedef *dev_si7021); 		
//Set configuration register
void SI7021_Set_Confuguration(uint8_t measurement_resolution, uint8_t heater_en, uint8_t heater_value, SI7021_typedef *dev_si7021); 
//Get firmware revision
void SI7021_Get_Electronic_Serial_Number(SI7021_typedef *dev_si7021);





