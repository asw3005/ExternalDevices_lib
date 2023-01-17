/* Created: 4.29.2020
 * htu21d.h
 * 
 */	
#pragma once
#include "stm32l4xx_hal.h"

/**
 ** @brief Constants
 **/
#define HTU21D_ADDR_SHIFTED             0x80	//7 bit MSB 0x76 (address) + 1 bit LSB 0x00 (read/write)
#define HTU21D_ADDR						0x40	//It's not shifted address

/**
 ** @brief Internal command SI7021
 **/
static enum {
	
	HTU21D_TRIGGER_TEMPERATURE_MEASUREMENT_HOLD_MASTER		= 0xE3,
	HTU21D_TRIGGER_HUMIDITY_MEASUREMENT_HOLD_MASTER			= 0xE5,
	HTU21D_TRIGGER_TEMPERATURE_MEASUREMENT_NO_HOLD_MASTER	= 0xF3,
	HTU21D_TRIGGER_HUMIDITY_MEASUREMENT_NO_HOLD_MASTER		= 0xF5,
	HTU21D_WRITE_USER_REGISTER								= 0xE6,
	HTU21D_READ_USER_REGISTER								= 0xE7,
	HTU21D_SOFT_RESET_HTU21D								= 0xFE,

} InternalCommandHTU21D_enum;

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
typedef int8_t(*htu21d_communication_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
//typedef int8_t(*htu21d_communication_alt_fptr)(uint8_t DevAddress, uint8_t *pData, uint16_t Size);
typedef void(*htu21d_delay_fptr)(uint32_t period);

/**
 ** @brief TempHumPress struct
 **/
typedef struct {
	float TemperatureC;
	float TemperatureF;
	float HumidityRH;
	
} TempHumStructHTU21D_typedef;

/**
 ** @brief Configuration struct
 **/
typedef struct {
	union {
		uint8_t ControlRegister;
		struct {
			uint8_t RES0		: 1;
			uint8_t OTPRELOAD	: 1;
			uint8_t HTRE		: 1;
			uint8_t RSVD1		: 1;
			uint8_t RSVD2		: 1;
			uint8_t RSVD3		: 1;
			uint8_t VDDS		: 1;
			uint8_t RES1		: 1;
						
		}bitsControlRegister;
	};
		
} ConfigStructHTU21D_typedef;

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
	
} DataReceiveStructHTU21D_typedef;

/**
 ** @brief BME280 instance struct
 **/
typedef struct {
	
	uint8_t dev_address;
	uint8_t dev_cmd;
	ConfigStructHTU21D_typedef dev_configuration;
	DataReceiveStructHTU21D_typedef dev_uncompensated_data;
	htu21d_communication_fptr read_data_i2c;
	htu21d_communication_fptr write_data_i2c;
	htu21d_delay_fptr delay;	
	TempHumStructHTU21D_typedef dev_compensated_data;
		
} HTU21D_typedef;

/**
 ** @brief Public function prototype
 **/

//Get temperature
void HTU21D_GetTemp(HTU21D_typedef *dev_htu21d);	
//Get humidity
void HTU21D_GetHum(HTU21D_typedef *dev_htu21d); 		
//Set configuration register
void HTU21D_SetConfuguration(HTU21D_typedef *dev_htu21d, uint8_t measurement_resolution, uint8_t heater_en);
