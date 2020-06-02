/** Created: 4.28.2020
 ** pca9546.h
 **/

#pragma once

/**
 ** @brief Configuration i2c multiplexor
 **/
#define ADDR_PCA9546            0x70
#define ADDR_PCA9546_SHIFTED	0xE0

#define CHANNEL_0		0x02
#define CHANNEL_1		0x01
#define CHANNEL_2		0x04
#define CHANNEL_3		0x08

//#define CHANNEL_BME280	CHANNEL_1
//#define CHANNEL_SI7021	CHANNEL_0
//#define CHANNEL_HTS221	CHANNEL_2
//#define CHANNEL_LPS22	CHANNEL_3


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
typedef int8_t(*pca9546_communication_fptr)(uint8_t dev_addr, uint8_t data, uint16_t len);
typedef void(*pca9546_reset_pin_driver_fptr)(uint8_t drive_reset_pin);
typedef void(*pca9546_delay_fptr)(uint32_t period);

/**
 ** @brief Channels
 **/
typedef enum
{
	CHANNEL_BME280  =    CHANNEL_1,
	CHANNEL_SI7021  =    CHANNEL_0,
	CHANNEL_HTS221  =    CHANNEL_2,
	CHANNEL_LPS22   =    CHANNEL_3,
	CHANNEL_HTU21D  =    CHANNEL_1
	
} CHANNEL_enum;

/**
 ** @brief Reset pin state
 **/
typedef enum
{
	PCA9546_RSTPIN_RESET = 0x00,
    PCA9546_RSTPIN_SET = 0x01,
	
} RESET_PIN_STATE_enum;

/**
 ** @brief Multiplexer config struct
 **/
typedef struct
{
	uint8_t dev_address;	
	uint8_t dev_configuration;
	pca9546_communication_fptr write_data_i2c;
	pca9546_reset_pin_driver_fptr drive_reset_pin;
	pca9546_delay_fptr delay;	
	
} PCA9546_CONF_typedef;

/**
 ** @brief Public function prototype
 **/

//Select pca9546 channel
void selectChannel(CHANNEL_enum channel, PCA9546_CONF_typedef *dev_instance);