/** Created: 4.28.2020
 ** pca9546.h
 **/

#pragma once

/**
 ** @brief Configuration i2c multiplexor
 **/
#define PCA9546_ADDR            0x70
#define PCA9546_ADDR_SHIFTED	0xE0

#define PCA9546_CHANNEL_0		0x02
#define PCA9546_CHANNEL_1		0x01
#define PCA9546_CHANNEL_2		0x04
#define PCA9546_CHANNEL_3		0x08

/**
 ** @brief Device specific function type
 ** 
 ** @param[in] dev_addr : Device address on the I2C bus
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
	PCA9546_CHANNEL_BME280  =  PCA9546_CHANNEL_1,
	PCA9546_CHANNEL_SI7021  =  PCA9546_CHANNEL_0,
	PCA9546_CHANNEL_HTS221  =  PCA9546_CHANNEL_2,
	PCA9546_CHANNEL_LPS22   =  PCA9546_CHANNEL_3,
	PCA9546_CHANNEL_HTU21D  =  PCA9546_CHANNEL_1
	
} PCA9546_Channel;

/**
 ** @brief Reset pin state
 **/
typedef enum
{
	PCA9546_RSTPIN_RESET = 0x00,
    PCA9546_RSTPIN_SET = 0x01,
	
} PCA9546_ResetPinState;

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
	
} PCA9546_Conf_typedef;

/**
 ** @brief Public function prototype
 **/

//Select pca9546 channel
void PCA9546_SelectChannel(PCA9546_Channel channel, PCA9546_Conf_typedef *dev_instance);