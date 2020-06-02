/** Created: 4.28.2020
 ** pca9546.c
 **/

#include "stm32l4xx_hal.h"
#include "pca9546.h"

//PCA9546_CONF_typedef pca9546_instance0 = { 
//	
//	.dev_address = ADDR_PCA9546_SHIFTED,
//};

/**
 ** @brief Select channel function
 **/
void selectChannel(CHANNEL_enum channel, PCA9546_CONF_typedef *dev_instance)
{
	dev_instance->write_data_i2c(dev_instance->dev_address, channel, 1);
	dev_instance->delay(1);
		
}