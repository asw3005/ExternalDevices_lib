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
void PCA9546_SelectChannel(PCA9546_Conf_typedef *dev_instance, PCA9546_Channel channel)
{
	dev_instance->dev_configuration = channel;
	dev_instance->write_data_i2c(dev_instance->dev_address, &dev_instance->dev_configuration, 1);
	dev_instance->delay(1);		
}