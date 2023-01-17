/* Created: 4.29.2020
 * htu21d.c
 * 
 * 
 * 
 */

#include "htu21d.h"
#include "stm32l4xx_hal.h"

/**
 ** @brief Private function prototype
 **/
static float HTU21D_TemperatureConv(HTU21D_typedef *dev_htu21d); 
static float HTU21D_HumidityConv(HTU21D_typedef *dev_htu21d);  

/**
 ** @brief Get temperature
 **/				
void HTU21D_GetTemp(HTU21D_typedef *dev_htu21d)                              
{	
	dev_htu21d->read_data_i2c(dev_htu21d->dev_address, HTU21D_TRIGGER_TEMPERATURE_MEASUREMENT_HOLD_MASTER, 1, &dev_htu21d->dev_uncompensated_data.TemperatureMSB, 3);
	dev_htu21d->delay(50);
	dev_htu21d->dev_compensated_data.TemperatureC = HTU21D_TemperatureConv(dev_htu21d);
	dev_htu21d->dev_compensated_data.TemperatureF = (dev_htu21d->dev_compensated_data.TemperatureC * 1.8f) + 32;
}

/**
 ** @brief Get humidity
 **/					
void HTU21D_GetHum(HTU21D_typedef *dev_htu21d)                              
{
	dev_htu21d->read_data_i2c(dev_htu21d->dev_address, HTU21D_TRIGGER_HUMIDITY_MEASUREMENT_HOLD_MASTER, 1, &dev_htu21d->dev_uncompensated_data.HumidityMSB, 3);
	dev_htu21d->delay(50);
	dev_htu21d->dev_compensated_data.HumidityRH = HTU21D_HumidityConv(dev_htu21d);
}

/**
 ** @brief Set configuration register
 ** 
 **	RES1	RES0	RH			Temp
 **	0		0		12bits		14bits	
 ** 0		1		8bits		12bits
 ** 1		0		10bits		13bits
 ** 1		1		11bits		11bits
 ** 
 ** HTRE 1 is On-chip heater enable, 0 is On-chip heater disble
 ** 
 **/
void HTU21D_SetConfuguration(HTU21D_typedef *dev_htu21d, uint8_t measurement_resolution, uint8_t heater_en)                              
{	
	dev_htu21d->dev_configuration.bitsControlRegister.RES0 = (measurement_resolution & 0x01);
	dev_htu21d->dev_configuration.bitsControlRegister.RES1 = (measurement_resolution & 0x02);
	dev_htu21d->dev_configuration.bitsControlRegister.HTRE = heater_en; 	/// On-chip heater disable

	dev_htu21d->write_data_i2c(dev_htu21d->dev_address, HTU21D_WRITE_USER_REGISTER, 1, &dev_htu21d->dev_configuration.ControlRegister, 1);
	dev_htu21d->delay(50);	
}

/**
 ** @brief Returns temperature in DegC, 21540 is 21,540 degree C.
 **/
static float HTU21D_TemperatureConv(HTU21D_typedef *dev_htu21d)                              
{ 
	uint16_t TempCode;
	float Temperature;
	
	TempCode = (dev_htu21d->dev_uncompensated_data.TemperatureMSB << 8 | ((dev_htu21d->dev_uncompensated_data.TemperatureLSB & 0xFC) >> 2));
	
	Temperature = ((175.72f * TempCode) / 65536) - 46.85f;
	
	return Temperature;
}

/**
 ** @brief Returns humidity in %RH, 61053 is 61,053 %RH
 **/
static float HTU21D_HumidityConv(HTU21D_typedef *dev_htu21d)                              
{
	uint32_t RH_Code = 0;
	float RelativeHumidity;
	
	RH_Code = ((dev_htu21d->dev_uncompensated_data.HumidityMSB << 8) | ((dev_htu21d->dev_uncompensated_data.HumidityLSB & 0xFC) >> 2));
	
	RelativeHumidity = ((125.0f * (float)RH_Code) / 65536) - 6;	
	RelativeHumidity = RelativeHumidity + (25 - dev_htu21d->dev_compensated_data.TemperatureC) * (-0.15f);
	
	if (RelativeHumidity > 100)
	{
		RelativeHumidity = 100;
	}
	if (RelativeHumidity < 0)
	{
		RelativeHumidity = 0;
	}
	
	return RelativeHumidity;
}
