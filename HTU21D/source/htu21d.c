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
static int32_t HTU21DTemperatureConv(HTU21D_typedef *dev_htu21d); 
static uint32_t HTU21DHumidityConv(HTU21D_typedef *dev_htu21d);  

/**
 ** @brief Get temperature
 **/				
TempHumStructHTU21D_typedef* getHTU21DTemp(HTU21D_typedef *dev_htu21d)                              
{	
	dev_htu21d->read_data_i2c(dev_htu21d->dev_address, TRIGGER_TEMPERATURE_MEASUREMENT_HOLD_MASTER, 1, &dev_htu21d->dev_uncompensated_data.TemperatureMSB, 3);
	dev_htu21d->delay(15);
	dev_htu21d->dev_compensated_data.TemperatureC = HTU21DTemperatureConv(dev_htu21d) / 10;
	dev_htu21d->dev_compensated_data.TemperatureF = ((dev_htu21d->dev_compensated_data.TemperatureC * 18 * 10) + 320000) / 100;
	return &dev_htu21d->dev_compensated_data;
}

/**
 ** @brief Get humidity
 **/					
TempHumStructHTU21D_typedef* getHTU21DHum(HTU21D_typedef *dev_htu21d)                              
{
	dev_htu21d->read_data_i2c(dev_htu21d->dev_address, TRIGGER_HUMIDITY_MEASUREMENT_HOLD_MASTER, 1, &dev_htu21d->dev_uncompensated_data.HumidityMSB, 3);
	dev_htu21d->delay(15);
	dev_htu21d->dev_compensated_data.HumidityRH = HTU21DHumidityConv(dev_htu21d);
	return &dev_htu21d->dev_compensated_data;
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
void setConfugurationHTU21D(uint8_t measurement_resolution, uint8_t heater_en, HTU21D_typedef *dev_htu21d)                              
{	
	dev_htu21d->dev_configuration.bitsControlRegister.RES0 = (measurement_resolution & 0x01);
	dev_htu21d->dev_configuration.bitsControlRegister.RES1 = (measurement_resolution & 0x02);
	dev_htu21d->dev_configuration.bitsControlRegister.HTRE = heater_en; 	/// On-chip heater disable

	dev_htu21d->write_data_i2c(dev_htu21d->dev_address, WRITE_USER_REGISTER, 1, &dev_htu21d->dev_configuration.ControlRegister, 1);
	dev_htu21d->delay(1);	
}

/**
 ** @brief Returns temperature in DegC, 21540 is 21,540 degree C.
 **/
static int32_t HTU21DTemperatureConv(HTU21D_typedef *dev_htu21d)                              
{ 
	uint16_t TempCode;
	float Temperature;
	
	TempCode = (dev_htu21d->dev_uncompensated_data.TemperatureMSB << 8 | dev_htu21d->dev_uncompensated_data.TemperatureLSB);
	
	Temperature = ((175.72f * TempCode) / 65536) - 46.85f;
	
	return (Temperature * 1000);
}

/**
 ** @brief Returns humidity in %RH, 61053 is 61,053 %RH
 **/
static uint32_t HTU21DHumidityConv(HTU21D_typedef *dev_htu21d)                              
{
	uint32_t RH_Code;
	float RelativeHumidity;
	
	RH_Code = (dev_htu21d->dev_uncompensated_data.HumidityMSB << 8 | dev_htu21d->dev_uncompensated_data.HumidityLSB);
	
	RelativeHumidity = ((125.0f * RH_Code) / 65536) - 6;
	if (RelativeHumidity > 100)
	{
		RelativeHumidity = 100;
	}
	if (RelativeHumidity < 0)
	{
		RelativeHumidity = 0;
	}
	
	return RelativeHumidity * 1000;
}
