/* Created: 14.04.2019
 * si7021v2.c
 * 
 * 
 * 
 */

#include "si7021v2.h"
#include "stm32l4xx_hal.h"

/**
 ** @brief Private function prototype
 **/
static int32_t SI7021_Temperature_Conv(SI7021_typedef *dev_si7021); 
static uint32_t SI7021_Humidity_Conv(SI7021_typedef *dev_si7021);  

/**
 ** @brief Get temperature
 **/				
SI7021_TempHumStruct_typedef* SI7021_Get_Temp(SI7021_typedef *dev_si7021)                              
{	
	dev_si7021->read_data_i2c(dev_si7021->dev_address, MEASURE_THOLD_MASTER, 1, &dev_si7021->dev_uncompensated_data.TemperatureMSB, 3);
	dev_si7021->delay(15);
	dev_si7021->dev_compensated_data.TemperatureC = SI7021_Temperature_Conv(dev_si7021) / 10;
	dev_si7021->dev_compensated_data.TemperatureF = ((dev_si7021->dev_compensated_data.TemperatureC * 18 * 10) + 320000) / 100;
	return &dev_si7021->dev_compensated_data;
}

/**
 ** @brief Get humidity
 **/					
SI7021_TempHumStruct_typedef* SI7021_Get_Hum(SI7021_typedef *dev_si7021)                              
{
	dev_si7021->read_data_i2c(dev_si7021->dev_address, MEASURE_RHHOLD_MASTER, 1, &dev_si7021->dev_uncompensated_data.HumidityMSB, 3);
	dev_si7021->delay(15);
	dev_si7021->dev_compensated_data.HumidityRH = SI7021_Humidity_Conv(dev_si7021);
	return &dev_si7021->dev_compensated_data;
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
void SI7021_Set_Confuguration(uint8_t measurement_resolution, uint8_t heater_en, uint8_t heater_value, SI7021_typedef *dev_si7021)                              
{	
	dev_si7021->dev_configuration.bitsControlRegister.RES0 = (measurement_resolution & 0x01);
	dev_si7021->dev_configuration.bitsControlRegister.RES1 = (measurement_resolution & 0x02);
	dev_si7021->dev_configuration.bitsControlRegister.HTRE = heater_en;	/// On-chip heater disable
	
	if(heater_en)
	{
		dev_si7021->dev_configuration.HeaterControlRegister = heater_value;
		dev_si7021->write_data_i2c(dev_si7021->dev_address, WRITE_HEATER_CR, 1, &dev_si7021->dev_configuration.HeaterControlRegister, 1);
		dev_si7021->delay(5);
	}
	
	dev_si7021->write_data_i2c(dev_si7021->dev_address, WRITE_RHT_USER_REG_1, 1, &dev_si7021->dev_configuration.ControlRegister, 1);
	dev_si7021->delay(5);	
}

/**
 ** @brief Get firmware revision
 **/
void SI7021_Get_Electronic_Serial_Number(SI7021_typedef *dev_si7021)                              
{
	//write receive's register address to  firmware revision Struct
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_msb = READ_ELECTRONIC_ID_FIRST_BYTE_PART1;
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_lsb = READ_ELECTRONIC_ID_FIRST_BYTE_PART2;
	
	dev_si7021->read_data_i2c(dev_si7021->dev_address, dev_si7021->dev_twin_cmd, 2, &dev_si7021->dev_electronic_serial_number.SNA_3, 8);
	dev_si7021->delay(5);
	
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_msb = READ_ELECTRONIC_ID_SECOND_BYTE_PART1;
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_lsb = READ_ELECTRONIC_ID_SECOND_BYTE_PART2;
	
	dev_si7021->read_data_i2c(dev_si7021->dev_address, dev_si7021->dev_twin_cmd, 2, &dev_si7021->dev_electronic_serial_number.SNB_3, 6);
	dev_si7021->delay(5);
	
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_msb = READ_FIRMWARE_REVISION_PART1;
	dev_si7021->dev_twin_cmd_elements.dev_twin_cmd_lsb = READ_FIRMWARE_REVISION_PART2;
	
	dev_si7021->read_data_i2c(dev_si7021->dev_address, dev_si7021->dev_twin_cmd, 2, &dev_si7021->dev_compensated_data.FirmwareRevision, 1);
	dev_si7021->delay(5);	
	
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNA_3 = dev_si7021->dev_electronic_serial_number.SNA_3;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNA_2 = dev_si7021->dev_electronic_serial_number.SNA_2;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNA_1 = dev_si7021->dev_electronic_serial_number.SNA_1;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNA_0 = dev_si7021->dev_electronic_serial_number.SNA_0;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNB_3 = dev_si7021->dev_electronic_serial_number.SNB_3;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNB_2 = dev_si7021->dev_electronic_serial_number.SNB_2;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNB_1 = dev_si7021->dev_electronic_serial_number.SNB_1;
	dev_si7021->dev_compensated_data.ElementsSerialNumber.SNB_0 = dev_si7021->dev_electronic_serial_number.SNB_0;
	dev_si7021->dev_compensated_data.Dev_ID = dev_si7021->dev_electronic_serial_number.SNB_3;
}

/**
 ** @brief Returns temperature in DegC, 21540 is 21,540 degree C.
 **/
static int32_t SI7021_Temperature_Conv(SI7021_typedef *dev_si7021)                              
{ 
	uint16_t TempCode;
	float Temperature;
	
	TempCode = (dev_si7021->dev_uncompensated_data.TemperatureMSB << 8 | dev_si7021->dev_uncompensated_data.TemperatureLSB);
	
	Temperature = ((175.72f * TempCode) / 65536) - 46.85f;
	
	return (Temperature * 1000);
}

/**
 ** @brief Returns humidity in %RH, 61053 is 61,053 %RH
 **/
static uint32_t SI7021_Humidity_Conv(SI7021_typedef *dev_si7021)                              
{
	uint32_t RH_Code;
	float RelativeHumidity;
	
	RH_Code = (dev_si7021->dev_uncompensated_data.HumidityMSB << 8 | dev_si7021->dev_uncompensated_data.HumidityLSB);
	
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
