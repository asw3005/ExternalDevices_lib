/* Created: 27.05.2018
 * bme280v2.c
 *  
 */	

#include "stm32l4xx_hal.h"
#include "bme280v2.h"

/**
 ** @brief Public variable
 **/
static int32_t t_fine;

/**
 ** @brief Private function prototype
 **/
static void BME280_Preparation_Config(void);											
static int32_t BME280_Temperature_Conv(BME280_typedef *dev_bme280);                   
static uint32_t BME280_Humidity_Conv(BME280_typedef *dev_bme280);                     
static uint32_t BME280_Pressure_Conv(BME280_typedef *dev_bme280);   					
static void BME280_Set_Profile(BME280_Profile profile_type, BME280_typedef *dev_bme280); 


/**
 ** @brief Init device  
 ** 
 ** @param[in] meas_profile : profile type from the  BME280_PROFILES_enum.
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
void BME280_Init_Device(BME280_Profile meas_profile, BME280_typedef *dev_bme280)                              
{	
	//Config work mode for device
	BME280_Set_Profile(meas_profile, dev_bme280);	
	BME280_Get_Calibration_Data(dev_bme280);
}

/**
 ** @brief Get T, H, P 
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/				
BME280_TempHumPressStruct_typedef* BME280_Get_Data_Press_Temp_Hum(BME280_typedef *dev_bme280)                              
{
	dev_bme280->dev_compensated_data.TemperatureC = BME280_Temperature_Conv(dev_bme280);
	dev_bme280->dev_compensated_data.TemperatureF = ((BME280_Temperature_Conv(dev_bme280) * 18) + 32000) / 10;
	dev_bme280->dev_compensated_data.HumidityRH = (BME280_Humidity_Conv(dev_bme280) * 1000) / 1024;
	dev_bme280->dev_compensated_data.PressurePa = BME280_Pressure_Conv(dev_bme280);
	dev_bme280->dev_compensated_data.PressuremmHg = (((uint64_t) dev_bme280 ->dev_compensated_data.PressurePa * 1000000)) / 133322;
	
	dev_bme280->read_data_i2c(dev_bme280->dev_address, CTRL_HUM_ADDR, 1, (uint8_t *)(&dev_bme280->dev_uncompensated_data), sizeof(dev_bme280->dev_uncompensated_data));
	dev_bme280->delay(1);
	
	return  &dev_bme280->dev_compensated_data;
}

/**
 ** @brief Get calibration data full range
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
void BME280_Get_Calibration_Data(BME280_typedef *dev_bme280)                              
{
	//write receive's register address to bme280 Config Struct
	dev_bme280->read_data_i2c(dev_bme280->dev_address, CALIB00_START_ADDR, 1, (uint8_t *)&dev_bme280->dev_calibration_data, sizeof(dev_bme280->dev_calibration_data));
	dev_bme280->delay(1);
}

/**
 ** @brief Get device's id
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
void BME280_GetID(BME280_typedef *dev_bme280)                              
{
	//write receive's register address to bme280 Config Struct
	dev_bme280->read_data_i2c(dev_bme280->dev_address, ID_ADDR, 1, (uint8_t *)(&dev_bme280->dev_calibration_data.bmeID), 1);
	dev_bme280->delay(1);
}

/**
 ** @brief Get calibration data zero range
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
void BME280_GetCalibration_Data0(BME280_typedef *dev_bme280)                              
{
	//write receive's register address to bme280 Config Struct
	dev_bme280->read_data_i2c(dev_bme280->dev_address, CALIB00_START_ADDR, 1, (uint8_t *)(&dev_bme280->dev_calibration_data), 26);
	dev_bme280->delay(1);
}

/**
 ** @brief Get calibration data firt range
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
void BME280_Get_Calibration_Data1(BME280_typedef *dev_bme280)                              
{
	//write receive's register address to bme280 Config Struct
	dev_bme280->read_data_i2c(dev_bme280->dev_address, CALIB26_START_ADDR, 1, (uint8_t *)(&dev_bme280->dev_calibration_data.Dig_H2), 7);
	dev_bme280->delay(1);
}

/**
 ** @brief Measurement profiles
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **
 **/
static void BME280_Set_Profile(BME280_Profile profile_type, BME280_typedef *dev_bme280)                              
{
	switch (profile_type)
	{
		case WEATHER_MONITORING:

		dev_bme280->dev_configuration.bitsDataCtrlHum.osrs_h = OSRS_H_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataCtrlMeas.mode = FORCED_MODE;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_p = OSRS_P_OVERSAMPLING_1;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_t = OSRS_T_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataConfig.spi3w_en = SPI3WIRE_DIS;
		dev_bme280->dev_configuration.bitsDataConfig.t_sb = T_SB_1000mS;
		dev_bme280->dev_configuration.bitsDataConfig.filter = FILTER_OFF; 
		
		break;
		
		case HUMIDITY_SENSING:

		dev_bme280->dev_configuration.bitsDataCtrlHum.osrs_h = OSRS_H_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataCtrlMeas.mode = FORCED_MODE;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_p = OSRS_P_SKIPPED;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_t = OSRS_T_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataConfig.spi3w_en = SPI3WIRE_DIS;
		dev_bme280->dev_configuration.bitsDataConfig.t_sb = T_SB_1000mS;
		dev_bme280->dev_configuration.bitsDataConfig.filter = FILTER_OFF; 
		
		break;
		
		case INDOOR_NAVIGATION:

		dev_bme280->dev_configuration.bitsDataCtrlHum.osrs_h = OSRS_H_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataCtrlMeas.mode = NORMAL_MODE;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_p = OSRS_P_OVERSAMPLING_16;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_t = OSRS_T_OVERSAMPLING_2;
	
		dev_bme280->dev_configuration.bitsDataConfig.spi3w_en = SPI3WIRE_DIS;
		dev_bme280->dev_configuration.bitsDataConfig.t_sb = T_SB_500mS;
		dev_bme280->dev_configuration.bitsDataConfig.filter = FILTER_COEFF16; 
		
		break;
		
		case GAMING:

		dev_bme280->dev_configuration.bitsDataCtrlHum.osrs_h = OSRS_H_SKIPPED;
	
		dev_bme280->dev_configuration.bitsDataCtrlMeas.mode = NORMAL_MODE;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_p = OSRS_P_OVERSAMPLING_4;
		dev_bme280->dev_configuration.bitsDataCtrlMeas.osrs_t = OSRS_T_OVERSAMPLING_1;
	
		dev_bme280->dev_configuration.bitsDataConfig.spi3w_en = SPI3WIRE_DIS;
		dev_bme280->dev_configuration.bitsDataConfig.t_sb = T_SB_500mS;
		dev_bme280->dev_configuration.bitsDataConfig.filter = FILTER_COEFF16; 
		
		break;
		
		default: break;
	}
	
	dev_bme280->write_data_i2c(dev_bme280->dev_address, CTRL_HUM_ADDR, 1, &dev_bme280->dev_configuration.DataCtrlHum, sizeof(dev_bme280->dev_configuration.DataCtrlHum));
	dev_bme280->delay(1);
	dev_bme280->write_data_i2c(dev_bme280->dev_address, CTRL_MEAS_ADDR, 1, &dev_bme280->dev_configuration.DataCtrlMeas, sizeof(dev_bme280->dev_configuration.DataCtrlMeas));
	dev_bme280->delay(1);
	dev_bme280->write_data_i2c(dev_bme280->dev_address, CONFIG_ADDR, 1, &dev_bme280->dev_configuration.DataConfig, sizeof(dev_bme280->dev_configuration.DataConfig));
	dev_bme280->delay(1);
 
}

/**
 ** @brief Temperature compensation
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 **  
 ** Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 ** t_fine carries fine temperature as global value.
 ** Data compensation formula/algoritm taken from www.bosch-sensortec.com
 **
 **/
static int32_t BME280_Temperature_Conv(BME280_typedef *dev_bme280)                              
{ 
	int32_t var1, var2, T, T_min = -4000, T_max = 8500;
	uint32_t adc_T;
	uint16_t dig_T1;
	int16_t dig_T2, dig_T3;
  
	dig_T1 = dev_bme280->dev_calibration_data.Dig_T1;
	dig_T2 = dev_bme280->dev_calibration_data.Dig_T2;
	dig_T3 = dev_bme280->dev_calibration_data.Dig_T3;
	
	adc_T = ((((dev_bme280->dev_uncompensated_data.Temp_msb << 8) | dev_bme280->dev_uncompensated_data.Temp_lsb) << 8) | dev_bme280->dev_uncompensated_data.Temp_xlsb_7_4) >> 4;
	
	var1 = (int32_t)((adc_T / 8) - ((int32_t)dig_T1 * 2));
	var1 = (var1 * ((int32_t)dig_T2)) / 2048;
	var2 = (int32_t)((adc_T / 16) - ((int32_t)dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)dig_T3)) / 16384;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) / 256;

	if (T < T_min)
    {T = T_min; }
	else if (T > T_max)
    {T = T_max; }
  
	return T;
}

/**
 ** @brief Humidity compensation
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 ** 
 ** Returns humidity in % RH as unsigned 32 bit integer in Q22.10 format(22 integer and 10 fractional bits).
 ** Output value of “47445” represents 47445/1024 = 46.333 %RH 
 ** Data compensation formula/algoritm taken from www.bosch-sensortec.com
 **
 **/
static uint32_t BME280_Humidity_Conv(BME280_typedef *dev_bme280)                              
{
	int32_t  var1, var2, var3, var4, var5;
	int16_t  dig_H2, dig_H4, dig_H5, dig_H6;
	uint16_t dig_H1, dig_H3;
	uint32_t adc_H, Hum, Hum_max = 102400; 
  
	dig_H1 = dev_bme280->dev_calibration_data.Dig_H1;
	dig_H2 = dev_bme280->dev_calibration_data.Dig_H2;
	dig_H3 = dev_bme280->dev_calibration_data.Dig_H3;
	dig_H4 = (dev_bme280->dev_calibration_data.Dig_H4_11_4 << 4) | (0x0F & dev_bme280->dev_calibration_data.Dig_H5_H4);
	dig_H5 = (dev_bme280->dev_calibration_data.Dig_H5_11_4 << 4) | ((0xF0 & dev_bme280->dev_calibration_data.Dig_H5_H4) >> 4);
	dig_H6 = dev_bme280->dev_calibration_data.Dig_H6;
	
	adc_H = (dev_bme280->dev_uncompensated_data.Hum_msb << 8) | dev_bme280->dev_uncompensated_data.Hum_lsb;

	var1 = t_fine - ((int32_t)76800);
	var2 = (int32_t)(adc_H * 16384);
	var3 = (int32_t)(((int32_t)dig_H4) * 1048576);
	var4 = ((int32_t)dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	Hum = (uint32_t)(var5 / 4096);
	if (Hum > Hum_max){Hum = Hum_max; }

	return Hum;
}

/**
 ** @brief Pressure compensation
 ** 
 ** @param[in] *dev_bme280 : pointer to instance BME280_typedef.
 ** 
 ** Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
 ** Data compensation formula/algoritm taken from www.bosch-sensortec.com
 **
 **/

static uint32_t BME280_Pressure_Conv(BME280_typedef *dev_bme280)                              
{
	int32_t var1, var2, var3, var4;  
	int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	uint16_t dig_P1;
	uint32_t P, adc_P, P_min = 30000, P_max = 110000, var5;
  
	dig_P1 = dev_bme280->dev_calibration_data.Dig_P1;
	dig_P2 = dev_bme280->dev_calibration_data.Dig_P2;
	dig_P3 = dev_bme280->dev_calibration_data.Dig_P3;
	dig_P4 = dev_bme280->dev_calibration_data.Dig_P4;
	dig_P5 = dev_bme280->dev_calibration_data.Dig_P5;
	dig_P6 = dev_bme280->dev_calibration_data.Dig_P6;
	dig_P7 = dev_bme280->dev_calibration_data.Dig_P7;
	dig_P8 = dev_bme280->dev_calibration_data.Dig_P8;
	dig_P9 = dev_bme280->dev_calibration_data.Dig_P9;
	
	adc_P = ((((dev_bme280->dev_uncompensated_data.Press_msb << 8) | dev_bme280->dev_uncompensated_data.Press_lsb) << 8) | dev_bme280->dev_uncompensated_data.Press_xlsb_7_4) >> 4;

	var1 = (((int32_t)t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)dig_P4) * 65536);
	var3 = (dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)dig_P1)) / 32768;
	/* avoid exception caused by division by zero */
	if (var1) 
	{
		var5 = (uint32_t)((uint32_t)1048576) - adc_P;
		P = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (P < 0x80000000)        
		{
			P = (P << 1) / ((uint32_t)var1);
		}
		else
		{
			P = (P / (uint32_t)var1) * 2;
		}
		var1 = (((int32_t)dig_P9) * ((int32_t)(((P / 8) * (P / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(P / 4)) * ((int32_t)dig_P8)) / 8192;
		P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) / 16));

		if (P < P_min){P = P_min; }       
		else 
		{
			if (P > P_max)
			   {P = P_max; }
		}
	} 
	else 
	{
		P = P_min;
	}

	return P;
}


