/** Created: 5.02.2020
 ** HTS221.c
 **/	

#include "stm32l4xx_hal.h"
#include "hts221.h"


/**
 ** @brief Private function prototype
 ***/				
static int32_t HTS221_Temperature_Conv(HTS221_typedef *dev_hts221);                   
static uint32_t HTS221_Humidity_Conv(HTS221_typedef *dev_hts221);     
static void HTS221_Get_Calibration_Data(HTS221_typedef *dev_hts221); 
static void HTS221_Get_ID(HTS221_typedef *dev_hts221);
static void HTS221_Set_Av_Conf(HTS221_typedef *dev_hts221, HTS221_AvConf av_conf_t, HTS221_AvConf av_conf_h);
static void HTS221_Set_Ctrl_Reg_1(HTS221_typedef *dev_hts221, HTS221_CtrlReg1 odr, HTS221_CtrlReg1 bdu, HTS221_CtrlReg1 pd);
static void HTS221_Set_Ctrl_Reg_2(HTS221_typedef *dev_hts221, HTS221_CtrlReg2 one_shot, HTS221_CtrlReg2 heater, HTS221_CtrlReg2 boot); 
static void HTS221_Set_Ctrl_Reg_3(HTS221_typedef *dev_hts221, HTS221_CtrlReg3 drdy_en, HTS221_CtrlReg3 pp_od, HTS221_CtrlReg3 drdy_h_l);



/**
 ** @brief Init device  
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 **
 **/
void HTS221_Init_Device(HTS221_typedef *dev_hts221)                              
{	
	
	//Config work mode for device
	HTS221_Set_Av_Conf(dev_hts221, HTS221_AVG_T16_H32, HTS221_AVG_T16_H32);
	HTS221_Set_Ctrl_Reg_1(dev_hts221, HTS221_ODR_RATE_H1T1Hz, HTS221_BDU_CONTINUOUS_UPDATE, HTS221_PD_ACTIVE_MODE);
	HTS221_Set_Ctrl_Reg_2(dev_hts221, HTS221_ONE_SHOT_DIS, HTS221_HEATER_DIS, HTS221_BOOT_NORMAL_MODE);
	HTS221_Set_Ctrl_Reg_3(dev_hts221, HTS221_DRDY_DIS, HTS221_PP_OD_PUSH_PULL, HTS221_DRDY_H_L_ACTIVE_HIGH);
	
	HTS221_Get_Calibration_Data(dev_hts221);
	HTS221_Get_ID(dev_hts221);
	
}

/**
 ** @brief Get T, H.
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 **
 **/				
HTS221_TempHumPressStruct_typedef* HTS221_Get_Data_Temp_Hum(HTS221_typedef *dev_hts221)                              
{
	dev_hts221->dev_compensated_data.TemperatureC = HTS221_Temperature_Conv(dev_hts221);
	dev_hts221->dev_compensated_data.TemperatureF = ((HTS221_Temperature_Conv(dev_hts221) * 18) + 32000) / 10;
	dev_hts221->dev_compensated_data.HumidityRH = HTS221_Humidity_Conv(dev_hts221);
	
	dev_hts221->read_data_i2c(dev_hts221->dev_address, HTS221_STATUS_REG, 1, (uint8_t *)(&dev_hts221->dev_uncompensated_data), sizeof(dev_hts221->dev_uncompensated_data));
	dev_hts221->delay(1);
	
	return  &dev_hts221->dev_compensated_data;
}

/**
 ** @brief Get calibration data full range
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 **
 **/
static void HTS221_Get_Calibration_Data(HTS221_typedef *dev_hts221)                              
{
	//write receive's register address to hts221 Config Struct
	dev_hts221->read_data_i2c(dev_hts221->dev_address, HTS221_CALIB_START, 1, (uint8_t *)&dev_hts221->dev_calibration_data, sizeof(dev_hts221->dev_calibration_data));
	dev_hts221->delay(1);
}

/**
 ** @brief Get device's id
 **
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 **
 **/
static void HTS221_Get_ID(HTS221_typedef *dev_hts221)                              
{
	//write receive's register address to hts221 Config Struct
	dev_hts221->read_data_i2c(dev_hts221->dev_address, HTS221_WHO_AM_I, 1, (uint8_t *)(&dev_hts221->dev_compensated_data.dev_id), sizeof(dev_hts221->dev_compensated_data.dev_id));
	dev_hts221->delay(1);
}

/**
 ** @brief Configuration AV_CONF register.
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @param[in] av_conf_t : select the numbers of averaged temperature samples, refer value to HTS221_Av_Conf_enum.	
 **	@param[in] av_conf_h : select the numbers of averaged humidity samples, refer value to HTS221_Av_Conf_enum.
 **	
 **/
static void HTS221_Set_Av_Conf(HTS221_typedef *dev_hts221, HTS221_AvConf av_conf_t, HTS221_AvConf av_conf_h)                              
{
	dev_hts221->dev_configuration.bitsAvConf.avgt_5_3 = av_conf_t;
	dev_hts221->dev_configuration.bitsAvConf.avgh_2_0 = av_conf_h;
	dev_hts221->dev_configuration.bitsAvConf.reserved_7_6 = 0;
	
	dev_hts221->write_data_i2c(dev_hts221->dev_address, HTS221_AV_CONF, 1, &dev_hts221->dev_configuration.Av_Conf, sizeof(dev_hts221->dev_configuration.Av_Conf));
	dev_hts221->delay(1); 
}

/**
 ** @brief Configuration CTRL_REG_1 register.
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @param[in] odr : output data rate selection.	
 **	@param[in] bdu : block data update, 0: continuous update, 1: output registers not updated until MSB and LSB reading.
 **	@param[in] pd : power-down control, 0: power-down mode, 1: active mode.
 **	
 **/
static void HTS221_Set_Ctrl_Reg_1(HTS221_typedef *dev_hts221, HTS221_CtrlReg1 odr, HTS221_CtrlReg1 bdu, HTS221_CtrlReg1 pd)                              
{
	dev_hts221->dev_configuration.bitsCtrlReg1.odr_1_0 = odr;
	dev_hts221->dev_configuration.bitsCtrlReg1.bdu = bdu;
	dev_hts221->dev_configuration.bitsCtrlReg1.pd = pd;	
	
	dev_hts221->write_data_i2c(dev_hts221->dev_address, HTS221_CTRL_REG_1, 1, &dev_hts221->dev_configuration.CtrlReg1, sizeof(dev_hts221->dev_configuration.CtrlReg1));
	dev_hts221->delay(1); 
}

/**
 ** @brief Configuration CTRL_REG_2 register.
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @param[in] one_shot : one-shot enable, 0: waiting for start of conversion, 1: start for a new dataset.	
 **	@param[in] heater : 0: heater disable; 1: heater enable.
 **	@param[in] boot : reboot memory content, 0: normal mode, 1: reboot memory content.
 **	
 **/
static void HTS221_Set_Ctrl_Reg_2(HTS221_typedef *dev_hts221, HTS221_CtrlReg2 one_shot, HTS221_CtrlReg2 heater, HTS221_CtrlReg2 boot)                              
{
	dev_hts221->dev_configuration.bitsCtrlReg2.one_shot = one_shot;
	dev_hts221->dev_configuration.bitsCtrlReg2.heater = heater;
	dev_hts221->dev_configuration.bitsCtrlReg2.boot = boot;	
	
	dev_hts221->write_data_i2c(dev_hts221->dev_address, HTS221_CTRL_REG_2, 1, &dev_hts221->dev_configuration.CtrlReg2, sizeof(dev_hts221->dev_configuration.CtrlReg2));
	dev_hts221->delay(1); 
}

/**
 ** @brief Configuration CTRL_REG_3 register.
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @param[in] one_shot : one-shot enable, 0: waiting for start of conversion, 1: start for a new dataset.	
 **	@param[in] heater : 0: heater disable; 1: heater enable.
 **	@param[in] boot : reboot memory content, 0: normal mode, 1: reboot memory content.
 **	
 **/
static void HTS221_Set_Ctrl_Reg_3(HTS221_typedef *dev_hts221, HTS221_CtrlReg3 drdy_en, HTS221_CtrlReg3 pp_od, HTS221_CtrlReg3 drdy_h_l)                              
{
	dev_hts221->dev_configuration.bitsCtrlReg3.drdy_en = drdy_en;
	dev_hts221->dev_configuration.bitsCtrlReg3.pp_od = pp_od;
	dev_hts221->dev_configuration.bitsCtrlReg3.drdy_h_l = drdy_h_l;	
	
	dev_hts221->write_data_i2c(dev_hts221->dev_address, HTS221_CTRL_REG_3, 1, &dev_hts221->dev_configuration.CtrlReg3, sizeof(dev_hts221->dev_configuration.CtrlReg3));
	dev_hts221->delay(1); 
}

/**
 ** @brief Temperature compensation
 **
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @return Temperature value that must be divided by 100 to get the value in ['C].
 **
 **/
static int32_t HTS221_Temperature_Conv(HTS221_typedef *dev_hts221)                              
{ 
	int32_t T_degC = 0;
	int16_t T1_degC, T0_degC;
	int16_t T10_Out = 0;	
	
	T1_degC = ((uint16_t)dev_hts221->dev_calibration_data.T1_T0_msb & 0x0C) << 6;
	T1_degC |= ((uint16_t)(dev_hts221->dev_calibration_data.T1_degC_x8));
	T1_degC = T1_degC >> 3;
	
	T0_degC = ((uint16_t)dev_hts221->dev_calibration_data.T1_T0_msb & 0x03) << 8;
	T0_degC |= ((uint16_t)(dev_hts221->dev_calibration_data.T0_degC_x8));
	T0_degC = T0_degC >> 3;
 
	T10_Out = dev_hts221->dev_calibration_data.T1_OUT - dev_hts221->dev_calibration_data.T0_OUT;	
	
	T_degC = (T1_degC - T0_degC) * (dev_hts221->dev_uncompensated_data.Temp_Out - dev_hts221->dev_calibration_data.T0_OUT) * 100;
	T_degC = (T_degC / T10_Out) + T0_degC * 100 ;
	
	if(T_degC < -40) T0_degC = -40;
	if (T0_degC > 120) T0_degC = 120;
	
	return T_degC;
}


/**
 ** @brief Humidity compensation
 ** 
 ** @param[in] *dev_hts221 : pointer to instance HTS221_typedef.
 ** @return Humidity value that must be divided by 100 to get the value in [%].
 **
 **/
static uint32_t HTS221_Humidity_Conv(HTS221_typedef *dev_hts221)                             
{ 
	int32_t H_Rh = 0;
	int16_t H10_rH = 0;
	int16_t H10_T0 = 0;	
	
	H10_rH = (dev_hts221->dev_calibration_data.H1_rH_x2 >> 1) - (dev_hts221->dev_calibration_data.H0_rH_x2 >> 1);		// >> 1 is divide on 2
	H10_T0 = (dev_hts221->dev_calibration_data.H1_T0_OUT - dev_hts221->dev_calibration_data.H0_T0_OUT);
	
	H_Rh = (H10_rH * (dev_hts221->dev_uncompensated_data.Humidity_Out - dev_hts221->dev_calibration_data.H0_T0_OUT) * 100);
	H_Rh = (H_Rh / H10_T0) + (dev_hts221->dev_calibration_data.H0_rH_x2 >> 1) * 100;
	
	if (H_Rh > 10000 || H_Rh < 0) H_Rh = 10000;
	
	return H_Rh;
}



