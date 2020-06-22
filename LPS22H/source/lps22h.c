/** Created: 5.17.2020
 ** lps22h.c
 **/	

#include "stm32l4xx_hal.h"
#include "lps22h.h"

/**
 ** @brief Private function prototype
 ***/		
static void LPS22H_Get_ID(LPS22H_typedef *dev_lps22h);
static void LPS22H_Set_ConfigFifo(LPS22H_typedef *dev_lps22h, LPS22H_FifoCtrl wtm_fifo, LPS22H_FifoCtrl f_mode);
static void LPS22H_Set_OffesetCompensation(LPS22H_typedef *dev_lps22h, int16_t offeset);
static void LPS22H_Set_ReferencePressure(LPS22H_typedef *dev_lps22h, int32_t reference);
static void LPS22H_Set_ThresholdP(LPS22H_typedef *dev_lps22h, uint16_t threshold_p);
static void LPS22H_Set_InterruptSource(LPS22H_typedef *dev_lps22h, LPS22H_IntSource ph, LPS22H_IntSource pl, LPS22H_IntSource ia);
static void LPS22H_Set_InterruptMode(LPS22H_typedef *dev_lps22h, LPS22H_InterruptCfg phe, LPS22H_InterruptCfg ple,
									LPS22H_InterruptCfg lir,  LPS22H_InterruptCfg diff_en, LPS22H_InterruptCfg  reset_az,
									LPS22H_InterruptCfg autozero, LPS22H_InterruptCfg reset_arp, LPS22H_InterruptCfg autorifp);
static void LPS22H_Set_CtrlReg1(LPS22H_typedef *dev_lps22h,	LPS22H_CtrlReg1 spi_mode, LPS22H_CtrlReg1 bdu, 
								LPS22H_CtrlReg1 lpf_state, LPS22H_CtrlReg1 lpf_cfg,	LPS22H_CtrlReg1 odr_mode);
static void LPS22H_Set_CtrlReg2(LPS22H_typedef *dev_lps22h, LPS22H_CtrlReg2 one_shot, LPS22H_CtrlReg2 swreset,
								LPS22H_CtrlReg2 i2c_state, LPS22H_CtrlReg2 if_add_inc, LPS22H_CtrlReg2 stop_on_fth,
								LPS22H_CtrlReg2 fifo_state, LPS22H_CtrlReg2 boot_state);
static void LPS22H_Set_CtrlReg3(LPS22H_typedef *dev_lps22h, LPS22H_CtrlReg3 int_s2_1, LPS22H_CtrlReg3 drdy_state,
								LPS22H_CtrlReg3 f_ovr, LPS22H_CtrlReg3 f_fth, LPS22H_CtrlReg3 f_fss5,
								LPS22H_CtrlReg3 pp_od, LPS22H_CtrlReg3 int_h_l);
static void LPS22H_Set_Profile(LPS22H_Profile profile_type, LPS22H_typedef *dev_lps22h);
static float LPS22H_PressureConv(LPS22H_typedef *dev_lps22h);
static void LPS22H_Reset_Filter(LPS22H_typedef *dev_lps22h);

/**
 ** @brief Init device  
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 **
 **/
void LPS22H_InitDevice(LPS22H_typedef *dev_lps22h)                              
{	
	LPS22H_Set_Profile(LPS22H_PROFILE_FIFO_BYPASS, dev_lps22h);
}

/**
 ** @brief Get P, T. Also it gets values of BootStatus, FifoStatus, Status registers.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 **
 **/				
LPS22H_TempPressStruct_typedef* LPS22H_Get_TempPress(LPS22H_typedef *dev_lps22h)                              
{
	dev_lps22h->dev_compensated_data.PressuremBar = LPS22H_PressureConv(dev_lps22h);
	dev_lps22h->dev_compensated_data.PressuremmHg = dev_lps22h->dev_compensated_data.PressuremBar / 1.333f;
	dev_lps22h->dev_compensated_data.PressurePa = dev_lps22h->dev_compensated_data.PressuremBar * 100;
	
	dev_lps22h->dev_compensated_data.TemperatureC = (float)dev_lps22h->dev_uncompensated_data.Temp_Out / 100;	
	dev_lps22h->dev_compensated_data.TemperatureF = (dev_lps22h->dev_compensated_data.TemperatureC * 1.8f) + 32;
	
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_INT_SOURCE, 1, (uint8_t *)(&dev_lps22h->dev_uncompensated_data), sizeof(dev_lps22h->dev_uncompensated_data) - 1);
	dev_lps22h->delay(1);
	
	return  &dev_lps22h->dev_compensated_data;
}

/**
 ** @brief Reset filter.
 **
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 **
 **/
static void LPS22H_Reset_Filter(LPS22H_typedef *dev_lps22h)                              
{
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_LPFP_RES, 1, (uint8_t *)(&dev_lps22h->dev_uncompensated_data.LpfpRes), sizeof(dev_lps22h->dev_uncompensated_data.LpfpRes));
	dev_lps22h->delay(10);
}

/**
 ** @brief Get device's id
 **
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 **
 **/
static void LPS22H_Get_ID(LPS22H_typedef *dev_lps22h)                              
{
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_WHO_AM_I, 1, (uint8_t *)(&dev_lps22h->dev_compensated_data.dev_id), sizeof(dev_lps22h->dev_compensated_data.dev_id));
	dev_lps22h->delay(1);
}

/**
 ** @brief Configuration fifo.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] wtm_fifo : watermark level selection.  
 ** @param[in] f_mode : fifo mode selection.
 ** 
 ** To convert the WTM bits into the number of levels stored in the FIFO, it is 
 ** sufficient to convert from binary to decimal the value of the WTM 
 ** bits and add 1. The FIFO buffer can store up to 32 levels of data.
 **
 **/
static void LPS22H_Set_ConfigFifo(LPS22H_typedef *dev_lps22h, LPS22H_FifoCtrl wtm_fifo, LPS22H_FifoCtrl f_mode)                              
{
	
	dev_lps22h->dev_configuration.bitsFifoCtrl.f_mode_2_0 = f_mode;
	dev_lps22h->dev_configuration.bitsFifoCtrl.wtm_4_0 = wtm_fifo;
	
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_FIFO_CTRL, 1, (uint8_t *)&dev_lps22h->dev_configuration.FifoCtrl, sizeof(dev_lps22h->dev_configuration.FifoCtrl));
	dev_lps22h->delay(1);
}

/**
 ** @brief Offset compensation (OPC)
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] offset : If, after the soldering of the component, 
 ** a residual offset is still present, it can be removed with a 
 ** one-point calibration (OPC). After soldering, the measured offset 
 ** can be stored in the RPDS (18h, 19h) registers and automatically 
 ** subtracted from the pressure output registers the output pressure 
 ** register PRESS_OUT (28h, 29h and 2Ah) is provided as the difference 
 ** between the measured pressure and the content of the register 
 ** 256*RPDS (18h, 19h) (DIFF_EN = ‘0’, AUTOZERO =‘0’, AUTORIFP= ‘0’).
 **
 **/
static void LPS22H_Set_OffesetCompensation(LPS22H_typedef *dev_lps22h, int16_t offeset)                              
{	
	dev_lps22h->dev_configuration.Rpds = offeset;
	
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_RPDS_L, 1, (uint8_t *)&dev_lps22h->dev_configuration.Rpds, sizeof(dev_lps22h->dev_configuration.Rpds));
	dev_lps22h->delay(1);
}

/**
 ** @brief Set reference pressure value.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] reference : reference pressure value, [23:0]. 
 ** The value is expressed as 2’s complement (three bytes).
 **
 **/
static void LPS22H_Set_ReferencePressure(LPS22H_typedef *dev_lps22h, int32_t reference)                              
{	
	
	dev_lps22h->dev_configuration.RefP = (((reference & 0x80000000) >> 8) | reference); 
	///WARNING! Return only 3 low bytes.
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_REF_P_XL, 1, (uint8_t *)&dev_lps22h->dev_configuration.RefP, sizeof(dev_lps22h->dev_configuration.RefP) - 1);
	dev_lps22h->delay(1);
}

/**
 ** @brief The threshold value for pressure interrupt generation. 
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] threshold_p : it is a 15-bit unsigned right-justified value.
 ** Interrupt threshold (hPA) = THS_P / 16
 **
 **/
static void LPS22H_Set_ThresholdP(LPS22H_typedef *dev_lps22h, uint16_t threshold_p)                              
{	
	dev_lps22h->dev_configuration.ThresholdP = threshold_p;
	
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_THS_P_L, 1, (uint8_t *)&dev_lps22h->dev_configuration.ThresholdP, sizeof(dev_lps22h->dev_configuration.ThresholdP));
	dev_lps22h->delay(1);
}

/**
 ** @brief Configuration fifo.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] ph : differential pressure High. 
 ** @param[in] pl : differential pressure Low.
 ** @param[in] ia : interrupt active.
 **
 **/
static void LPS22H_Set_InterruptSource(LPS22H_typedef *dev_lps22h, LPS22H_IntSource ph, LPS22H_IntSource pl, LPS22H_IntSource ia)                              
{	
	dev_lps22h->dev_configuration.bitsIntSource.ph = ph;
	dev_lps22h->dev_configuration.bitsIntSource.pl = pl;
	dev_lps22h->dev_configuration.bitsIntSource.ia = ia;
	
	dev_lps22h->read_data_i2c(dev_lps22h->dev_address, LPS22H_INT_SOURCE, 1, (uint8_t *)&dev_lps22h->dev_configuration.IntSource, sizeof(dev_lps22h->dev_configuration.IntSource));
	dev_lps22h->delay(1);
}

/**
 ** @brief Interrupt mode settings.
 ** 
 ** The LPS22HB can be configured to generate interrupt events related 
 ** to pressure acquisition and FIFO status. A dedicated pad (INT_DRDY) 
 ** can be set for selected interrupt events.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] phe : enable interrupt generation on pressure high event.	
 **	@param[in] ple : enable interrupt generation on pressure low event.
 **	@param[in] lir : latch interrupt request to the INT_SOURCE (25h) register.
 **	@param[in] diff_en : enable interrupt generation.
 **	@param[in] reset_az : reset Autozero function.
 **	@param[in] autozero : enable Autozero function.
 **	@param[in] reset_arp : reset AutoRifP function.
 **	@param[in] autorifp	: enable AUTORIFP function.
 **	
 **/
static void LPS22H_Set_InterruptMode(LPS22H_typedef *dev_lps22h, LPS22H_InterruptCfg phe, LPS22H_InterruptCfg ple, LPS22H_InterruptCfg lir,
								LPS22H_InterruptCfg diff_en, LPS22H_InterruptCfg  reset_az, LPS22H_InterruptCfg autozero,
								LPS22H_InterruptCfg reset_arp, LPS22H_InterruptCfg autorifp)                              
{
	dev_lps22h->dev_configuration.bitsInterruptCfg.phe = phe;
	dev_lps22h->dev_configuration.bitsInterruptCfg.ple = ple;
	dev_lps22h->dev_configuration.bitsInterruptCfg.lir = lir;	
	dev_lps22h->dev_configuration.bitsInterruptCfg.diff_en = diff_en;
	dev_lps22h->dev_configuration.bitsInterruptCfg.reset_az = reset_az;
	dev_lps22h->dev_configuration.bitsInterruptCfg.autozero = autozero;
	dev_lps22h->dev_configuration.bitsInterruptCfg.reset_arp = reset_arp;
	dev_lps22h->dev_configuration.bitsInterruptCfg.autorifp = autorifp;
	
	dev_lps22h->write_data_i2c(dev_lps22h->dev_address, LPS22H_INTERRUPT_CFG, 1, &dev_lps22h->dev_configuration.InterruptCfg, sizeof(dev_lps22h->dev_configuration.InterruptCfg));
	dev_lps22h->delay(1); 
}

/**
 ** @brief Configuration CTRL_REG1 register.
 ** 
 ** @param[in] *dev_lps22h	: pointer to instance LPS22H_typedef.
 ** @param[in] spi_mode		: serial Interface Mode selection.	
 **	@param[in] bdu			: block data update.
 **	@param[in] lpf_state	: enable/disable low-pass filter on pressure data when continuous mode is used.
 **	@param[in] lpf_cfg		: low-pass filter configuration.
 **	@param[in] odr_mode		: output data rate selection.
 **	
 **/
static void LPS22H_Set_CtrlReg1(LPS22H_typedef *dev_lps22h, LPS22H_CtrlReg1 spi_mode, LPS22H_CtrlReg1 bdu, 
								LPS22H_CtrlReg1 lpf_state, LPS22H_CtrlReg1 lpf_cfg, LPS22H_CtrlReg1 odr_mode)                              
{
	dev_lps22h->dev_configuration.bitsCtrlReg1.sim = spi_mode;
	dev_lps22h->dev_configuration.bitsCtrlReg1.bdu = bdu;
	dev_lps22h->dev_configuration.bitsCtrlReg1.lpfp_cfg = lpf_cfg;
	dev_lps22h->dev_configuration.bitsCtrlReg1.en_lpfp = lpf_state;
	dev_lps22h->dev_configuration.bitsCtrlReg1.odr_2_0 = odr_mode;
	
	dev_lps22h->write_data_i2c(dev_lps22h->dev_address, LPS22H_CTRL_REG1, 1, &dev_lps22h->dev_configuration.CtrlReg1, sizeof(dev_lps22h->dev_configuration.CtrlReg1));
	dev_lps22h->delay(1); 
}

/**
 ** @brief Configuration CTRL_REG2 register.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] one_shot : one-shot enable
 **	@param[in] swreset : software reset.
 **	@param[in] i2c_state : disable/enable I2C interface
 **	@param[in] if_add_inc : register address automatically incremented 
 **	during a multiple byte access with a serial interface (I2C or SPI).
 **	@param[in] stop_on_fth : stop on FIFO watermark.
 **	@param[in] fifo_state : fifo disable/enable.
 **	@param[in] boot_state : reboot memory content.
 **	
 **/
static void LPS22H_Set_CtrlReg2(LPS22H_typedef *dev_lps22h, LPS22H_CtrlReg2 one_shot, LPS22H_CtrlReg2 swreset, LPS22H_CtrlReg2 i2c_state,
						LPS22H_CtrlReg2 if_add_inc, LPS22H_CtrlReg2 stop_on_fth, LPS22H_CtrlReg2 fifo_state, LPS22H_CtrlReg2 boot_state)                              
{
	dev_lps22h->dev_configuration.bitsCtrlReg2.one_shot = one_shot;
	dev_lps22h->dev_configuration.bitsCtrlReg2.swreset = swreset;
	dev_lps22h->dev_configuration.bitsCtrlReg2.i2c_dis = i2c_state;
	dev_lps22h->dev_configuration.bitsCtrlReg2.if_add_inc = if_add_inc;
	dev_lps22h->dev_configuration.bitsCtrlReg2.stop_on_fth = stop_on_fth;
	dev_lps22h->dev_configuration.bitsCtrlReg2.fifo_en = fifo_state;
	dev_lps22h->dev_configuration.bitsCtrlReg2.boot = boot_state;
	
	dev_lps22h->write_data_i2c(dev_lps22h->dev_address, LPS22H_CTRL_REG2, 1, &dev_lps22h->dev_configuration.CtrlReg2, sizeof(dev_lps22h->dev_configuration.CtrlReg2));
	dev_lps22h->delay(1); 
	
}

/**
 ** @brief Configuration CTRL_REG3 register.
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** @param[in] int_s2_1 : data signal on INT_DRDY pin control bits.
 **	@param[in] drdy_state : data-ready signal on INT_DRDY pin.
 **	@param[in] f_ovr : FIFO overrun interrupt on INT_DRDY pin.
 **	@param[in] f_fth : FIFO watermark status on INT_DRDY pin.
 **	@param[in] f_fss5 : FIFO full flag on INT_DRDY pin.
 **	@param[in] pp_od : push-pull/open drain selection on interrupt pads.
 **	@param[in] int_h_l : interrupt active-high/low.
 **	
 **/
static void LPS22H_Set_CtrlReg3(LPS22H_typedef *dev_lps22h, LPS22H_CtrlReg3 int_s2_1, LPS22H_CtrlReg3 drdy_state, LPS22H_CtrlReg3 f_ovr,
								LPS22H_CtrlReg3 f_fth, LPS22H_CtrlReg3 f_fss5, LPS22H_CtrlReg3 pp_od, LPS22H_CtrlReg3 int_h_l)                              
{
	dev_lps22h->dev_configuration.bitsCtrlReg3.int_s_2_1 = int_s2_1;
	dev_lps22h->dev_configuration.bitsCtrlReg3.drdy = drdy_state;
	dev_lps22h->dev_configuration.bitsCtrlReg3.f_ovr = f_ovr;
	dev_lps22h->dev_configuration.bitsCtrlReg3.f_fth = f_fth;
	dev_lps22h->dev_configuration.bitsCtrlReg3.f_fss5 = f_fss5;
	dev_lps22h->dev_configuration.bitsCtrlReg3.pp_od = pp_od;
	dev_lps22h->dev_configuration.bitsCtrlReg3.int_h_l = int_h_l;
	
	dev_lps22h->write_data_i2c(dev_lps22h->dev_address, LPS22H_CTRL_REG3, 1, &dev_lps22h->dev_configuration.CtrlReg3, sizeof(dev_lps22h->dev_configuration.CtrlReg3));
	dev_lps22h->delay(1); 
}

/**
 ** @brief Measurement profiles
 ** 
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 **
 **/
static void LPS22H_Set_Profile(LPS22H_Profile profile_type, LPS22H_typedef *dev_lps22h)                              
{
	switch (profile_type)
	{
	case LPS22H_PROFILE_FIFO_BYPASS:
		
		LPS22H_Get_ID(dev_lps22h);
		LPS22H_Set_ConfigFifo(dev_lps22h, 0, LPS22H_F_MODE_BYPASS_MODE);
		LPS22H_Set_OffesetCompensation(dev_lps22h, 0);
		LPS22H_Set_ReferencePressure(dev_lps22h, 0);
		LPS22H_Set_ThresholdP(dev_lps22h, 0);
		LPS22H_Set_CtrlReg1(dev_lps22h, LPS22H_SIM_3_WIRE_INTERFACE, LPS22H_BDU_DISCONTINUOUS_UPDATE, 
							LPS22H_LPFP_CFG_LPF_ENABLE_ODR_DIV9, LPS22H_DIS_LPFP, LPS22H_ODR_P1Hz_T1Hz);
		LPS22H_Set_CtrlReg2(dev_lps22h, LPS22H_ONE_SHOT_IDLE_MODE, LPS22H_SWRESET_NORMAL_MODE, LPS22H_I2C_EN, 
							LPS22H_IF_ADD_INC_EN, LPS22H_STOP_ON_FTH_DIS, LPS22H_FIFO_DIS, LPS22H_BOOT_NORMAL_MODE);
		LPS22H_Set_CtrlReg3(dev_lps22h, LPS22H_INT_S_IN_ORDER_OF_PRIORITY, LPS22H_DRDY_EN, LPS22H_F_OVR_DIS,
							LPS22H_F_FTH_DIS, LPS22H_F_FSS5_DIS, LPS22H_PP_OD_PUSH_PULL, LPS22H_INT_H_L_ACTIVE_LOW);
		LPS22H_Set_InterruptMode(dev_lps22h, LPS22H_PHE_DIS, LPS22H_PLE_DIS, LPS22H_LIR_DIS, LPS22H_DIFF_DIS,
							LPS22H_RESET_AZ_NORMAL_MODE, LPS22H_AUTOZERO_NORMAL_MODE, LPS22H_RESET_ARP_NORMAL_MODE, 
							LPS22H_AUTORIFP_NORMAL_MODE);
		break;
		
	default : break;
	}
}

/**
 ** @brief 
 **
 ** @param[in] *dev_lps22h : pointer to instance LPS22H_typedef.
 ** 
 **/
static float LPS22H_PressureConv(LPS22H_typedef *dev_lps22h)                              
{ 
	///Test 
	int32_t pressure;
	uint32_t sign;
	
	sign = (uint32_t)(dev_lps22h->dev_uncompensated_data.Press_Out_H & 0x80) << 24;
	pressure = (uint32_t)(dev_lps22h->dev_uncompensated_data.Press_Out_H & 0x7F) << 16;
	pressure |= (uint32_t)(dev_lps22h->dev_uncompensated_data.Press_Out_L) << 8;
	pressure |= (uint32_t)dev_lps22h->dev_uncompensated_data.Press_Out_XL;
	
	return (float)pressure / 4096;	
}






