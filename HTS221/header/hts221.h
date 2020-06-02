/** Created: 5.02.2020
 ** HTS221.h
 **/	
#pragma once
//#ifndef HTS221_H_
//#define HTS221_H_

#include "stm32l4xx_hal.h"


/**
 ** @brief Device address
 **/
#define ADDR_HTS221_SHIFTED             0xBE                                //7 bit MSB 0x76 (address) + 1 bit LSB 0x00 (read/write)
#define ADDR_HTS221						0x5F								//It's not shifted address
///An 8 - bit sub - address(SUB) will be transmitted : the
///7 LSB represents the actual register address while the MSB enables address autoincrement.
///If the MSB of the SUB field is ‘1’, the SUB(register address) will be automatically
///increased to allow multiple data read / write.
#define ADDR_AUTOINC_ENABLE				0x80

/**
 ** @brief Register mapping
 ** 
 ** Provides a list of the 8-bit registers embedded in the device and the related
 ** addresses.
 **/
typedef enum 
{
	///Device identification, type R, defalt 0xBC
	WHO_AM_I		= 0x0F,
	///Humidity and temperature resolution mode, type R/W, default 0x1B
	AV_CONF			= 0x10,
	///control register 1
	CTRL_REG_1		= 0x20,
	///Control register 2
	CTRL_REG_2		= 0x21,
	///Control register 3
	CTRL_REG_3		= 0x22,
	///Status register
	STATUS_REG		= 0x27 | ADDR_AUTOINC_ENABLE,
	///Humidity output registers
	HUMIDITY_OUT_L	= 0x28 | ADDR_AUTOINC_ENABLE,
	HUMIDITY_OUT_H	= 0x29,
	///Temperature output registers
	TEMP_OUT_L		= 0x2A | ADDR_AUTOINC_ENABLE,
	TEMP_OUT_H		= 0x2B,
	///Calibration data registers, DON'T MODIFY, range 0x30 - 0x3F
	CALIB_START		= 0x30 | ADDR_AUTOINC_ENABLE,
	CALIB_STOP		= 0x3F,
	
	
}  HTS221_AddressReg;

/**
 ** @brief Averaged samples select, default temperature samples 16, humidity samples 32
 ** 
 **/
typedef enum 
{
	///averaged temperature samples 2, averaged humidity samples 4
	AVG_T2_H4		= 0,
	///averaged temperature samples 4, averaged humidity samples 8
	AVG_T4_H8		= 1,
	///averaged temperature samples 8, averaged humidity samples 16
	AVG_T8_H16		= 2,
	///averaged temperature samples 16, averaged humidity samples 32
	AVG_T16_H32		= 3,
	///averaged temperature samples 32, averaged humidity samples 64
	AVG_T32_H64		= 4,
	///averaged temperature samples 64, averaged humidity samples 128
	AVG_T64_H128	= 5,
	///averaged temperature samples 128, averaged humidity samples 256
	AVG_T128_H256	= 6,
	///averaged temperature samples 256, averaged humidity samples 512
	AVG_T256_H512	= 7,
	
	
}  HTS221_AvConf;

/**
 ** @brief Control register 1 const config
 ** 
 **/
typedef enum 
{
	///output data rate selection, one-shot.
	ODR_RATE_ONE_SHOT			= 0,
	///output data rate selection, humidity 1Hz, temperature 1Hz.
	ODR_RATE_H1T1Hz				= 1,
	///output data rate selection, humidity 7Hz, temperature 7Hz.
	ODR_RATE_H7T7Hz				= 2,
	///output data rate selection, humidity 12.5Hz, temperature 12.5Hz.
	ODR_RATE_H125T125Hz			= 3,
	///block data update, 0: continuous update, 1: output registers not updated until MSB and LSB reading.
	BDU_CONTINUOUS_UPDATE		= 0,
	BDU_DISCONTINUOUS_UPDATE	= 1,
	///power-down control, 0: power-down mode, 1: active mode.
	PD_POWER_DOWN				= 0,
	PD_ACTIVE_MODE				= 1,	
	
}  HTS221_CtrlReg1;

/**
 ** @brief Control register 2 const config
 ** 
 **/
typedef enum 
{
	///One-shot enable, 0: waiting for start of conversion, 1: start for a new dataset.
	ONE_SHOT_DIS = 0,
	ONE_SHOT_EN = 1,
	///Heater, 0: heater disable, 1: heater enable.
	HEATER_DIS = 0,
	HEATER_EN = 1,
	///BOOT: Reboot memory content, 0: normal mode, 1: reboot memory content.
	BOOT_NORMAL_MODE = 0,
	BOOT_REBOOT_MEMORY_CONTENT = 1,
	
}  HTS221_CtrlReg2;

/**
 ** @brief Control register 3 const config
 ** 
 **/
typedef enum 
{
	///Data Ready enable, 0: Data Ready disabled - default, 1: Data Ready signal available on pin 3.
	DRDY_DIS = 0,
	DRDY_EN = 1,
	///Push-pull / Open Drain selection on pin 3 (DRDY), 0: push-pull - default, 1: open drain.
	PP_OD_PUSH_PULL = 0,
	PP_OD_OPEN_DRAIN = 1,
	///Data Ready output signal active high, low, 0: active high - default, 1: active low.
	DRDY_H_L_ACTIVE_HIGH = 0,
	DRDY_H_L_ACTIVE_LOW = 1,
	
}  HTS221_CtrlReg3;

/**
 ** @brief Device specific function type
 ** 
 ** @param[in] DevAddress : Device address on the I2C bus
 ** @param[in] MemAddress : Register address for device
 ** @param[in] MemAddSize : Size address of memory
 ** @param[in] *pData : Pointer on data struct instance
 ** @param[in] Size : Size of transmition/reception data
 **
 ** @return Result of API execution status
 ** @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 **/
typedef int8_t(*hts221_communication_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
typedef void(*hts221_delay_fptr)(uint32_t period);

/**
 ** @brief Compensated data
 **/
typedef struct {
	uint8_t dev_id;	
	int32_t TemperatureC;
	int32_t TemperatureF;
	uint32_t HumidityRH;
	
} HTS221_TempHumPressStruct_typedef;

/**
 ** @brief Configuration registers
 ** 
 ** AV_CONF register. Numbers of averaget samples. Default value is 011.
 ** AVGx2:0		Temperature		Humidity 
 **	  000			2				4
 **	  001			4				8
 **	  010			8				16
 **	  011			16				32
 **	  100			32				64
 **	  101			64				128
 **	  110			128				256
 **	  111			256				512
 **	  
 ** CTRL_REG1. Control register 1
 ** Output data rate selection
 **	  ODR		Humidity(Hz)	Temperature(Hz) 
 **	   00		  one_shot		    one_shot
 **	   01			 1				  1	
 **    10			 7				  7
 **    11			12.5			 12.5
 **    
 **/
typedef struct {
	union {	
		///Humidity and temperature resolution mode
		uint8_t Av_Conf;
		struct {
			///averaged humidity samples
			uint8_t		avgh_2_0		: 3;   
			///averaged temperature samples
			uint8_t		avgt_5_3		: 3;	
			uint8_t		reserved_7_6	: 2;
		} bitsAvConf;				
	};

	union {
		///Control register 1 (20h)
		uint8_t CtrlReg1;
		struct {
			///output data rate selection, detailed see upper.
			uint8_t		odr_1_0			: 2;   
			///block data update. 
			///0: continuous update, 1: output registers not updated until MSB and LSB reading. 
			uint8_t		bdu				: 1;
			uint8_t		reserved_6_3	: 4;	
			///power-down control
			///0: power-down mode, 1: active mode.
			uint8_t		pd				: 1; 
		} bitsCtrlReg1;
	};

	union {
		///Control register 2 (21h)
		uint8_t CtrlReg2;
		struct {
			///One-shot enable
			///0: waiting for start of conversion, 1: start for a new dataset.
			uint8_t   one_shot			: 1;   
			///Heater.
			///0: heater disable; 1: heater enable
			uint8_t   heater			: 1;
			uint8_t   reserved_6_2		: 5; 
			///BOOT: Reboot memory content
			///0: normal mode; 1: reboot memory content
			uint8_t   boot				: 1; 
		} bitsCtrlReg2;
	};
	
	union {
		///Control register 3 (22h)
		uint8_t CtrlReg3;
		struct {
			uint8_t		reserved_1_0	: 2;   
			///DRDY_EN: Data Ready enable.
			///0: Data Ready disabled - default, 1: Data Ready signal available on pin 3.
			uint8_t		drdy_en			: 1;
			uint8_t		reserved_5_3	: 3; 
			///PP_OD: Push-pull / Open Drain selection on pin 3 (DRDY).
			///0: push-pull - default, 1: open drain.
			uint8_t		pp_od			: 1; 
			///DRDY_H_L: Data Ready output signal active high, low.
			///0: active high - default, 1: active low.
			uint8_t		drdy_h_l		: 1;
			
		} bitsCtrlReg3;
	};
	
}  HTS221_ConfigStruct_typedef;

/**
 ** @brief Data receive parametr
 **/
typedef struct __attribute__((aligned(1), packed)) {
	///Bit 0 T_DA: Temperature data available. 
	///0: new data for temperature is not yet available, 1: new data for temperature is available.
	///Bit 1 H_DA: Humidity data available. 
	///0: new data for humidity is not yet available, 1: new data for humidity is available.
	uint8_t Status_Reg;	
	union 
	{
		int16_t Humidity_Out;
		struct 
		{
			///Relative humidity data (LSB)
			uint8_t Humidity_Out_L;
			///Relative humidity data (MSB)
			uint8_t Humidity_Out_H;
		} partHumidity;
	};
	
	union 
	{
		int16_t Temp_Out;
		struct 
		{
			///Temperature data (LSB)
			uint8_t Temp_Out_L;
			///Temperature data (MSB)
			uint8_t Temp_Out_H;
		} partTemp;
	};
		
} HTS221_DataReceiveStruct_typedef;

/**
 ** @brief Calibration data
 **/
typedef struct __attribute__((aligned(1), packed)) {

	uint8_t H0_rH_x2;
	uint8_t H1_rH_x2;
	uint8_t T0_degC_x8;
	uint8_t T1_degC_x8;
	uint8_t Reserved_0;
	uint8_t T1_T0_msb;
	int16_t H0_T0_OUT;
	uint16_t Reserved_1;
	int16_t H1_T0_OUT;
	int16_t T0_OUT;	
	int16_t T1_OUT;	
	
} HTS221_CalibrationCoefficientsStruct_typedef;


/**
 ** @brief HTS221 instance struct
 **/
typedef struct {
	
	uint8_t dev_address;
	HTS221_ConfigStruct_typedef dev_configuration;
	HTS221_DataReceiveStruct_typedef dev_uncompensated_data;
	HTS221_CalibrationCoefficientsStruct_typedef dev_calibration_data;
	hts221_communication_fptr read_data_i2c;
	hts221_communication_fptr write_data_i2c;
	hts221_delay_fptr delay;	
	HTS221_TempHumPressStruct_typedef dev_compensated_data;
	
	
} HTS221_typedef;

/**
 ** @brief Public function prototype
 **/
void HTS221_Init_Device(HTS221_typedef *dev_hts221);
HTS221_TempHumPressStruct_typedef* HTS221_Get_Data_Temp_Hum(HTS221_typedef *dev_hts221);
void HTS221_Set_Av_Conf(HTS221_typedef *dev_hts221, HTS221_AvConf av_conf_t, HTS221_AvConf av_conf_h);
void HTS221_Set_Ctrl_Reg_1(HTS221_typedef *dev_hts221, HTS221_CtrlReg1 odr, HTS221_CtrlReg1 bdu, HTS221_CtrlReg1 pd);
void HTS221_Set_Ctrl_Reg_2(HTS221_typedef *dev_hts221, HTS221_CtrlReg2 one_shot, HTS221_CtrlReg2 heater, HTS221_CtrlReg2 boot); 
void HTS221_Set_Ctrl_Reg_3(HTS221_typedef *dev_hts221, HTS221_CtrlReg3 drdy_en, HTS221_CtrlReg3 pp_od, HTS221_CtrlReg3 drdy_h_l);

//#endif /* HTS221_H_ */


