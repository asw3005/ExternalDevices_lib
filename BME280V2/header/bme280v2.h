/** Created: 27.05.2018
 ** BME280.h
 **/	
#pragma once
//#ifndef BME280_H_
//#define BME280_H_

#include "stm32l4xx_hal.h"

/**
 ** @brief Device address
 **/
#define ADDR_BME280_SHIFTED             0xEC                                //7 bit MSB 0x76 (address) + 1 bit LSB 0x00 (read/write)
#define ADDR_BME280						0x76								//It's not shifted address
#define ADDR_BME280_SHIFTED_ALTERNATIVE 0xEE                                //
#define ADDR_BME280_ALTERNATIVE			0x77								//It's not shifted address

/**
 ** @brief Software reset cmd.
 **/
#define RESET_STATE                     0xB6

/**
 ** @brief Internal registers addresses
 **/
typedef enum 
{
	///The trimming parameters are programmed into the devices’ non-volatile memory (NVM) during production
	CALIB00_START_ADDR     =  0x88,
	CALIB25_FINISH_ADDR    =  0xA1,
	///Chip identification number chip_id[7:0], which is 0x60.
	ID_ADDR                =  0xD0,
	///The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is written to the
	///register, the device is reset. Writing other values than 0xB6 has no effect.
	RESET_ADDR             =  0xE0,
	///The trimming parameters are programmed into the devices’ non - volatile memory(NVM) during production
	CALIB26_START_ADDR     =  0xE1,
	CALIB41_FINISH_ADDR    =  0xF0,
	///The “ctrl_hum” register sets the humidity data acquisition options of the device.Changes to this
	///register only become effective after a write operation to “ctrl_meas”.
	CTRL_HUM_ADDR          =  0xF2,
	///The “status” register contains two bits which indicate the status of the device.
	///Bit 3 "measuring[0]" Automatically set to ‘1’ whenever a conversion is running and back to ‘0’ when the results have been
	///transferred to the data registers. 
	///Bit 0 "im_update[0]" Automatically set to ‘1’ when the NVM data are being copied to image registers and back to ‘0’ when the
	///copying is done. The data are copied at power-on-reset and before every conversion.
	STATUS_ADDR            =  0xF3,
	///The “ctrl_meas” register sets the pressure and temperature data acquisition options of the device.
	///The register needs to be written after changing “ctrl_hum” for the changes to become effective.
	CTRL_MEAS_ADDR         =  0xF4,
	///The “config” register sets the rate, filter and interface options of the device. Writes to the “config”
	///register in normal mode may be ignored.In sleep mode writes are not ignored.
	CONFIG_ADDR            =  0xF5,
	///The “press” register contains the raw pressure measurement output data up[19:0].
	///Contains the MSB part up[19:12] of the raw pressure measurement output data.
	PRESS_MSB_ADDR         =  0xF7,
	///Contains the LSB part up[11:4] of the raw pressure measurement output data.
	PRESS_LSB_ADDR         =  0xF8,
	///Contains the XLSB part up[3:0] of the raw pressure measurement output data (bit 7, 6, 5, 4). Contents depend on temperature resolution.
	PRESS_XLSB_ADDR        =  0xF9,
	///The “temp” register contains the raw temperature measurement output data ut[19:0].
	///Contains the MSB part ut[19:12] of the raw temperature measurement output data.
	TEMP_MSB_ADDR          =  0xFA,
	///Contains the LSB part ut[11:4] of the raw temperature measurement output data.
	TEMP_LSB_ADDR          =  0xFB,
	///Contains the XLSB part ut[3:0] of the raw temperature measurement output data (bit 7, 6, 5, 4). Contents depend on pressure resolution.
	TEMP_XLSB_ADDR         =  0xFC,
	///The “temp” register contains the raw temperature measurement output data ut[19:0]. 
	///Contains the MSB part uh[15:8] of the raw humidity measurement output data.
	HUM_MSB_ADDR           =  0xFD,
	///Contains the LSB part uh[7:0] of the raw humidity measurement output data.
	HUM_LSB_ADDR           =  0xFE,     
	
} BME280_AddrReg;

/**
 ** @brief Controls oversampling of humidity data.
 **/
typedef enum
{
	///Skipped (output set to 0x8000).
	OSRS_H_SKIPPED          =  0x00,
	OSRS_H_OVERSAMPLING_1   =  0x01,
	OSRS_H_OVERSAMPLING_2   =  0x02,
	OSRS_H_OVERSAMPLING_4   =  0x03,
	OSRS_H_OVERSAMPLING_8   =  0x04,
	///Value 0x05 or others.
	OSRS_H_OVERSAMPLING_16  =  0x05,
	
} BME280_CtrlHum;

/**
 ** @brief Pressure and temperature data acquisition options of the device.
 **/
typedef enum
{
	///Controls oversampling of pressure data.
	///Skipped (output set to 0x80000).
	OSRS_P_SKIPPED           =       0x00,
	OSRS_P_OVERSAMPLING_1    =       0x01,
	OSRS_P_OVERSAMPLING_2    =       0x02,
	OSRS_P_OVERSAMPLING_4    =       0x03,
	OSRS_P_OVERSAMPLING_8    =       0x04,
	///Value is 0x05 or other.
	OSRS_P_OVERSAMPLING_16   =       0x05,
							 			 
	///Controls oversampling of temperature data.
	///Skipped (output set to 0x80000).
	OSRS_T_SKIPPED           =       0x00,
	OSRS_T_OVERSAMPLING_1    =       0x01,
	OSRS_T_OVERSAMPLING_2    =       0x02,
	OSRS_T_OVERSAMPLING_4    =       0x03,
	OSRS_T_OVERSAMPLING_8    =       0x04,
	///Value is 0x05 or other.
	OSRS_T_OVERSAMPLING_16   =       0x05,
							 			 
	///Controls the sensor mode of the device.
	SLEEP_MODE               =       0x00,
	FORCED_MODE              =       0x01,
	//FORCED_MODE              =       0x02,
	NORMAL_MODE              =       0x03,
	
} BME280_CtrlMeas;

/**
 ** @brief Configuration speed, filter and interface options of the device.
 **/
typedef enum
{
	///Controls inactive duration t standby in normal mode.
	T_SB_500uS               =       0x00,
	T_SB_62500uS             =       0x01,
	T_SB_125mS               =       0x02,
	T_SB_250mS               =       0x03,
	T_SB_500mS               =       0x04,
	T_SB_1000mS              =       0x05,
	T_SB_10mS                =       0x06,
	T_SB_20mS                =       0x07,
							 			 
	///Controls the time constant of the IIR filter.
	FILTER_OFF               =       0x00,
	FILTER_COEFF2            =       0x01,
	FILTER_COEFF4            =       0x02,
	FILTER_COEFF8            =       0x03,
	FILTER_COEFF16           =       0x04,
							 		
	///Enables 3-wire SPI.
	SPI3WIRE_EN              =       0x01,
	SPI3WIRE_DIS             =       0x00,
	
} BME280_Config;
/**
 ** @brief Device specific function type.
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
typedef int8_t(*bme280_communication_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
typedef void(*bme280_delay_fptr)(uint32_t period);

/**
 ** @brief Device measurement profiles.
 ** 
 ** It is recommended medes of operations. See datasheet BME280, page 17.
 ** Mode "CUSTOM_PROFILE_0" is custom. You need to make settings yourself by 
 ** adding your configuration to the function BME280_Set_Profile.
 **
 **/
typedef enum 
{ 
	WEATHER_MONITORING,
	HUMIDITY_SENSING,
	INDOOR_NAVIGATION,
	GAMING,
	CUSTOM_PROFILE_0,
	
} BME280_Profile;

/**
 ** @brief Compensated data struct.
 **/
typedef struct {
	int32_t TemperatureC;
	int32_t TemperatureF;
	uint32_t HumidityRH;
	uint32_t PressurePa;
	uint32_t PressuremmHg;
	
} BME280_TempHumPressStruct_typedef;

/**
 ** @brief Configuration registers.
 **/
typedef struct {

	union {	
		///Controls oversampling of humidity data.
		///Look at the BME280_Ctrl_Hum_enum on top.
		uint8_t DataCtrlHum;
		struct {
			uint8_t   osrs_h			: 3;   
			uint8_t   RESERVED_0		: 5;			
		} bitsDataCtrlHum;				
	};
	
	union {
		///Pressure and temperature data acquisition options of the device.
		///Look at the BME280_Ctrl_Meas_enum on top.
		uint8_t DataCtrlMeas;
		struct {
			///Controls the sensor mode of the device.
			uint8_t   mode				: 2;   
			///Controls oversampling of pressure data.
			uint8_t   osrs_p			: 3;
			///Controls oversampling of temperature data.
			uint8_t   osrs_t			: 3; 
		} bitsDataCtrlMeas;
	};
	
	union {
		///The “config” register sets the rate, filter and interface options of the device.
		///Look at the BME280_Config_enum on top.
		uint8_t DataConfig;
		struct {
			///Enables 3 - wire SPI interface when set to ‘1’.
			uint8_t   spi3w_en			: 1;   
			uint8_t   RESERVED_1		: 1;
			///Controls the time constant of the IIR filter.
			uint8_t   filter			: 3;   
			///Controls inactive duration t standby in normal mode.
			uint8_t   t_sb				: 3; 
		} bitsDataConfig;
	};
	
} BME280_ConfigStruct_typedef;

/**
 ** @brief Raw data struct.
 ** 
 ** Raw data humidity, temperature and pressure. Need compensation to use raw data.
 **
 **/
typedef struct {
	uint8_t	Ctrl_Hum;		                        
	uint8_t	Status;			                          
	uint8_t	Ctrl_Meas;		                       
	uint8_t	Config;			                          
	uint8_t Reserved_0;	
	uint8_t	Press_msb;		                       
	uint8_t	Press_lsb;		                       
	uint8_t	Press_xlsb_7_4;	                      
	uint8_t	Temp_msb;		                        
	uint8_t	Temp_lsb;		                        
	uint8_t	Temp_xlsb_7_4;	                       
	uint8_t	Hum_msb;			                         
	uint8_t	Hum_lsb;			 
	
} BME280_DataReceiveStruct_typedef;

/**
 ** @brief Calibration data struct.
 ** 
 ** The trimming parameters are programmed into the devices’ non-volatile memory (NVM) during
 ** production and cannot be altered by the customer. Each compensation word is a 16-bit signed or
 ** unsigned integer value stored in two’s complement. As the memory is organized into 8-bit words,
 ** two words must always be combined in order to represent the compensation word. The 8-bit
 ** registers are named calib00…calib41 and are stored at memory addresses 0x88…0xA1 and
 ** 0xE1…0xE7. The corresponding compensation words are named dig_T# for temperature
 ** compensation related values, dig_P# for pressure related values and dig_H# for humidity related
 ** values.
 **
 **/
typedef struct __attribute__((aligned(1), packed)) {
	uint16_t Dig_T1;
	int16_t Dig_T2;
	int16_t Dig_T3;
	uint16_t Dig_P1;
	int16_t Dig_P2;
	int16_t Dig_P3;
	int16_t Dig_P4;
	int16_t Dig_P5;
	int16_t Dig_P6;
	int16_t Dig_P7;
	int16_t Dig_P8;
	int16_t Dig_P9;
	uint8_t Reserved_0;
	uint8_t Dig_H1;
	uint8_t Reserved_1[46];
	uint8_t bmeID;
	uint8_t Reserved_2[15];
	uint8_t Reset;
	int16_t Dig_H2;
	uint8_t Dig_H3;
	int8_t Dig_H4_11_4;		
	uint8_t Dig_H5_H4;
	int8_t Dig_H5_11_4;
	int8_t Dig_H6;
	
} BME280_CalibrationCoefficientsStruct_typedef;


/**
 ** @brief BME280 instance struct.
 **/
typedef struct {
	
	uint8_t dev_address;
	uint8_t dev_id;
	BME280_ConfigStruct_typedef dev_configuration;
	BME280_DataReceiveStruct_typedef dev_uncompensated_data;
	BME280_CalibrationCoefficientsStruct_typedef dev_calibration_data;
	bme280_communication_fptr read_data_i2c;
	bme280_communication_fptr write_data_i2c;
	bme280_delay_fptr delay;	
	BME280_TempHumPressStruct_typedef dev_compensated_data;
		
} BME280_typedef;

/**
 ** @brief Public function prototype.
 **/
void BME280_Init_Device(BME280_PROFILES_enum meas_profil, BME280_typedef *dev_bme280);
BME280_TempHumPressStruct_typedef* BME280_Get_Data_Press_Temp_Hum(BME280_typedef *dev_bme280); 
void BME280_Write_Config(void); 								  
void BME280_Get_Calibration_Data(BME280_typedef *dev_bme280); 
void BME280_Get_Calibration_Data0(BME280_typedef *dev_bme280); 
void BME280_Get_Calibration_Data1(BME280_typedef *dev_bme280);  
void BME280_GetID(BME280_typedef *dev_bme280); 
 
//#endif /* BME280_H_ */


