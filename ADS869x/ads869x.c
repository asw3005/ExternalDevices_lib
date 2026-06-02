/*
 * @brief Common driver for ADS8691, ADS8695, ADS8699 from the Texas Instruments.
 * Created 05.04.26 by asw3005. 
 *
 **/

#include "ads869x.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include <stdint.h>

/* External variables. */
extern SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef* ADS869x_SPI = &hspi2;

/* Private function prototypes. */
static void ADS869x_SPI_TxCheck(void);
static void ADS869x_SPI_RxCheck(void);


/*
 * @brief ADC init function.
 *
 **/
void ADS869x_Init(void) {

	/* Set SPI configuration and GPIO function of SDO1 pin. */
	//ADS869x_SdoCtrl(ADS869xGetDataStruct(), 0, 0, ADS869x_SDO1_GPO);	
	/* Enable LED on the ADC's pin. */
	//ADS869x_SdoPinSetReset(ADS869xGetDataStruct(), 1);	
	/* Enable ADC's test data sequence. */
	ADS869xGetDataStruct()->ADC0Struct.isTxReady = 0;
	ADS869x_DataOutCtrl(&(ADS869xGetDataStruct()->ADC0Struct), ADS869x_CONVDATA, 0, 0, ADS869x_ACTIVE_IN_DO_NOT_INCL, ADS869x_ACTIVE_VDD_DO_NOT_INCL, 0);
	ADS869x_SPI_TxCheck();

	/* Selecting ADC input range. */
	ADS869xGetDataStruct()->ADC0Struct.isTxReady = 0;
	ADS869x_RangeSel(&(ADS869xGetDataStruct()->ADC0Struct), ADS869x_P1_25VREF, 0);
	ADS869x_SPI_TxCheck();
}

/*
 * @brief Reset ADC (hardware pin).
 *
 * @param state : 0 - ADC reset,
 *				  1 - ADC normal mode.
 *
 **/
void ADS869x_RST(uint8_t state) {
	
	if (state > 0) {
		ADS869x_RST_ADC_GPIO_Port->BSRR = ADS869x_RST_ADC_Pin;
	}
	else {
		ADS869x_RST_ADC_GPIO_Port->BSRR = ADS869x_RST_ADC_Pin << 16;
	}	
	HAL_Delay(5);
}

/*
 *	@brief Scale factor.
 *	
 * @param scale : ADS869x_VALUE_OF_DIVISION_5_12P  			= +5.12V
 *				  ADS869x_VALUE_OF_DIVISION_6_144P 			= +6.144V
 *				  ADS869x_VALUE_OF_DIVISION_10_24P 			= +10.24V
 *				  ADS869x_VALUE_OF_DIVISION_12_288P 		= +12.288V
 *				  ADS869x_VALUE_OF_DIVISION_2_56P_2_56N 	= +2.56V to -2.56V
 *				  ADS869x_VALUE_OF_DIVISION_5_12P_5_122N 	= +5.12V to -5.12V
 *				  ADS869x_VALUE_OF_DIVISION_6_144P_6_144N 	= +6.144V to -6.144V
 *				  ADS869x_VALUE_OF_DIVISION_10_24P_10_24N 	= +10.24V to -10.24V
 *				  ADS869x_VALUE_OF_DIVISION_12_288P_12_288N = +12.288V to -12.288V
 * 
 **/
float ADS869x_GetVoltage(float scale) {	

	ADS869xGetDataStruct()->ADC0Struct.tx_byte_cnt = &ADS869x_SPI->TxXferCount;
	ADS869xGetDataStruct()->ADC0Struct.rx_byte_cnt = &ADS869x_SPI->RxXferCount;

	return  /* ADS869x_INPUT_RANGE * */ scale * (uint32_t)(ADS869x_ReadADC(&(ADS869xGetDataStruct()->ADC0Struct)).DataWord >> 14);
}

/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @return 14-bit ADC data type of ADS869x_OutputDataWord_t.
 *
 **/
ADS869x_OutputDataWord_t ADS869x_ReadADC(ADS869x_GInst_t* device)
{
	ADS869x_OutputDataWord_t DataWord;
	
	device->data.ADDRESS = ADS869x_NOP;
	device->data.COMMAND = ADS869x_NOP;
	device->data.REG_DATA_LSB = ADS869x_NOP;
	device->data.REG_DATA_MSB = ADS869x_NOP;

	device->spi_tx(&device->data.Command, 4);
	ADS869x_SPI_TxCheck();	
	
	//device->delay(1);
	device->spi_rx(&device->data.Command, 4);
	ADS869x_SPI_RxCheck();
	//device->delay(1);
	
	DataWord.DataWord_LSW_LSB = device->data.REG_DATA_LSB;
	DataWord.DataWord_LSW_MSB = device->data.REG_DATA_MSB;
	DataWord.DataWord_HSW_LSB = device->data.Address;
	DataWord.DataWord_HSW_MSB = device->data.Command;
	
	return DataWord;
}

/*
 * @brief Write/read device register.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param address : Device address that you wish.
 *
 **/
uint16_t ADS869x_W_R_REG(ADS869x_GInst_t* device, uint8_t address)
{
	uint16_t Data = 0;
	
	device->data.ADDRESS = address;
	device->data.COMMAND = ADS869x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);
	
	device->delay(1);
	device->spi_rx((uint8_t*)&device->data, 2);
	device->delay(1);
	return Data  = (device->data.REG_DATA_MSB << 8) | device->data.REG_DATA_LSB;	
}

/*
 * @brief Controls the reset and power-down features.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables the NAP mode of the converter, 1 enables the converter to enter NAP mode if CONVST/CS 
 *        is held high after the current conversion completes.
 * @param rstn_app : If 0 RST pin functions as a POR class reset (causes full device initialization) if 1 RST pin 
 *        functions as an application reset (only user-programmed modes are cleared).
 * @param in_al_dis : If 0 input alarm is enabled, 1 input alarm is disabled.
 * @param vdd_al_dis : If 0 VDD alarm is enabled, 1 VDD alarm is disabled.
 *
 **/
void ADS869x_RstPwdn(ADS869x_GInst_t* device, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis)
{
	ADS869x_RstPwrCtrl_t RstPwrCtlReg;
	
	RstPwrCtlReg.PWRDN = pwrdn;
	RstPwrCtlReg.NAP_EN = nap_en;
	RstPwrCtlReg.RSTn_APP = rstn_app;
	RstPwrCtlReg.IN_AL_DIS = in_al_dis;
	RstPwrCtlReg.VDD_AL_DIS = vdd_al_dis;
	RstPwrCtlReg.WKEY = ADS869x_WKEY;
	
	device->data.ADDRESS = ADS869x_RST_PWRCTL_LSW;
	device->data.COMMAND = ADS869x_WRITE_MSB;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = ADS869x_WKEY;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.COMMAND = ADS869x_WRITE_LSB;
	device->data.REG_DATA_LSB = (uint8_t)RstPwrCtlReg.RstPwrCtrlReg_LSW;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Configures the protocol used for writing data.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param protocol : Selects the SPI protocol, see ADS869x_SPIProtocol enum (default ADS869x_CPOL0_CPHASE0).
 *
 **/
void ADS869x_SdiCtrl(ADS869x_GInst_t* device, ADS869x_SPIProtocol protocol)
{
//	ADS869x_SdiCtrl_t SdiCtrlReg;
//	SdiCtrlReg.SDI_MODE = protocol;
	
	device->data.ADDRESS = ADS869x_SDI_CTL_LSW;
	device->data.COMMAND = ADS869x_WRITE_LSB;
	device->data.REG_DATA_LSB = protocol;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Controls data protocol used to transmit data from the SDO-x pins of the device.
 * NOTE. This function resets the GPO pin (GPO_VAL) to zero.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param sdo_mode : If 0xb, SDO mode follows the same SPI protocol as that used for SDI (default), see the SDI_CTL_REG register.
 *        If 10b, invalid configuration. If 11b SDO mode follows the ADC master clock or source-synchronous protocol.
 * @param ssync_clk : If 0b, external SCLK selected (no division, default), 1b - internal clock selected (no division).
 * @param sdo1_config : It used to configure ALARM/SDO-1/GPO, see the ADS869x_SDO1Mode enum.
 *
 *
 **/
void ADS869x_SdoCtrl(ADS869x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk, ADS869x_SDO1Mode sdo1_config)
{
	ADS869x_SdoCtrl_t SdoCtrlReg;
	
	SdoCtrlReg.SDO_MODE = sdo_mode;
	SdoCtrlReg.SSYNC_CLK = ssync_clk;
	SdoCtrlReg.SDO1_CONFIG = sdo1_config;
	SdoCtrlReg.GPO_VAL = 0;
	
	device->data.ADDRESS = ADS869x_SDO_CTL_LSW;
	device->data.COMMAND = ADS869x_WRITE_HWORD;	
	device->data.REG_DATA_LSB = SdoCtrlReg.SdoCtrlReg_LSW;
	device->data.REG_DATA_MSB = SdoCtrlReg.SdoCtrlReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);		
}

/*
 * @brief Drives logical level of general purpoise pin (SDO1 as a GPO pin, that alternative function should be 
 * selected in advance by ADS869x_SdoCtrl function above).
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param gpo_val : 1-bit value for the output on the GPO pin (can be 0 or 1).
 *
 **/
void ADS869x_SdoPinSetReset(ADS869x_GInst_t* device, uint8_t sdo_val)
{	
	device->data.ADDRESS = ADS869x_SDO_CTL_LSW;	
	if (sdo_val) {
		device->data.COMMAND = ADS869x_SET_HWORD;
	}
	else {
		device->data.COMMAND = ADS869x_RESET_HWORD;
	}	
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0x10;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects data format for the output data.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param data_val : It controls the data value output by the converter, see the ADS869x_DataVal enum.
 * @param par_en : If 0b output data does not contain parity information, if 1b two parity bits (ADC output
 *	      and output data frame) are appended to the LSBs of the output data.
 * @param range_incl : 0b do not include the range configuration register value, 1b include the range configuration
 *        register value.
 * @param in_active_alarm_incl : Control to include the active input ALARM flags in the SDO-x output bit stream.
 *        See the ADS869x_InActiveAlarm enum.
 * @param vdd_active_alarm_incl : Control to include the active VDD ALARM flags in the SDO-x output bit stream.
 *        See the ADS869x_VddActiveAlarm enum.
 * @param device_addr_incl :  0b do not include the register value, 1b include the register value.
 *
 **/
void ADS869x_DataOutCtrl(ADS869x_GInst_t* device, ADS869x_DataVal data_val, uint8_t par_en, uint8_t range_incl, 
	ADS869x_InActiveAlarm in_active_alarm_incl, ADS869x_VddActiveAlarm vdd_active_alarm_incl, uint8_t device_addr_incl)
{
	ADS869x_DataOut_t DataOutReg;
	
	DataOutReg.DATA_VAL = data_val;
	DataOutReg.PAR_EN = par_en;
	DataOutReg.RANGE_INCL = range_incl;
	DataOutReg.IN_ACTIVE_ALARM_INCL = in_active_alarm_incl;
	DataOutReg.VDD_ACTIVE_ALARM_INCL = vdd_active_alarm_incl;
	DataOutReg.DEVICE_ADDR_INCL = device_addr_incl;
	
	device->data.ADDRESS = ADS869x_DATAOUT_CTL_LSW;
	device->data.COMMAND = ADS869x_WRITE_HWORD;
	device->data.REG_DATA_LSB = DataOutReg.DataOutReg_LSW;
	device->data.REG_DATA_MSB = DataOutReg.DataOutReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects either internal or external reference and selects input range.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param range_sel : It selects one of nine input ranges, see the ADS869x_InputRange enum.
 * @param intref_dis : 0 enables internal voltage reference, 1 disables this one.
 *
 **/
void ADS869x_RangeSel(ADS869x_GInst_t* device, ADS869x_InputRange range_sel, uint8_t intref_dis)
{
	ADS869x_RangeSel_t RangeSelReg;
	
	RangeSelReg.RANGE_SEL = range_sel;
	RangeSelReg.INTREF_DIS = intref_dis;
	
	device->data.ADDRESS = ADS869x_RANGE_SEL_LSW;
	device->data.COMMAND = ADS869x_WRITE_HWORD;
	device->data.REG_DATA_LSB = RangeSelReg.RangeSelReg_LSW;
	device->data.REG_DATA_MSB = RangeSelReg.RangeSelReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Return output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 *
 **/
ADS869x_Alarm_t ADS869x_ReadAlarm(ADS869x_GInst_t* device)
{
	ADS869x_Alarm_t AlarmReg;
	
	device->data.ADDRESS = ADS869x_ALARM_LSW;
	device->data.COMMAND = ADS869x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_rx(&device->data.Command, 4);
	device->delay(1);
	device->spi_rx((uint8_t*)&device->data, 2);
	AlarmReg.AlarmReg_LSW = ((uint16_t)device->data.REG_DATA_MSB << 8) | (uint16_t)device->data.REG_DATA_LSB;
	
	return AlarmReg;
}

/*
 * @brief Sets hysteresis and high threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param inp_alrm_high_th : 14-bit threshold for comparison is INP_ALRM_HIGH_TH.
 * @param inp_alrm_hyst : 4-bit hysteresis value for the input ALARM.
 *
 **/
void ADS869x_SetAlarmHTh(ADS869x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst)
{
	ADS869x_AlarmHTh_t AlarmHthReg;
	
	AlarmHthReg.RESERVED1_0 = 0;
	AlarmHthReg.RESERVED27_24 = 0;
	AlarmHthReg.INP_ALRM_HIGH_TH = inp_alrm_high_th;	
	AlarmHthReg.INP_ALRM_HYST = inp_alrm_hyst;
	
	device->data.ADDRESS = ADS869x_ALARM_H_TH_LSW;
	device->data.COMMAND = ADS869x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_LSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.ADDRESS = ADS869x_ALARM_H_TH_HSW;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_HSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_HSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS869x_GInst_t.
 * @param inp_alrm_low_th : 14-bit threshold for comparison is INP_ALRM_LOW_TH.
 *
 **/
void ADS869x_SetAlarmLTh(ADS869x_GInst_t* device, uint16_t inp_alrm_low_th)
{
	ADS869x_AlarmLTh_t AlarmLThReg;
	
	AlarmLThReg.RESERVED1_0 = 0;
	AlarmLThReg.INP_ALRM_LOW_TH = inp_alrm_low_th;
	
	device->data.ADDRESS = ADS869x_ALARM_L_TH_LSW;
	device->data.COMMAND = ADS869x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmLThReg.AlarmLTh_LSW;
	device->data.REG_DATA_MSB = AlarmLThReg.AlarmLTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Data strucut getter/setter.
 * 
 *
 **/
ADS869x_MGInst_t* ADS869xGetDataStruct(void) {

	/* General data struct of ADC unit. */
	static ADS869x_MGInst_t ads869x_minst = { 

		.ADC0Struct.delay = HAL_Delay,
		.ADC0Struct.spi_tx = ADS869x_SPI_Tx,
		.ADC0Struct.spi_rx = ADS869x_SPI_Rx
	};

	// ads869x_minst.ADC0Struct.delay = HAL_Delay;
	// ads869x_minst.ADC0Struct.spi_tx = ADS869x_SPI_Tx;
	// ads869x_minst.ADC0Struct.spi_rx = ADS869x_SPI_Rx;


	return &ads869x_minst;
}


/* Hardware dependent functions. */

/*
 * @brief SPI chip select.
 *
 * @param gpio : Either CS_ADC_GPIO_Port or CS_DAC_GPIO_Port.
 * @param gpio_pin : Either CS_ADC_Pin or CS_DAC_Pin.
 * @param state : Either GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
void ADS869x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {

	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}
}

/*
 * @brief  Tx check.
 *
 **/
 static void ADS869x_SPI_TxCheck(void) {

	// ADS869xGetDataStruct()->ADC0Struct.isTxReady = 0;
	do {
		if (ADS869xGetDataStruct()->ADC0Struct.isTxReady) {
			__NOP();
			ADS869xGetDataStruct()->ADC0Struct.isTxReady = 0;
			break;
		}
	}
	while(1);	
}

/*
 * @brief  Tx check.
 *
 **/
static void ADS869x_SPI_RxCheck(void) {
	
	// ADS869xGetDataStruct()->ADC0Struct.isRxReady = 0;
	do {
		if (ADS869xGetDataStruct()->ADC0Struct.isRxReady) {
			__NOP();
			ADS869xGetDataStruct()->ADC0Struct.isRxReady = 0;
			break;
		}
	}
	while(1);	
}

/*
 * @brief
 *
 **/
void ADS869x_SPI_Tx(uint8_t *pData, uint8_t size) {
	
	//uint16_t timeout = 10000;

	ADS869x_SPI_CS(ADS869x_CS_ADC_GPIO_Port, ADS869x_CS_ADC_Pin, 0);
	HAL_SPI_Transmit_IT(ADS869x_SPI, pData, size);
	//HAL_Delay(3);
	//ADS869x_SPI_CS(ADS869x_CS_ADC_GPIO_Port, ADS869x_CS_ADC_Pin, 1);
}

/*
 * @brief
 * 
 *
 **/
void ADS869x_SPI_Rx(uint8_t *pData, uint8_t size) {
	
	//uint16_t timeout = 10000;

	ADS869x_SPI_CS(ADS869x_CS_ADC_GPIO_Port, ADS869x_CS_ADC_Pin, 0);
	HAL_SPI_Receive_IT(ADS869x_SPI, pData, size);
	//HAL_Delay(3);
	//ADS869x_SPI_CS(ADS869x_CS_ADC_GPIO_Port, ADS869x_CS_ADC_Pin, 1);
}





