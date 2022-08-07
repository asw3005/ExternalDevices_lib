/*
 * @brief Common driver for ADS8671, ADS8675 from the Texas Instruments.
 * Created 08.02.21 by asw3005. 
 *
 **/

#include "ads867x.h"
#include "spi.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;


/* Private function prototypes. */
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/*
 * @brief ADC init function.
 *
 **/
void ADS867x_Init(void) {
	
	/* General data struct of ADC unit. */
	ADS867x_GInst_t ads8671 = { 		
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};
	
	/* Set SPI configuration and GPIO function of SDO1 pin. */
	//ADS867x_SdoCtrl(&ads8671, 0, 0, ADS867x_SDO1_GPO);	
	/* Enable LED on the ADC's pin. */
	//ADS867x_SdoPinSetReset(&ads8671, 1);	
	/* Enable ADC's test data sequence. */
	ADS867x_DataOutCtrl(&ads8671, ADS867x_CONVDATA, 0, 0, ADS867x_ACTIVE_IN_DO_NOT_INCL, ADS867x_ACTIVE_VDD_DO_NOT_INCL, 0);
	/* Selecting ADC input range. */
	ADS867x_RangeSel(&ads8671, ADS867x_P1_25VREF, 0);
}

/*
	@brief
*/
float ADS867x_GetVoltage(void) {
	
	/* General data struct of ADC unit. */
	ADS867x_GInst_t ads8671 = { 
		.tx_byte_cnt = &hspi1.TxXferCount,
		.rx_byte_cnt = &hspi1.RxXferCount,
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};

	return  /* ADS867x_INPUT_RANGE * */ ADS867x_VALUE_OF_DIVISION * (uint16_t)(ADS867x_ReadADC(&ads8671).DataWord >> 18);
}

/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @return 14-bit ADC data type of ADS867x_OutputDataWord_t.
 *
 **/
ADS867x_OutputDataWord_t ADS867x_ReadADC(ADS867x_GInst_t* device)
{
	uint16_t timeout = 10000;
	
	ADS867x_OutputDataWord_t DataWord;
	
	device->data.ADDRESS = ADS867x_NOP;
	device->data.COMMAND = ADS867x_NOP;
	device->data.REG_DATA_LSB = ADS867x_NOP;
	device->data.REG_DATA_MSB = ADS867x_NOP;
	device->spi_tx(&device->data.Command, 4);
	
	while (*device->tx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}	
	
	//device->delay(1);
	device->spi_rx(&device->data.Command, 4);
	timeout = 10000;
	while (*device->rx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}	
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
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param address : Device address that you wish.
 *
 **/
uint16_t ADS867x_W_R_REG(ADS867x_GInst_t* device, uint8_t address)
{
	uint16_t Data = 0;
	
	device->data.ADDRESS = address;
	device->data.COMMAND = ADS867x_READ_HWORD;
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
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables the NAP mode of the converter, 1 enables the converter to enter NAP mode if CONVST/CS 
 *        is held high after the current conversion completes.
 * @param rstn_app : If 0 RST pin functions as a POR class reset (causes full device initialization) if 1 RST pin 
 *        functions as an application reset (only user-programmed modes are cleared).
 * @param in_al_dis : If 0 input alarm is enabled, 1 input alarm is disabled.
 * @param vdd_al_dis : If 0 VDD alarm is enabled, 1 VDD alarm is disabled.
 *
 **/
void ADS867x_RstPwdn(ADS867x_GInst_t* device, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis)
{
	ADS867x_RstPwrCtrl_t RstPwrCtlReg;
	
	RstPwrCtlReg.PWRDN = pwrdn;
	RstPwrCtlReg.NAP_EN = nap_en;
	RstPwrCtlReg.RSTn_APP = rstn_app;
	RstPwrCtlReg.IN_AL_DIS = in_al_dis;
	RstPwrCtlReg.VDD_AL_DIS = vdd_al_dis;
	RstPwrCtlReg.WKEY = ADS867x_WKEY;
	
	device->data.ADDRESS = ADS867x_RST_PWRCTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_MSB;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = ADS867x_WKEY;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.COMMAND = ADS867x_WRITE_LSB;
	device->data.REG_DATA_LSB = (uint8_t)RstPwrCtlReg.RstPwrCtrlReg_LSW;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Configures the protocol used for writing data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param protocol : Selects the SPI protocol, see ADS867x_SPIProtocol enum (default ADS867x_CPOL0_CPHASE0).
 *
 **/
void ADS867x_SdiCtrl(ADS867x_GInst_t* device, ADS867x_SPIProtocol protocol)
{
//	ADS867x_SdiCtrl_t SdiCtrlReg;
//	SdiCtrlReg.SDI_MODE = protocol;
	
	device->data.ADDRESS = ADS867x_SDI_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_LSB;
	device->data.REG_DATA_LSB = protocol;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Controls data protocol used to transmit data from the SDO-x pins of the device.
 * NOTE. This function resets the GPO pin (GPO_VAL) to zero.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param sdo_mode : If 0xb, SDO mode follows the same SPI protocol as that used for SDI (default), see the SDI_CTL_REG register.
 *        If 10b, invalid configuration. If 11b SDO mode follows the ADC master clock or source-synchronous protocol.
 * @param ssync_clk : If 0b, external SCLK selected (no division, default), 1b - internal clock selected (no division).
 * @param sdo1_config : It used to configure ALARM/SDO-1/GPO, see the ADS867x_SDO1Mode enum.
 *
 *
 **/
void ADS867x_SdoCtrl(ADS867x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk, ADS867x_SDO1Mode sdo1_config)
{
	ADS867x_SdoCtrl_t SdoCtrlReg;
	
	SdoCtrlReg.SDO_MODE = sdo_mode;
	SdoCtrlReg.SSYNC_CLK = ssync_clk;
	SdoCtrlReg.SDO1_CONFIG = sdo1_config;
	SdoCtrlReg.GPO_VAL = 0;
	
	device->data.ADDRESS = ADS867x_SDO_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;	
	device->data.REG_DATA_LSB = SdoCtrlReg.SdoCtrlReg_LSW;
	device->data.REG_DATA_MSB = SdoCtrlReg.SdoCtrlReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);		
}

/*
 * @brief Drives logical level of general purpoise pin (SDO1 as a GPO pin, that alternative function should be 
 * selected in advance by ADS867x_SdoCtrl function above).
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param gpo_val : 1-bit value for the output on the GPO pin (can be 0 or 1).
 *
 **/
void ADS867x_SdoPinSetReset(ADS867x_GInst_t* device, uint8_t sdo_val)
{	
	device->data.ADDRESS = ADS867x_SDO_CTL_LSW;	
	if (sdo_val) {
		device->data.COMMAND = ADS867x_SET_HWORD;
	}
	else {
		device->data.COMMAND = ADS867x_RESET_HWORD;
	}	
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0x10;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects data format for the output data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param data_val : It controls the data value output by the converter, see the ADS867x_DataVal enum.
 * @param par_en : If 0b output data does not contain parity information, if 1b two parity bits (ADC output
 *	      and output data frame) are appended to the LSBs of the output data.
 * @param range_incl : 0b do not include the range configuration register value, 1b include the range configuration
 *        register value.
 * @param in_active_alarm_incl : Control to include the active input ALARM flags in the SDO-x output bit stream.
 *        See the ADS867x_InActiveAlarm enum.
 * @param vdd_active_alarm_incl : Control to include the active VDD ALARM flags in the SDO-x output bit stream.
 *        See the ADS867x_VddActiveAlarm enum.
 * @param device_addr_incl :  0b do not include the register value, 1b include the register value.
 *
 **/
void ADS867x_DataOutCtrl(ADS867x_GInst_t* device, ADS867x_DataVal data_val, uint8_t par_en, uint8_t range_incl, 
	ADS867x_InActiveAlarm in_active_alarm_incl, ADS867x_VddActiveAlarm vdd_active_alarm_incl, uint8_t device_addr_incl)
{
	ADS867x_DataOut_t DataOutReg;
	
	DataOutReg.DATA_VAL = data_val;
	DataOutReg.PAR_EN = par_en;
	DataOutReg.RANGE_INCL = range_incl;
	DataOutReg.IN_ACTIVE_ALARM_INCL = in_active_alarm_incl;
	DataOutReg.VDD_ACTIVE_ALARM_INCL = vdd_active_alarm_incl;
	DataOutReg.DEVICE_ADDR_INCL = device_addr_incl;
	
	device->data.ADDRESS = ADS867x_DATAOUT_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = DataOutReg.DataOutReg_LSW;
	device->data.REG_DATA_MSB = DataOutReg.DataOutReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects either internal or external reference and selects input range.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param range_sel : It selects one of nine input ranges, see the ADS867x_InputRange enum.
 * @param intref_dis : 0 enables internal voltage reference, 1 disables this one.
 *
 **/
void ADS867x_RangeSel(ADS867x_GInst_t* device, ADS867x_InputRange range_sel, uint8_t intref_dis)
{
	ADS867x_RangeSel_t RangeSelReg;
	
	RangeSelReg.RANGE_SEL = range_sel;
	RangeSelReg.INTREF_DIS = intref_dis;
	
	device->data.ADDRESS = ADS867x_RANGE_SEL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = RangeSelReg.RangeSelReg_LSW;
	device->data.REG_DATA_MSB = RangeSelReg.RangeSelReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Return output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 *
 **/
ADS867x_Alarm_t ADS867x_ReadAlarm(ADS867x_GInst_t* device)
{
	ADS867x_Alarm_t AlarmReg;
	
	device->data.ADDRESS = ADS867x_ALARM_LSW;
	device->data.COMMAND = ADS867x_READ_HWORD;
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
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param inp_alrm_high_th : 14-bit threshold for comparison is INP_ALRM_HIGH_TH.
 * @param inp_alrm_hyst : 4-bit hysteresis value for the input ALARM.
 *
 **/
void ADS867x_SetAlarmHTh(ADS867x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst)
{
	ADS867x_AlarmHTh_t AlarmHthReg;
	
	AlarmHthReg.RESERVED1_0 = 0;
	AlarmHthReg.RESERVED27_24 = 0;
	AlarmHthReg.INP_ALRM_HIGH_TH = inp_alrm_high_th;	
	AlarmHthReg.INP_ALRM_HYST = inp_alrm_hyst;
	
	device->data.ADDRESS = ADS867x_ALARM_H_TH_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_LSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.ADDRESS = ADS867x_ALARM_H_TH_HSW;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_HSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_HSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param inp_alrm_low_th : 14-bit threshold for comparison is INP_ALRM_LOW_TH.
 *
 **/
void ADS867x_SetAlarmLTh(ADS867x_GInst_t* device, uint16_t inp_alrm_low_th)
{
	ADS867x_AlarmLTh_t AlarmLThReg;
	
	AlarmLThReg.RESERVED1_0 = 0;
	AlarmLThReg.INP_ALRM_LOW_TH = inp_alrm_low_th;
	
	device->data.ADDRESS = ADS867x_ALARM_L_TH_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmLThReg.AlarmLTh_LSW;
	device->data.REG_DATA_MSB = AlarmLThReg.AlarmLTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}


/* Hardware dependent functions. */

/*
 * @brief
 *
 **/
void ADS867x_SPI_Tx(uint8_t *pData, uint8_t size) {
	
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Transmit(&hspi1, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief
 * 
 *
 **/
void ADS867x_SPI_Rx(uint8_t *pData, uint8_t size) {
	
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Receive(&hspi1, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief SPI chip select.
 * 
 * @param gpio : Either CS_ADC_GPIO_Port or CS_DAC_GPIO_Port.
 * @param gpio_pin : Either CS_ADC_Pin or CS_DAC_Pin.
 * @param state : Either GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}