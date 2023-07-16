/*
 * @brief Common driver for ADS8681, ADS8685, ADS8689 from the Texas Instruments.
 * Created 07.10.23 by asw3005. 
 *
 **/

#include "ads868x.h"
#include "spi.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;


/* Private function prototypes. */
static void ADS868x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/*
 * @brief ADC init function.
 *
 **/
void ADS868x_Init(void) {
	
	/* General data struct of ADC unit. */
	ADS868x_GInst_t ads8681 = { 		
		.delay = HAL_Delay,
		.spi_tx = ADS868x_SPI_Tx,
		.spi_rx = ADS868x_SPI_Rx
	};
	
	/* Set SPI configuration and GPIO function of SDO1 pin. */
	//ADS868x_SdoCtrl(&ads8681, 0, 0, ADS868x_SDO1_GPO);	
	/* Enable LED on the ADC's pin. */
	//ADS868x_SdoPinSetReset(&ads8681, 1);	
	/* Enable ADC's test data sequence. */
	ADS868x_DataOutCtrl(&ads8681, ADS868x_CONVDATA, 0, 0, ADS868x_ACTIVE_IN_DO_NOT_INCL, ADS868x_ACTIVE_VDD_DO_NOT_INCL, 0);
	/* Selecting ADC input range. */
	ADS868x_RangeSel(&ads8681, ADS868x_P1_25VREF, 0);
}

/*
	@brief
*/
float ADS868x_GetVoltage(void) {
	
	/* General data struct of ADC unit. */
	ADS868x_GInst_t ads8681 = { 
		.tx_byte_cnt = &hspi1.TxXferCount,
		.rx_byte_cnt = &hspi1.RxXferCount,
		.delay = HAL_Delay,
		.spi_tx = ADS868x_SPI_Tx,
		.spi_rx = ADS868x_SPI_Rx
	};

	return  /* ADS868x_INPUT_RANGE * */ ADS868x_VALUE_OF_DIVISION * (uint16_t)(ADS868x_ReadADC(&ads8681).DataWord >> 16);
}

/*
 * @brief Read ADC convertion data.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @return 16-bit ADC data type of ADS868x_OutputDataWord_t.
 *
 **/
ADS868x_OutputDataWord_t ADS868x_ReadADC(ADS868x_GInst_t* device)
{
	uint16_t timeout = 10000;
	
	ADS868x_OutputDataWord_t DataWord;
	
	device->data.ADDRESS = ADS868x_NOP;
	device->data.COMMAND = ADS868x_NOP;
	device->data.REG_DATA_LSB = ADS868x_NOP;
	device->data.REG_DATA_MSB = ADS868x_NOP;
	device->spi_tx(&device->data.Command, 4);
	
	while (*device->tx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}	
	
	//device->delay(1);
	device->spi_rx(&device->data.DataWord_HSW_MSB, 4);
	timeout = 10000;
	while (*device->rx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}	
	//device->delay(1);
	DataWord.DataWord_LSW_LSB = device->data.DataWord_LSW_LSB;
	DataWord.DataWord_LSW_MSB = device->data.DataWord_LSW_MSB;
	DataWord.DataWord_HSW_LSB = device->data.DataWord_HSW_LSB;
	DataWord.DataWord_HSW_MSB = device->data.DataWord_HSW_MSB;
	
	return DataWord;
}

/*
 * @brief Read device register.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param address : Device address that you wish.
 *
 **/
uint16_t ADS868x_R_REG(ADS868x_GInst_t* device, uint8_t address)
{
	uint16_t Data = 0;
	
	device->data.ADDRESS = address;
	device->data.COMMAND = ADS868x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);
	
	device->delay(1);
	device->spi_rx(&device->data.DataWord_LSW_MSB, 2);
	device->delay(1);
	return Data  = ((uint16_t)device->data.DataWord_LSW_MSB << 8) | (uint16_t)device->data.DataWord_LSW_LSB;	
}

/*
 * @brief Controls the reset and power-down features.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables the NAP mode of the converter, 1 enables the converter to enter NAP mode if CONVST/CS 
 *        is held high after the current conversion completes.
 * @param rstn_app : If 0 RST pin functions as a POR class reset (causes full device initialization) if 1 RST pin 
 *        functions as an application reset (only user-programmed modes are cleared).
 * @param in_al_dis : If 0 input alarm is enabled, 1 input alarm is disabled.
 * @param vdd_al_dis : If 0 VDD alarm is enabled, 1 VDD alarm is disabled.
 *
 **/
void ADS868x_RstPwdn(ADS868x_GInst_t* device, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis)
{
	ADS868x_RstPwrCtrl_t RstPwrCtlReg;
	
	RstPwrCtlReg.PWRDN = pwrdn;
	RstPwrCtlReg.NAP_EN = nap_en;
	RstPwrCtlReg.RSTn_APP = rstn_app;
	RstPwrCtlReg.IN_AL_DIS = in_al_dis;
	RstPwrCtlReg.VDD_AL_DIS = vdd_al_dis;
	RstPwrCtlReg.WKEY = ADS868x_WKEY;
	
	device->data.ADDRESS = ADS868x_RST_PWRCTL_LSW;
	device->data.COMMAND = ADS868x_WRITE_MSB;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = ADS868x_WKEY;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.COMMAND = ADS868x_WRITE_LSB;
	device->data.REG_DATA_LSB = (uint8_t)RstPwrCtlReg.RstPwrCtrlReg_LSW;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Configures the protocol used for writing data.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param protocol : Selects the SPI protocol, see ADS868x_SPI_PROTOCOL_t enum (default ADS868x_CPOL0_CPHASE0).
 *
 **/
void ADS868x_SdiCtrl(ADS868x_GInst_t* device, ADS868x_SPI_PROTOCOL_t protocol)
{
	device->data.ADDRESS = ADS868x_SDI_CTL_LSW;
	device->data.COMMAND = ADS868x_WRITE_LSB;
	device->data.REG_DATA_LSB = protocol;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Controls data protocol used to transmit data from the SDO-x pins of the device.
 * NOTE. This function resets the GPO pin (GPO_VAL) to zero.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param sdo_mode : If 0xb, SDO mode follows the same SPI protocol as that used for SDI (default), see the SDI_CTL_REG register.
 *        If 10b, invalid configuration. If 11b SDO mode follows the ADC master clock or source-synchronous protocol.
 * @param ssync_clk : If 0b, external SCLK selected (no division, default), 1b - internal clock selected (no division).
 * @param sdo1_config : It used to configure ALARM/SDO-1/GPO, see the ADS868x_SDO1_MODE_t enum.
 *
 *
 **/
void ADS868x_SdoCtrl(ADS868x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk, ADS868x_SDO1_MODE_t sdo1_config)
{
	ADS868x_SdoCtrl_t SdoCtrlReg;
	
	SdoCtrlReg.SDO_MODE = sdo_mode;
	SdoCtrlReg.SSYNC_CLK = ssync_clk;
	SdoCtrlReg.SDO1_CONFIG = sdo1_config;
	SdoCtrlReg.GPO_VAL = 0;
	
	device->data.ADDRESS = ADS868x_SDO_CTL_LSW;
	device->data.COMMAND = ADS868x_WRITE_HWORD;	
	device->data.REG_DATA_LSB = SdoCtrlReg.SdoCtrlReg_LSW;
	device->data.REG_DATA_MSB = SdoCtrlReg.SdoCtrlReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);		
}

/*
 * @brief Drives logical level of general purpoise pin (SDO1 as a GPO pin, that alternative function should be 
 * selected in advance by ADS868x_SdoCtrl function above).
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param gpo_val : 1-bit value for the output on the GPO pin (can be 0 or 1).
 *
 **/
void ADS868x_SdoPinSetReset(ADS868x_GInst_t* device, uint8_t sdo_val)
{	
	device->data.ADDRESS = ADS868x_SDO_CTL_LSW;	
	if (sdo_val) {
		device->data.COMMAND = ADS868x_SET_HWORD;
	}
	else {
		device->data.COMMAND = ADS868x_RESET_HWORD;
	}	
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0x10;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects data format for the output data.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param data_val : It controls the data value output by the converter, see the ADS868x_DATA_VAL_t enum.
 * @param par_en : If 0b output data does not contain parity information, if 1b two parity bits (ADC output
 *	      and output data frame) are appended to the LSBs of the output data. See ADS868x_PAR_t enum.
 * @param range_incl : 0b do not include the range configuration register value, 1b include the range configuration
 *        register value. See ADS868x_RANGE_INCL_t enum.
 * @param in_active_alarm_incl : Control to include the active input ALARM flags in the SDO-x output bit stream.
 *        See the ADS868x_IN_ACTIVE_ALARM_t enum.
 * @param vdd_active_alarm_incl : Control to include the active VDD ALARM flags in the SDO-x output bit stream.
 *        See the ADS868x_VDD_ACTIVE_ALARM_t enum.
 * @param device_addr_incl :  0b do not include the register value, 1b include the register value. See ADS868x_DEV_ADDR_INCL_t enum.
 *
 **/
void ADS868x_DataOutCtrl(ADS868x_GInst_t* device, ADS868x_DATA_VAL_t data_val, ADS868x_PAR_t par_en, ADS868x_RANGE_INCL_t range_incl, 
	ADS868x_IN_ACTIVE_ALARM_t in_active_alarm_incl, ADS868x_VDD_ACTIVE_ALARM_t vdd_active_alarm_incl, ADS868x_DEV_ADDR_INCL_t device_addr_incl)
{
	ADS868x_DataOut_t DataOutReg;
	
	DataOutReg.DATA_VAL = data_val;
	DataOutReg.PAR_EN = par_en;
	DataOutReg.RANGE_INCL = range_incl;
	DataOutReg.IN_ACTIVE_ALARM_INCL = in_active_alarm_incl;
	DataOutReg.VDD_ACTIVE_ALARM_INCL = vdd_active_alarm_incl;
	DataOutReg.DEVICE_ADDR_INCL = device_addr_incl;
	
	device->data.ADDRESS = ADS868x_DATAOUT_CTL_LSW;
	device->data.COMMAND = ADS868x_WRITE_HWORD;
	device->data.REG_DATA_LSB = DataOutReg.DataOutReg_LSW;
	device->data.REG_DATA_MSB = DataOutReg.DataOutReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects either internal or external reference and selects input range.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param range_sel : It selects one of nine input ranges, see the ADS868x_INPUT_RANGE_t enum.
 * @param intref_dis : 0 enables internal voltage reference, 1 disables this one.
 *
 **/
void ADS868x_RangeSel(ADS868x_GInst_t* device, ADS868x_INPUT_RANGE_t range_sel, uint8_t intref_dis)
{
	ADS868x_RangeSel_t RangeSelReg;
	
	RangeSelReg.RANGE_SEL = range_sel;
	RangeSelReg.INTREF_DIS = intref_dis;
	
	device->data.ADDRESS = ADS868x_RANGE_SEL_LSW;
	device->data.COMMAND = ADS868x_WRITE_HWORD;
	device->data.REG_DATA_LSB = RangeSelReg.RangeSelReg_LSW;
	device->data.REG_DATA_MSB = RangeSelReg.RangeSelReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Return output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 *
 **/
ADS868x_Alarm_t ADS868x_ReadAlarm(ADS868x_GInst_t* device)
{
	ADS868x_Alarm_t AlarmReg;
	
	device->data.ADDRESS = ADS868x_ALARM_LSW;
	device->data.COMMAND = ADS868x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->spi_rx(&device->data.DataWord_LSW_MSB, 2);
	AlarmReg.AlarmReg_LSW = ((uint16_t)device->data.DataWord_LSW_MSB << 8) | (uint16_t)device->data.DataWord_LSW_LSB;
	
	return AlarmReg;
}

/*
 * @brief Sets hysteresis and high threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param inp_alrm_high_th : 16-bit threshold for comparison is INP_ALRM_HIGH_TH.
 * @param inp_alrm_hyst : 6-bit hysteresis value for the input ALARM.
 *
 **/
void ADS868x_SetAlarmHTh(ADS868x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst)
{
	ADS868x_AlarmHTh_t AlarmHthReg;
	
	AlarmHthReg.RESERVED23_16 = 0;
	AlarmHthReg.RESERVED25_24 = 0;
	AlarmHthReg.INP_ALRM_HIGH_TH = inp_alrm_high_th;	
	AlarmHthReg.INP_ALRM_HYST = inp_alrm_hyst;
	
	device->data.ADDRESS = ADS868x_ALARM_H_TH_LSW;
	device->data.COMMAND = ADS868x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_LSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.ADDRESS = ADS868x_ALARM_H_TH_HSW;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_HSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_HSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS868x_GInst_t.
 * @param inp_alrm_low_th : 16-bit threshold for comparison is INP_ALRM_LOW_TH.
 *
 **/
void ADS868x_SetAlarmLTh(ADS868x_GInst_t* device, uint16_t inp_alrm_low_th)
{
	ADS868x_AlarmLTh_t AlarmLThReg;
	
	AlarmLThReg.INP_ALRM_LOW_TH = inp_alrm_low_th;
	
	device->data.ADDRESS = ADS868x_ALARM_L_TH_LSW;
	device->data.COMMAND = ADS868x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmLThReg.AlarmLTh_LSW;
	device->data.REG_DATA_MSB = AlarmLThReg.AlarmLTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}


/* Hardware dependent functions. */

/*
 * @brief
 *
 **/
void ADS868x_SPI_Tx(uint8_t *pData, uint8_t size) {
	
	ADS868x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Transmit(&hspi1, pData, size, 10);
	ADS868x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief
 * 
 *
 **/
void ADS868x_SPI_Rx(uint8_t *pData, uint8_t size) {
	
	ADS868x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Receive(&hspi1, pData, size, 10);
	ADS868x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief SPI chip select.
 * 
 * @param gpio : Either CS_ADC_GPIO_Port or CS_DAC_GPIO_Port.
 * @param gpio_pin : Either CS_ADC_Pin or CS_DAC_Pin.
 * @param state : Either GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void ADS868x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}