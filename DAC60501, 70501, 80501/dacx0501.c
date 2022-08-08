/*
 * @brief Common driver for DAC60501, DAC70501, DAC80501 from the Texas Instruments.
 * Created 07.19.21 by asw3005. 
 *
 **/

#include "dacx0501.h"
#include "spi.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;

/* Private function prototypes. */
static void DACx0501_CheckRef(void* reference);
static void DACx0501_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/* SPI set of functions. */

/*
 * @brief Write RAW DAC data.
 *
 * @param voltage : data from 0 to 4095.
 *
 **/
void DAC60501_WriteRAWData(uint16_t data) {

	uint16_t dac_data = 0;

	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac60501 = {
		.delay = HAL_Delay,
		.spi_tx = DACx0501_SPI_Tx
	};

	dac_data = data;
	if (dac_data > DAC60501_12BIT) { dac_data = DAC60501_12BIT; }

	DACx0501_SPI_WriteData(&dac60501, DACx0501_DAC60501, dac_data);
}

/*
 * @brief Write RAW DAC data.
 *
 * @param voltage : data from 0 to 16383.
 *
 **/
void DAC70501_WriteRAWData(uint16_t data) {

	uint16_t dac_data = 0;

	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac70501 = {
		.delay = HAL_Delay,
		.spi_tx = DACx0501_SPI_Tx
	};

	dac_data = data;
	if (dac_data > DAC70501_14BIT) { dac_data = DAC70501_14BIT; }

	DACx0501_SPI_WriteData(&dac70501, DACx0501_DAC70501, dac_data);
}

/*
 * @brief Write RAW DAC data.
 *
 * @param voltage : data from 0 to 65535.
 *
 **/
void DAC80501_WriteRAWData(uint16_t data) {

	uint16_t dac_data = 0;

	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac80501 = {
		.delay = HAL_Delay,
		.spi_tx = DACx0501_SPI_Tx
	};

	dac_data = data;
	if (dac_data > DAC80501_16BIT) { dac_data = DAC80501_16BIT; }

	DACx0501_SPI_WriteData(&dac80501, DAC80501_16BIT, dac_data);
}

/*
 * @brief Set DAC voltage.
 * 
 * @param voltage : Voltage from 0 to 5000 mV.
 *
 **/
void DAC60501_SetVoltage(float voltage) {
	
	uint16_t dac_data = 0;
	
	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac60501 = { 			
		.delay = HAL_Delay,		
		.spi_tx = DACx0501_SPI_Tx
	};
	
	dac_data = (uint16_t)(voltage * DAC60501_VALUE_OF_DIVISION);
	if (dac_data > DAC60501_12BIT) { dac_data = DAC60501_12BIT; }
	
	DACx0501_SPI_WriteData(&dac60501, DACx0501_DAC60501, dac_data);
}

/*
 * @brief Set DAC voltage.
 *
 * @param voltage : Voltage from 0 to 5000 mV.
 *
 **/
void DAC70501_SetVoltage(float voltage) {

	uint16_t dac_data = 0;

	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac70501 = {
		.delay = HAL_Delay,
		.spi_tx = DACx0501_SPI_Tx
	};

	dac_data = (uint16_t)(voltage * DAC70501_VALUE_OF_DIVISION);
	if (dac_data > DAC70501_14BIT) { dac_data = DAC70501_14BIT; }

	DACx0501_SPI_WriteData(&dac70501, DACx0501_DAC70501, dac_data);
}

/*
 * @brief Set DAC voltage.
 *
 * @param voltage : Voltage from 0 to 5000 mV.
 *
 **/
void DAC80501_SetVoltage(float voltage) {

	uint16_t dac_data = 0;

	/* General data struct of DAC unit. */
	DACx0501_GInst_t dac70501 = {
		.delay = HAL_Delay,
		.spi_tx = DACx0501_SPI_Tx
	};

	dac_data = (uint16_t)(voltage * DAC80501_VALUE_OF_DIVISION);
	if (dac_data > DAC80501_16BIT) { dac_data = DAC80501_16BIT; }

	DACx0501_SPI_WriteData(&dac70501, DACx0501_DAC80501, dac_data);
}


/*
 * @brief Enable or disable dac sync function.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param dac_sync : When set to 1, the DAC output is set to update in response to an LDAC trigger (synchronous mode).
 *        When cleared to 0 ,the DAC output is set to update immediately (asynchronous mode), default.
 *
 **/
void DACx0501_SPI_Sync(DACx0501_GInst_t *device, uint8_t dac_sync) {
	
	DACx0501_Sync_t dac_sync_reg;
	
	dac_sync_reg.DAC_SYNC_EN = dac_sync;
	
	device->Command = DACx0501_SYNC;
	device->DataLSB = dac_sync_reg.SyncLSB;	
	device->DataMSB = dac_sync_reg.SyncMSB;	
	device->spi_tx(&device->Command, 3);	
}

/*
 * @brief Configuration internal reference and power-down mode.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param dac_pwdwn : When set to 1, the DAC in power-down mode and the DAC output is connected to GND through a 1-kΩ internal 
 *        resistor.
 * @param ref_pwdwn : When set to 1, this bit disables the device internal reference.
 *
 **/
void DACx0501_SPI_Config(DACx0501_GInst_t *device, uint8_t dac_pwdwn, uint8_t ref_pwdwn) {
	
	DACx0501_Config_t dac_config_reg;
	
	dac_config_reg.DAC_PWDWN = dac_pwdwn;
	dac_config_reg.REF_PWDWN = ref_pwdwn;
	
	device->Command = DACx0501_CONFIG;
	device->DataLSB = dac_config_reg.ConfigLSB;	
	device->DataMSB = dac_config_reg.ConfigMSB;
	device->spi_tx(&device->Command, 3);
}

/*
 * @brief Selecting gain of buffer and divider for the reference voltage.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param buff_gain : When set to 1 (default), the buffer amplifier for corresponding DAC has a gain of 2. When cleared to 0, the buffer 
 *		  amplifier for corresponding DAC has a gain of 1.
 * @param ref_div : When REF-DIV set to 1, the reference voltage is internally divided by a factor of 2. When REF-DIV is cleared
 *        to 0 (default), the reference voltage is unaffected.
 *
 **/
void DACx0501_SPI_Gain(DACx0501_GInst_t *device, uint8_t buff_gain, uint8_t ref_div) {
	
	DACx0501_Gain_t dac_gain_reg;
	
	dac_gain_reg.BUFF_GAIN = buff_gain;
	dac_gain_reg.REF_DIV = ref_div;
	
	device->Command = DACx0501_GAIN;
	device->DataLSB = dac_gain_reg.GainLSB;	
	device->DataMSB = dac_gain_reg.GainMSB;
	device->spi_tx(&device->Command, 3);
}

/*
 * @brief Starts soft-reset and synchronous mode control.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param soft_reset : When set to the reserved code of DACx0501_SOFT_RESET of the DACx0501_Command enum (binary 1010), this bit
 *        resets the device to the default state. These bits are self resetting. 
 * @param ldac : Set this bit to 1 to synchronously load the DAC in synchronous mode. This bit is self resetting.
 *
 **/
void DACx0501_SPI_Trigger(DACx0501_GInst_t *device, uint8_t soft_reset, uint8_t ldac) {
	
	DACx0501_Trigger_t dac_trigger_reg;
	
	dac_trigger_reg.SOFT_RESET = soft_reset;
	dac_trigger_reg.LDAC = ldac;
	
	device->Command = DACx0501_TRIGGER;
	device->DataLSB = dac_trigger_reg.TriggerLSB;	
	device->DataMSB = dac_trigger_reg.TriggerMSB;
	device->spi_tx(&device->Command, 3);	
}

/*
 * @brief Writes data to the DAC data register.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param dac_type : Type of DAC listed in the DACx0501_Type enum. May be DACx0501_DAC60501, DACx0501_DAC70501, DACx0501_DAC80501.
 * @param dac_data : Digital code that conversion to voltage on the output of the DAC.
 *
 **/
void DACx0501_SPI_WriteData(DACx0501_GInst_t *device, uint8_t dac_type, uint16_t dac_data) {
	
	DACx0501_DACReg_t dac_data_reg;
	
	if (dac_type == DACx0501_DAC60501) {
		dac_data_reg.DAC60501_DATA = dac_data;
	} else if (dac_type == DACx0501_DAC70501) {
		dac_data_reg.DAC70501_DATA = dac_data;
	} else if (dac_type == DACx0501_DAC80501) {
		dac_data_reg.DAC80501_DATA = dac_data;
	}
	
	device->Command = DACx0501_DAC_DATA;
	device->DataLSB = dac_data_reg.DataLSB;	
	device->DataMSB = dac_data_reg.DataMSB;
	device->spi_tx(&device->Command, 3);
}




/* I2C set of functions. */

/*
 * @brief Enable or disable dac sync function.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param dac_sync : When set to 1, the DAC output is set to update in response to an LDAC trigger (synchronous mode).
 *        When cleared to 0 ,the DAC output is set to update immediately (asynchronous mode), default.
 *
 **/
void DACx0501_I2C_Sync(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_sync) {
	
	DACx0501_Sync_t dac_sync_reg;
	
	dac_sync_reg.DAC_SYNC_EN = dac_sync;
	
	device->Command = DACx0501_SYNC;
	device->DataLSB = dac_sync_reg.SyncLSB;	
	device->DataMSB = dac_sync_reg.SyncMSB;
	device->i2c_tx(address, device->Command, 1,  &device->DataMSB, 2);	
}

/*
 * @brief Read device ID and scale mode.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 *
 **/
DACx0501_Devid_t DACx0501_I2C_ReadDevId(DACx0501_GInst_t *device, DACx0501_I2C_Addr address) {
	
	DACx0501_Devid_t dac_devid_reg;
	
	device->Command = DACx0501_DEVID;
	device->i2c_rx(address, device->Command, 1, &device->DataMSB, 2);
	/* A bit delay is need. */
	device->delay(1);
	dac_devid_reg.DevIdLSB = device->DataLSB;
	dac_devid_reg.DevIdMSB = device->DataMSB;
	
	return dac_devid_reg;
}

/*
 * @brief Read reference alarm flag.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 *
 **/
DACx0501_Status_t DACx0501_I2C_ReadStatus(DACx0501_GInst_t *device, DACx0501_I2C_Addr address) {
	
	DACx0501_Status_t dac_status_reg;
	
	device->Command = DACx0501_STATUS;
	device->i2c_rx(address, device->Command, 1, &device->DataMSB, 2);
	/* A bit delay is need. */
	device->delay(1);
	dac_status_reg.StatusLSB = device->DataLSB;
	dac_status_reg.StatusMSB = device->DataMSB;
	
	return dac_status_reg;
}

/*
 * @brief Configuration internal reference and power-down mode.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param dac_pwdwn : When set to 1, the DAC in power-down mode and the DAC output is connected to GND through a 1-kΩ internal 
 *        resistor.
 * @param ref_pwdwn : When set to 1, this bit disables the device internal reference.
 *
 **/
void DACx0501_I2C_Config(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_pwdwn, uint8_t ref_pwdwn) {
	
	DACx0501_Config_t dac_config_reg;
	
	dac_config_reg.DAC_PWDWN = dac_pwdwn;
	dac_config_reg.REF_PWDWN = ref_pwdwn;
	
	device->Command = DACx0501_CONFIG;
	device->DataLSB = dac_config_reg.ConfigLSB;	
	device->DataMSB = dac_config_reg.ConfigMSB;
	device->i2c_tx(address, device->Command, 1, &device->DataMSB, 2);
}

/*
 * @brief Selecting gain of buffer and divider for the reference voltage.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param buff_gain : When set to 1, the buffer amplifier for corresponding DAC has a gain of 2. When cleared to 0, the buffer amplifier
 *        for corresponding DAC has a gain of 1.
 * @param ref_div : When REF-DIV set to 1, the reference voltage is internally divided by a factor of 2. When REF-DIV is cleared
 *        to 0, the reference voltage is unaffected.
 *
 **/
void DACx0501_I2C_Gain(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t buff_gain, uint8_t ref_div) {
	
	DACx0501_Gain_t dac_gain_reg;
	
	dac_gain_reg.BUFF_GAIN = buff_gain;
	dac_gain_reg.REF_DIV = ref_div;
	
	device->Command = DACx0501_GAIN;
	device->DataLSB = dac_gain_reg.GainLSB;	
	device->DataMSB = dac_gain_reg.GainMSB;
	device->i2c_tx(address, device->Command, 1, &device->DataMSB, 2);
}

/*
 * @brief Starts soft-reset and synchronous mode control.
 *
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param soft_reset : When set to the reserved code of DACx0501_SOFT_RESET of the DACx0501_Command unum (binary 1010), this bit
 *        resets the device to the default state. These bits are self resetting. 
 * @param ldac : Set this bit to 1 to synchronously load the DAC in synchronous mode, This bit is self resetting.
 *
 **/
void DACx0501_I2C_Trigger(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t soft_reset, uint8_t ldac) {
	
	DACx0501_Trigger_t dac_trigger_reg;
	
	dac_trigger_reg.SOFT_RESET = soft_reset;
	dac_trigger_reg.LDAC = ldac;
	
	device->Command = DACx0501_TRIGGER;
	device->DataLSB = dac_trigger_reg.TriggerLSB;	
	device->DataMSB = dac_trigger_reg.TriggerMSB;
	device->i2c_tx(address, device->Command, 1, &device->DataMSB, 2);	
}

/*
 * @brief Writes data to the DAC data register.
 * 
 * @param *device : Instance of general data struct DACx0501_GInst_t.
 * @param address : I2C device address, must be shifted 1 bit to the left.
 * @param dac_type : Type of DAC listed in the DACx0501_Type enum. May be DACx0501_DAC60501, DACx0501_DAC70501, DACx0501_DAC80501.
 * @param dac_data : Digital code that conversion to voltage on the output of the DAC.
 *
 **/
void DACx0501_I2C_WriteData(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_type, uint16_t dac_data) {
	
	DACx0501_DACReg_t dac_data_reg;
	
	if (dac_type == DACx0501_DAC60501) {
		dac_data_reg.DAC60501_DATA = dac_data;
	}
	else if (dac_type == DACx0501_DAC70501) {
		dac_data_reg.DAC70501_DATA = dac_data;
	}
	else if (dac_type == DACx0501_DAC80501) {
		dac_data_reg.DAC80501_DATA = dac_data;
	}
	
	device->Command = DACx0501_DAC_DATA;
	device->DataLSB = dac_data_reg.DataLSB;	
	device->DataMSB = dac_data_reg.DataMSB;
	device->i2c_tx(address, device->Command, 1, &device->DataMSB, 2);
}


/*
 * @brief 
 *
 **/
static void DACx0501_CheckRef(void* reference) {
	
	if (reference == NULL) {
		while(1) {
			/* Reference error occured. */
		}
	}	
}


/* Hardware dependent functions. */

/*
 * @brief
 *
 **/
void DACx0501_SPI_Tx(uint8_t *pData, uint8_t size) {
	
	DACx0501_SPI_CS(CS_DAC_GPIO_Port, CS_DAC_Pin, 0);
	HAL_SPI_Transmit(&hspi1, pData, size, 10);
	DACx0501_SPI_CS(CS_DAC_GPIO_Port, CS_DAC_Pin, 1);
}

/*
 * @brief SPI chip select.
 * 
 * @param gpio : Either CS_ADC_GPIO_Port or CS_DAC_GPIO_Port.
 * @param gpio_pin : Either CS_ADC_Pin or CS_DAC_Pin.
 * @param state : Either GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void DACx0501_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}