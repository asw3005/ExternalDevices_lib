/*
 * ina226.c source file.
 *
 *  Created on: Jan 9, 2025
 *  Author: asw3005
 */

#include "ina226.h"
#include <stm32f1xx_hal.h>



/* External variables. */
extern I2C_HandleTypeDef hi2c1;

/* Private variables. */
static I2C_HandleTypeDef* INA226_I2C = &hi2c1;

/* Private function prototypes. */
static void INA226_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);
static void INA226_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);

/* Init general struct. */
static INA226_GStr_t ina226_inst = {
		.DevAddress = INA226_CURRENT_ADDR,
		.delay_fp = HAL_Delay,
		.i2c_rx_fp = INA226_I2CRxData,
		.i2c_tx_fp = INA226_I2CTxData
};

/*
 * @brief Initial configuration of the chip.
 */
void INA226_Init(void) {

	if(INA226_ReadID().DieID == INA226_ID_DIE) {

		/* Set INA226_CURRENT_LSB = INA226_MAX_EXPECTED_CURRENT / 32768.0f. */

		//INA226_WriteReg(uint8_t reg_address, uint16_t value);

		/* Set INA226_POWER_LSB = 25.0f * INA226_CURRENT_LSB. */

		/* Set INA226_CALIBRATION_VAL = 0.00512f / (INA226_CURRENT_LSB * RSHUNT). */

		__NOP();
	}

	__NOP();
}

/*
 * @brief Read shunt voltage, bus voltage, power, current.
 */
INA226_Meas_t INA226_GetMeas(void) {

	static INA226_Meas_t Meas;




	return Meas;
}

/*
 * @brief Read chip manufacturer ID and die ID.
 *
 */
INA226_ChipID_t INA226_ReadID(void) {

	uint8_t IdData[4];
	static INA226_ChipID_t ChipID;

	ina226_inst.i2c_rx_fp(INA226_MANUFACTURER_ID, &IdData[0], 4);
	ChipID.ManufactID = ((uint16_t)IdData[0] << 8) | IdData[1];
	ChipID.DieID = ((uint16_t)IdData[2] << 8) | IdData[3];
	return ChipID;
}

/*
 * @brief RW mask/enable register.
 *
 * @param len: When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit resets to the idle
 * 			states when the fault has been cleared. When the Alert Latch Enable bit is set to Latch mode, the Alert pin and
 * 			Alert Flag bit remains active following a fault until the Mask/Enable Register has been read.
 * 				1 - Latch enabled,
 * 				0 - Transparent (default).
 *
 * @param apol: 1 - Inverted (active-high open collector),
 * 				0 - Normal (active-low open collector) (default).
 *
 * @param ovf: This bit is set to '1' if an arithmetic operation resulted in an overflow error. The bit indicates that
 * 			current and power data can be invalid.
 *
 * @param cvrf: Although the device can be read at any time, and the data from the last conversion is available, the
 * 			Conversion Ready Flag bit is provided to help coordinate one-shot or triggered conversions. The Conversion Ready
 * 			Flag bit is set after all conversions, averaging, and multiplications are complete. Conversion Ready Flag bit
 * 			clears under the following conditions:
 *				1.) Writing to the Configuration Register (except for Power-Down selection)
 *				2.) Reading the Mask/Enable Register
 *
 * @param aff: While only one Alert Function can be monitored at the Alert pin at a time, the Conversion Ready can also
 * 			be enabled to assert the Alert pin. Reading the Alert Function Flag following an alert allows the user to
 * 			determine if the Alert Function is the source of the Alert.
 *			When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit clears only when the Mask/Enable
 *			Register is read. When the Alert Latch Enable bit is set to Transparent mode, the Alert Function Flag bit is
 *			cleared following the next conversion that does not result in an Alert condition.
 *
 * @param cnvr: Setting this bit high configures the Alert pin to be asserted if the Power calculation made following
 *  		a bus voltage measurement exceeds the value programmed in the Alert Limit Register.
 *
 * @param pol: Setting this bit high configures the Alert pin to be asserted if the Power calculation made following
 *  		a bus voltage measurement exceeds the value programmed in the Alert Limit Register.
 *
 * @param bul: Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement following
 * 			a conversion drops below the value programmed in the Alert Limit Register.
 *
 * @param bol: Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement following
 * 			a conversion exceeds the value programmed in the Alert Limit Register.
 *
 * @param sul: Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement following
 * 			a conversion drops below the value programmed in the Alert Limit Register.
 *
 * @param sol: Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement following
 * 			a conversion exceeds the value programmed in the Alert Limit Register.
 *
 * @return INA226_MaskEn_t: Return last read value.
 *
 */
INA226_MaskEn_t INA226_RWMaskEnReg(uint8_t rw, uint8_t len, uint8_t apol, uint8_t ovf, uint8_t cvrf, uint8_t aff,
				uint8_t cnvr, uint8_t pol, uint8_t bul, uint8_t bol, uint8_t sul, uint8_t sol) {

	uint8_t MaskEnData[2];
	static INA226_MaskEn_t MaskEnReg;

	if(!rw) {
		MaskEnReg.LEN = len;
		MaskEnReg.APOL = apol;
		MaskEnReg.OVF = ovf;
		MaskEnReg.CVRF = cvrf;
		MaskEnReg.AFF = aff;
		MaskEnReg.CNVR = cnvr;
		MaskEnReg.POL = pol;
		MaskEnReg.BUL = bul;
		MaskEnReg.BOL = bol;
		MaskEnReg.SUL = sul;
		MaskEnReg.SOL = sol;
		MaskEnReg.RESERVED9_5 = 0;

		ina226_inst.Data_LSB = MaskEnReg.MaskEnReg_LSB;
		ina226_inst.Data_MSB = MaskEnReg.MaskEnReg_MSB;
		ina226_inst.i2c_tx_fp(INA226_MASK_EN, &ina226_inst.Data_MSB, 2);
	} else {
		ina226_inst.i2c_rx_fp(INA226_MASK_EN, &MaskEnData[0], 2);
		MaskEnReg.MaskEnReg_LSB = MaskEnData[1];
		MaskEnReg.MaskEnReg_MSB = MaskEnData[0];
	}
	return MaskEnReg;
}


/*
 * @brief Configuration set (default 0x4127).
 *
 * @param mode: chip operating mode, Selects continuous, triggered, or power-down mode of operation (default 7).
 * 				0 - Power-Down (or Shutdown),
 * 				1 - Shunt Voltage, Triggered,
 * 				2 - Bus Voltage, Triggered,
 * 				3 - Shunt and Bus, Triggered,
 * 				4 - Power-Down (or Shutdown),
 * 				5 - Shunt Voltage, Continuous,
 * 				6 - Bus Voltage, Continuous,
 * 				7 - Shunt and Bus, Continuous.
 *
 * @param vshct: shunt voltage conversion time, Sets the conversion time for the shunt voltage measurement (default 4).
 * 				0 - 104us,   1 - 204us,
 * 				2 - 332us, 	 3 - 588us,
 * 				4 - 1.1ms,   5 - 2.116ms,
 * 				6 - 4.156ms, 7 - 8.244ms.
 *
 * @param vbusct: bus voltage conversion time, sets the conversion time for the bus voltage measurement (default 4).
 * 				0 - 104us,   1 - 204us,
 * 				2 - 332us, 	 3 - 588us,
 * 				4 - 1.1ms,   5 - 2.116ms,
 * 				6 - 4.156ms, 7 - 8.244ms.
 *
 * @param avg: averaging mode, determines the number of samples that are collected and averaged (default 0).
 * 				0 - 1,   1 - 4,
 * 				2 - 16,  3 - 64,
 * 				4 - 128, 5 - 256,
 * 				6 - 512, 7 - 1024.
 *
 * @param reset: reset bit, setting this bit to '1' generates a system reset that is the same as power-on reset.
 * 				Resets all registers to default values, this bit self-clears.
 *
 */
void INA226_SetCfgReg(uint8_t mode, uint8_t vshct, uint8_t vbusct, uint8_t avg, uint8_t reset) {

	INA226_CfgReg_t ConfigReg;

	ConfigReg.MODE2_0 = mode;
	ConfigReg.VSHCT2_0 = vshct;
	ConfigReg.VBUSCT2_0 = vbusct;
	ConfigReg.AVG2_0 = avg;
	ConfigReg.RST = reset;
	ConfigReg.RSVD14_12 = 4;

	ina226_inst.Data_LSB = ConfigReg.CfgReg_LSB;
	ina226_inst.Data_MSB = ConfigReg.CfgReg_MSB;
	ina226_inst.i2c_tx_fp(INA226_CFG, &ina226_inst.Data_MSB, 2);
}

/*
 * @brief Read register value.
 *
 * @param reg_address: register address to read.
 */
uint16_t INA226_ReadReg(uint8_t reg_address) {

	//static uint16_t RegData;

	ina226_inst.i2c_rx_fp(reg_address, &ina226_inst.Data_MSB, 2);
	return ((uint16_t)ina226_inst.Data_MSB << 8) | ina226_inst.Data_LSB;
}

/*
 * @brief Write register value.
 *
 * @param reg_address: register address to write.
 */
void INA226_WriteReg(uint8_t reg_address, uint16_t value) {

	ina226_inst.Data_LSB = value;
	ina226_inst.Data_MSB = value >> 8;

	ina226_inst.i2c_tx_fp(reg_address, &ina226_inst.Data_MSB, 2);
}



/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
static void INA226_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Read(INA226_I2C, ina226_inst.DevAddress, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

/*
 * @brief Transmit data to the chip.
 */
static void INA226_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Write(INA226_I2C, ina226_inst.DevAddress, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}









