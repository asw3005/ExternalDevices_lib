/*
 * ext_load_drv.cpp
 *
 *  Created on: Aug 14, 2026
 *      Author: asw3005
 */


#include "ext_load_drv.hpp"


ExtLoadDriver::ExtLoadDriver(TIM_HandleTypeDef* tim, uint32_t tim_ch, DAC_HandleTypeDef* dac, uint32_t dac_ch, OPAMP_HandleTypeDef* opamp) :
		tim(tim),
		tim_ch1(tim_ch),
		dac(dac),
		dac_ch1(dac_ch),
		opamp(opamp)

{
}

ExtLoadDriver::ExtLoadDriver(TIM_HandleTypeDef* tim, uint32_t tim_ch1, uint32_t tim_ch2, DAC_HandleTypeDef* dac, uint32_t dac_ch1, uint32_t dac_ch2, OPAMP_HandleTypeDef* opamp) :
		tim(tim),
		tim_ch1(tim_ch1),
		tim_ch2(tim_ch2),
		dac(dac),
		dac_ch1(dac_ch1),
		dac_ch2(dac_ch2),
		opamp(opamp)

{}

/*
 * @brief Init.
 */
void ExtLoadDriver::ELD_InitOneChannel(void) {

	DRV8955_Init(this->tim, this->tim_ch1, this->dac_ch1);
	ExtLoadDriver::ELD_SetCurrentProfile(ELD_CURRENT_PROFILE_t::ILOAD_1CH_600mA);
	HAL_DAC_Start(this->dac, this->dac_ch1);
	HAL_OPAMP_Start(this->opamp);
	ExtLoadDriver::ELD_OutCtrl(ELD_OUTPUT_SELECT_t::OUT34, ELD_OUTPUT_STATE_t::LOAD_OFF);
	ExtLoadDriver::ELD_SleepCtrl(ELD_SLEEP_CONTROL_t::WAKE_UP);
}

/*
 * @brief Init.
 */
void ExtLoadDriver::ELD_InitTwoChannels(void) {

	DRV8955_Init(this->tim, this->tim_ch1, this->tim_ch2);
	ExtLoadDriver::ELD_SetCurrentProfile(ELD_CURRENT_PROFILE_t::ILOAD_2CH_600mA);
	HAL_DAC_Start(this->dac, this->dac_ch1);
	HAL_DAC_Start(this->dac, this->dac_ch2);
	HAL_OPAMP_Start(this->opamp);
	ExtLoadDriver::ELD_OutCtrl(ELD_OUTPUT_SELECT_t::OUT12, ELD_OUTPUT_STATE_t::LOAD_OFF);
	ExtLoadDriver::ELD_OutCtrl(ELD_OUTPUT_SELECT_t::OUT34, ELD_OUTPUT_STATE_t::LOAD_OFF);
	ExtLoadDriver::ELD_SleepCtrl(ELD_SLEEP_CONTROL_t::WAKE_UP);
}

/*
 * @brief Set RAW value.
 */
void ExtLoadDriver::ELD_SetCurrentProfile(ELD_CURRENT_PROFILE_t i_profile) {

	switch(i_profile) {

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_150mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_150mA;
		break;

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_500mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_500mA;
		break;

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_600mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_600mA;
		break;

	default:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_150mA;
		break;

	}
	HAL_DAC_SetValue(this->dac, this->dac_ch1, DAC_ALIGN_12B_R, static_cast<uint32_t>(DacVoltage));
}

/*
 * @brief Set RAW value.
 */
void ExtLoadDriver::ELD_SetCurrentProfile(ELD_OUTPUT_SELECT_t channel, ELD_CURRENT_PROFILE_t i_profile) {

	switch(i_profile) {

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_150mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_150mA;
		break;

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_500mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_500mA;
		break;

	case ELD_CURRENT_PROFILE_t::ILOAD_1CH_600mA:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_600mA;
		break;

	default:
		DacVoltage = ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_150mA;
		break;

	}

	if (channel == ELD_OUTPUT_SELECT_t::OUT12) {
		HAL_DAC_SetValue(this->dac, this->dac_ch1, DAC_ALIGN_12B_R, static_cast<uint32_t>(DacVoltage));
	} else if (channel == ELD_OUTPUT_SELECT_t::OUT34) {
		HAL_DAC_SetValue(this->dac, this->dac_ch2, DAC_ALIGN_12B_R, static_cast<uint32_t>(DacVoltage));
	}
}

/*
 * @brief Set RAW value.
 */
void ExtLoadDriver::ELD_SetCurrent(uint32_t load_current) {

	if (load_current > static_cast<uint32_t>(ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_MAX)) {
		load_current = static_cast<uint32_t>(ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_MAX);
	}
	HAL_DAC_SetValue(this->dac, this->dac_ch1, DAC_ALIGN_12B_R, load_current);
}

/*
 * @brief Set RAW value.
 */
void ExtLoadDriver::ELD_SetCurrent(ELD_OUTPUT_SELECT_t channel, uint32_t load_current) {

	if (load_current > static_cast<uint32_t>(ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_MAX)) {
		load_current = static_cast<uint32_t>(ELD_DAC_OUT_VOLTAGE_t::DACV_1CH_MAX);
	}

	if (channel == ELD_OUTPUT_SELECT_t::OUT12) {
		HAL_DAC_SetValue(this->dac, this->dac_ch1, DAC_ALIGN_12B_R, load_current);
	} else if (channel == ELD_OUTPUT_SELECT_t::OUT34) {
		HAL_DAC_SetValue(this->dac, this->dac_ch2, DAC_ALIGN_12B_R, load_current);
	}
}

/*
 * @brief
 */
void ExtLoadDriver::ELD_SleepCtrl(ELD_SLEEP_CONTROL_t state) {

	DRV8955_SleepCtrl(static_cast<uint8_t>(state));
}

/*
 * @brief
 */
void ExtLoadDriver::ELD_OutCtrl(ELD_OUTPUT_SELECT_t output, ELD_OUTPUT_STATE_t out_state) {


	switch (output) {
		#ifndef ONE_OUT_CTRL
		case ELD_OUTPUT_SELECT_t::OUT12:
			DRV8955_IN2Ctrl(static_cast<uint8_t>(out_state));
			break;
		#endif

		case ELD_OUTPUT_SELECT_t::OUT34:
			DRV8955_IN4Ctrl(static_cast<uint8_t>(out_state));
			break;

		default:
			DRV8955_IN4Ctrl(static_cast<uint8_t>(out_state));
			break;
	}
}
