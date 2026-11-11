/*
 * ext_load_drv.hpp
 *
 *  Created on: Aug 14, 2026
 *      Author: asw3005
 */

#ifndef EXT_LOAD_DRV_HPP_
#define EXT_LOAD_DRV_HPP_


#include "drv8955.h"


class ExtLoadDriver {

private:

	TIM_HandleTypeDef *tim;
	uint32_t tim_ch1;
	uint32_t tim_ch2;
	DAC_HandleTypeDef* dac;
	uint32_t dac_ch1;
	uint32_t dac_ch2;
	OPAMP_HandleTypeDef* opamp;


public:

	enum class ECL_LOAD_CURRENT_t {

		ILOAD_150mA = 150,
		ILOAD_420mA = 420,
		ILOAD_500mA = 500,
		ILOAD_600mA = 600,
		ILOAD_MAX 	= 600
	};

	enum class ELD_DAC_OUT_VOLTAGE_t {

		DACV_2CH_720mA 	= 4095,
		DACV_2CH_600mA 	= 3407,
		DACV_2CH_300mA 	= 1704,
		DACV_2CH_150mA 	= 861,

		DACV_1CH_600mA 	= 1704,
		DACV_1CH_500mA 	= 1420,
		DACV_1CH_420mA	= 1120,
		DACV_1CH_150mA 	= 431,

		DACV_1CH_MAX 	= 1704

	} DacVoltage;

	enum class ELD_SLEEP_CONTROL_t {

		SLEEP,
		WAKE_UP
	};

	enum class ELD_OUTPUT_STATE_t {

		LOW_SIDE_SWITCH_ON,
		HIGH_SIDE_SWITCH_ON,
		LOAD_ON = 0,
		LOAD_OFF
	};

	enum class ELD_OUTPUT_SELECT_t {

		OUT12 = 1,
		OUT34
	};

	ExtLoadDriver(TIM_HandleTypeDef* tim, uint32_t tim_ch, DAC_HandleTypeDef* dac, uint32_t dac_ch, OPAMP_HandleTypeDef* opamp);
	ExtLoadDriver(TIM_HandleTypeDef* tim, uint32_t tim_ch1, uint32_t tim_ch2, DAC_HandleTypeDef* dac, uint32_t dac_ch1, uint32_t dac_ch2, OPAMP_HandleTypeDef* opamp);

	void ELD_InitOneChannel(void);
	void ELD_InitTwoChannels(void);

	void ELD_SetCurrent(uint32_t load_current);
	void ELD_SetCurrent(ELD_OUTPUT_SELECT_t channel, uint32_t load_current);

	void ELD_SetCurrentProfile(ECL_LOAD_CURRENT_t i_profile);
	void ELD_SetCurrentProfile(ELD_OUTPUT_SELECT_t channel, ECL_LOAD_CURRENT_t i_profile);

	void ELD_SleepCtrl(ELD_SLEEP_CONTROL_t state);
	void ELD_OutCtrl(ELD_OUTPUT_SELECT_t output, ELD_OUTPUT_STATE_t out_state);
};



#endif /* EXT_LOAD_DRV_HPP_ */
