/*
 *	@brief AM2301 driver.
 *	Created 08.24.2020 by asw3005.
 *
 **/

#include "stm32f4xx_hal.h"
#include "am2301.h"

#ifdef HARDWARE_SENSOR_READ
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#endif /* HARDWARE_SENSOR_READ */

/* Private variables. */
static AM2301_ConvertedData_t am2301_data = { 0 };

#ifdef HARDWARE_SENSOR_READ

/* Periferal instance. */
TIM_HandleTypeDef htim10;
/* Extern variables. */
extern SemaphoreHandle_t AM2301DataReady;
/* Private variables. */
static AM2301_RxDataBuff_t RxData = { 0 };
/* Callbacks' prototypes. */
static void TIM10_InputCaptureCallbackImpl(TIM_HandleTypeDef* htim);
static void MX_TIM10_Init(void);

#endif /* HARDWARE_SENSOR_READ */

/* Private function prototypes. */
static void AM2301_RxStart(void);

#ifndef HARDWARE_SENSOR_READ

/*
	@brief Software reading temperature and humidity from the sensor with the polling input pin of the MCU.
*/
AM2301_ConvertedData_t* AM2301_GetData(void) {

	uint32_t TLowCnt = 0, THighCnt = 0;
	uint32_t CntTimeout = AM2301_TIMEOUT;
	uint8_t	bit_cnt = 0, tmp_data = 0;
	uint8_t *pData = (uint8_t*)&am2301_data.data[3];

	/* NOTE: If you use this with RTOS this must be a critical section. */

	AM2301_RxStart();
	/* Read the Tgo bit. */
	while (HAL_GPIO_ReadPin(GPIO_DATA_PORT, GPIO_DATA_PIN) && CntTimeout != 0) {
		CntTimeout--;
	}
	CntTimeout = AM2301_TIMEOUT;
	/* Read the Responce (responce is pulling to low the bus). */
	while (!HAL_GPIO_ReadPin(GPIO_DATA_PORT, GPIO_DATA_PIN) && CntTimeout != 0) {
		TLowCnt++;
		CntTimeout--;
	}
	/* Wait HIGH teil of response. */
	while (HAL_GPIO_ReadPin(GPIO_DATA_PORT, GPIO_DATA_PIN) && CntTimeout != 0) {
		THighCnt++;
		CntTimeout--;
	}
	/* Check if presense pulse occured? */
	if (THighCnt - TLowCnt == AM2301_TIMEOUT) {
		__NOP();
		return NULL;
	}
	TLowCnt = 0;
	THighCnt = 0;

	/* Read the 40 data bits. */
	for (uint8_t i = 0; i < AM2301_RX_DATA; i++) {
		/* Wait LOW teil of data. */
		CntTimeout = AM2301_TIMEOUT;
		while (!HAL_GPIO_ReadPin(GPIO_DATA_PORT, GPIO_DATA_PIN) && CntTimeout != 0) {
			TLowCnt++;
			CntTimeout--;
		}
		CntTimeout = AM2301_TIMEOUT;
		/* Count HIGH teil of data. */
		while (HAL_GPIO_ReadPin(GPIO_DATA_PORT, GPIO_DATA_PIN) && CntTimeout != 0) {
			THighCnt++;
			CntTimeout--;
		}

		/* Create the data byte. The receiving is MSB first. */
		if (TLowCnt < THighCnt) {
			tmp_data |= 0x01;
		}

		bit_cnt++;
		if (bit_cnt == 8) {
			*pData = tmp_data;
			pData--;
			bit_cnt = 0;
			tmp_data = 0;
		}
		tmp_data <<= 1;
		TLowCnt = 0;
		THighCnt = 0;
	}
	/* Check parity. */	
	tmp_data = am2301_data.data[0] + am2301_data.data[1] + am2301_data.data[2] + am2301_data.data[3];
	if (am2301_data.Parity != tmp_data) {
		am2301_data.Temperature = AM2301_DATA_CHECK_ERROR;
	}

	#ifdef BELOW_ZERO_TEST
	am2301_data.data[0] = 0x65;
	am2301_data.data[1] = 0x80;
	#endif /* BELOW_ZERO */

	/* Check is  temperature below zero? */
	if (am2301_data.Temperature != AM2301_DATA_CHECK_ERROR) {
		if (am2301_data.data[1] & 0x80) {
			am2301_data.Temperature = -(((uint16_t)am2301_data.data[0] | (uint16_t)am2301_data.data[1] << 8) & 0x7FFF);
		}
		/* Check the parameters for MAX and MIN values. */
		if (am2301_data.Temperature > AM2301_TMAX) {
			am2301_data.Temperature = AM2301_TMAX;
		}
		if (am2301_data.Temperature < AM2301_TMIN) {
			am2301_data.Temperature = AM2301_TMIN;
		}
		if (am2301_data.Humidity > AM2301_HMAX) {
			am2301_data.Humidity = AM2301_HMAX;
		}
	}
	   	 
	/* Check timeout. */
	if (!CntTimeout) {
		return NULL;
	}
	CntTimeout = 0;
	__NOP();

	return &am2301_data;
}

/* Hardware dependent functions. */

/*
	@brief Software receiving data pin init.
*/
static void AM2301_RxStart(void) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*Configure GPIO pin. */
	GPIO_InitStruct.Pin = GPIO_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_DATA_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_DATA_PORT, GPIO_DATA_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIO_DATA_PORT, GPIO_DATA_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_DATA_PORT, &GPIO_InitStruct);
}


#endif /* HARDWARE_SENSOR_READ */

#ifdef HARDWARE_SENSOR_READ

/*
	@brief
*/
uint8_t AM2301_InitPerif(void) {

	MX_TIM10_Init();
	HAL_TIM_RegisterCallback(&htim10, HAL_TIM_IC_CAPTURE_CB_ID, TIM10_InputCaptureCallbackImpl);
}

/*
	@brief Hardware reading temperature and humidity from the sensor with the timer capture input.
*/
AM2301_ConvertedData_t* AM2301_GetData(void) {

	uint8_t bit_cnt = 0, tmp_data = 0;
	uint8_t* pData = (uint8_t*)& am2301_data.data[3];
	
	AM2301_RxStart();
	/* Wait data receiving. */
	if (xSemaphoreTake(AM2301DataReady, 100) == pdPASS) {
		if (RxData.RAWData[1] < AM2301_RESPONSE_TIME_MIN && RxData.RAWData[1] > AM2301_RESPONSE_TIME_MAX) {
			return NULL;
		} 		

		/* Decoding the data. */
		for (uint8_t i = 2; i < 42; i++) {
			/* Create the data byte. The receiving is MSB first. */
			if (RxData.RAWData[i] > AM2301_ONE_BIT_MIN && RxData.RAWData[i] < AM2301_ONE_BIT_MAX) {
				tmp_data |= 0x01;
			}
			bit_cnt++;
			if (bit_cnt == 8) {
				*pData = tmp_data;
				pData--;
				bit_cnt = 0;
				tmp_data = 0;
			}
			tmp_data <<= 1;
		}

		/* Check parity. */
		tmp_data = am2301_data.data[0] + am2301_data.data[1] + am2301_data.data[2] + am2301_data.data[3];
		if (am2301_data.Parity != tmp_data) {
			am2301_data.Temperature = AM2301_DATA_CHECK_ERROR;
		}

		#ifdef BELOW_ZERO_TEST
		am2301_data.data[0] = 0x65;
		am2301_data.data[1] = 0x80;
		#endif /* BELOW_ZERO */

		/* Check is temperature below zero? */
		if (am2301_data.Temperature != AM2301_DATA_CHECK_ERROR) {
			if (am2301_data.data[1] & 0x80) {
				am2301_data.Temperature = -(((uint16_t)am2301_data.data[0] | (uint16_t)am2301_data.data[1] << 8) & 0x7FFF);
			}
			/* Check the parameters for MAX and MIN values. */
			if (am2301_data.Temperature > AM2301_TMAX) {
				am2301_data.Temperature = AM2301_TMAX;
			}
			if (am2301_data.Temperature < AM2301_TMIN) {
				am2301_data.Temperature = AM2301_TMIN;
			}
			if (am2301_data.Humidity > AM2301_HMAX) {
				am2301_data.Humidity = AM2301_HMAX;
			}
		}
		__NOP();

	}
	return &am2301_data;
}


/* Hardware dependent functions. */

/*
	@brief Hardware receiving data pin init.
*/
static void AM2301_RxStart(void) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*Configure GPIO pin. */
	GPIO_InitStruct.Pin = GPIO_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_DATA_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_DATA_PORT, GPIO_DATA_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIO_DATA_PORT, GPIO_DATA_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
	HAL_GPIO_Init(GPIO_DATA_PORT, &GPIO_InitStruct);

	__HAL_TIM_SetCounter(&htim10, 0);
	TIM_CCxChannelCmd(TIM10, TIM_CHANNEL_1, TIM_CCx_ENABLE);
}

/* Callbacks. */

/*
	@brief Timer callback implementation.
*/
static void TIM10_InputCaptureCallbackImpl(TIM_HandleTypeDef* htim) {

	/* Read the all data bytes and then stop (every falling age read for one byte). */
	RxData.RAWData[RxData.CntRxData++] = TIM10->CCR1;
	__HAL_TIM_SetCounter(&htim10, 0);
	/* If reading is finished. */
	if (RxData.CntRxData == AM2301_RX_DATA + AM2301_RX_HARD_PRESENCE) {

		TIM_CCxChannelCmd(TIM10, TIM_CHANNEL_1, TIM_CCx_DISABLE);
		RxData.CntRxData = 0;
		xSemaphoreGiveFromISR(AM2301DataReady, NULL);
	}
}

/* Periferal initialization functions. */

/* TIM10 init function */
static void MX_TIM10_Init(void) {

	TIM_IC_InitTypeDef sConfigIC = { 0 };
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_TIM10_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 167;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 65535;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim10);
	HAL_TIM_IC_Init(&htim10);

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1);

	/* TIM10 interrupt Init */
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
	TIM_CCxChannelCmd(TIM10, TIM_CHANNEL_1, TIM_CCx_DISABLE);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void) {

	HAL_TIM_IRQHandler(&htim10);
}

#endif /* HARDWARE_SENSOR_READ */

