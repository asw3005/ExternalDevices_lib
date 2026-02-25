/*
 * @brief C for MC33CD1020.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "drv8955.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include <stdint.h>

#define HARD_SPI_NSS

 /* External variables. */
extern TIM_HandleTypeDef htim3;
TIM_HandleTypeDef* TIM3_INx_PWM = &htim3;

/* Private variables. */


/* Private function prototypes. */
static void DRV8955_PinCtrl(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);
static void DRV8955_PWMCtrl(TIM_HandleTypeDef* timer, uint32_t channel, uint8_t state);

/* General struct. */


/*
 * @brief 
 *
**/

/*
 * @brief Init control pins.
 *
**/
void DRV8955_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : input pin 4 */
    GPIO_InitStruct.Pin = IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);

    #ifndef ONE_OUT_CTRL
    /*Configure GPIO pin : input pin 2 */
    GPIO_InitStruct.Pin = IN2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);
    #endif
    #ifdef FOUR_OUT_CTRL
    /*Configure GPIO pin : input pin 1 and 3 */
    GPIO_InitStruct.Pin = IN1_PIN | IN3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);
    #endif

    /*Configure GPIO pin : COIL_FAULT__Pin */
    GPIO_InitStruct.Pin = nFAULT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : COIL_SLEEP__Pin */
    GPIO_InitStruct.Pin = nSLEEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(nSLEEP_GPIO_Port, &GPIO_InitStruct);

}

/*
 * @brief Sleep control. 
 *
 * @paaram state :  0 - sleep (DRV8955_SLEEP),
 *                  1 - wake-up (DRV8955_EN).
 *
 **/
void DRV8955_SleepCtrl(uint8_t state) {
	
	if (state > 0) {
		DRV8955_PinCtrl(nSLEEP_GPIO_Port, nSLEEP_PIN, GPIO_PIN_SET);
	}
	else {
		DRV8955_PinCtrl(nSLEEP_GPIO_Port, nSLEEP_PIN, GPIO_PIN_RESET);
	}	
}

#ifndef ONE_OUT_CTRL
/*
 * @brief No PWM control of input 2. 
 *
 * @paaram state :  0 - out low-side ON,
 *                  1 - out high-side on.
 *
 **/
void DRV8955_IN2Ctrl(uint8_t state) {
	
	if (state > 0) {
		DRV8955_PinCtrl(IN2_GPIO_Port, IN2_PIN, GPIO_PIN_SET);
	}
	else {
		DRV8955_PinCtrl(IN2_GPIO_Port, IN2_PIN, GPIO_PIN_RESET);
	}	
}
#endif

/*
 * @brief No PWM control of input 4. 
 *
 * @paaram state :  0 - out low-side ON,
 *                  1 - out high-side on.
 *
 **/
void DRV8955_IN4Ctrl(uint8_t state)  {
	
	if (state > 0) {
		DRV8955_PinCtrl(IN4_GPIO_Port, IN4_PIN, GPIO_PIN_SET);
	}
	else {
		DRV8955_PinCtrl(IN4_GPIO_Port, IN4_PIN, GPIO_PIN_RESET);
	}	
}

#ifdef FOUR_OUT_CTRL
/*
 * @brief No PWM control of input 1. 
 *
 * @paaram state :  0 - out low-side ON,
 *                  1 - out high-side on.
 *
 **/
void DRV8955_IN1Ctrl(uint8_t state) {
	
	if (state > 0) {
		DRV8955_PinCtrl(IN1_GPIO_Port, IN1_PIN, GPIO_PIN_SET);
	}
	else {
		DRV8955_PinCtrl(IN1_GPIO_Port, IN1_PIN, GPIO_PIN_RESET);
	}	
}

/*
 * @brief No PWM control of input 3. 
 *
 * @paaram state :  0 - out low-side ON,
 *                  1 - out high-side on.
 *
 **/
void DRV8955_IN3Ctrl(uint8_t state) {
	
	if (state > 0) {
		DRV8955_PinCtrl(IN3_GPIO_Port, IN3_PIN, GPIO_PIN_SET);
	}
	else {
		DRV8955_PinCtrl(IN3_GPIO_Port, IN3_PIN, GPIO_PIN_RESET);
	}	
}
#endif

/*
 * @brief PWM control of input 2 (). 
 *
 * @paaram state :  0 - stop PWM,
 *                  1 - start PWM.
 *
 **/
void DRV8955_IN2PWMCtrl(uint8_t state) {

	//HAL_TIM_PWM_Start(TIM3_INx_PWM, TIM_CHANNEL_2);
    DRV8955_PWMCtrl(TIM3_INx_PWM, TIM_CHANNEL_2, state);
}

 /* Hardware dependent functions. */

 /*
 * @brief No PWM control of input 0. 
 *
 **/
static void DRV8955_PinCtrl(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}


/*
 * @brief No PWM control of input 2. 
 *
 * @paaram state :  0 - stop PWM,
 *                  1 - start PWM.
 *
 **/
static void DRV8955_PWMCtrl(TIM_HandleTypeDef* timer, uint32_t channel, uint8_t state) {

    if(state > 0) {
	    HAL_TIM_PWM_Start(timer, channel);
    } else {
        HAL_TIM_PWM_Stop(timer, channel);
    }
}


/* Interrupt callbacks. */

/*
	@brief  EXTI line detection callbacks.
	@param  GPIO_Pin: Specifies the pins connected EXTI line
	@retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(GPIO_Pin == GPIO_PIN_2) {

		/* Disable external line ITs. */
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	   /* Notify the task. */
	//    xTaskNotifyFromISR( PassHandle_TH,
	// 		   	   	   	   BtnClickBit,
	// 					   eSetBits,
	// 					   &xHigherPriorityTaskWoken );
	}
}