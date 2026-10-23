/*
 * @brief Common source for brt38.
 * Created 04.29.26 by asw3005. 
 *
 **/
#include "brt38.h"
#include "cmsis_gcc.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>

/* External variables. */
extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef* BRT38_SPI = &hspi1;

/* Private variables. */


/* Private function prototypes. */
static void BRT38Start(GPIO_TypeDef* gpio, uint16_t gpio_pin);
static void BRT38_Rx(uint8_t *pData, uint8_t size);

/*
 * @brief 
 *
**/

/*
 * @brief Read encoder data (non blocking IT mode).
 *
**/
void BRT38_ReadPosition(ENCData_t* enc_inst) {

    /* Two dummy reads and one real to update out data (this is hardware pause, above 50uS). ??? */
    BRT38_Rx((uint8_t *)&enc_inst->EncReadBack.Data, sizeof(enc_inst->EncReadBack.Data));
    BRT38_Rx((uint8_t *)&enc_inst->EncReadBack.Data, sizeof(enc_inst->EncReadBack.Data));
    BRT38_Rx((uint8_t *)&enc_inst->EncReadBack.Data, sizeof(enc_inst->EncReadBack.Data));

    enc_inst->Single = ((uint16_t)enc_inst->EncReadBack.SINGLE11_8 << 8) | (uint16_t)enc_inst->EncReadBack.SINGLE7_0;
    enc_inst->Turns = enc_inst->EncReadBack.TURNS;
    enc_inst->AbsPosition = enc_inst->Single + ( enc_inst->FactoryResolution * enc_inst->Turns );
    enc_inst->KeepAliveDelta = enc_inst->AbsPosition - enc_inst->PreviousPos;
    enc_inst->PreviousPos = enc_inst->AbsPosition;
    enc_inst->FullDistance = enc_inst->LimMax - enc_inst->LimMin;
}

/*
 * @brief Encoder data setter/getter.
 */
ENCInst_t* EncoderGetInst(void) {

	static ENCInst_t EncInst = {
		.EncInst0.FactoryTurns 		= 16,
		.EncInst0.FactoryResolution	= 4096,
		.EncInst0.LimMin			= 500,
		.EncInst0.LimMax 			= 65000,
		.EncInst0.HomePos			= 1,
		.EncInst0.KeepAliveDelta	= 10,
		.EncInst0.PreviousPos		= 0
	};
	return &EncInst;
}

 /* Hardware dependent functions. */

/*
 * @brief Set zero position.
 */
void BRT38_SetSero(void) {

	HAL_GPIO_WritePin(ZERO_GPIO_Port, ZERO_PIN, GPIO_PIN_SET);
	HAL_Delay(210);
	HAL_GPIO_WritePin(ZERO_GPIO_Port, ZERO_PIN, GPIO_PIN_RESET);
}

/*
 * @brief Set middle position.
 */
void BRT38_SetMiddle(void) {

	HAL_GPIO_WritePin(MID_GPIO_Port, MID_PIN, GPIO_PIN_SET);
	HAL_Delay(210);
	HAL_GPIO_WritePin(MID_GPIO_Port, MID_PIN, GPIO_PIN_RESET);
}

 /*
 * @brief CLK HIGH to LOW transition.
 * 
 * @param gpio      : START_PIN.
 * @param gpio_pin  : START_GPIO_Port.
 *
 **/
static void BRT38Start(GPIO_TypeDef* gpio, uint16_t gpio_pin) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    /* RST pin state. */
    gpio->BSRR = gpio_pin << 16;
    for(uint8_t i = 0; i < 30; i++) {
        __NOP();
    }
    /* SET pin high. */
    //gpio->BSRR = gpio_pin;

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
}


/*
 * @brief SPI Rx data.
 * 
 **/
static void BRT38_Rx(uint8_t *pData, uint8_t size) {
	
    BRT38Start(START_GPIO_Port, START_PIN);
	HAL_SPI_Receive_IT(BRT38_SPI, pData, size);
    HAL_Delay(1);

}


