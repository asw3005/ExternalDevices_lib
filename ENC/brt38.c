/*
 * @brief Common source for brt38.
 * Created 04.29.26 by asw3005. 
 *
 **/
#include "brt38.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>


/* Private variables. */
static BRT38_InterfaceParam_t ENC_IO;
static ENC_Data_t EncInst = { 0 };

/* Private function prototypes. */
static void BRT38Start(void);
static void BRT38_Rx(uint8_t *pData, uint8_t size);

/*
 * brief Init IO
 */
void BRT38_IO_Init(SPI_HandleTypeDef* spi,
					GPIO_TypeDef* set_zero_port, uint16_t set_zero_pin,
					GPIO_TypeDef* set_middle_port,	uint16_t set_middle_pin,
					GPIO_TypeDef* clock_start_port, uint16_t clock_start_pin) {

	ENC_IO.Spi = spi;
	ENC_IO.SetZeroPin = set_zero_pin;
	ENC_IO.SetMiddlePin = set_middle_pin;
	ENC_IO.ClockStartPin = clock_start_pin;
	ENC_IO.SetZeroPort = set_zero_port;
	ENC_IO.SetMiddlePort = set_middle_port;
	ENC_IO.ClockStartPort = clock_start_port;
	ENC_IO.isInitialized = 1;
}

/*
 * @brief Read encoder data (non blocking IT mode).
 *
**/
ENC_Data_t* BRT38_UpdatePosition(void) {

	BRT38_Data_t EncReadBack;
    /* Two dummy reads and one real to update out data (this is hardware pause, above 50uS). ??? */
    BRT38_Rx((uint8_t *)&EncReadBack.Data, sizeof(EncReadBack.Data));
    BRT38_Rx((uint8_t *)&EncReadBack.Data, sizeof(EncReadBack.Data));
    BRT38_Rx((uint8_t *)&EncReadBack.Data, sizeof(EncReadBack.Data));

    EncInst.Single = ((uint16_t)EncReadBack.SINGLE11_8 << 8) | (uint16_t)EncReadBack.SINGLE7_0;
    EncInst.Turns = EncReadBack.TURNS;

    return &EncInst;
}


 /* Hardware dependent functions. */

/* Hardware dependent functions. */

/*
* @brief Set zero position.
*/
void BRT38_SetSero(void) {

	HAL_GPIO_WritePin(ENC_IO.SetZeroPort, ENC_IO.SetZeroPin, GPIO_PIN_SET);
	HAL_Delay(210);
	HAL_GPIO_WritePin(ENC_IO.SetZeroPort, ENC_IO.SetZeroPin, GPIO_PIN_RESET);
}

/*
* @brief Set middle position.
*/
void BRT38_SetMiddle(void) {

	HAL_GPIO_WritePin(ENC_IO.SetMiddlePort, ENC_IO.SetMiddlePin, GPIO_PIN_SET);
	HAL_Delay(210);
	HAL_GPIO_WritePin(ENC_IO.SetMiddlePort, ENC_IO.SetMiddlePin, GPIO_PIN_RESET);
}

/*
* @brief CLK HIGH to LOW transition.
*
* @param gpio      : START_PIN.
* @param gpio_pin  : START_GPIO_Port.
*
**/
static void BRT38Start(void) {

   GPIO_InitTypeDef GPIO_InitStruct = {0};

   GPIO_InitStruct.Pin = ENC_IO.ClockStartPin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   //GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
   HAL_GPIO_Init(ENC_IO.ClockStartPort, &GPIO_InitStruct);
	
   /* RST pin state. */
   ENC_IO.ClockStartPort->BSRR = ENC_IO.ClockStartPin << 16;
   for(uint8_t i = 0; i < 30; i++) {
       __NOP();
   }
   /* SET pin high. */
   //gpio->BSRR = gpio_pin;

   GPIO_InitStruct.Pin = ENC_IO.ClockStartPin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
   HAL_GPIO_Init(ENC_IO.ClockStartPort, &GPIO_InitStruct);
	
}


/*
* @brief SPI Rx data.
*
**/
static void BRT38_Rx(uint8_t *pData, uint8_t size) {
	
   BRT38Start();
   HAL_SPI_Receive(ENC_IO.Spi, pData, size, 5);
   HAL_Delay(1);

}


