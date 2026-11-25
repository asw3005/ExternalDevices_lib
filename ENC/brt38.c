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
//static void BRT38Start(void);
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

	/* Two dummy reads and one real to update out data (encoder changes his output data only for third time). ??? */
    BRT38_Rx(&EncReadBack.Data_MSB_H, sizeof(EncReadBack));
    for (uint16_t i = 0; i < 500; i++);
    BRT38_Rx(&EncReadBack.Data_MSB_H, sizeof(EncReadBack));
    for (uint16_t i = 0; i < 500; i++);
    BRT38_Rx(&EncReadBack.Data_MSB_H, sizeof(EncReadBack));
    EncInst.Single = ((uint16_t)EncReadBack.SINGLE11_9 << 9) | ((uint16_t)EncReadBack.SINGLE8_1 << 1) | (uint16_t)EncReadBack.SINGLE0;
    EncInst.Turns = EncReadBack.TURNS;

    return &EncInst;
}

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
* @brief SPI Rx data.
*
**/
static void BRT38_Rx(uint8_t *pData, uint8_t size) {
	
	uint8_t dummy_tx[3] = { 0 };
   //HAL_SPI_Receive(ENC_IO.Spi, pData, size, 5);
	HAL_SPI_TransmitReceive(ENC_IO.Spi, dummy_tx, pData, size, 2);

}


