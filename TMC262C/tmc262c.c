/*
 * @brief Common C for TMC262C.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "tmc262c.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "spi.h"
#include <stdint.h>

//#define HARD_SPI_NSS

 /* External variables. */
extern SPI_HandleTypeDef hspi3;

/* Private variables. */
static SPI_HandleTypeDef* TMC262C_SpiInst = &hspi3;


/* Private function prototypes. */
static uint8_t TMC262C_TxCheck(void);
static uint8_t TMC262C_RxCheck(void);
static void TMC262C_Tx(uint8_t *pData, uint8_t size);
static void TMC262C_Rx(uint8_t *pData, uint8_t size);

/* General struct. */
static TMC262C_GInst_t tmc262c_inst = {

	.delay = HAL_Delay,
	.spi_rx = TMC262C_Rx,
	.spi_tx = TMC262C_Tx
};

/*
 * @brief 
 *
**/

/*
 * @brief Init control pins.
 *
**/
void TMC262C_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : input pin 4 */
    GPIO_InitStruct.Pin = TMC262C_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC262C_CS_GPIO_Port, &GPIO_InitStruct);

}



 /* Hardware dependent functions. */

#ifndef HARD_SPI_NSS
/*
 * @brief SPI chip select. You must disable hardware SPI_NSS to use this function.
 * 
 * @param gpio      : TMC262C_CS_GPIO_Port.
 * @param gpio_pin  : TMC262C_CS_Pin.
 * @param state     : GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void TMC262C_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}
#endif 

/*
 * @brief SPI Tx data.
 *
 **/
static void TMC262C_Tx(uint8_t *pData, uint8_t size) {

	#ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_RESET);
    #endif
	HAL_SPI_Transmit(TMC262C_SpiInst, pData, size, 10);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}

/*
 * @brief SPI Rx data.
 * 
 *
 **/
static void TMC262C_Rx(uint8_t *pData, uint8_t size) {
	
	#ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_RESET);
    #endif
	HAL_SPI_Receive(TMC262C_SpiInst, pData, size, 10);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}


 /*
 * @brief Transmit complete check.
 *
**/
static uint8_t TMC262C_TxCheck(void) {

	tmc262c_inst.delay(5);
	return 0;
}

 /*
 * @brief Receive complete check.
 *
**/
static uint8_t TMC262C_RxCheck(void) {

	tmc262c_inst.delay(5);
	return 0;
}