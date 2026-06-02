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
 * @brief RAW Position data.
 *
**/
uint32_t BRT38_GetRawPosition(void) {

    static BRT38_Data_t Data;

    Data = BRT38_GetRawData();
    return (uint32_t)((Data.TURNS*4096) + (Data.SINGLE11_8 << 8 | Data.SINGLE7_0));
}

/*
 * @brief Read encoder data (blocking mode).
 *
**/
BRT38_Data_t BRT38_GetRawData(void) {

    static BRT38_Data_t Data;

    /* Two dummy reads and one real to updata out data. */
    BRT38_Rx((uint8_t *)&Data, sizeof(Data));
    BRT38_Rx((uint8_t *)&Data, sizeof(Data));
    BRT38_Rx((uint8_t *)&Data, sizeof(Data));
    return Data;
}

 /* Hardware dependent functions. */

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
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
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


