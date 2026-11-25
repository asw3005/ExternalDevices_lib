/*
 * @brief Common header for brt38.
 * Created 04.29.26 by asw3005. 
 *
 **/

#ifndef BRT38_H_
#define BRT38_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*brt38_delay_fptr)(uint32_t delay);

/*
 * @brief BRT38 typedef.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    struct {
        uint8_t Data_MSB_H;
        uint8_t Data_LSB_H;
        uint8_t Data_MSB_L;
    };

    /* 12-bit, 16-turns */
    struct {
        uint8_t SINGLE11_9  : 3;
        uint8_t TURNS       : 4;
    	uint8_t DUMMY0		: 1;

        uint8_t SINGLE8_1   : 8;

        uint8_t DEAD_TIME	: 7;
        uint8_t SINGLE0		: 1;

    };

 } BRT38_Data_t;

 /*
  * @brief Encoder interface parameters.
  */
typedef struct {

	uint8_t isInitialized;
	uint16_t SetZeroPin;
	uint16_t SetMiddlePin;
	uint16_t ClockStartPin;
	GPIO_TypeDef* SetZeroPort;
	GPIO_TypeDef* SetMiddlePort;
	GPIO_TypeDef* ClockStartPort;
	SPI_HandleTypeDef* Spi;

} BRT38_InterfaceParam_t;


 /*
  * @brief Dynamic parameters struct typedef.
  */
 typedef struct {

	/* Dynamic data. */
	uint16_t Single;
	uint16_t Turns;

 } ENC_Data_t;


/* Public function prototypes. */
void BRT38_SetSero(void);
void BRT38_SetMiddle(void);
ENC_Data_t* BRT38_UpdatePosition(void);
void BRT38_IO_Init(SPI_HandleTypeDef* spi,
					GPIO_TypeDef* set_zero_port, uint16_t set_zero_pin,
					GPIO_TypeDef* set_middle_port,	uint16_t set_middle_pin,
					GPIO_TypeDef* clock_start_port, uint16_t clock_start_pin);

void ENC_CalcFullDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* BRT38_H_ */








