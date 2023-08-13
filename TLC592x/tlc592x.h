/*
 * @brief Common header for the TLC5926, TLC5927 led driver.
 * Created 03.14.22 by asw3005.
 *
 **/

#ifndef TLC592x_H_
#define TLC592x_H_


#include "stm32f4xx.h"


#define ZERO_LEADING_DELETE


 /*
  * @brief Data exchange function typedefs.
  *
  **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*espi_txrx_fptr)(GPIO_TypeDef* gpio, uint8_t* pData, uint8_t size);



/*
 * @brief Led display data struct.
 *
 **/
typedef struct {
	uint8_t Data[2];
} TLC592x_Data;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	TLC592x_Data TxData;
	delay_fptr delay;
	espi_txrx_fptr spi_tx;

} TLC592x_GInst_t;

/* Public function prototypes. */

void TLC592x_Clr_Display(TLC592x_GInst_t* device);
void TLC592x_Set_Address(TLC592x_GInst_t* device, uint8_t address);
void TLC592x_Set_Dot0(TLC592x_GInst_t* device, uint8_t state);
void TLC592x_Set_Dot1(TLC592x_GInst_t* device, uint8_t state);

void ESPI_ClockData(GPIO_TypeDef* gpio);
void ESPI_LatchDataEn(GPIO_TypeDef* gpio);
void ESPI_LatchDataDis(GPIO_TypeDef* gpio);
void ESPI_TxData(GPIO_TypeDef* gpio, uint8_t* pData, uint8_t Size);


#endif /* TLC592x_H_ */
