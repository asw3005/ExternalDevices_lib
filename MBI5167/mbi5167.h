/*
 * @brief Common header for the led address indicator.
 * Created 09.03.21 by asw3005. 
 *
 **/

#ifndef MBI5167_H_
#define MBI5167_H_

#include "stm32f103xb.h"


#define ZERO_LEADING_DELETE


/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*espi_txrx_fptr)(GPIO_TypeDef* gpio, uint8_t *pData, uint8_t size);



/*
 * @brief Led display data struct.
 *
 **/
typedef struct {	
	uint8_t Data[2];	
} MBI5167_Data;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {
	
	MBI5167_Data TxData;
	delay_fptr delay;
	espi_txrx_fptr spi_tx;

} MBI5167_GInst_t;

/* Public function prototypes. */

void MBI5167_Clr_Display(MBI5167_GInst_t *device);
void MBI5167_Set_Address(MBI5167_GInst_t *device, uint8_t address);
void MBI5167_Set_Dot0(MBI5167_GInst_t *device, uint8_t state);
void MBI5167_SET_Dot1(MBI5167_GInst_t *device, uint8_t state);

void ESPI_ClockData(GPIO_TypeDef* gpio);
void ESPI_LatchDataEn(GPIO_TypeDef* gpio);
void ESPI_LatchDataDis(GPIO_TypeDef* gpio);
void ESPI_TxData(GPIO_TypeDef* gpio, uint8_t *pData, uint8_t Size);


#endif /* MBI5167_H_ */
