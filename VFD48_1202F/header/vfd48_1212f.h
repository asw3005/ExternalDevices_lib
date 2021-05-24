/*
 *	@brief VFD48_1202F driver.
 *	Created 04.24.21
 *	VFD48_1202F_H_
 *	
 **/

#ifndef VFD48_1202F_H_				  
#define VFD48_1202F_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief Levels of brightness og the display.
 *
 **/
typedef enum
{
	VFD48_LOWEST_BRIGHTNESS,
	VFD48_BRIGHTNESS1,
	VFD48_BRIGHTNESS2,
	VFD48_BRIGHTNESS3,
	VFD48_BRIGHTNESS4,
	VFD48_BRIGHTNESS5,
	VFD48_BRIGHTNESS6,
	VFD48_HIGHEST_BRIGHTNESS
	
} VFD48_Brightness;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 * @brief Display data on yhe screen.
 *
 **/
typedef void(*vfd48_display_data_fptr)(void);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*vfd48_tx_data_fptr)(uint8_t segment, uint8_t dig, uint8_t state);

/*
 *	@brief Rx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*vfd48_rx_data_fptr)(uint8_t segment, uint8_t dig);




/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointers for the rx, tx and delay functions.
	delay_fptr delay_fptr;
	vfd48_display_data_fptr display_data_fptr;
	vfd48_tx_data_fptr tx_data_fptr;
	vfd48_rx_data_fptr rx_data_fptr;
	
} VFD48_GInst_t;


/* Public function prototypes. */
void VFD48_Init(VFD48_GInst_t *device);
void VFD48_TestScreen(VFD48_GInst_t *device);
void VFD48_SetTime(VFD48_GInst_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds);

/* Second layer functions. */
uint8_t VFD48_RxData(uint8_t segment, uint8_t dig);
uint8_t VFD48_TxData(uint8_t segment, uint8_t dig, uint8_t state);
void VFD48_DisplayData(void);

/* Hardware dependent functions. */
void SPI_Latch(uint8_t state);
int8_t SPI_TxData(uint8_t *buffer, uint16_t size);
int8_t SPI_RxData(uint8_t *buffer, uint16_t size);

#endif /* VFD48_1202F_H_ */