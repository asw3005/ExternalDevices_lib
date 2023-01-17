/*
 *	@brief IV18 driver.
 *	Created 06.01.21 by asw3005
 *	IV18_H_
 *	
 **/

#ifndef IV18_H_				  
#define IV18_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/* Definations. */
//#define PT6315_DEBUG_MODE

/*
 * @brief Levels of brightness og the display.
 *
 **/
typedef enum
{
	IV18_LOWEST_BRIGHTNESS,
	IV18_BRIGHTNESS1,
	IV18_BRIGHTNESS2,
	IV18_BRIGHTNESS3,
	IV18_BRIGHTNESS4,
	IV18_BRIGHTNESS5,
	IV18_BRIGHTNESS6,
	IV18_HIGHEST_BRIGHTNESS
	
} IV18_Brightness;

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
typedef void(*iv18_display_data_fptr)(void);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*iv18_tx_data_fptr)(uint8_t segment, uint8_t dig, uint8_t state);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*iv18_tx_7seg_data_fptr)(uint8_t dig, uint8_t data);

/*
 *	@brief Rx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*iv18_rx_data_fptr)(uint8_t segment, uint8_t dig);




/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointers for the rx, tx and delay functions.
	delay_fptr delay_fptr;
	iv18_display_data_fptr display_data_fptr;
	iv18_tx_data_fptr tx_data_fptr;
	iv18_tx_7seg_data_fptr tx_7seg_data_fptr;
	iv18_rx_data_fptr rx_data_fptr;
	
} IV18_GInst_t;


/* Public function prototypes. */
void IV18_Init(IV18_GInst_t *device);
void IV18_TestScreen0(IV18_GInst_t *device);
void IV18_TestScreen1(IV18_GInst_t *device);
void IV18_SetTime(IV18_GInst_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds);
void IV18_DotState(IV18_GInst_t *device, uint8_t state);

/* Second layer functions. */
uint8_t IV18_RxData(uint8_t segment, uint8_t dig);
uint8_t IV18_TxData(uint8_t segment, uint8_t dig, uint8_t state);
uint8_t IV18_TxData7Seg(uint8_t dig, uint8_t data);
void IV18_DisplayData(void);

/* Hardware dependent functions. */
//void SPI_Latch(uint8_t state);
//int8_t SPI_TxData(uint8_t *buffer, uint16_t size);
//int8_t SPI_RxData(uint8_t *buffer, uint16_t size);


#endif /* IV18_1202F_H_ */