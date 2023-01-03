/*
 *	@brief IV11 driver.
 *	Created 01.01.23 by asw3005
 *	IV11_H_
 *	
 **/

#ifndef IV11_H_				  
#define IV11_H_

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
	IV11_LOWEST_BRIGHTNESS,
	IV11_BRIGHTNESS1,
	IV11_BRIGHTNESS2,
	IV11_BRIGHTNESS3,
	IV11_BRIGHTNESS4,
	IV11_BRIGHTNESS5,
	IV11_BRIGHTNESS6,
	IV11_HIGHEST_BRIGHTNESS
	
} IV11_Brightness;

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
typedef void(*IV11_display_data_fptr)(void);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*IV11_tx_data_fptr)(uint8_t segment, uint8_t dig, uint8_t state);

/*
 *	@brief Tx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*IV11_tx_7seg_data_fptr)(uint8_t dig, uint8_t data);

/*
 *	@brief Rx function typedef pointer. 
 *	
 *	@param 
 *	@param 
 *
 **/
typedef uint8_t(*IV11_rx_data_fptr)(uint8_t segment, uint8_t dig);




/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointers for the rx, tx and delay functions.
	delay_fptr delay_fptr;
	IV11_display_data_fptr display_data_fptr;
	IV11_tx_data_fptr tx_data_fptr;
	IV11_tx_7seg_data_fptr tx_7seg_data_fptr;
	IV11_rx_data_fptr rx_data_fptr;
	
} IV11_GInst_t;


/* Public function prototypes. */
void IV11_Init(IV11_GInst_t *device);
void IV11_TestScreen0(IV11_GInst_t *device);
void IV11_TestScreen1(IV11_GInst_t *device);
void IV11_SetTime(IV11_GInst_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds);
void IV11_DotState(IV11_GInst_t *device, uint8_t dot_number, uint8_t state);

/* Second layer functions. */
uint8_t IV11_RxData(uint8_t segment, uint8_t dig);
uint8_t IV11_TxData(uint8_t segment, uint8_t dig, uint8_t state);
uint8_t IV11_TxData7Seg(uint8_t dig, uint8_t data);
void IV11_DisplayData(void);

/* Hardware dependent functions. */
//void SPI_Latch(uint8_t state);
//int8_t SPI_TxData(uint8_t *buffer, uint16_t size);
//int8_t SPI_RxData(uint8_t *buffer, uint16_t size);


#endif /* IV11_H_ */