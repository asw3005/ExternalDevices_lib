/**\In developing
 **brief Heder files.
 **
 ***/
#pragma once

/**\brief Includes.
 **
 ***/
#include "stm32f1xx_hal.h"

/**\brief Definitions.
 **
 ***/

///Define parameters of led strip and buffer.
#define THREE_COLORS_SPI				///<THREE_COLORS  - strip colors, if not def, the colors set to four colors led.
#define LED_NUMBERS_OF_STRIP_SPI 16		///<LED_NUMBERS_OF_STRIP - led numbers in the led strip. 
#define COLORS_DATA_BITS_SPI 24			///<BUFFER_SIZE_STRIP - compute the buffer size.    
/// \cond
#ifdef THREE_COLORS_SPI
#define COLOR_NUMBERS_OF_LED_SPI 3	///<Numbers of led in strip.
    //#define DATA_BYTES_QUANTITY 3 
#else  
#define COLOR_NUMBERS_OF_LED 4
    //#define DATA_BYTES_QUANTITY_SPI 4
#endif
/// \endcond
#define BUFFER_SIZE_STRIP_SPI (LED_NUMBERS_OF_STRIP_SPI * COLOR_NUMBERS_OF_LED_SPI * 8)		///<Size for data buffer.  

///Define parameters for spi, input clock 7MHz, period 142.8nS.
///<Frequency is 800kHz, step SPI is 152.8nS (determined timer's input clock).
#define SPI_0LEVEL				192 ///<Low level constant, signal set to 0.35uS HL + 0.9uS LL, tolerance +-150nS. 
#define SPI_1LEVEL				252	///<High level constant, signal set to 0.9uS HL + 0.35uS LL, tolerance +-150nS. 
#define LOAD_FRAME_TIME_SPI     270	///<Low level frame constant, it's no less 50uS PWM signal shold be 0.



///Data typedef.

///Buffer typedef struct.
///Why 200 instead of 40 - my leds don't work, if  reset code less then 250uS, buf in datasheet this value is >=50uS
///Set to 235, about 300uS, 
typedef struct 
{
	uint8_t loadFirstFrame[LOAD_FRAME_TIME_SPI];   			///<Start reset frame, length is 40 * 1.25uS = 50uS (operation frequency's period)
	uint8_t bitBuffer[BUFFER_SIZE_STRIP_SPI];  	///<It's data buffer, GRB format, 24 bytes to 3 bytes of color. 
	//uint8_t loadSecondFrame; 				///<Reset frame at end of transmission, need if disable and enable dma.

} DATA_BUFFER_SPI_Typedef;

/**\brief Public function prototype.
 **
 ***/
void initHardwareSPI(SPI_TypeDef *timer, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio);
void testCircleSPI(uint8_t  color);
void testColorCircleSPI(uint8_t color);///Color circle, one diode.
void testFillAllSPI();///Fill all diode.
