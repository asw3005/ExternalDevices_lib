/**\brief Heder files.
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
//#define SYSCLOCK_72
//#define SYSCLOCK_64
#define SYSCLOCK_56
#define THREE_COLORS_PWM				///<THREE_COLORS  - strip colors, if not def, the colors set to four colors led.
#define LED_NUMBERS_OF_STRIP_PWM 8		///<LED_NUMBERS_OF_STRIP - led numbers in the led strip. 
#define COLORS_DATA_BITS_PWM 24			///<BUFFER_SIZE_STRIP - compute the buffer size.    
/// \cond
#ifdef THREE_COLORS_PWM
#define COLOR_NUMBERS_OF_LED_PWM 3	///<Numbers of led in strip.
    //#define DATA_BYTES_QUANTITY 3 
#else  
#define COLOR_NUMBERS_OF_LED_PWM 4
    //#define DATA_BYTES_QUANTITY 4
#endif
/// \endcond
#define BUFFER_SIZE_STRIP_PWM (LED_NUMBERS_OF_STRIP_PWM * COLOR_NUMBERS_OF_LED_PWM * 8)///<Size for data buffer.  

///Define parameters for timer and timer's PWM channel
#define TIM_PRESCALER       1   ///<Prescaler for input clock, output clock 36MHz, period 27,7nS. Output clock 32MHz, period 31.25nS.
#ifdef SYSCLOCK_72
#define TIM_PWM_PERIOD      44  ///<PWM period constant, PWM frequency is 800kHz, step PWM is 27,7nS (determined timer's input clock).      
#define TIM_0LEVEL          12  ///<Low level constant, PWM signal set to 0.35uS HL + 0.9uS LL, tolerance +-150nS.   
#define TIM_1LEVEL          32  ///<High level constant, PWM signal set to 0.9uS HL + 0.35uS LL, tolerance +-150nS.
#else 
#ifdef SYSCLOCK_64
#define TIM_PWM_PERIOD      39  ///<PWM period constant, PWM frequency is 800kHz, step PWM is 31,25nS (determined timer's input clock).      
#define TIM_0LEVEL          11  ///<Low level constant, PWM signal set to 0.35uS HL + 0.9uS LL, tolerance +-150nS.   
#define TIM_1LEVEL          29  ///<High level constant, PWM signal set to 0.9uS HL + 0.35uS LL, tolerance +-150nS.
#else
#ifdef SYSCLOCK_56
#define TIM_PWM_PERIOD      34  ///<PWM period constant, PWM frequency is 800kHz, step PWM is 35.71nS (determined timer's input clock).      
#define TIM_0LEVEL          10  ///<Low level constant, PWM signal set to 0.35uS HL + 0.9uS LL, tolerance +-150nS.   
#define TIM_1LEVEL          25  ///<High level constant, PWM signal set to 0.9uS HL + 0.35uS LL, tolerance +-150nS.
#endif
#endif
#endif
#define LOAD_FRAME_TIME_PWM 235   ///<Low level frame constant, it's no less 50uS PWM signal shold be 0.

/// Enum.
enum error_led_param
{
	InputParam_Ok,
	NumbersOfLed_Err,
	ColorParam_Err
		
} error_led_param_enum;

/// Enum colors.
enum colors
{
	green = 1,
	red =	2,
	blue =	3
		
} colors_enum;


///Data typedef.

///In developing
typedef struct 
{
	uint32_t _RCC_AHBENR_PERIPH_ENABLE;
	uint32_t _RCC_APB1ENR_PERIPH_ENABLE;
	uint32_t _GPIO_PIN;
	uint32_t _GPIO_CONFIG;
	GPIO_TypeDef *_GPIO;
	TIM_TypeDef  *_TIMER;
	DMA_Channel_TypeDef *_DMA;	


} CONFIGURATOR_PWM_Typedef;

///Buffer typedef struct.
///Why 200 instead of 40 - my leds don't work, if  reset code less then 250uS, buf in datasheet this value is >=50uS
///Set to 235, about 300uS, 
typedef struct 
{
	uint8_t loadFirstFrame[LOAD_FRAME_TIME_PWM];  			///<Start reset frame, length is 40 * 1.25uS = 50uS (operation frequency's period)
	uint8_t bitBuffer[BUFFER_SIZE_STRIP_PWM]; 	///<It's data buffer, GRB format, 24 bytes to 3 bytes of color. 
	//uint8_t loadSecondFrame; 				///<Reset frame at end of transmission, need if disable and enable dma.

} DATA_BUFFER_Typedef;

/**\brief Public function prototype.
 **
 ***/
void initHardwarePWM(TIM_TypeDef *timer, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio);
void testCirclePWM(uint8_t  color);
void testColorCirclePWM(uint8_t color);///Color circle, one diode.
void testFillAllPWM();///Fill all diode.
