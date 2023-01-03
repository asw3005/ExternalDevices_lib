/**
 **\file ws2812B_PWM.c
 **\brief Module for WS2812B leds
 **
***/

#include "ws2812b_pwm.h"
#include "stm32f1xx_hal.h"
#include "HSVtoRGB.h"
/**\brief Private function prototype
 ***/
static void updateLedColorsPWM(uint8_t green, uint8_t red, uint8_t blue,const uint8_t position);
static enum error_led_param updateAllLedsPWM(uint8_t green, uint8_t red, uint8_t blue);
static enum error_led_param updatePositionLedPWM(uint8_t green, uint8_t red, uint8_t blue, uint8_t position);
/**\brief Instance of PWM_BUFFER_Typedef
 ***/
static DATA_BUFFER_Typedef dataLedBufferPWM;      
                              
static CONFIGURATOR_PWM_Typedef conf[2] = 
{ 
	{ RCC_AHBENR_DMA1EN, RCC_APB1ENR_TIM3EN, GPIO_CRL_CNF6, (uint32_t)(GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0), GPIOA, TIM1, DMA1_Channel1 },
	{ RCC_AHBENR_DMA1EN, RCC_APB1ENR_TIM2EN, GPIO_CRL_CNF2, (uint32_t)(GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0), GPIOB, TIM2, DMA1_Channel1 },
};		

/**\brief Init GPIO, Timer, DMA
 ** 
 **
***/
//void initHardwarePWM(TIM_TypeDef *timer, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio) {
//
//	UNUSED(timer);
//	UNUSED(dma_channel);
//	UNUSED(gpio);		
//	
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;	
//	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
//	DMA1_Channel2->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | /*DMA_CCR_MSIZE_0 |*/ DMA_CCR_PSIZE_0 /*| DMA_CCR_PL*/;	
//	DMA1_Channel2->CNDTR = (uint16_t)sizeof(dataLedBufferPWM);
//	DMA1_Channel2->CPAR = (uint32_t)(&TIM1->CCR1);
//	DMA1_Channel2->CMAR = (uint32_t)((uint8_t *)&dataLedBufferPWM);		
//  
//	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
//	GPIOA->CRH	&= ~GPIO_CRH_CNF8;                                  
//	GPIOA->CRH	|= GPIO_CRH_CNF8_1;                               
//	GPIOA->CRH  |= GPIO_CRH_MODE8_0;      
//  	  
//	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//	TIM1->PSC = TIM_PRESCALER;     				                  
//	TIM1->ARR = TIM_PWM_PERIOD;    					  		          
//	TIM1->CCR1 = 10;    
//	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;                 
//    TIM1->CCER |= TIM_CCER_CC1E;  			          
//    TIM1->DIER |= TIM_DIER_CC1DE;
//	TIM1->BDTR |= TIM_BDTR_MOE;	
//	TIM1->CR1 |= TIM_CR1_CEN; 
//	
//	for (volatile uint16_t i = 0; i < sizeof(dataLedBufferPWM.bitBuffer); i++) dataLedBufferPWM.bitBuffer[i] = TIM_0LEVEL;
//	
//	DMA1_Channel2->CCR |= DMA_CCR_EN;
//}
//void initHardwarePWM(TIM_TypeDef *timer, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio) 
//{	
//	UNUSED(timer);
//	UNUSED(dma_channel);
//	UNUSED(gpio);	
//	
//	///Enable dma channel for use in timer, enable timer.	
//	///Enable transport complete interrupt, dma direction set to "read from memory",
//	///increment memory address, peripheral data size is 16 bit(timer CCR register),
//	///memory size is 8 bit.	
//	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//	DMA1_Channel6->CCR &= ~DMA_CCR_EN;																									
//	DMA1_Channel6->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | /*DMA_CCR_MSIZE_0 |*/ DMA_CCR_PSIZE_0 /*| DMA_CCR_PL*/;	
//	DMA1_Channel6->CNDTR = (uint16_t)sizeof(dataLedBufferPWM); 			///<Set number of data transfer.
//	DMA1_Channel6->CPAR = (uint32_t)(&TIM3->CCR1);					///<Set the address where data will be written
//	DMA1_Channel6->CMAR = (uint32_t)((uint8_t *)&dataLedBufferPWM); 	///<Set the address from where data will be read	
//  
//	///Config GPIO for timer channel, configuration output pin, reset CNF to 0, set AF-PP, out speed max 10MHz
//	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
//	GPIOA->CRL	&= ~GPIO_CRL_CNF6;                                   
//	GPIOA->CRL	|= GPIO_CRL_CNF6_1;                               
//	GPIOA->CRL  |= GPIO_CRL_MODE6_0;       
//  	 
//	///Set input clock prescaler to TIM_PRESCALER, set period PWM to TIM_PWM_PERIOD(look define in *.h),  
//	///set compare register first to 0, PWM mode 1, enable PWM channel X, enable DMA request.
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//	TIM3->PSC = TIM_PRESCALER;     				                  
//	TIM3->ARR = TIM_PWM_PERIOD;    					  		          
//	TIM3->CCR1 = 10;    
//	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;                
//    TIM3->CCER |= TIM_CCER_CC1E;
//	TIM3->CR1 |= TIM_CR1_URS /* | TIM_CR1_ARPE*/;    			          
//    TIM3->DIER |= TIM_DIER_CC1DE;
//	TIM3->CR1 |= TIM_CR1_CEN; 
//	
//	///Initialise data buffer
//	for(volatile uint16_t i = 0 ; i < sizeof(dataLedBufferPWM.bitBuffer) ; i++) dataLedBufferPWM.bitBuffer[i] = TIM_0LEVEL;
//	///Enable DMA
//	DMA1_Channel6->CCR |= DMA_CCR_EN;
//}

void initHardwarePWM(TIM_TypeDef *timer, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio) 
{	
	UNUSED(timer);
	UNUSED(dma_channel);
	UNUSED(gpio);	
	
	///Enable dma channel for use in timer, enable timer.	
	///Enable transport complete interrupt, dma direction set to "read from memory",
	///increment memory address, peripheral data size is 16 bit(timer CCR register),
	///memory size is 8 bit.	
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;																									
	DMA1_Channel5->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | /*DMA_CCR_MSIZE_0 |*/ DMA_CCR_PSIZE_0 /*| DMA_CCR_PL*/;	
	DMA1_Channel5->CNDTR = (uint16_t)sizeof(dataLedBufferPWM);  		///<Set number of data transfer.
	DMA1_Channel5->CPAR = (uint32_t)(&TIM2->CCR1); 						///<Set the address where data will be written
	DMA1_Channel5->CMAR = (uint32_t)((uint8_t *)&dataLedBufferPWM);  	///<Set the address from where data will be read	
  
	///Config GPIO for timer channel, configuration output pin, reset CNF to 0, set AF-PP, out speed max 10MHz
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	GPIOA->CRL	&= ~GPIO_CRL_CNF0;                                   
	GPIOA->CRL	|= GPIO_CRL_CNF0_1;                               
	GPIOA->CRL  |= GPIO_CRL_MODE0_0;       
  	 
	///Set input clock prescaler to TIM_PRESCALER, set period PWM to TIM_PWM_PERIOD(look define in *.h),  
	///set compare register first to 0, PWM mode 1, enable PWM channel X, enable DMA request.
	///Enable timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = TIM_PRESCALER;     				                  
	TIM2->ARR = TIM_PWM_PERIOD;    					  		          
	TIM2->CCR1 = 0;    
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;                
	TIM2->CCER |= TIM_CCER_CC1E;  			          
	TIM2->DIER |= TIM_DIER_CC1DE;	
	TIM2->CR1 |= TIM_CR1_CEN; 
	
	///Initialise data buffer
	for(volatile uint16_t i = 0 ; i < sizeof(dataLedBufferPWM.bitBuffer) ; i++) dataLedBufferPWM.bitBuffer[i] = TIM_0LEVEL;
	///Enable DMA
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

/**\brief Function smootly chenges color.
 **
 ***/
void testCirclePWM(uint8_t color)
{
	static int8_t position = 0;
	static uint8_t 	colorG = 0, colorR = 0, colorB = 0;

	switch (color)
	{
	case green:	colorG = 128; colorR = 0; colorB = 0;
		break;
	case red:	colorG = 0;	colorR = 128; colorB = 0;
		break;
	case blue:	colorG = 0;	colorR = 0;	colorB = 128;		
		break;			
	default:
		break;
	}
	
	if(position == 8) { updatePositionLedPWM(colorG, colorR, colorB, 0); }
	else updatePositionLedPWM(colorG, colorR, colorB, position);
	
	if (position == 0) updatePositionLedPWM(0, 0, 0, 7);
	else { 
		updatePositionLedPWM(0, 0, 0, position - 1); 
		if (position == 8) position = 0; 
	}
	
	position++;
}

///Color circle, one diode.
void testColorCirclePWM(uint8_t color)
{
	UNUSED(color);
	
	RGB_DATA_Typedef localRGB;
	static int8_t position = 0;	 
	static uint16_t colorH = 0;
	static float sat = 0;
	
	localRGB = HSVtoRGB(colorH, 0.95f, 1.0f);
	colorH += 1;
	if (colorH > 360) colorH = 0;		

	if(position == 8) { updatePositionLedPWM(localRGB.green, localRGB.red, localRGB.blue, 0); }
	else updatePositionLedPWM(localRGB.green, localRGB.red, localRGB.blue, position);
	
	if (position == 0) { updatePositionLedPWM(0, 0, 0, 7);}
	else { 
		updatePositionLedPWM(0, 0, 0, position - 1); 
		if (position == 8) position = 0; 
	}
	position++;

}

///Fill all diode.
void testFillAllPWM()
{
		
	RGB_DATA_Typedef localRGB;
	static int8_t position = 0;	 
	static uint16_t colorH = 0;
	static float sat = 0;
	
	localRGB = HSVtoRGB(colorH, 0.9f, 0.5f);
	colorH += 1;
	if (colorH > 360) colorH = 0;		

	updateAllLedsPWM(localRGB.green, localRGB.red, localRGB.blue);

}

/**\brief Function position led.
 **
 ***/
static enum error_led_param updatePositionLedPWM(uint8_t green, uint8_t red, uint8_t blue, uint8_t position)
{ 
	if (position > LED_NUMBERS_OF_STRIP_PWM) return NumbersOfLed_Err;
	if (green > 255 || red > 255 || blue > 255) return ColorParam_Err;
	
	updateLedColorsPWM(green, red, blue, position);
	
	return InputParam_Ok;
}

/**\brief Function fill whole buffer, before shold be enable CIRC DMA.
 **
 ***/
static enum error_led_param updateAllLedsPWM(uint8_t green, uint8_t red, uint8_t blue)
{ 	
	if (green > 255 || red > 255 || blue > 255) return ColorParam_Err;
	
	for (volatile uint16_t i = 0; i < LED_NUMBERS_OF_STRIP_PWM; i++)
		updateLedColorsPWM(green, red, blue, i);
	
	return InputParam_Ok;
}

/**\brief Function "updateLedColors" send data into led massive (strip, pcb led and other).
 **
 ***/
static void updateLedColorsPWM(uint8_t green, uint8_t red, uint8_t blue, const uint8_t position)
{
	uint8_t i;
	uint8_t prepColors;	
	
	prepColors = green;	
	for (i = 0; i < 8 ; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + position*COLORS_DATA_BITS_PWM] = TIM_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + position*COLORS_DATA_BITS_PWM] = TIM_0LEVEL;
		prepColors <<= 1;
	}

	prepColors = red;
	for (i = 0; i < 8; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + 8 + position*COLORS_DATA_BITS_PWM] = TIM_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + 8 + position*COLORS_DATA_BITS_PWM] = TIM_0LEVEL;
		prepColors <<= 1;
	}

	prepColors = blue;
	for (i = 0; i < 8; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + 16 + position*COLORS_DATA_BITS_PWM] = TIM_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + 16 + position*COLORS_DATA_BITS_PWM] = TIM_0LEVEL;
		prepColors <<= 1;
	}
		
	//dataLedBufferPWM.loadSecondFrame = 0;
}
