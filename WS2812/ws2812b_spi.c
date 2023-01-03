/**In developing
 **\file ws2812B_SPI.c
 **\brief Module for WS2812B leds, need clock 7MHz (6.4 with correction constant), 8 bit mode.
 **
***/

#include "ws2812b_spi.h"
#include "stm32f1xx_hal.h"
#include "HSVtoRGB.h"
/// Enum.
static enum error_led_param
{
	InputParam_Ok,
	NumbersOfLed_Err,
	ColorParam_Err
		
} error_led_param_enum;
/// Enum colors.
static enum colors
{
	green = 1,
	red =	2,
	blue =	3
		
} colors_enum;



/**\brief Private function prototype
 ***/
static void updateLedColorsSPI(uint8_t green, uint8_t red, uint8_t blue, const uint8_t position);
static enum error_led_param updateAllLedsSPI(uint8_t green, uint8_t red, uint8_t blue);
static enum error_led_param updatePositionLedSPI(uint8_t green, uint8_t red, uint8_t blue, uint8_t position);
/**\brief Instance of PWM_BUFFER_Typedef
 ***/
static DATA_BUFFER_SPI_Typedef dataLedBufferPWM;      


/**\brief Init GPIO, Timer, DMA
 **
 ** 
***/
void initHardwareSPI(SPI_TypeDef *spi, DMA_Channel_TypeDef *dma_channel, GPIO_TypeDef *gpio) 
{
	UNUSED(spi);
	UNUSED(dma_channel);
	UNUSED(gpio);	
	
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;	
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0;	
	DMA1_Channel3->CNDTR = (uint16_t)sizeof(dataLedBufferPWM);
	DMA1_Channel3->CPAR = (uint32_t)(&SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t)((uint8_t *)&dataLedBufferPWM);		
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN  | RCC_APB2ENR_AFIOEN;
	GPIOA->CRL	&= ~GPIO_CRL_CNF7;                                
	GPIOA->CRL	|= GPIO_CRL_CNF7_1;                               
	GPIOA->CRL  |= GPIO_CRL_MODE7_0;
	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |= SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
	SPI1->CR2 |= SPI_CR2_TXDMAEN;	
	
	for (volatile uint16_t i = 0; i < sizeof(dataLedBufferPWM.bitBuffer); i++) dataLedBufferPWM.bitBuffer[i] = SPI_0LEVEL;
	
	DMA1_Channel3->CCR |= DMA_CCR_EN;
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
}

/**\brief Function smootly chenges color.
 **
 ***/
void testCircleSPI(uint8_t  color)
{
	static int8_t position = 0;
	static uint8_t 	flagRedUpDown = 1, colorG = 0, colorR = 0, colorB = 0;		
	
		switch(color)
	{
	case green:	colorG = 128; colorR = 0; colorB = 0;
		break;
	case red: colorG = 0; colorR = 128; colorB = 0;
		break;
	case blue: colorG = 0; colorR = 0; colorB = 128;		
		break;			
	default:
		break;
	}
		
	if (position == 16) { updatePositionLedSPI(colorG, colorR, colorB, 0); }
	else updatePositionLedSPI(colorG, colorR, colorB, position);
		
	if (position == 0) updatePositionLedSPI(0, 0, 0, 15);
	else { 
		updatePositionLedSPI(0, 0, 0, position - 1); 
		if (position == 16) position = 0; 
	}
	
	position++;
}

///Color circle, one diode.
void testColorCircleSPI(uint8_t color)
{
	UNUSED(color);
	
	RGB_DATA_Typedef localRGB;
	static int8_t position = 0;	
	static uint16_t colorH = 0;
	static float sat = 0;
	
	localRGB = HSVtoRGB(colorH, 0.95f, 1.0f);
	colorH += 1;
	if (colorH > 360) colorH = 0;		

	if(position == 16) { updatePositionLedSPI(localRGB.green, localRGB.red, localRGB.blue, 0); }
	else updatePositionLedSPI(localRGB.green, localRGB.red, localRGB.blue, position);
	
	if (position == 0) { updatePositionLedSPI(0, 0, 0, 15); }
	else { 
		updatePositionLedSPI(0, 0, 0, position - 1); 
		if (position == 16) position = 0; 
	}
	position++;
}

///Fill all diode.
void testFillAllSPI()
{
		
	RGB_DATA_Typedef localRGB;
	static int8_t position = 0;	 
	static uint16_t colorH = 0;
	static float sat = 0;
	
	localRGB = HSVtoRGB(colorH, 0.9f, 0.5f);
	colorH += 1;
	if (colorH > 360) colorH = 0;		

	updateAllLedsSPI(localRGB.green, localRGB.red, localRGB.blue);

}

/**\brief Function position led.
 **
 ***/
static enum error_led_param updatePositionLedSPI(uint8_t green, uint8_t red, uint8_t blue, uint8_t position)
{ 
	if(position > LED_NUMBERS_OF_STRIP_SPI) return NumbersOfLed_Err ;
	if(green > 255 || red > 255 || blue > 255) return ColorParam_Err ;
	
	updateLedColorsSPI(green, red, blue, position);
	
	return InputParam_Ok ;
}

/**\brief Function fill whole buffer, before shold be enable CIRC DMA.
 **
 ***/
static enum error_led_param updateAllLedsSPI(uint8_t green, uint8_t red, uint8_t blue)
{ 	
	if(green > 255 || red > 255 || blue > 255) return ColorParam_Err ;
	
	for(volatile uint16_t i = 0 ; i < LED_NUMBERS_OF_STRIP_SPI ; i++)
	updateLedColorsSPI(green, red, blue, i) ;
	
	return InputParam_Ok ;
}

/**\brief Function "updateLedColors" send data into led massive (strip, pcb led and other).
 **
 ***/
static void updateLedColorsSPI(uint8_t green, uint8_t red, uint8_t blue, const uint8_t position)
{
	uint8_t i ;
	uint8_t prepColors ;	
	
	prepColors = green ;	
	for(i = 0 ; i < 8 ; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + position*COLORS_DATA_BITS_SPI] = SPI_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + position*COLORS_DATA_BITS_SPI] = SPI_0LEVEL;
		prepColors <<= 1 ;
	}

	prepColors = red ;
	for(i = 0 ; i < 8 ; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + 8 + position*COLORS_DATA_BITS_SPI] = SPI_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + 8 + position*COLORS_DATA_BITS_SPI] = SPI_0LEVEL;
		prepColors <<= 1 ;
	}

	prepColors = blue ;
	for(i = 0 ; i < 8 ; i++)
	{
		if ((prepColors & 0x80) != 0) dataLedBufferPWM.bitBuffer[i + 16 + position*COLORS_DATA_BITS_SPI] = SPI_1LEVEL;
		else dataLedBufferPWM.bitBuffer[i + 16 + position*COLORS_DATA_BITS_SPI] = SPI_0LEVEL;
		prepColors <<= 1 ;
	}
		
	//dataLedBufferPWM.loadSecondFrame = 0;
}
