/*
 *	@brief IV11 driver.
 *	Created 01.01.23 by asw3005
 *
 **/

#include "stm32f1xx_hal.h"
#include "iv11.h"
#include "pt6315.h"

/* Extern variables. */
extern SPI_HandleTypeDef hspi2; 

/* Private variables. */
/* Dot symbol. */
static const uint8_t IV11_CLOCK_DOT[6][2] = 
{ 
	{ 8, 1 }, { 8, 2 }, { 8, 3 }, 
	{ 8, 4 }, { 8, 5 }, { 8, 6 } 
};

/* Number graph constants.  */
static const uint8_t IV11_CLOCK_NUMBERS_PACK[12] = {		
	// "0" "1"	 "2"
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x00, 0x40
};

/* Private function prototypes. */
static void SPI_Latch(uint8_t state);
static int8_t SPI_TxData(uint8_t *buffer, uint16_t size);
static int8_t SPI_RxData(uint8_t *buffer, uint16_t size);

/* Instance of PT6315_GInst_t data struct. */
static PT6315_GInst_t pt6315_ginst = { 
		
	.delay_fptr = HAL_Delay,
	.latch_control_fptr = SPI_Latch,
	.spi_tx_data_fptr = SPI_TxData,
	.spi_rx_data_fptr = SPI_RxData,
};

/* Private function prototypes. */


/* Public functions. */

/*
 * @brief Clear RAM into the external VFD driver and drow the clock face.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 *
 **/
void IV11_Init(IV11_GInst_t *device)
{
	PT6315_Init(&pt6315_ginst, PT6315_6DIG_22SEG, PT6315_DISPLAY_ON, PT6315_PULSE_WIDTH_14_16);
}

/*
 * @brief Clear display.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 *
 **/
void IV11_ClearDisplay(IV11_GInst_t *device)
{
	device->tx_7seg_data_fptr(8, 0);
	device->tx_7seg_data_fptr(7, 0);
	device->tx_7seg_data_fptr(5, 0);
	device->tx_7seg_data_fptr(4, 0);
	device->tx_7seg_data_fptr(2, 0);
	device->tx_7seg_data_fptr(1, 0);	
}

/*
 * @brief Drives the clock, its arrows.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 * @param hours : values from 0 to 23.
 * @param minutes : values from 0 to 59.
 * @param seconds : values from 0 to 59.
 *
 **/
void IV11_SetTime(IV11_GInst_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	/* Hour offset and previous state in the points of Clock. */
	uint8_t keys_code = 0,digit = 0;
	static uint8_t LedState = 0;

	
	/* Clear old data. */
	device->tx_7seg_data_fptr(8, 0);
	device->tx_7seg_data_fptr(7, 0);
	device->tx_7seg_data_fptr(5, 0);
	device->tx_7seg_data_fptr(4, 0);
	device->tx_7seg_data_fptr(2, 0);
	device->tx_7seg_data_fptr(1, 0);
	
	/* Hours. */
	digit = hours / 10;
	device->tx_7seg_data_fptr(1, IV11_CLOCK_NUMBERS_PACK[digit]);
	digit = hours % 10;
	device->tx_7seg_data_fptr(2, IV11_CLOCK_NUMBERS_PACK[digit]);
	
	/* Minutes. */
	digit = minutes / 10;
	device->tx_7seg_data_fptr(3, IV11_CLOCK_NUMBERS_PACK[digit]);
	digit = minutes % 10;
	device->tx_7seg_data_fptr(4, IV11_CLOCK_NUMBERS_PACK[digit]);
	
	/* Seconds. */
	digit = seconds / 10;
	device->tx_7seg_data_fptr(5, IV11_CLOCK_NUMBERS_PACK[digit]);
	digit = seconds % 10;
	device->tx_7seg_data_fptr(6, IV11_CLOCK_NUMBERS_PACK[digit]);

	/* Toggle the led 1 on the board every time when it called.*/
	if (LedState) {
		PT6315_LedEn(&pt6315_ginst, 1, 1);
		//IV11_DotState(device, 2, 1);
		LedState = 0;		
	}
	else {
		PT6315_LedEn(&pt6315_ginst, 1, 0);
		//IV11_DotState(device, 2, 0);
		LedState = 1;
	}
	
	/* Refresh display RAM. */
	device->display_data_fptr();
}

/*
 * @brief Set/reset dot on the screen.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 * @param dot_number : number of dot on the six digit screen, from 1 to 6.
 * @param state : if 1 the dot on the screen is lightning, if 0 does not.
 *
 **/
void IV11_DotState(IV11_GInst_t *device, uint8_t dot_number, uint8_t state)
{
	/* Dot setting/resetting. */
	device->tx_data_fptr(IV11_CLOCK_DOT[dot_number - 1][0], IV11_CLOCK_DOT[dot_number - 1][1], state);
	/* Refresh display RAM. */
	device->display_data_fptr();
}

/*
 * @brief Screen test function.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 *
 **/
void IV11_TestScreen0(IV11_GInst_t *device)
{
	
	/* Displays dot. */
	device->tx_data_fptr(IV11_CLOCK_DOT[1][0], IV11_CLOCK_DOT[1][1], 1);
	device->display_data_fptr();
	device->delay_fptr(250);
	
	/* Displays the clock numbers. */
	for (uint8_t i = 0; i < 12; i++) {
			
		
		device->display_data_fptr();
		device->delay_fptr(250);
	}
		
	/* Clear the clock numbers. */
	for (uint8_t i = 0; i < 60; i++) {
			
		
		device->display_data_fptr();
		device->delay_fptr(250);
	}
	
	/* Clear dot. */
	device->tx_data_fptr(IV11_CLOCK_DOT[1][0], IV11_CLOCK_DOT[1][1], 0);
	device->display_data_fptr();
	device->delay_fptr(250);	
}

/*
 * @brief Screen test function. Emulates the clock, fast mode.
 * 
 *
 **/
void IV11_TestScreen1(IV11_GInst_t *device)
{
	/*Clock Variables. */
	static uint8_t hour = 0, min = 0, sec = 0, keys_code = 0;
	
	/* Clock is running now. */
	IV11_SetTime(device, hour, min, sec);		
	sec++;
	if (hour > 23) hour = 0;		
	if (sec > 59)
	{
		sec = 0;
		min++;
	}
			
	if (min > 59) {
		min = 0; 
		hour++;			
	}
	
	/* Little delay for the clock. */
	device->delay_fptr(250);
	/* Read keys. */
	PT6315_ReadKeys(&pt6315_ginst);	
	keys_code = pt6315_ginst.keys.SG1_4KS1_4Keys.COL1_KEY1;
	keys_code |= pt6315_ginst.keys.SG1_4KS1_4Keys.COL1_KEY2;
	keys_code |= pt6315_ginst.keys.SG1_4KS1_4Keys.COL2_KEY1;
}

/*
 * @brief Brightness control.
 * 
 * @param device : instance of the IV11_GInst_t struct.
 * @param level : value of brightness from the IV11_Brightness enum.
 *
 **/
void IV11_SetBrightness(IV11_GInst_t *device, IV11_Brightness level)
{
	PT6315_SetBrightness(&pt6315_ginst, (uint8_t)level);	
}

/*
 * @brief Write data to the RAM of the display driver (pt6315).
 *
 **/
void IV11_DisplayData(void)
{
	PT6315_WriteRAM(&pt6315_ginst);
}

/*
 * @brief Transmit data to the buffer.
 * 
 * @param segment : number of segment of the display.
 * @param dig : number of greed of the display.
 * @param state : state of the segment, if 0 the segment is off, if 1 the segment is on.
 *
 **/
uint8_t IV11_TxData(uint8_t segment, uint8_t dig, uint8_t state)
{
	PT6315_SetResetRAMBuff(&pt6315_ginst, segment, dig, state);	
	return 0;
}

/*
 * @brief Transmit data to the buffer.
 * 
 * @param dig : number of greed of the display.
 * @param data : data byte that means eight segments of 7 seg indicators + one dot.
 *
 **/
uint8_t IV11_TxData7Seg(uint8_t dig, uint8_t data)
{
	PT6315_SetResetRAMBuff7Seg(&pt6315_ginst, dig, data);
	return 0;
}

/*
 * @brief Receive data from the buffer.
 * 
 * @param segment : number of segment of the display.
 * @param dig : number of greed of the display.
 * @return : state of the particular segment in the screen RAM.
 *
 **/
uint8_t IV11_RxData(uint8_t segment, uint8_t dig)
{
	return PT6315_ReadRAMBuff(&pt6315_ginst, segment, dig);	
}

/* Private functions. */


/* Hardware dependent functions. */

/*
 * @brief SPI transmit data.
 * 
 * @param state : Zero is pull down, more than zero is pull up.
 *
 **/
static void SPI_Latch(uint8_t state)
{
	if (!state) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}	
}

/*
 * @brief SPI transmit data.
 *
 **/
static int8_t SPI_TxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Transmit(&hspi2, buffer, size, 100);
	//HAL_SPI_Transmit_IT(&hspi2, buffer, size);
	//HAL_SPI_Transmit_DMA(&hspi2, buffer, size);

}

/*
 * @brief SPI receive data.
 *
 **/
static int8_t SPI_RxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Receive(&hspi2, buffer, size, 100);
	//return HAL_SPI_Receive_IT(&hspi2, buffer, size);
	//HAL_SPI_Receive_DMA(&hspi2, buffer, size);
}

