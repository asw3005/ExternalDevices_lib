/*
 *	@brief VFD48_1202F driver.
 *	Created 04.24.21 by asw3005
 *
 **/

#include "stm32f1xx_hal.h"
#include "vfd48_1212f.h"
#include "pt6315.h"

/* Public variables. */
extern SPI_HandleTypeDef hspi2; 

/* Private variables. */
/* Dot symbol. */
static const uint8_t VFD48_CLOCK_DOT[2] = { 1, 1 };

/* Number graph constants.  */
static const uint8_t VFD48_CLOCK_NUMBERS[12][2] = {		
		//  1         2         3         4
		{ 2, 1 }, { 2, 2 },	{ 2, 3 }, { 2, 4 }, 
		//  5         6         7         8
		{ 2, 5 }, { 2, 6 },	{ 2, 7 }, { 2, 8 },
		//  9        10         11         12
		{ 2, 9 }, { 2, 10 }, { 2, 11 }, { 2, 12 }	
};

/* Clock hands_up data (S segments). First element starts from 12 + 1 O'clock. */
static const uint8_t VFD48_CLOCK_HANDS_UP[60][2] = {		
		//  60
		{ 8, 12 }, 
		//  1          2          3         4         5         6         7         8         9
		{ 7, 12 }, { 4, 12 }, { 3, 1 }, { 9, 1 }, { 8, 1 }, { 7, 1 }, { 6, 1 }, { 5, 2 }, { 9, 2 },
		// 10         11         12         13        14        15        16        17        18        19
		{ 8, 2 },  { 7, 2 },  { 4, 2 },  { 3, 3 }, { 9, 3 }, { 8, 3 }, { 7, 3 }, { 6, 3 }, { 5, 4 }, { 9, 4 },	
		// 20         21         22         23        24        25        26        27        28        29
		{ 8, 4 },  { 7, 4 },  { 4, 4 },  { 3, 5 }, { 9, 5 }, { 8, 5 }, { 7, 5 }, { 6, 5 }, { 5, 6 }, { 9, 6 },
		// 30         31         32         33        34        35        36        37        38        39
		{ 8, 6 },  { 7, 6 },  { 4, 6 },  { 3, 7 }, { 9, 7 }, { 8, 7 }, { 7, 7 }, { 6, 7 }, { 5, 8 }, { 9, 8 },
		// 40         41         42         43        44        45        46        47        48        49
		{ 8, 8 },  { 7, 8 },  { 4, 8 },  { 3, 9 }, { 9, 9 }, { 8, 9 }, { 7, 9 }, { 6, 9 }, { 5, 10 }, { 9, 10 },
		// 50         51         52         53         54         55         56         57         58         59
		{ 8, 10 }, { 7, 10 }, { 4, 10 }, { 3, 11 }, { 9, 11 }, { 8, 11 }, { 7, 11 }, { 6, 11 }, { 5, 12 }, { 9, 12 }

};

/* Clock hands_down data (R segments). First element starts from 12 + 1 O'clock. */
static const uint8_t VFD48_CLOCK_HANDS_DOWN[60][2] = {	
		//  60
		{ 15, 12 },
		//   1           2           3          4          5          6          7         8          9
		{ 14, 12 }, { 11, 12 }, { 10, 1 }, { 16, 1 }, { 15, 1 }, { 14, 1 }, { 13, 1 }, { 12, 2 }, { 16, 2 },
		//  10          11          12          13         14         15         16         17         18         19
		{ 15, 2 },  { 14, 2 },  { 11, 2 },  { 10, 3 }, { 16, 3 }, { 15, 3 }, { 14, 3 }, { 13, 3 }, { 12, 4 }, { 16, 4 },	
		//  20          21          22          23         24         25         26         27         28         29
		{ 15, 4 },  { 14, 4 },  { 11, 4 },  { 10, 5 }, { 16, 5 }, { 15, 5 }, { 14, 5 }, { 13, 5 }, { 12, 6 }, { 16, 6 },
		//  30          31          32          33         34         35         36         37         38         39
		{ 15, 6 },  { 14, 6 },  { 11, 6 },  { 10, 7 }, { 16, 7 }, { 15, 7 }, { 14, 7 }, { 13, 7 }, { 12, 8 }, { 16, 8 },
		//  40          41          42          43         44         45         46         47         48          49
		{ 15, 8 },  { 14, 8 },  { 11, 8 },  { 10, 9 }, { 16, 9 }, { 15, 9 }, { 14, 9 }, { 13, 9 }, { 12, 10 }, { 16, 10 },
		//  50          51          52          53          54          55          56          57          58          59
		{ 15, 10 }, { 14, 10 }, { 11, 10 }, { 10, 11 }, { 16, 11 }, { 15, 11 }, { 14, 11 }, { 13, 11 }, { 12, 12 }, { 16, 12 }

};

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
 * @brief Cleare RAM into the external VFD driver and drow the clock face.
 * 
 * @param device : instance of the VFD48_GInst_t struct.
 *
 **/
void VFD48_Init(VFD48_GInst_t *device)
{
	PT6315_Init(&pt6315_ginst);
	
	for (uint8_t i = 0; i < 12; i++) {			
		device->tx_data_fptr(VFD48_CLOCK_NUMBERS[i][0], VFD48_CLOCK_NUMBERS[i][1], 1);
		device->display_data_fptr();
	}
}



/*
 * @brief Drives the clock, its arrows.
 * 
 * @param device : instance of the VFD48_GInst_t struct.
 * @param hours : values from 0 to 23.
 * @param minutes : values from 0 to 59.
 * @param seconds : values from 0 to 59.
 *
 **/
void VFD48_SetTime(VFD48_GInst_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	/* Hour offset and previous state in the points of Clock. */
	uint8_t HourOffset = 0;
	static uint8_t HourState = 0, MinState = 0, SecState = 0, LedState = 0;
	
	uint8_t keys_code = 0;
	/* Hour arrow. */
	HourOffset = (minutes / 12);	
	device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[HourState][0], VFD48_CLOCK_HANDS_DOWN[HourState][1], 0);
	if (hours < 12) {	
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[(hours * 5) + HourOffset][0], VFD48_CLOCK_HANDS_DOWN[(hours * 5) + HourOffset][1], 1);
		HourState = (hours * 5) + HourOffset;
	}
	else {
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[((hours - 12) * 5) + HourOffset][0], VFD48_CLOCK_HANDS_DOWN[((hours - 12) * 5) + HourOffset][1], 1);
		HourState = ((hours - 12) * 5) + HourOffset;
	}
	
	/* Minute arrow. */
	if (MinState == HourState) {
		device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[MinState][0], VFD48_CLOCK_HANDS_UP[MinState][1], 0);
	}
	else {
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[MinState][0], VFD48_CLOCK_HANDS_DOWN[MinState][1], 0);
		device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[MinState][0], VFD48_CLOCK_HANDS_UP[MinState][1], 0);
	}

	device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[minutes][0], VFD48_CLOCK_HANDS_DOWN[minutes][1], 1);
	device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[minutes][0], VFD48_CLOCK_HANDS_UP[minutes][1], 1);
	MinState = minutes;
	
	/* Second arrow. */
	if (SecState == HourState) {
		if (SecState != MinState) {
			device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[SecState][0], VFD48_CLOCK_HANDS_UP[SecState][1], 0);
		}
	}
	else if (SecState != MinState) {
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[SecState][0], VFD48_CLOCK_HANDS_DOWN[SecState][1], 0);
		device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[SecState][0], VFD48_CLOCK_HANDS_UP[SecState][1], 0);
	}

	device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[seconds][0], VFD48_CLOCK_HANDS_DOWN[seconds][1], 1);
	device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[seconds][0], VFD48_CLOCK_HANDS_UP[seconds][1], 1);
	SecState = seconds;	

	/* Toggle the led 1 on the board every time when it called.*/
	if (LedState) {
		PT6315_LedEn(&pt6315_ginst, 1, 1);
		LedState = 0;		
	}
	else {
		PT6315_LedEn(&pt6315_ginst, 1, 0);
		LedState = 1;
	}
	
	/* Refresh display RAM. */
	device->display_data_fptr();
}

/*
 * @brief Set/reset dot on the screen.
 * 
 * @param device : instance of the VFD48_GInst_t struct.
 * @param state : if 1 the dot on the screen is lightning, if 0 does not.
 *
 **/
void VFD48_DotState(VFD48_GInst_t *device, uint8_t state)
{
	/* Dot setting/resetting. */
	device->tx_data_fptr(VFD48_CLOCK_DOT[0], VFD48_CLOCK_DOT[1], state);
	/* Refresh display RAM. */
	device->display_data_fptr();
}

/*
 * @brief Screen test function.
 * 
 * @param device : instance of the VFD48_GInst_t struct.
 *
 **/
void VFD48_TestScreen0(VFD48_GInst_t *device)
{
	
	/* Displays dot. */
	device->tx_data_fptr(VFD48_CLOCK_DOT[0], VFD48_CLOCK_DOT[1], 1);
	device->display_data_fptr();
	device->delay_fptr(250);
	
	/* Displays the clock numbers. */
	for (uint8_t i = 0; i < 12; i++) {
			
		device->tx_data_fptr(VFD48_CLOCK_NUMBERS[i][0], VFD48_CLOCK_NUMBERS[i][1], 1);
		device->display_data_fptr();
		device->delay_fptr(250);
	}
	
	/* Displays the clock arrows. */
	for (uint8_t i = 0; i < 60; i++) {
			
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[i][0], VFD48_CLOCK_HANDS_DOWN[i][1], 1);
		device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[i][0], VFD48_CLOCK_HANDS_UP[i][1], 1);
		device->display_data_fptr();
		device->delay_fptr(250);
	}
		
	/* Clear the clock numbers. */
	for (uint8_t i = 0; i < 60; i++) {
			
		device->tx_data_fptr(VFD48_CLOCK_HANDS_DOWN[i][0], VFD48_CLOCK_HANDS_DOWN[i][1], 0);
		device->tx_data_fptr(VFD48_CLOCK_HANDS_UP[i][0], VFD48_CLOCK_HANDS_UP[i][1], 0);
		device->display_data_fptr();
		device->delay_fptr(250);
	}
	
	/* Clear the clock arrows. */
	for (uint8_t i = 0; i < 12; i++) {
			
		device->tx_data_fptr(VFD48_CLOCK_NUMBERS[i][0], VFD48_CLOCK_NUMBERS[i][1], 0);
		device->display_data_fptr();
		device->delay_fptr(250);
	}
	
	/* Clear dot. */
	device->tx_data_fptr(VFD48_CLOCK_DOT[0], VFD48_CLOCK_DOT[1], 0);
	device->display_data_fptr();
	device->delay_fptr(250);	
}

/*
 * @brief Screen test function. Emulates the clock, fast mode.
 * 
 *
 **/
void VFD48_TestScreen1(VFD48_GInst_t *device)
{
	/*Clock Variables. */
	static uint8_t hour = 0, min = 0, sec = 0, keys_code = 0;
	
	/* Clock is running now. */
	VFD48_SetTime(device, hour, min, sec);		
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
 * @param device : instance of the VFD48_GInst_t struct.
 * @param level : value of brightness from the VFD48_Brightness enum.
 *
 **/
void VFD48_SetBrightness(VFD48_GInst_t *device, VFD48_Brightness level)
{
	PT6315_SetBrightness(&pt6315_ginst, (uint8_t)level);	
}

/*
 * @brief Write data to the RAM of the display driver (pt6315).
 *
 **/
void VFD48_DisplayData(void)
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
uint8_t VFD48_TxData(uint8_t segment, uint8_t dig, uint8_t state)
{
	PT6315_SetResetRAMBuff(&pt6315_ginst, segment, dig, state);	
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
uint8_t VFD48_RxData(uint8_t segment, uint8_t dig)
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
void SPI_Latch(uint8_t state)
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
int8_t SPI_TxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Transmit(&hspi2, buffer, size, 100);
	//HAL_SPI_Transmit_IT(&hspi2, buffer, size);
	//HAL_SPI_Transmit_DMA(&hspi2, buffer, size);

}

/*
 * @brief SPI receive data.
 *
 **/
int8_t SPI_RxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Receive(&hspi2, buffer, size, 100);
	//return HAL_SPI_Receive_IT(&hspi2, buffer, size);
	//HAL_SPI_Receive_DMA(&hspi2, buffer, size);
}
