/*
 *	@brief VFD48_1202F driver.
 *	Created 04.24.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "vfd48_1212f.h"
#include "pt6315.h"

/* Public variables. */
extern SPI_HandleTypeDef hspi2; 

/* Private function prototypes. */
static int8_t SPI_TxData(uint8_t *buffer, uint16_t size);
static int8_t SPI_RxData(uint8_t *buffer, uint16_t size);


/* Private variables. */

/* Dot symbol. */
static const uint8_t PT6315_DOT[2] = { 1, 1 };

/* Number graph constants.  */
static const uint8_t PT6315_NUMBERS[12][2] = {		
		//  1         2         3         4
		{ 2, 1 }, { 2, 2 },	{ 2, 3 }, { 2, 4 }, 
		//  5         6         7         8
		{ 2, 5 }, { 2, 6 },	{ 2, 7 }, { 2, 8 },
		//  9        10         11         12
		{ 2, 9 }, { 2, 10 }, { 2, 11 }, { 2, 12 }	
};

/* Clock hands_up data (S segments). First element starts from 12 O'clock. */
static const uint8_t PT6315_CLOCK_HANDS_UP[60][2] = {		
		// 60         1          2          3         4         5         6         7         8         9
		{ 8, 12 }, { 7, 12 }, { 4, 12 }, { 3, 1 }, { 9, 1 }, { 8, 1 }, { 7, 1 }, { 6, 1 }, { 5, 2 }, { 9, 2 },
		// 10         11         12         13        14        15        16        17        18        19
		{ 8, 2 },  { 9, 2 },  { 4, 2 },  { 3, 3 }, { 9, 3 }, { 8, 3 }, { 7, 3 }, { 6, 3 }, { 5, 4 }, { 9, 4 },	
		// 20         21         22         23        24        25        26        27        28        29
		{ 8, 4 },  { 7, 4 },  { 4, 4 },  { 3, 5 }, { 9, 5 }, { 8, 5 }, { 7, 5 }, { 6, 5 }, { 5, 6 }, { 9, 6 },
		// 30         31         32         33        34        35        36        37        38        39
		{ 8, 6 },  { 7, 6 },  { 4, 6 },  { 3, 7 }, { 9, 7 }, { 8, 7 }, { 7, 7 }, { 6, 7 }, { 5, 8 }, { 9, 8 },
		// 40         41         42         43        44        45        46        47        48        49
		{ 8, 8 },  { 7, 8 },  { 4, 8 },  { 3, 9 }, { 9, 9 }, { 8, 9 }, { 7, 9 }, { 6, 9 }, { 5, 10 }, { 9, 10 },
		// 50         51         52         53         54         55         56         57         58         59
		{ 8, 10 }, { 7, 10 }, { 4, 10 }, { 3, 11 }, { 9, 11 }, { 8, 11 }, { 7, 11 }, { 6, 11 }, { 5, 12 }, { 9, 12 },
};

/* Clock hands_down data (R segments). First element starts from 12 O'clock. */
static const uint8_t PT6315_CLOCK_HANDS_DOWN[60][2] = {		
		// 60         1          2          3         4         5         6         7         8         9
		{ 8, 12 }, { 7, 12 }, { 4, 12 }, { 3, 1 }, { 9, 1 }, { 8, 1 }, { 7, 1 }, { 6, 1 }, { 5, 2 }, { 9, 2 },
		// 10         11         12         13        14        15        16        17        18        19
		{ 8, 2 },  { 9, 2 },  { 4, 2 },  { 3, 3 }, { 9, 3 }, { 8, 3 }, { 7, 3 }, { 6, 3 }, { 5, 4 }, { 9, 4 },	
		// 20         21         22         23        24        25        26        27        28        29
		{ 8, 4 },  { 7, 4 },  { 4, 4 },  { 3, 5 }, { 9, 5 }, { 8, 5 }, { 7, 5 }, { 6, 5 }, { 5, 6 }, { 9, 6 },
		// 30         31         32         33        34        35        36        37        38        39
		{ 8, 6 },  { 7, 6 },  { 4, 6 },  { 3, 7 }, { 9, 7 }, { 8, 7 }, { 7, 7 }, { 6, 7 }, { 5, 8 }, { 9, 8 },
		// 40         41         42         43        44        45        46        47        48        49
		{ 8, 8 },  { 7, 8 },  { 4, 8 },  { 3, 9 }, { 9, 9 }, { 8, 9 }, { 7, 9 }, { 6, 9 }, { 5, 10 }, { 9, 10 },
		// 50         51         52         53         54         55         56         57         58         59
		{ 8, 10 }, { 7, 10 }, { 4, 10 }, { 3, 11 }, { 9, 11 }, { 8, 11 }, { 7, 11 }, { 6, 11 }, { 5, 12 }, { 9, 12 },
};

/* Instance of PT6315_GInst_t data struct. */
static PT6315_GInst_t pt6315_ginst = { 
		
	.delay_fptr = HAL_Delay,
	.spi_tx_data_fptr = SPI_TxData,
	.spi_rx_data_fptr = SPI_RxData,
};

/* Private function prototypes. */





/* Public functions. */




/* Private functions. */


/* Hardware dependent functions. */

/*
 * @brief SPI transmit data.
 *
 **/
static int8_t SPI_TxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Transmit(&hspi2, buffer, size, 10);
	//HAL_SPI_Transmit_IT(&hspi2, buffer, size);
	//HAL_SPI_Transmit_DMA(&hspi2, buffer, size);

}

/*
 * @brief SPI receive data.
 *
 **/
static int8_t SPI_RxData(uint8_t *buffer, uint16_t size)
{
	return HAL_SPI_Receive(&hspi2, buffer, size, 10);
	//HAL_SPI_Receive_IT(&hspi2, buffer, size);
	//HAL_SPI_Receive_DMA(&hspi2, buffer, size);
}
