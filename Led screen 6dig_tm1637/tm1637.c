/*
 * @brief Driver for the TM1637.
 *	Created 08.16.22 by asw3005
 **/

#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#include "tm1637.h"

/* Private variables.*/
static TM1637_RamData_t ram_data;

/* Private function prototypes. */
static void TM1637_StartBus(void);
static void TM1637_StopBus(void);
static uint8_t TM1637_ReadACK(void);
static void TM1637_WriteData(uint8_t command, uint8_t *pData, uint8_t Size);
static void TM1637_ReadData(uint8_t command, uint8_t *pData, uint8_t Size);
static void TM1637_NOPDelay(void);

__INLINE static void TurnDownCLK(void);
__INLINE static void TurnUpCLK(void);
__INLINE static void TurnDownData(void);
__INLINE static void TurnUpData(void);

/*
 * @brief Write 7-seg display's code.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 * @param grid : Display's grid, may be set from 0 to 5.
 * @param segment : One of seven segments of 7-seg display. May be from 0 to 7.
 * @param state : If 0 the segment is power down, otherwise is power up.
 *
 **/
void TM1637_WriteSeg(uint8_t grid, uint8_t segment, uint8_t state) {
	
	if (grid > 5 || segment > 7) return;
	if (state) {		
		ram_data.Gridx[grid].SegData |= 1 << segment;
	} else {
		ram_data.Gridx[grid].SegData &= ~(1 << segment);
	}	
}

/*
 * @brief Reads scan code of keys.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 *
 **/
TM1637_KeysData_t TM1637_ReadKeys(TM1637_GInst_t* device) {

	TM1637_KeysData_t scan_code;

	/* Prepare reading scan code of keys. (6 bytes). */
	ram_data.DataCmd.DATA_WR_MODE = TM1637_READ_KEY_SCAN_DATA;
	ram_data.DataCmd.ADDR_ADDING_MODE = TM1637_AUTOMATIC_ADDR_ADDING;
	ram_data.DataCmd.TEST_MODE_SETTING = TM1637_NORMAL_MODE;
	ram_data.DataCmd.RESERVED5_4 = 0;
	ram_data.DataCmd.DATA_CMD_WORD = TM1637_DATA_CMD;

	device->rx_data(ram_data.DataCmd.DataCommand, &scan_code, 1);
	return scan_code;
}

/*
 * @brief Power up or power down display, configurate brightness of display.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 *
 **/
void TM1637_DisplayCtrl(TM1637_GInst_t* device, uint8_t state, uint8_t pulse_width) {
	
	TM1637_DisplayCmd_t display_cmd;
	
	/* Prepare display control command byte. */
	display_cmd.PULSE_WIDTH = pulse_width;
	display_cmd.DISPLAY_ON_OFF = state;
	display_cmd.RESERVED5_4 = 0;
	display_cmd.DISPLAY_CMD_WORD = TM1637_DISPLAY_CTRL_CMD;
	
	device->tx_data(display_cmd.DisplayCommand, NULL, 0);	
}

/* Private functions. */

/*
 * @brief Writes data to display RAM buffer.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 *
 **/
void TM1637_WriteRAM(TM1637_GInst_t* device) {
	
	/* Set up address to zero position. */
	ram_data.AddressCmd.ADDRESS = TM1637_C0H;
	ram_data.AddressCmd.RESERVED5_4 = 0;
	ram_data.AddressCmd.ADDR_CMD_WORD = TM1637_ADDR_CMD;
	
	/* Prepare writing whole data pack (6 bytes). */
	ram_data.DataCmd.DATA_WR_MODE = TM1637_WRITE_DATA_TO_DISPLAY;
	ram_data.DataCmd.ADDR_ADDING_MODE = TM1637_AUTOMATIC_ADDR_ADDING;
	ram_data.DataCmd.TEST_MODE_SETTING = TM1637_NORMAL_MODE;
	ram_data.DataCmd.RESERVED5_4 = 0;
	ram_data.DataCmd.DATA_CMD_WORD = TM1637_DATA_CMD;
	
	device->tx_data(ram_data.DataCmd.DataCommand, NULL, 0);
	device->tx_data(ram_data.AddressCmd.AddressCommand, &ram_data.Gridx, 6);
}


/* Hardware dependent functions. */

/*
	@brief Write data to screen.
*/
void TM1637_WriteTo(uint8_t command, void* pData, uint8_t size) {

	TM1637_WriteData(command, pData, size);
}

/*
	@brief Read data from screen.
*/
void TM1637_ReadFrom(uint8_t command, void* pData, uint8_t size) {

	TM1637_ReadData(command, pData, size);
}

/*
 * @brief 
 *
 **/
static void TM1637_StartBus(void) {
	
	TurnUpCLK();
	TurnUpData();
	TM1637_NOPDelay();
	TurnDownData();
	TM1637_NOPDelay();
}

/*
 * @brief 
 *
 **/
static void TM1637_StopBus(void) {
		
	TurnDownCLK();
	//TM1637_NOPDelay();
	TurnDownData();
	TM1637_NOPDelay();
	TurnUpCLK();
	TM1637_NOPDelay();
	TurnUpData();
	TM1637_NOPDelay();
}

/*
	@brief Reading ACK from IC.
*/
static uint8_t TM1637_ReadACK(void) {

	uint8_t ack_state = 1;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*Configure GPIO pins : PF7 */
	GPIO_InitStruct.Pin = TM1637_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TM1637_DATA_PORT, &GPIO_InitStruct);

	/* Down the clock pin. */
	TurnDownCLK();
	TM1637_NOPDelay();
	/* Up the clock pin. */
	TurnUpCLK();
	TM1637_NOPDelay();
	/* Read the 9th bit on the data pin (check out ACK pulse). */

	ack_state = 1;
	ack_state = HAL_GPIO_ReadPin(TM1637_DATA_PORT, TM1637_DATA_PIN);

	/*Configure GPIO pins : PF7 */
	GPIO_InitStruct.Pin = TM1637_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TM1637_DATA_PORT, &GPIO_InitStruct);

	return ack_state;
}

/*
 * @brief 
 *
 **/
static void TM1637_WriteData(uint8_t command, uint8_t *pData, uint8_t Size) {
		
	uint8_t TxData = 0, ack_state = 1;
	
	/* Generating start condition. */
	TM1637_StartBus();
	
	/* Transmitting the command. */
	TxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		TurnDownCLK();
		TM1637_NOPDelay();
		/* MSB first. */
		TM1637_DATA_PORT->BSRR = (TxData & 0x01) ? TM1637_DATA_PIN : (TM1637_DATA_PIN << 16);
		TM1637_NOPDelay();
		/* Up the clock pin. */
		TurnUpCLK();
		TM1637_NOPDelay();
		/* Getting next bit.*/
		TxData >>= 1;
	}

	/* Read ACK. */
	ack_state = TM1637_ReadACK();
	/* Stop the bus if only command wants to transmit. */
	if (pData == NULL && Size == 0) {
		TM1637_StopBus();
	}
	
	/* Transmit the data to the data line. */
	if (pData != NULL && Size != 0) {	
		for (uint8_t i = Size; i > 0; i--) {	
			TxData = *pData;	
			for (uint8_t j = 8; j > 0; j--) {	
				/* Down the clock pin. */
				TurnDownCLK();
				TM1637_NOPDelay();
				/* MSB first. */
				TM1637_DATA_PORT->BSRR = (TxData & 0x01) ? TM1637_DATA_PIN : (TM1637_DATA_PIN << 16);
				TM1637_NOPDelay();
				/* Up the clock pin. */
				TurnUpCLK();
				TM1637_NOPDelay();
				/* Getting next bit.*/
				TxData >>= 1;
			}
			/* Read ACK. */
			ack_state = TM1637_ReadACK();
			/* Increment data byte to be transmitted. */
			pData++;				 
		}
		/* Generating stop condition. */
		TM1637_StopBus();
		__NOP();
	}
		
}


/*
 * @brief 
 *
 **/
static void TM1637_ReadData(uint8_t command, uint8_t *pData, uint8_t Size) {
		
	uint8_t TxRxData = 0, ack_state;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Generating start condition. */
	TM1637_StartBus();
	
	/* Transmitting the command. */
	TxRxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		TurnDownCLK();
		/* MSB first. */
		TM1637_DATA_PORT->BSRR = (TxRxData & 0x01) ? TM1637_DATA_PIN : (TM1637_DATA_PIN << 16);
		TM1637_NOPDelay();
		/* Up the clock pin. */
		TurnUpCLK();
		TM1637_NOPDelay();
		/* Getting next bit.*/
		TxRxData >>= 1;
	}
	/* Read ACK. */
	ack_state = TM1637_ReadACK();
	/* Stop the bus if only command wants to transmit. */
	if (pData == NULL && Size == 0) {
		TM1637_StopBus();
	}
	/* Data port like an input. */
	GPIO_InitStruct.Pin = TM1637_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TM1637_DATA_PORT, &GPIO_InitStruct);
	if (pData != NULL && Size != 0) {
		/* Read the data from the data line. */	
		for (uint8_t i = 0; i < 8; i++) {
			/* Down the clock pin. */
			TurnDownCLK();
			TM1637_NOPDelay();
			/* Up the clock pin. */
			TurnUpCLK();
			TM1637_NOPDelay();
			/* Reading data bit. MSB first. */
			TxRxData |= (HAL_GPIO_ReadPin(TM1637_DATA_PORT, TM1637_DATA_PIN) << i);
		}
		/* Data port like an output. */
		GPIO_InitStruct.Pin = TM1637_DATA_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(TM1637_DATA_PORT, &GPIO_InitStruct);

		*pData = TxRxData;
		/* Read ACK. */
		ack_state = TM1637_ReadACK();
		/* Generating stop condition. */
		TM1637_StopBus();
		
	}
}

/*
	@brief NOP delay (about two 2us for 180MHz core clock).
	For the OD output in the F429 delay should be 1000 (about 100us).
*/
static void TM1637_NOPDelay(void) {

	for (uint16_t i = 1000; i > 0; i--) {
		__NOP();
	}
}

/*
	@brief Turn down the CLK.
*/
__INLINE static void TurnDownCLK(void) {

	/* Down the clock pin. */
	TM1637_CLK_PORT->BSRR = TM1637_CLK_PIN << 16;
}

/*
	@brief Turn up the CLK.
*/
__INLINE static void TurnUpCLK(void) {
	TM1637_CLK_PORT->BSRR = TM1637_CLK_PIN;
}

/*
	@brief Turn down the DATA.
*/
__INLINE static void TurnDownData(void) {
	TM1637_DATA_PORT->BSRR = TM1637_DATA_PIN << 16;
}

/*
	@brief Turn up the DATA.
*/
__INLINE static void TurnUpData(void) {
	TM1637_DATA_PORT->BSRR = TM1637_DATA_PIN;
}
