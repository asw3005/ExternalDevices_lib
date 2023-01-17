/*
 * @brief Driver for the TM1618.
 *	Created 08.16.22 by asw3005
 **/

#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#include "TM1618.h"

/* Private variables.*/
static TM1618_RamData_t ram_data;

/* Private function prototypes. */
static void TM1618_WriteData(uint8_t command, uint16_t *pData, uint8_t Size);
static void TM1618_ReadData(uint8_t command, uint8_t *pData, uint8_t Size);
static void TM1618_NOPDelay(void);

__INLINE static void TM1618_TurnDownSTB(void);
__INLINE static void TM1618_TurnUpSTB(void);
__INLINE static void TM1618_TurnDownCLK(void);
__INLINE static void TM1618_TurnUpCLK(void);
__INLINE static void TM1618_TurnDownData(void);
__INLINE static void TM1618_TurnUpData(void);

/*
 * @brief Write 7-seg display's code.
 * 
 * @param device : General instance of data struct of TM1618_GInst_t type.
 * @param grid : Display's grid, may be set from 0 to 6.
 * @param segment : One of seven segments of 7-seg display. May be from 0 to 4, 11 to 13 
					(hardware number of segments 1 to 5 and 12 to 14).
 * @param state : If 0 the segment is power down, otherwise is power up.
 *
 **/
void TM1618_WriteSeg(uint8_t grid, uint8_t segment, uint8_t state) {
	
	if (grid > 6 || segment > 14 || (segment > 5 & segment < 11)) return;
	
	if (segment < 8) {
		if (state) {
			ram_data.Gridx[grid].SegData1 |= 1 << segment;
		} else {
			ram_data.Gridx[grid].SegData1 &= ~(1 << segment);
		}
	} else {
		if (state) {
			ram_data.Gridx[grid].SegData2 |= 1 << (segment - 8);
		} else {
			ram_data.Gridx[grid].SegData2 &= ~(1 << (segment - 8));
		}
	}
}

/*
 * @brief Reads scan code of keys.
 * 
 * @param device : General instance of data struct of TM1618_GInst_t type.
 *
 **/
TM1618_KeysData_t TM1618_ReadKeys(TM1618_GInst_t* device) {

	TM1618_KeysData_t scan_code;

	/* Prepare reading scan code of keys. (3 bytes). */
	ram_data.DataCmd.DATA_WR_MODE = TM1618_READ_KEY_SCAN_DATA;
	ram_data.DataCmd.ADDR_ADDING_MODE = TM1618_AUTOMATIC_ADDR_ADDING;
	ram_data.DataCmd.TEST_MODE_SETTING = TM1618_NORMAL_MODE;
	ram_data.DataCmd.RESERVED5_4 = 0;
	ram_data.DataCmd.DATA_CMD_WORD = TM1618_DATA_CMD;

	device->rx_data(ram_data.DataCmd.DataCommand, &scan_code.ScanCode1, 3);
	return scan_code;
}

/*
 * @brief Power up or power down display, configurate brightness of display.
 * 
 * @param device : General instance of data struct of TM1618_GInst_t type.
 *
 **/
void TM1618_PwrCtrl(TM1618_GInst_t* device, uint8_t state, uint8_t pulse_width) {
	
	TM1618_DisplayCmd_t display_cmd;
	
	/* Prepare display control command byte. */
	display_cmd.PULSE_WIDTH = pulse_width;
	display_cmd.DISPLAY_ON_OFF = state;
	display_cmd.RESERVED5_4 = 0;
	display_cmd.DISPLAY_CMD_WORD = TM1618_DISPLAY_CTRL_CMD;
	
	device->tx_data(display_cmd.DisplayCommand, NULL, 0);	
}

/*
 * @brief Configuration for grid/segment ration.
 *
 * @param device : General instance of data struct of TM1618_GInst_t type.
 * @param grid_seg: It can be TM1618_4grid_8SEGMENT, TM1618_5grid_7SEGMENT,
							   TM1618_6grid_6SEGMENT, TM1618_7grid_5SEGMENT.
 *
 **/
void TM1618_GridSegCfg(TM1618_GInst_t* device, uint8_t grid_seg) {

	TM1618_DisplayCfg_t display_cfg;

	/* Prepare display control command byte. */
	display_cfg.DISPLAY_CFG = grid_seg;
	display_cfg.RESERVED = 0;
	display_cfg.DISPLAY_CFG_WORD = TM1618_DISPLAY_CFG;

	device->tx_data(display_cfg.DisplayConfig, NULL, 0);
}

/* Private functions. */

/*
 * @brief Writes data to display RAM buffer.
 * 
 * @param device : General instance of data struct of TM1618_GInst_t type.
 *
 **/
void TM1618_WriteRAM(TM1618_GInst_t* device) {
	
	/* Set up address to zero position. */
	ram_data.AddressCmd.ADDRESS = TM1618_C0H;
	ram_data.AddressCmd.RESERVED5_4 = 0;
	ram_data.AddressCmd.ADDR_CMD_WORD = TM1618_ADDR_CMD;
	
	/* Prepare writing whole data pack (6 bytes). */
	ram_data.DataCmd.DATA_WR_MODE = TM1618_WRITE_DATA_TO_DISPLAY;
	ram_data.DataCmd.ADDR_ADDING_MODE = TM1618_AUTOMATIC_ADDR_ADDING;
	ram_data.DataCmd.TEST_MODE_SETTING = TM1618_NORMAL_MODE;
	ram_data.DataCmd.RESERVED5_4 = 0;
	ram_data.DataCmd.DATA_CMD_WORD = TM1618_DATA_CMD;
	
	device->tx_data(ram_data.DataCmd.DataCommand, NULL, 0);
	device->tx_data(ram_data.AddressCmd.AddressCommand, &ram_data.Gridx, 7);
}


/* Hardware dependent functions. */

/*
	@brief Write data to screen.
*/
void TM1618_WriteTo(uint8_t command, void* pData, uint8_t size) {
	TM1618_WriteData(command, pData, size);
}

/*
	@brief Read data from screen.
*/
void TM1618_ReadFrom(uint8_t command, void* pData, uint8_t size) {
	TM1618_ReadData(command, pData, size);
}

/*
 * @brief 
 *
 **/
static void TM1618_WriteData(uint8_t command, uint16_t *pData, uint8_t Size) {
		
	uint16_t TxData = 0, ack_state = 1;
	
	/* Generating start condition. */
	TM1618_TurnDownSTB();
	TM1618_NOPDelay();
	/* Transmitting the command. */
	TxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		TM1618_TurnDownCLK();
		TM1618_NOPDelay();
		/* MSB first. */
		TM1618_DATA_PORT->BSRR = (TxData & 0x01) ? TM1618_DATA_PIN : (TM1618_DATA_PIN << 16);
		TM1618_NOPDelay();
		/* Up the clock pin. */
		TM1618_TurnUpCLK();
		TM1618_NOPDelay();
		/* Getting next bit.*/
		TxData >>= 1;
	}
	/* Stop the bus if only command wants to transmit. */
	if (pData == NULL && Size == 0) {
		TM1618_TurnUpSTB();
		TM1618_NOPDelay();
	}
	
	/* Transmit the data to the data line. */
	if (pData != NULL && Size != 0) {	
		for (uint8_t i = Size; i > 0; i--) {	
			TxData = *pData;
			for (uint8_t j = 16; j > 0; j--) {	
				/* Down the clock pin. */
				TM1618_TurnDownCLK();
				TM1618_NOPDelay();
				/* MSB first. */
				TM1618_DATA_PORT->BSRR = (TxData & 0x01) ? TM1618_DATA_PIN : (TM1618_DATA_PIN << 16);
				TM1618_NOPDelay();
				/* Up the clock pin. */
				TM1618_TurnUpCLK();
				TM1618_NOPDelay();
				/* Getting next bit.*/
				TxData >>= 1;
			}
			/* Increment data byte to be transmitted. */
			pData++;				 
		}
		/* Generating stop condition. */
		TM1618_TurnUpSTB();
		TM1618_NOPDelay();
		__NOP();
	}		
}


/*
 * @brief 
 *
 **/
static void TM1618_ReadData(uint8_t command, uint8_t *pData, uint8_t Size) {
		
	uint8_t TxRxData = 0, ack_state;

	/* Generating start condition. */
	TM1618_TurnDownSTB();
	TM1618_NOPDelay();
	/* Transmitting the command. */
	TxRxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		TM1618_TurnDownCLK();
		/* MSB first. */
		TM1618_DATA_PORT->BSRR = (TxRxData & 0x01) ? TM1618_DATA_PIN : (TM1618_DATA_PIN << 16);
		TM1618_NOPDelay();
		/* Up the clock pin. */
		TM1618_TurnUpCLK();
		TM1618_NOPDelay();
		/* Getting next bit.*/
		TxRxData >>= 1;
	}
	/* Stop the bus if only command wants to transmit. */
	if (pData == NULL && Size == 0) {
		TM1618_TurnUpSTB();
	}

	if (pData != NULL && Size != 0) {
		/* Read the data from the data line. */	
		for (uint8_t i = Size; i > 0; i--) {
			for (uint8_t j = 0; j < 8; j++) {
				/* Down the clock pin. */
				TM1618_TurnDownCLK();
				TM1618_NOPDelay();
				/* Up the clock pin. */
				TM1618_TurnUpCLK();
				TM1618_NOPDelay();
				/* Reading data bit. MSB first. */
				TxRxData |= (HAL_GPIO_ReadPin(TM1618_DATA_PORT, TM1618_DATA_PIN) << j);
			}
			*pData = TxRxData;
			pData++;
		}
		/* Generating stop condition. */
		TM1618_TurnUpSTB();
	}
}

/*
	@brief NOP delay (about two 2us for 180MHz core clock).
*/
static void TM1618_NOPDelay(void) {
	for (uint16_t i = 10; i > 0; i--) {
		__NOP();
	}
}

/*
	@brief Turn down the CLK.
*/
__INLINE static void TM1618_TurnDownCLK(void) {

	/* Down the clock pin. */
	TM1618_CLK_PORT->BSRR = TM1618_CLK_PIN << 16;
}

/*
	@brief Turn up the CLK.
*/
__INLINE static void TM1618_TurnUpCLK(void) {
	TM1618_CLK_PORT->BSRR = TM1618_CLK_PIN;
}

/*
	@brief Turn down the DATA.
*/
__INLINE static void TM1618_TurnDownData(void) {
	TM1618_DATA_PORT->BSRR = TM1618_DATA_PIN << 16;
}

/*
	@brief Turn up the DATA.
*/
__INLINE static void TM1618_TurnUpData(void) {
	TM1618_DATA_PORT->BSRR = TM1618_DATA_PIN;
}

/*
	@brief Turn down the STB.
*/
__INLINE static void TM1618_TurnDownSTB(void) {
	TM1618_STB_PORT->BSRR = TM1618_STB_PIN << 16;
}

/*
	@brief Turn up the STB.
*/
__INLINE static void TM1618_TurnUpSTB(void) {
	TM1618_STB_PORT->BSRR = TM1618_STB_PIN;
}





