/*
 * @brief Driver for the TM1637.
 *
 **/

#include "stm32f103xb.h"
#include "tm1637.h"

/* Private function prototypes. */
static void TM1637_WriteRAM(TM1637_GInst_t* device);

static void TM1637_StartBus(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin);
static void TM1637_StopBus(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin);
static void TM1637_WriteData(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin, uint8_t command, uint8_t *pData, uint8_t Size);
static void TM1637_ReadData(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin, uint8_t command, uint8_t *pData, uint8_t Size);

/* Private variables.*/
static TM1637_RamData_t tm1637_ram; 
//static TM1637_GInst_t tm1637_inst = {
//
//		
//		
//	};

/*
 * @brief Write 7-seg display's code.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 * @param grid : Display's grid, may be set from 0 to 5.
 * @param segment : One of seven segments of 7-seg display. May be from 0 to 7.
 * @param state : If 0 the segment is power down, otherwise is power up.
 *
 **/
void TM1637_WriteSeg(TM1637_GInst_t* device, uint8_t grid, uint8_t segment, uint8_t state) {
	
	if (grid > 5 || segment > 7) return;
	if (state) {		
		tm1637_ram.RamData[grid] |= 1 << segment;		
	} else {
		tm1637_ram.RamData[grid] &= ~(1 << segment);
	}	
	
	TM1637_WriteRAM(device);
}

/*
 * @brief Reads scan code of keys.
 * 
 * @param device : General instance of data struct of TM1637_GInst_t type.
 *
 **/
TM1637_RamData_t TM1637_ReadKeys(TM1637_GInst_t* device) {

	TM1637_DataCmd_t data_cmd;
	TM1637_RamData_t scan_codes;

	/* Prepare reading scan code of keys. (6 bytes). */
	data_cmd.DATA_WR_MODE = TM1637_READ_KEY_SCAN_DATA;
	data_cmd.ADDR_ADDING_MODE = TM1637_AUTOMATIC_ADDR_ADDING;
	data_cmd.TEST_MODE_SETTING = TM1637_NORMAL_MODE;
	data_cmd.RESERVED5_4 = 0;
	data_cmd.DATA_CMD_WORD = TM1637_DATA_CMD;
	
	device->rx_data(data_cmd.DataCommand, &scan_codes, 6);
	return scan_codes;
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
static void TM1637_WriteRAM(TM1637_GInst_t* device) {
	
	TM1637_AddressCmd_t addr_cmd;
	TM1637_DataCmd_t data_cmd;
	
	/* Set up address to zero position. */
	addr_cmd.ADDRESS = TM1637_C0H;
	addr_cmd.RESERVED5_4 = 0;
	addr_cmd.ADDR_CMD_WORD = TM1637_ADDR_CMD;
	
	/* Prepare writing whole data pack (6 bytes). */
	data_cmd.DATA_WR_MODE = TM1637_WRITE_DATA_TO_DISPLAY;
	data_cmd.ADDR_ADDING_MODE = TM1637_AUTOMATIC_ADDR_ADDING;
	data_cmd.TEST_MODE_SETTING = TM1637_NORMAL_MODE;
	data_cmd.RESERVED5_4 = 0;
	data_cmd.DATA_CMD_WORD = TM1637_DATA_CMD;
	
	device->tx_data(addr_cmd.AddressCommand, NULL, 0);
	device->tx_data(data_cmd.DataCommand, &tm1637_ram, 6);	
}


/* Hardware dependent functions. */

/*
 * @brief 
 *
 **/
static void TM1637_StartBus(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin) {
	
	gpio->BSRR = data_gpio_pin;
	gpio->BSRR = clk_gpio_pin;
	gpio->BSRR = data_gpio_pin << 16;
}

/*
 * @brief 
 *
 **/
static void TM1637_StopBus(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin) {
		
	//gpio->BSRR = data_gpio_pin << 16;
	gpio->BSRR = clk_gpio_pin;
	gpio->BSRR = data_gpio_pin;
}



/*
 * @brief 
 *
 **/
static void TM1637_WriteData(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin, uint8_t command, uint8_t *pData, uint8_t Size) {
		
	uint16_t TxData;
	
	/* Generating start condition. */
	TM1637_StartBus(gpio, clk_gpio_pin, data_gpio_pin);
	
	/* Transmitting the command. */
	TxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		gpio->BSRR = clk_gpio_pin << 16;
		/* MSB first. */
		gpio->BSRR = (TxData & 0x80) ? data_gpio_pin : (data_gpio_pin << 16);
		/* Up the clock pin. */
		gpio->BSRR = clk_gpio_pin;		
		/* Getting next bit.*/
		TxData <<= 1;
	}
	/* Down the clock pin. */
	gpio->BSRR = clk_gpio_pin << 16;
	/* Up the clock pin. */
	gpio->BSRR = clk_gpio_pin;	
	/* Read the 9th bit on the data pin (check out ACK pulse). */
	
	
	/* Transmitting the data to the data line. */
	if (pData != NULL && Size != 0) {	
		for (uint8_t i = Size; i > 0; i--) {	
			TxData = *pData;	
			for (uint8_t j = 8; j > 0; j--) {	
				/* Down the clock pin. */
				gpio->BSRR = clk_gpio_pin << 16;
				/* MSB first. */
				gpio->BSRR = (TxData & 0x80) ? data_gpio_pin : (data_gpio_pin << 16);
				/* Up the clock pin. */
				gpio->BSRR = clk_gpio_pin;		
				/* Getting next bit.*/
				TxData <<= 1;
			}
			/* Down the clock pin. */
			gpio->BSRR = clk_gpio_pin << 16;
			/* Up the clock pin. */
			gpio->BSRR = clk_gpio_pin;	
			/* Read the 9th bit on the data pin (check out ACK pulse). */	
		
			/* Increment data byte to be transmitted. */
			pData++;		
		}
	}
		
	/* Down the data line. */
	gpio->BSRR = data_gpio_pin << 16;
	/* Generating stop condition. */
	TM1637_StopBus(gpio, clk_gpio_pin, data_gpio_pin);
}


/*
 * @brief 
 *
 **/
static void TM1637_ReadData(GPIO_TypeDef* gpio, uint16_t clk_gpio_pin, uint16_t data_gpio_pin, uint8_t command, uint8_t *pData, uint8_t Size) {
		
	uint16_t TxRxData;
	
	/* Generating start condition. */
	TM1637_StartBus(gpio, clk_gpio_pin, data_gpio_pin);
	
	/* Transmitting the command. */
	TxRxData = command;
	for (uint8_t i = 8; i > 0; i--) {	
		/* Down the clock pin. */
		gpio->BSRR = clk_gpio_pin << 16;
		/* MSB first. */
		gpio->BSRR = (TxRxData & 0x80) ? data_gpio_pin : (data_gpio_pin << 16);
		/* Up the clock pin. */
		gpio->BSRR = clk_gpio_pin;		
		/* Getting next bit.*/
		TxRxData <<= 1;
	}
	/* Down the clock pin. */
	gpio->BSRR = clk_gpio_pin << 16;
	/* Up the clock pin. */
	gpio->BSRR = clk_gpio_pin;	
	/* Read the 9th bit on the data pin (check out ACK pulse). */
	
	
	/* Reading the data from the data line. */
	for (uint8_t i = Size; i > 0; i--) {	
		//TxData = *pData;		
		for (uint8_t j = 8; j > 0; j--) {	
			/* Down the clock pin. */
			gpio->BSRR = clk_gpio_pin << 16;			
			/* Up the clock pin. */
			gpio->BSRR = clk_gpio_pin;		
			/* Reading data bit. MSB first. */
			//gpio->BSRR = (TxData & 0x80) ? data_gpio_pin : (data_gpio_pin << 16);
			TxRxData |= gpio->IDR; 
			
			/* Getting next bit.*/
			//TxData <<= 1;
		}
		/* Down the clock pin. */
		gpio->BSRR = clk_gpio_pin << 16;
		/* Up the clock pin. */
		gpio->BSRR = clk_gpio_pin;	
		/* Read the 9th bit on the data pin (check out ACK pulse). */	
		
		/* Increment data byte to be transmitted. */
		pData++;		
	}	
	/* Down the data line. */
	gpio->BSRR = data_gpio_pin << 16;
	/* Generating stop condition. */
	TM1637_StopBus(gpio, clk_gpio_pin, data_gpio_pin);
}