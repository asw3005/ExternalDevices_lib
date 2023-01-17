/*
	@brief NORDIC SEMI nRF24L01 driver.
	Created 08.31.22 by asw3005.
*/


#include "stm32f4xx_hal.h"
#include "nRF24l01.h"

/* Extern variables. */
#ifdef HAL_SPI_MODULE_ENABLED
SPI_HandleTypeDef hspi1;
#define NRF24L01_SPI hspi1
#endif /* HAL_SPI_MODULE_ENABLED */

/* Private variables. */



/* Private function prototypes. */
static void NRF24L01_InitPerif(void);
static void NRF24L01_NOPDelay(void);
//static void NRF24L01_TxStart(NRF24L01_GInst_t* device);
//static void NRF24L01_RxStart(NRF24L01_GInst_t* device);

/* Payload struct. */
NRF24L01_TxRxData_t nrf24l01_payload = { 0 };

/* General struct instance. */
static NRF24L01_GInst_t nRF24l01_ginst = {

	.delay_f = NRF24L01_Delay,
	.cepin_ctrl_f = SPI_CECtrl,
	.cspin_ctrl_f = SPI_CSCtrl,
	.spi_rx_data_f = SPI_RxData,
	.spi_tx_data_f = SPI_TxData
};

/*
	@brief nRF24L01 init.
*/
void NRF24L01_Init(NRF24L01_GInst_t* device) {
	
	NRF24L01_CfgReg_t config_reg;
	NRF24L01_EnRxAddr_t enrx_pipe;
	NRF24L01_RFChannel_t rf_channel;
	/* For P0 - P5. */
	NRF24L01_NumberBytes_t bytes_number[6];

	/* Init pins. */
	NRF24L01_InitPerif();

	/* Configure register's bits. */
	config_reg.PRIM_RX = 0;
	config_reg.PWR_UP = 0;
	config_reg.CRCO = 1;
	config_reg.EN_CRC = 1;
	config_reg.MASK_MAX_RT = 1;
	config_reg.MASK_TX_DS = 0;
	config_reg.MASK_RX_DR = 0;
	config_reg.RESERVED = 0;
	NRF24L01_WriteReg(device, NRF24L01_CONFIG, &config_reg.CfgReg, sizeof(config_reg.CfgReg));

	/* Configure register's bits. */
	enrx_pipe.ERX_P0 = 1;
	enrx_pipe.ERX_P1 = 1;
	enrx_pipe.ERX_P2 = 1;
	enrx_pipe.ERX_P3 = 1;
	enrx_pipe.ERX_P4 = 1;
	enrx_pipe.ERX_P5 = 1;
	enrx_pipe.RESERVED = 0;
	NRF24L01_WriteReg(device, NRF24L01_EN_RXADDR, &enrx_pipe.EnRxAddrReg, sizeof(enrx_pipe.EnRxAddrReg));

	/* Configure register's bits. */
	rf_channel.RF_CH = 2;
	rf_channel.RESERVED = 0;
	NRF24L01_WriteReg(device, NRF24L01_RF_CH, &rf_channel.RFChannelReg, sizeof(rf_channel.RFChannelReg));

	/* Configure register's bits. */
	bytes_number[0].RX_PW_Px = 32;
	bytes_number[0].RESERVED = 0;
	bytes_number[1].RX_PW_Px = 32;
	bytes_number[1].RESERVED = 0;
	bytes_number[2].RX_PW_Px = 32;
	bytes_number[2].RESERVED = 0;
	bytes_number[3].RX_PW_Px = 32;
	bytes_number[3].RESERVED = 0;
	bytes_number[4].RX_PW_Px = 32;
	bytes_number[4].RESERVED = 0;
	bytes_number[5].RX_PW_Px = 32;
	bytes_number[5].RESERVED = 0;
	NRF24L01_WriteReg(device, NRF24L01_RX_PW_P0, &bytes_number->NumberOfBytesReg, sizeof(bytes_number));

}

/*
	@brief Set TX address.

	@param device : General struct instance of function pointers.
	@param tx_addr : Tx address of nRF24L01 device.
*/
void NRF24L01_SetPipeTxAddr(NRF24L01_GInst_t* device, uint64_t tx_addr) {

	NRF24L01_Addr_t pipe_addr;

	pipe_addr.ADDR_ALL_BITS = tx_addr;
	/* Write data to nRF device. */
	NRF24L01_WriteReg(device, NRF24L01_TX_ADDR, pipe_addr.AddrByte, NRF24L01_FULL_ADDR_AMOUNT);
}

/*
	@brief Set RX address of specific pipe.

	@param device : General struct instance of function pointers.
	@param pipe : Number of RX pipe in nRF24L01 device (may be 0 to 5).
	@param addr : Addres of pipe that you need.
*/
uint8_t NRF24L01_SetPipeRxAddr(NRF24L01_GInst_t* device, NRF24L01_PIPE_NUMBER_t rx_pipe, uint64_t addr) {

	NRF24L01_Addr_t pipe_addr;

	if (rx_pipe < NRF24L01_PIPE2) {
		nrf24l01_payload.Size = NRF24L01_FULL_ADDR_AMOUNT;
		pipe_addr.ADDR_ALL_BITS = addr;
	} else if (rx_pipe < (NRF24L01_PIPE5 + 1)) {
		nrf24l01_payload.Size = NRF24L01_LOW_ADDR_AMOUNT;
		pipe_addr.ADDR_LOW_BITS = addr;
	} else return NRF24L01_SEND_ERR;
	/* Write data to nRF device. */
	NRF24L01_WriteReg(device, NRF24L01_RX_ADDR_P0 + rx_pipe, pipe_addr.AddrByte, nrf24l01_payload.Size);
	return NRF24L01_SEND_OK;
}

/*
	@brief Set Rx addresses of all pipes and TX address.

	@param device : General struct instance of function pointers.
	@param rx_pipeX_addr : Address of RX pipe that you need, where X is the pipe number from 0 to 5.
	@param tx_addr : Tx address of nRF24L01 device.
*/
void NRF24L01_SetPipeAddrBurst(NRF24L01_GInst_t* device, uint64_t rx_pipe0_addr, uint64_t rx_pipe1_addr, uint8_t rx_pipe2_addr,
	uint8_t rx_pipe3_addr, uint8_t rx_pipe4_addr, uint8_t rx_pipe5_addr, uint64_t tx_addr) {

	NRF24L01_SetAddr_t pipe_addresses;

	/* Write RX pipes' addresses. */
	pipe_addresses.RxAddrPipe0.ADDR_ALL_BITS = rx_pipe0_addr;
	pipe_addresses.RxAddrPipe1.ADDR_ALL_BITS = rx_pipe1_addr;
	pipe_addresses.RxAddrPipe2 = rx_pipe2_addr;
	pipe_addresses.RxAddrPipe3 = rx_pipe3_addr;
	pipe_addresses.RxAddrPipe4 = rx_pipe4_addr;
	pipe_addresses.RxAddrPipe5 = rx_pipe5_addr;
	/* Write TX addr. */
	pipe_addresses.TxAddr.ADDR_ALL_BITS = tx_addr;

	/* Write data to nRF device. */
	NRF24L01_WriteReg(device, NRF24L01_RX_ADDR_P0, pipe_addresses.RxAddrPipe0.AddrByte, NRF24L01_FULL_RXTX_ADDR_AMOUNT);
}

/*
	@brief Write payload.

	@param device : General struct instance of function pointers.
	@param pdata* : Pointer to data to transmit.
	@param size : Amount of bytes to transmit.
*/
void NRF24L01_WritePayload(NRF24L01_GInst_t* device, uint8_t* pdata, uint8_t size) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_W_TX_PAYLOAD };

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->spi_tx_data_f(pdata, size);
	/* Delay in milliseconds. */
	device->delay_f(NRF24L01_DELAY_BEFORE_CSHIGH);
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);
}

/*
	@brief Read payload.

	@param device : General struct instance of function pointers.
	@param size : Amount of bytes to receive.
*/
NRF24L01_TxRxData_t* NRF24L01_ReadPayload(NRF24L01_GInst_t* device, uint8_t size) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_R_RX_PAYLOAD };

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->spi_rx_data_f(nrf24l01_payload.Data, size);
	/* Delay in milliseconds. */
	device->delay_f(NRF24L01_DELAY_BEFORE_CSHIGH);
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);

	return &nrf24l01_payload;
}

/*
	@brief Read observe tx register.

	@param device : General struct instance of function pointers.
*/
NRF24L01_Observe_t NRF24L01_ReadObserveTx(NRF24L01_GInst_t* device) {

	NRF24L01_Observe_t observe_tx;

	/* Write data to nRF device. */
	observe_tx.ObserveTxReg = NRF24L01_ReadReg(device, NRF24L01_OBSERVE_TX);

	return observe_tx;
}

/*
	@brief Read carrier detect register.

	@param device : General struct instance of function pointers.
*/
uint8_t NRF24L01_ReadCarrierDet(NRF24L01_GInst_t* device) {

	NRF24L01_CarrierDet_t carrier_det;

	/* Write data to nRF device. */
	carrier_det.CarrierDetReg = NRF24L01_ReadReg(device, NRF24L01_CD);

	return carrier_det.CD;
}

/*
	@brief Read FIFO status register.

	@param device : General struct instance of function pointers.
*/
NRF24L01_FIFOStatus_t NRF24L01_ReadFIFOStatus(NRF24L01_GInst_t* device) {

	NRF24L01_FIFOStatus_t fifo_status;

	/* Write data to nRF device. */
	fifo_status.FIFOStatusReg = NRF24L01_ReadReg(device, NRF24L01_FIFO_STATUS);

	return fifo_status;
}

/*
	@brief Read status register through NOP command.

	@param device : General struct instance of function pointers.
*/
NRF24L01_Status_t NRF24L01_ReadStatusV1(NRF24L01_GInst_t* device) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_NOP };
	NRF24L01_Status_t Status;

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	/* Read status byte. */
	Status.StatusReg = (uint8_t)NRF24L01_SPI.Instance->DR;
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);

	return Status;
}

/*
	@brief Read status register through regular reading command.

	@param device : General struct instance of function pointers.
*/
NRF24L01_Status_t NRF24L01_ReadStatusV2(NRF24L01_GInst_t* device) {

	static NRF24L01_RWCmd_t rw_cmd;
	NRF24L01_Status_t Status;

	rw_cmd.RW_CMD = NRF24L01_R_REGISTER;
	rw_cmd.MEM_MAP_ADDR = NRF24L01_STATUS;

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->spi_rx_data_f(&Status.StatusReg, 1);
	NRF24L01_NOPDelay();
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);

	return Status;
}

/*
	@brief Flush TX FIFO, used in TX mode.

	@param device : General struct instance of function pointers.
*/
void NRF24L01_FlushTx(NRF24L01_GInst_t* device) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_FLUSH_TX };

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);
}

/*
	@brief Flush RX FIFO, used in RX mode.

	@param device : General struct instance of function pointers.
*/
void NRF24L01_FlushRx(NRF24L01_GInst_t* device) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_FLUSH_RX };

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);
}

/*
	@brief Reuse last send payload.

	@param device : General struct instance of function pointers.
*/
void NRF24L01_ReuseTxPayload(NRF24L01_GInst_t* device) {

	static NRF24L01_RWCmd_t rw_cmd = { .RWCmd = NRF24L01_REUSE_TX_PL };

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);
}

/*
	@brief Write some register.

	@param device : General struct instance of function pointers.
	@param reg_addr : Register address from NRF24L01_MEMORY_MAP_t enum.
	@param *reg_value : Pointer to the data to send.
	@param size : Amount of data to send.
*/
void NRF24L01_WriteReg(NRF24L01_GInst_t* device, uint8_t reg_addr, uint8_t* reg_value, uint8_t size) {

	NRF24L01_RWCmd_t rw_cmd;

	rw_cmd.RW_CMD = NRF24L01_W_REGISTER;
	rw_cmd.MEM_MAP_ADDR = reg_addr;
	nrf24l01_payload.Data[0] = rw_cmd.RWCmd;
	for (uint8_t i = 0; i < size; i++) {
		nrf24l01_payload.Data[i + 1] = *reg_value;
		reg_value++;
	}

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(nrf24l01_payload.Data, NRF24L01_CMD_BYTE_AMOUNT + size);
	device->delay_f(NRF24L01_DELAY_BEFORE_CSHIGH);
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);
}

/*
	@brief Read some register.

	@param device : General struct instance of function pointers.
	@param reg_addr : Register address from NRF24L01_MEMORY_MAP_t enum.
*/
uint8_t NRF24L01_ReadReg(NRF24L01_GInst_t * device, uint8_t reg_addr) {

	static NRF24L01_RWCmd_t rw_cmd;
	uint8_t data;

	rw_cmd.RW_CMD = NRF24L01_R_REGISTER;
	rw_cmd.MEM_MAP_ADDR = reg_addr;

	/* Activate nRF SPI and send data to. */
	device->cspin_ctrl_f(NRF24L01_CS_ACTIVE);
	device->spi_tx_data_f(&rw_cmd.RWCmd, 1);
	/* Wait command to send. */
	NRF24L01_NOPDelay();
	device->spi_rx_data_f(&data, 1);
	NRF24L01_NOPDelay();
	device->cspin_ctrl_f(NRF24L01_CS_NOTACTIVE);

	return data;
}

/*
	@brief Go to the Standby-1 mode (low current mode).

	@param device : General struct instance of function pointers.
*/
void NRF24L01_GoStandby1(NRF24L01_GInst_t* device) {

	device->cepin_ctrl_f(NRF24L01_CE_NOTACTIVE);
}

/*
	@brief Go to the Standby-2 mode.

	@param device : General struct instance of function pointers.
*/
void NRF24L01_GoStandby2(NRF24L01_GInst_t* device) {

	device->cepin_ctrl_f(NRF24L01_CE_ACTIVE);
}

/*
	@brief Start transmission. 
	After transmition device goes to the Standby-1 mode if CE stay in LOW level.
*/
void NRF24L01_TxStart(NRF24L01_GInst_t* device) {

	device->cepin_ctrl_f(NRF24L01_CE_ACTIVE);
	/* Should be minimum 10uS. */
	for (uint8_t i = 100; i > 0; i--) {
		__NOP();
	}
	device->cepin_ctrl_f(NRF24L01_CE_NOTACTIVE);
}

/*
	@brief Start reception.
*/
void NRF24L01_RxStart(NRF24L01_GInst_t* device) {

	device->cepin_ctrl_f(NRF24L01_CE_ACTIVE);
}

/* Hardware dependent functions. */

/*
	@brief Mini delay with NOP.
*/
static void NRF24L01_NOPDelay(void) {
	for (uint8_t i = 100; i > 0; i--) {
		__NOP();
	}
}

/*
	@brief Chip enable control.

	@param state : May be NRF24L01_CE_ACTIVE, NRF24L01_CE_NOTACTIVE.
*/
void SPI_CECtrl(NRF24L01_CSPIN_STATE_t state) {

	if (state) {
		HAL_GPIO_WritePin(CE_GPIO_PORT, CE_GPIO_PIN, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(CE_GPIO_PORT, CE_GPIO_PIN, GPIO_PIN_RESET);
	}
}

/*
	@brief Chip select control.

	@param state : May be NRF24L01_CS_ACTIVE, NRF24L01_CS_NOTACTIVE.
*/
void SPI_CSCtrl(NRF24L01_CSPIN_STATE_t state) {

	if (state) {
		HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET);
	}
}

/*
	@brief Tx data to the SPI bus.
*/
void SPI_TxData(uint8_t* pdata, uint16_t size) {

	HAL_SPI_Transmit(&hspi1, pdata, size, 25);
}

/*
	@brief Rx data from the SPI bus.
*/
void SPI_RxData(uint8_t* pdata, uint16_t size) {

	HAL_SPI_Receive(&hspi1, pdata, size, 25);
}

/*
	@brief Delay.
*/
void NRF24L01_Delay(uint32_t delay) {

	HAL_Delay(delay);
}

/*
	@brief HAL SPI init.
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (spiHandle->Instance == SPI1) {
		/**SPI1 GPIO Configuration
		PB3     ------> SPI1_SCK
		PB4     ------> SPI1_MISO
		PB5     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* SPI1 interrupt Init */
		HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
}

/*
	@brief Init SPI module and some auxiliary pins.
*/
static void NRF24L01_InitPerif(void) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* SPI1 clock enable */
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

#ifdef USE_INTERRUPT_PIN
	/* Configure INT pin. */
	GPIO_InitStruct.Pin = INTERRUPT_INPUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(INTERRUPT_INPUT_PORT, &GPIO_InitStruct);
#endif /* USE_INTERRUPT_PIN */

	/* Configure CE pin. */
	GPIO_InitStruct.Pin = CE_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CE_GPIO_PORT, &GPIO_InitStruct);

	/* Configure CS pin. */
	GPIO_InitStruct.Pin = CS_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_PORT, &GPIO_InitStruct);

	/* Configure SPI. */
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi1);
}
