/*
 * FpgaCtrl.c source file.
 *
 * Created on: Oct 20, 2024
 * Author: asw3005
 */

#include "FpgaCtrl.h"
#include "stm32f4xx_hal.h"
#include "main.h"

/* External variables. */
extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;

/* Private variables. */
static SPI_HandleTypeDef* FPGASpi = &hspi4;
//static DMA_HandleTypeDef* FPGASpiDma = &hdma_spi4_rx;

/* Private function prototypes. */
static void FPGA_SpiRxData(uint8_t *pData, uint8_t Size);
static void FPGA_SpiTxData(uint8_t *pData, uint8_t Size);
static void SPI_RxDmaFullCallback(SPI_HandleTypeDef *hspi);

/* Init general struct. */
static FPGA_GStr_t FPGA_Reg = {
		.delay_fp = HAL_Delay,
		.spi_rx_fp = FPGA_SpiRxData,
		.spi_tx_fp = FPGA_SpiTxData
};

/* ADC samples data struct. */
static FPGA_ADCData_t ADCData_inst;



/*
 * @brief Init FPGA registers.
 */
void FPGA_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = FPGA_NSS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(FPGA_NSS_PORT, &GPIO_InitStruct);
	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
	/* Clear SPI command and timers. */
	//HAL_GPIO_WritePin(FPGA_CLK_PORT, FPGA_CLK_PIN, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(FPGA_CLK_PORT, FPGA_CLK_PIN, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(FPGASpi, pData, Size, 25);
	/* Register SPI Receive callback. */
	/* Callback's register functions. */
	//HAL_DMA_RegisterCallback(FPGASpiDma, HAL_DMA_XFER_CPLT_CB_ID, SPI_RxDmaFullCallback);
	HAL_SPI_RegisterCallback(FPGASpi, HAL_SPI_RX_COMPLETE_CB_ID, SPI_RxDmaFullCallback);


	/* Reset FPGA registers(ADDRST - additional reset pin). */
	HAL_Delay(1000);
	HAL_GPIO_WritePin(DATA_SYNC_GPIO_Port, DATA_SYNC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADDRST_GPIO_Port, ADDRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(ADDRST_GPIO_Port, ADDRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);


//	/* Timer clock is 100MHz, so we have 10ns period. */
//	FPGA_WriteReg(FPGA_PERIOD_LIMITER_VAL, 100000);
//	FPGA_WriteReg(FPGA_FSTART_TIM_VAL, 10100);
//	FPGA_WriteReg(FPGA_SSTART_TIM_VAL, 15000);
//	/* 2000 * 10ns (100MHz timer clock frequency) = 20_000 ns,
//	 * 20_000ns : 5ns (200MHz sample rate) =  4000 samples,
//	 * 4000 * 4 (2 samples of 2 bytes each) = 16000 bytes.
//	*/
//	FPGA_WriteReg(FPGA_FSTOP_TIM_VAL, 1024);
//	FPGA_WriteReg(FPGA_SSTOP_TIM_VAL, 1024);
//	/* For the start acquisition module. */
//	FPGA_WriteReg(FPGA_THR_START_HIGH_VAL, 3128);
//	FPGA_WriteReg(FPGA_THR_START_LOW_VAL, 896);
//	/* One sample is 128 bit wide, so it's 8 samples of 16 bits each. (128*256 = 4096 byte). Now we have maximum 1024 samples of 128 bits. */
//	FPGA_WriteReg(FPGA_THR0_STOP_VAL, 128);
//	FPGA_WriteReg(FPGA_THR1_STOP_VAL, 128);

	while(1) {

		/* Timer clock is 100MHz, so we have 10ns period. */
		FPGA_WriteReg(FPGA_PERIOD_LIMITER_VAL, 100000);
		FPGA_WriteReg(FPGA_FSTART_TIM_VAL, 10100);
		FPGA_WriteReg(FPGA_SSTART_TIM_VAL, 15000);
		/* 2000 * 10ns (100MHz timer clock frequency) = 20_000 ns,
		 * 20_000ns : 5ns (200MHz sample rate) =  4000 samples,
		 * 4000 * 4 (2 samples of 2 bytes each) = 16000 bytes.
		*/
		FPGA_WriteReg(FPGA_FSTOP_TIM_VAL, 1024);
		FPGA_WriteReg(FPGA_SSTOP_TIM_VAL, 1024);
		/* For the start acquisition module. */
		FPGA_WriteReg(FPGA_THR_START_HIGH_VAL, 3128);
		FPGA_WriteReg(FPGA_THR_START_LOW_VAL, 896);
		/* One sample is 128 bit wide, so it's 8 samples of 16 bits each. (128*256 = 4096 byte). Now we have maximum 1024 samples of 128 bits. */
		FPGA_WriteReg(FPGA_THR0_STOP_VAL, 128);
		FPGA_WriteReg(FPGA_THR1_STOP_VAL, 128);

		FPGA_WriteCtrlReg(0, 0, 0, 0, 1, 1, 0);


		/* Sync the data (~450ns width). */
		//HAL_GPIO_WritePin(DATA_SYNC_GPIO_Port, DATA_SYNC_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(DATA_SYNC_GPIO_Port, DATA_SYNC_Pin, GPIO_PIN_RESET);

		/* Read numbers of data to read. */
		FPGA_SampleCnt_t* CounterData;
		//CounterData = FPGA_ReadSampleData();
		CounterData = FPGA_ReadTestSampleData();
		__NOP();



		/* Read FIFO data. */
		FPGA_ReadAdcData(4, SPI_RFIFO0, (uint8_t*)&ADCData_inst.Buff0, 0);
		FPGA_ReadAdcData(4, SPI_RFIFO1, (uint8_t*)&ADCData_inst.Buff1, 0);
		/* Read test data. */
		//FPGA_ReadTestAdcData(3, (uint8_t*)&ADCData_inst.TestDataBuff);

		/* Reset FIFO. */
		//FPGA_RstFifo();
		__NOP();
//		HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_RESET);

	}





}

/*
 * @brief Write FPGA control register.
 */
void FPGA_WriteCtrlReg(uint8_t StartSelBit, uint8_t StopSelBit, uint8_t ThrStartCh, uint8_t ThrStartHl, uint8_t PrgStartBit, uint8_t SyncBit,
						uint8_t RstBit) {

	FPGA_CtrlReg_t CtrlReg;

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_WRITE;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_COMMAND;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;
	/* Data prepare. */
	CtrlReg.START_SEL_BIT = StartSelBit;
	CtrlReg.STOP_SEL_BIT = StopSelBit;
	CtrlReg.THR_START_CH = ThrStartCh;
	CtrlReg.THR_START_HL = 0;
	CtrlReg.PRG_START_BIT = PrgStartBit;
	CtrlReg.SYNC_BIT = SyncBit;
	CtrlReg.RST_BIT = RstBit;
	CtrlReg.RSVD5_29 = 0;
	FPGA_Reg.DataOut[0] = CtrlReg.FpgaCtrlReg;
	//FPGA_Reg.DataOut[0] = 50000;

	FPGA_SpiTxData((uint8_t*)&FPGA_Reg.SpiCmdWord, 4);
}

/*
 * @brief Write FPGA registers' data.
 *
 * @param Address : address of register you need
 * @param Data : data to write
 *
 */
void FPGA_WriteReg(uint8_t Address, uint32_t Data) {

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_WRITE;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_COMMAND;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = Address;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;
	/* Data prepare. */
	FPGA_Reg.DataOut[0] = Data;

	FPGA_SpiTxData((uint8_t*)&FPGA_Reg.SpiCmdWord, 4);

}

/*
 * @brief Read test sample count data.
 */
FPGA_SampleCnt_t* FPGA_ReadTestSampleData(void) {

	uint32_t RawData;
	static FPGA_SampleCnt_t SampleCounters;

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_READ;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_SCNTTEST;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;
	/* Assert the CS and transmit data counter read command. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(FPGASpi, (uint8_t*)&FPGA_Reg.SpiCmdWord, 2, 5);

	//FPGA_Reg.delay_fp(10);
	/* 32-bit dummy read. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	/* 32-bit sample counter read. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	//FPGA_Reg.delay_fp(10);
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);

	SampleCounters.Fifo0Cnt = RawData;
	SampleCounters.Fifo1Cnt = RawData >> 16;

	return &SampleCounters;
}

/*
 * @brief Read sample count data.
 */
FPGA_SampleCnt_t* FPGA_ReadSampleData(void) {

	uint32_t RawData;
	static FPGA_SampleCnt_t SampleCounters;

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_READ;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_SAMPLECNT;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;

	/* Assert the CS and transmit test data counter read command. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(FPGASpi, (uint8_t*)&FPGA_Reg.SpiCmdWord, 2, 5);

	//FPGA_Reg.delay_fp(10);
	/* 32-bit dummy read. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	/* 32-bit sample counter read. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	//FPGA_Reg.delay_fp(10);
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);

	SampleCounters.Fifo0Cnt = RawData;
	SampleCounters.Fifo1Cnt = RawData >> 16;

	return &SampleCounters;
}

/*
 * @brief Read test ADC data.
 *
 * @param ReadCnt 	: number of test data read sequences.
 * @param Buff 		: data storage to write to.
 */
void FPGA_ReadTestAdcData(uint8_t ReadCnt, uint8_t* Buff) {

	uint32_t RawData;

	/* Check read counter. */
	if(ReadCnt == 0) { ReadCnt = 1; }

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_READ;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_ADCRTEST;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;

	/* Activate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	/* Write command. Blocking. */
	HAL_SPI_Transmit(FPGASpi, (uint8_t*)&FPGA_Reg.SpiCmdWord, 2, 5);
	/* 32-bit dummy read. Blocking. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	/* 128-bit test data read. Nonblocking.*/
	HAL_SPI_Receive_DMA(FPGASpi, Buff, 8*ReadCnt);
	//HAL_SPI_Receive_DMA(FPGASpi, Buff, 8*ReadCnt);
	while(!FPGA_Reg.IsReadADCCmpl) {
		/* Timeout is here :) */
		__NOP();
	}
	/* Deactivate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
	FPGA_Reg.IsReadADCCmpl = 0;
	__NOP();
}

/*
 * @brief Read ADC data.
 *
 * @param FifoNumber : SPI_RFIFO0 or SPI_RFIFO1.
 * @param Buff : data storage to write to.
 * @param Size : amount of data in 16-bit words times 4 (4 16-bit data sample by one ADC clock).
 */
void FPGA_ReadAdcData(uint16_t ReadDataNumber, uint8_t FifoNumber, uint8_t* Buff, uint8_t Size) {

	uint32_t RawData;

	/* Check the number of read data. */
	if(ReadDataNumber == 0) { ReadDataNumber = 1; }

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_READ;
	if(FifoNumber > SPI_RFIFO1) { FifoNumber = SPI_RFIFO0; }
	FPGA_Reg.SpiCmdWord.SPI_DC = FifoNumber;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;

	/* Activate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	/* Write command. Blocking. */
	HAL_SPI_Transmit(FPGASpi, (uint8_t*)&FPGA_Reg.SpiCmdWord, 2, 5);
	/* 32-bit dummy read. */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	/* Read data. */
	HAL_SPI_Receive_DMA(FPGASpi, Buff, 8*ReadDataNumber);
	while(!FPGA_Reg.IsReadADCCmpl) {
		/* Timeout is here :) */
		__NOP();
	}
	/* Deactivate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
	FPGA_Reg.IsReadADCCmpl = 0;
	__NOP();
}

/* Hardware dependent functions. */

/*
 * @brief Resets FIFO0 and FIFO1 to its initial state.
 * 			FIFO sample counter = 0, FIFO data = 0xFF.
 */
void FPGA_RstFifo(void) {

	/* Reset FIFO. */
	HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_RESET);

}

/*
 * @brief Receive data from the chip.
 */
static void FPGA_SpiRxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);

	//HAL_SPI_Receive(FPGASpi, pData, Size, 25);
	HAL_SPI_Receive_DMA(FPGASpi, pData, Size);

	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Transmit data to the chip.
 */
static void FPGA_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(FPGASpi, pData, Size, 25);
	//HAL_SPI_Transmit_DMA(FPGASpi, pData, Size);

	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief DMA full data read complete callback.
 */
static void SPI_RxDmaFullCallback(SPI_HandleTypeDef *hspi) {

	FPGA_Reg.IsReadADCCmpl = 1;

}

