/*
 * FpgaCtrl.c source file.
 *
 * Created on: Oct 20, 2024
 * Author: asw3005
 */

#include "FpgaCtrl.h"
#include "stm32f4xx_hal.h"
#include "main.h"

#include "DataExchUART.h"
#include "FreeRTOS.h"
#include "queue.h"

/* External variables. */
extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern QueueHandle_t CmdRxPool_qh;

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

	/* */
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

	/* Callback's register functions. */
	HAL_SPI_RegisterCallback(FPGASpi, HAL_SPI_RX_COMPLETE_CB_ID, SPI_RxDmaFullCallback);


	/* Reset FPGA registers(ADDRST - additional reset pin). May should wait more time because of SFP module that gives the clock. */
	HAL_Delay(100);
	FPGA_RstFpgaRegisters();
	HAL_Delay(100);

	FPGA_GetSetSettings()->FPGA_PeriodLimiter = 10000000;
	FPGA_GetSetSettings()->FPGA_FStartTim = 10100;
	FPGA_GetSetSettings()->FPGA_SStartTim = 10100;
	FPGA_GetSetSettings()->FPGA_FStopTim = 1024;
	FPGA_GetSetSettings()->FPGA_SStopTim = 1024;
	FPGA_GetSetSettings()->FPGA_ThrStartHigh = 3128;
	FPGA_GetSetSettings()->FPGA_ThrStartLow = 896;
	FPGA_GetSetSettings()->FPGA_Thr0Stop = DEFAULT_RSAMPLE;
	FPGA_GetSetSettings()->FPGA_Thr1Stop = DEFAULT_RSAMPLE;

	/* Timer clock is 100MHz, so we have 10ns period, 100 ms. */
	FPGA_WriteReg(FPGA_PERIOD_LIMITER_VAL, FPGA_GetSetSettings()->FPGA_PeriodLimiter);
	FPGA_WriteReg(FPGA_FSTART_TIM_VAL, FPGA_GetSetSettings()->FPGA_FStartTim);
	FPGA_WriteReg(FPGA_SSTART_TIM_VAL, FPGA_GetSetSettings()->FPGA_SStartTim);
	/* 2000 * 10ns (100MHz timer clock frequency) = 20_000 ns,
	 * 20_000ns : 5ns (200MHz sample rate) =  4000 samples,
	 * 4000 * 4 (2 samples of 2 bytes each) = 16000 bytes.
	*/
	FPGA_WriteReg(FPGA_FSTOP_TIM_VAL, FPGA_GetSetSettings()->FPGA_FStopTim);
	FPGA_WriteReg(FPGA_SSTOP_TIM_VAL, FPGA_GetSetSettings()->FPGA_SStopTim);
	/* For the start acquisition module. */
	FPGA_WriteReg(FPGA_THR_START_HIGH_VAL, FPGA_GetSetSettings()->FPGA_ThrStartHigh);
	FPGA_WriteReg(FPGA_THR_START_LOW_VAL, FPGA_GetSetSettings()->FPGA_ThrStartLow);
	/* One sample is 128 bit wide, so it's 8 samples of 16 bits each. (128*256 = 4096 byte). Now we have maximum 1024 samples of 128 bits. */
	FPGA_WriteReg(FPGA_THR0_STOP_VAL, FPGA_GetSetSettings()->FPGA_Thr0Stop);
	FPGA_WriteReg(FPGA_THR1_STOP_VAL, FPGA_GetSetSettings()->FPGA_Thr1Stop);

	/* Sync.  Init registers' update. */
	//FPGA_WriteCtrlReg(0, 0, 0, 0, 0, 1, 0);
	HAL_Delay(1);
	FPGA_UpdateRegisters();

	__NOP();
}

/*
 * @brief FPGA setting registers.
 */
FPGA_Settings_t* FPGA_GetSetSettings(void) {

	static FPGA_Settings_t FpgaSettings = { 0 };

	return &FpgaSettings;
}

/*
 * @brief Start update internal registers' values.
 */
void FPGA_UpdateRegisters(void) {

	/* Sync.  Init registers' update. */
	FPGA_WriteCtrlReg(0, 0, 0, 0, 0, 1, 0);
}

/*
 * @brief Software ADC sample acquisition start.
 */
void FPGA_StartAcq(void) {

	/* Software start. */
	FPGA_WriteCtrlReg(0, 0, 0, 0, 1, 0, 0);
}

/*
 * @brief Reset FIFO and start a new FIFO sample operation.
 */
FPGA_SampleCnt_t* FPGA_GetSample(void) {

	/* */
	FPGA_SampleCnt_t* CounterData;
	uint32_t TimeoutCnt = 0;
	uint8_t ReadErr = 0;

	/* Waiting read complete. */
	ReadErr = 0;
	//HAL_Delay(1);
	while(!FPGA_Reg.isFIFO0ReadCmpl || !FPGA_Reg.isFIFO1ReadCmpl) {
		/* Timeout is here :) */
		__NOP();
		TimeoutCnt++;
		if(TimeoutCnt == 2000000) {
			TimeoutCnt = 0;
			ReadErr = 1;
			break;
		}
	}

	if (ReadErr != 1) {

		CounterData = FPGA_ReadSampleData();

		if(CounterData->Fifo0Cnt > FIFO_DEPTH) {
			CounterData->Fifo0Cnt = DEFAULT_RSAMPLE;
		}

		if(CounterData->Fifo1Cnt > FIFO_DEPTH) {
			CounterData->Fifo1Cnt = DEFAULT_RSAMPLE;
		}


//		//FPGA_PipeDelayDummyRead();
//		//CounterData = FPGA_ReadSampleData();
//		FPGA_ReadAdcData(CounterData->Fifo0Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO0, (uint8_t*)&ADCData_inst.Buff0, 0);
//		FPGA_ReadAdcData(CounterData->Fifo1Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO1, (uint8_t*)&ADCData_inst.Buff1, 0);
//		//CounterData = FPGA_ReadSampleData();
//		__NOP();

		FPGA_PipeDelayDummyRead();
		//CounterData = FPGA_ReadSampleData();
		FPGA_ReadAdcData(CounterData->Fifo0Cnt, SPI_RFIFO0, (uint8_t*)&ADCData_inst.Buff0, 0);
		FPGA_ReadAdcData(CounterData->Fifo1Cnt, SPI_RFIFO1, (uint8_t*)&ADCData_inst.Buff1, 0);
		//CounterData = FPGA_ReadSampleData();
		__NOP();

	}


	/* Reset FIFO flags (for hardware reading in the future). */
	FPGA_Reg.isReadReqActive = 0;
	FPGA_Reg.isFIFO0ReadCmpl = 0;
	FPGA_Reg.isFIFO1ReadCmpl = 0;

	/* Reset FIFO. */
	FPGA_RstFifo();

	return CounterData;
}

/*
 * @brief Reading FIFO0 data.
 */
FPGA_ADCData_t* FPGA_GetFIfoData(void) {

	return &ADCData_inst;
}

/*
 * @brief Reading FIFO0 data.
 */
FPGA_ADCData_t* FPGA_GetFIfo0Data(void) {

	return &ADCData_inst;
}

/*
 * @brief Reading FIFO1 data.
 */
FPGA_ADCData_t* FPGA_GetFIfo1Data(void) {

	return &ADCData_inst;
}

/*
 * @brief Dummy read FIFO.
 *
 * @ret FPGA_SampleCnt_t : struct of sample numbers acquired from the ADC.
 */
FPGA_SampleCnt_t FPGA_PipeDelayDummyRead(void) {

	/* Read FIFO data. */
	uint32_t RawData;

	/* SPI command word prepare. */
	FPGA_Reg.SpiCmdWord.SPI_RW = SPI_READ;
	FPGA_Reg.SpiCmdWord.SPI_DC = SPI_RPIPE;
	FPGA_Reg.SpiCmdWord.SPI_ADDR = 0;
	FPGA_Reg.SpiCmdWord.SPI_RSVD5_10 = 0;

	/* Activate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_RESET);
	/* Write command. Blocking. */
	HAL_SPI_Transmit(FPGASpi, (uint8_t*)&FPGA_Reg.SpiCmdWord, 2, 5);
	/* 32-bit dummy read (generate some extra pipeline clocks for FIFO reading). */
	HAL_SPI_Receive(FPGASpi, (uint8_t*)&RawData, 2, 5);
	/* Deactivate CS. */
	HAL_GPIO_WritePin(FPGA_NSS_PORT, FPGA_NSS_PIN, GPIO_PIN_SET);

	return *(FPGA_ReadSampleData());
}

/*
 * @brief Test FPGA FIFO read.
 */
void FPGA_TestRead(void) {

	/* */
	FPGA_SampleCnt_t* CounterData;

	while(1) {

		/* Software start. */
		FPGA_WriteCtrlReg(0, 0, 0, 0, 1, 0, 0);
		HAL_Delay(1);
		/* Read numbers of data to read. */
		CounterData = FPGA_ReadSampleData();
		/* Read FIFO data. */
		//FPGA_PipeDelayDummyRead();
		CounterData = FPGA_ReadSampleData();
		__NOP();
		FPGA_ReadAdcData(CounterData->Fifo0Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO0, (uint8_t*)&ADCData_inst.Buff0, 0);
		FPGA_ReadAdcData(CounterData->Fifo1Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO1, (uint8_t*)&ADCData_inst.Buff1, 0);
		CounterData = FPGA_ReadSampleData();
		__NOP();

		/* Reset FIFO. */
		FPGA_RstFifo();
		HAL_Delay(1);
		__NOP();

		/* Software start. */
		FPGA_WriteCtrlReg(0, 0, 0, 0, 1, 0, 0);
		HAL_Delay(1);
		__NOP();
		CounterData = FPGA_ReadSampleData();
		//FPGA_PipeDelayDummyRead();
		CounterData = FPGA_ReadSampleData();
		FPGA_ReadAdcData(CounterData->Fifo0Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO0, (uint8_t*)&ADCData_inst.Buff0, 0);
		FPGA_ReadAdcData(CounterData->Fifo1Cnt + FIFO_PIPELINE_FACT, SPI_RFIFO1, (uint8_t*)&ADCData_inst.Buff1, 0);
		CounterData = FPGA_ReadSampleData();
		__NOP();
		HAL_Delay(10);

		/* Reset FIFO. */
		FPGA_RstFifo();
		HAL_Delay(1);
		__NOP();
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

	FPGA_GetSetSettings()->FPGA_CtrlReg = CtrlReg.FpgaCtrlReg;
	FPGA_Reg.DataOut[0] = FPGA_GetSetSettings()->FPGA_CtrlReg;

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
	static FPGA_SampleCnt_t SampleCounters = { 0 };

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
	static FPGA_SampleCnt_t SampleCounters = { 0 };

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

	SampleCounters.Fifo0Cnt = RawData >> 16;
	SampleCounters.Fifo1Cnt = RawData;

	return &SampleCounters;
}

/*
 * @brief Read test ADC data.
 *
 * @param ReadCnt 	: number of test data read sequences (FIFO depth).
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
 * @param ReadDataNumber : read data FIFO (max FIFO_DEPTH parameter).
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
//		if((FifoNumber != SPI_RFIFO1) && (FifoNumber != SPI_RFIFO0)) { FifoNumber = SPI_RFIFO0; }

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
 * @brief Resets internal registers.
 */
void FPGA_RstFpgaRegisters(void) {

	/* Reset registers. */
	HAL_GPIO_WritePin(ADDRST_GPIO_Port, ADDRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(ADDRST_GPIO_Port, ADDRST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

}

/*
 * @brief Resets FIFO0 and FIFO1 to its initial state.
 * 			FIFO sample counter = 0, FIFO data = 0xFF.
 */
void FPGA_RstFifo(void) {

	/* Disable interrupts from the external lines. */
//	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	/* Reset FIFO. */
	HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, FIFO_SCLR_Pin, GPIO_PIN_RESET);

	/* Manage and enable interrupts from the external lines. */
//	HAL_Delay(1);
//	__HAL_GPIO_EXTI_CLEAR_IT(WRFULL_FIFO0_Pin);
//	__HAL_GPIO_EXTI_CLEAR_IT(WRFULL_FIFO1_Pin);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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


/* Interrupt callbacks. */

/*
	@brief  EXTI line detection callbacks.
	@param  GPIO_Pin: Specifies the pins connected EXTI line
	@retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	CmdCode_t CmdCode = { 0 };

	if (GPIO_Pin == ADDR_SEL_Pin) {
		//xSemaphoreGiveFromISR(BtnPressed_sh, NULL);
	}

	if (GPIO_Pin == WRFULL_FIFO0_Pin) {
		FPGA_Reg.isFIFO0ReadCmpl = 1;

	} else if (GPIO_Pin == WRFULL_FIFO1_Pin) {
		FPGA_Reg.isFIFO1ReadCmpl = 1;

	}

	if(FPGA_Reg.isFIFO0ReadCmpl == 1 && FPGA_Reg.isFIFO1ReadCmpl == 1) {

		FPGA_Reg.isReadReqActive = 1;
		/* MCS_HardwareReq = 0x0801f858. */
		CmdCode.CommandCode = 0x0801f858;
		xQueueSendToBackFromISR(CmdRxPool_qh, &CmdCode, 0);
	}
}

/*
 * @brief DMA full data read complete callback.
 */
static void SPI_RxDmaFullCallback(SPI_HandleTypeDef *hspi) {

	FPGA_Reg.IsReadADCCmpl = 1;

}

