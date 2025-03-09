/*
 * FpgaCtrl.h header file.
 *
 * Created on: Oct 20, 2024
 * Author: asw3005
 */

#ifndef FPGACTRL_H_
#define FPGACTRL_H_

#include "stm32f4xx.h"

/* Read tag. */
#define SPI_READ			1
/* Write tag. */
#define SPI_WRITE			0
/* Data tag to R/W operation. */
#define SPI_DATA			1
/* Command tag to R/W operation. */
#define SPI_COMMAND			2
/* Sample data counter tag to R. */
#define SPI_SAMPLECNT		3
/* Sample test data counter tag to R test data. */
#define SPI_SCNTTEST		4
/* Sample ADC test data tag to R ADC test data. */
#define SPI_ADCRTEST		5
/* Read FIFO0 and FIFO1. */
#define SPI_RFIFO0			6
#define SPI_RFIFO1			7

#define DATA_IN_MAX16		(4*10)
#define DATA_OUT_MAX32		16
#define ADC_SAMPLE			32 //8192
#define ADC_SAMPLE_BUFF0	32 //8192
#define ADC_SAMPLE_BUFF1	32 //8192

/* Ports' selection. */
#define FPGA_FIFOCLK_PIN 	GPIO_PIN_3
#define FPGA_FIFOCLK_PORT 	GPIOE
#define FPGA_GRESET_PIN 	GPIO_PIN_13
#define FPGA_GRESET_PORT 	GPIOC
#define FPGA_ACQSTART_PIN 	GPIO_PIN_14
#define FPGA_ACQSTART_PORT 	GPIOC
#define FPGA_FIFOSEL_PIN 	GPIO_PIN_0
#define FPGA_FIFOSEL_PORT 	GPIOF
#define FPGA_FIFOSCLR_PIN 	GPIO_PIN_1
#define FPGA_FIFOSCLR_PORT 	GPIOF

/* Additional signals for a while. */
#define FPGA_ADDACQSTART_PIN 	GPIO_PIN_3
#define FPGA_ADDACQSTART_PORT 	GPIOF
#define FPGA_ADDGRESET_PIN 		GPIO_PIN_4
#define FPGA_ADDGRESET_PORT 	GPIOF

/* SPI4. */
#define FPGA_NSS_PIN 		GPIO_PIN_4
#define FPGA_CLK_PIN 		GPIO_PIN_2
#define FPGA_MOSI_PIN		GPIO_PIN_6
#define FPGA_MISO_PIN		GPIO_PIN_5
#define FPGA_NSS_PORT		GPIOE
#define FPGA_CLK_PORT 		GPIOE
#define FPGA_MOSI_PORT		GPIOE
#define FPGA_MISO_PORT		GPIOE

/*
 * @brief Register maps.
 */
typedef enum {
	FPGA_CTRL_REG,
	FPGA_PERIOD_LIMITER_VAL,
	FPGA_FSTART_TIM_VAL,
	FPGA_SSTART_TIM_VAL,
	FPGA_FSTOP_TIM_VAL,
	FPGA_SSTOP_TIM_VAL,
	FPGA_THR_START_HIGH_VAL,
	FPGA_THR_START_LOW_VAL,
	FPGA_THR0_STOP_VAL,
	FPGA_THR1_STOP_VAL,
	FPGA_RESERVED0,
	FPGA_RESERVED1,
	FPGA_RESERVED2,
	FPGA_RESERVED4,
	FPGA_RESERVED5,
	FPGA_RESERVED6

} FPGA_REG_MAPS_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);



/*
 * @brief SPI command word.
 */
typedef union {
	uint32_t SpiCmdWord;
	struct {
		uint32_t SPI_DUMMY		: 16;
		uint32_t SPI_ADDR 		: 5;
		uint32_t SPI_RSVD5_10 	: 6;
		uint32_t SPI_DC 		: 4;
		uint32_t SPI_RW 		: 1;
	};

} SPI_CmdWord_t;

/*
 * @brief FPGA control register.
 */
typedef union {
	uint32_t FpgaCtrlReg;
	struct {
		uint32_t START_SEL_BIT 	: 1;
		uint32_t STOP_SEL_BIT 	: 1;
		uint32_t THR_START_CH 	: 1;
		uint32_t THR_START_HL 	: 1;
		uint32_t PRG_START_BIT 	: 1;
		uint32_t RSVD5_29 		: 25;
		uint32_t SYNC_BIT 		: 1;
		uint32_t RST_BIT 		: 1;
	};

} FPGA_CtrlReg_t;

/*
 * @brief Sample counters.
 */
typedef struct {
	uint16_t Fifo0Cnt;
	uint16_t Fifo1Cnt;

} FPGA_SampleCnt_t;

/*
 * @brief ADC data.
 */
typedef struct {

	uint16_t TestDataBuff[ADC_SAMPLE];
	uint16_t Buff0[ADC_SAMPLE_BUFF0];
	uint16_t Buff1[ADC_SAMPLE_BUFF1];

} FPGA_ADCData_t;

/*
 * @brief General struct.
 */
typedef struct {

	uint16_t DataIn[DATA_IN_MAX16];
	SPI_CmdWord_t SpiCmdWord;
	uint32_t DataOut[DATA_OUT_MAX32];

	/* Read sample complete. */
	uint8_t IsReadADCCmpl;
	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr spi_rx_fp;
	rxtx_fptr spi_tx_fp;

} FPGA_GStr_t;


/* Public function prototypes. */

void FPGA_Init(void);
void FPGA_RstFifo(void);
FPGA_SampleCnt_t* FPGA_ReadSampleData(void);
FPGA_SampleCnt_t* FPGA_ReadTestSampleData(void);

void FPGA_ReadTestAdcData(uint8_t ReadCnt, uint8_t* Buff);
void FPGA_ReadAdcData(uint16_t ReadDataNumber, uint8_t FifoNumber, uint8_t* Buff, uint8_t Size);

void FPGA_WriteReg(uint8_t Address, uint32_t Data);
void FPGA_WriteCtrlReg(uint8_t StartSelBit, uint8_t StopSelBit, uint8_t ThrStartCh, uint8_t ThrStartHl, uint8_t PrgStartBit, uint8_t SyncBit,
						uint8_t RstBit);

#endif /* FPGACTRL_H_ */
