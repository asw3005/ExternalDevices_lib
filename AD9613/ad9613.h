/*
 * ad9613.h header file.
 *
 * Created on: Aug 23, 2023
 * Author: Supervisor
 */

#ifndef AD9613_H_
#define AD9613_H_

#include "stm32f1xx.h"

/* Chip ID. */
#define AD9613_CHIID 	0x83

/* Define chip select port and pin. */
#define AD9613_CS_PORT 	GPIOA
#define AD9613_CS_PIN 	GPIO_PIN_0



/*
 * @brief Register maps.
 */
typedef enum {
	/* Chip configuration registers. */
	AD9613_SPI_PORT_CFG,
	AD9613_CHIP_ID,
	AD9613_CHIP_GRADE,
	/* Channel index and transfer registers. */
	AD9613_CHANNEL_INDEX 				= 0x05,
	AD9613_TRANSFER 					= 0xFF,
	/* ADC functions. */
	AD9613_POWER_MODES 					= 0x08,
	AD9613_GLOBAL_CLOCK,
	AD9613_CLOCK_DIVIDE 				= 0x0B,
	AD9613_TEST_MODE 					= 0x0D,
	AD9613_OFFSET_ADJUST 				= 0x10,
	AD9613_OUTPUT_MODE 					= 0x14,
	AD9613_OUTPUT_ADJUST,
	AD9613_CLOCK_PHASE_CTRL,
	AD9613_DCO_OUTPUT_DELAY,
	AD9613_INPUT_SPAN_SEL,
	AD9613_USER_TEST_PATTERN1_LSB,
	AD9613_USER_TEST_PATTERN1_MSB,
	AD9613_USER_TEST_PATTERN2_LSB,
	AD9613_USER_TEST_PATTERN2_MSB,
	AD9613_USER_TEST_PATTERN3_LSB,
	AD9613_USER_TEST_PATTERN3_MSB,
	AD9613_USER_TEST_PATTERN4_LSB,
	AD9613_SYNC_CTRL 					= 0x3A

} AD9613_REG_MAPS_t;

/*
 * @brief Read/write command.
 */
typedef enum {
	AD9613_WRITE_CMD,
	AD9613_READ_CMD,

	AD9613_ONE_BYTE 					= 0x00,
	AD9613_TWO_BYTES,
	AD9613_THREE_BYTES,
	AD9613_STREAM_MODE

} AD9613_RW_CMD_t;

typedef enum {
	AD9613_250MSPS,
	AD9613_210MSPS,
	AD9613_170MSPS 						= 0x03

} AD9613_SPEED_GRADE_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);

/*
 * @brief
 */
typedef union {
	uint8_t SpiPortCfgReg;
	struct{
		uint8_t MUST_BE_ZERO 			: 1;
		uint8_t LSB_FIRST 				: 1;
		uint8_t SOFT_RST 				: 1;
		uint8_t MUST_BE_11 				: 2;
		uint8_t SOFT_RSTM 				: 1;
		uint8_t LSB_FIRSTM 				: 1;
		uint8_t MUST_BE_ZEROM 			: 1;
	};

} AD9613_SpiPortCfg_t;

/*
 * @brief Power modes  (local).
 */
typedef union {
	uint8_t PwrModesReg;
	struct{
		uint8_t INT_PWR_DOWN 			: 2;
		uint8_t RESERVED0 				: 3;
		uint8_t EXT_PWR_DOWN_PINF 		: 1;
		uint8_t RESERVED1 				: 2;
	};

} AD9613_PwrModes_t;

/*
 * @brief Clock divide.
 */
typedef union {
	uint8_t ClkDivideReg;
	struct{
		uint8_t CLK_DIV_RATIO 			: 3;
		uint8_t IN_CLK_DIV_PHASE_ADJ 	: 3;
		uint8_t RESERVED 				: 2;
	};

} AD9613_ClockDivide_t;

/*
 * @brief Test mode.
 */
typedef union {
	uint8_t TestModeReg;
	struct{
		uint8_t OUT_TEST_MODE 			: 4;
		uint8_t RST_PN_SHORT_GEN 		: 1;
		uint8_t RST_PN_LONG_GEN 		: 1;
		uint8_t RESERVED 				: 1;
		uint8_t USER_TEST_MODE_CTRL 	: 1;
	};

} AD9613_TestMode_t;

/*
 * @brief Offset adjust.
 */
typedef union {
	uint8_t OffsetAdjReg;
	struct{
		uint8_t OFFSET_ADJ 				: 5;
		uint8_t OFFSET_SIGN 			: 1;
		uint8_t RESERVED 				: 2;
	};

} AD9613_OffsetAdj_t;

/*
 * @brief Output mode.
 */
typedef union {
	uint8_t OutModeReg;
	struct{
		uint8_t OUT_FORMAT 				: 2;
		uint8_t OUT_INVERT 				: 1;
		uint8_t RESERVED0 				: 1;
		uint8_t OUT_EN_BAR 				: 1;
		uint8_t RESERVED1 				: 3;
	};

} AD9613_OutMode_t;

/*
 * @brief Clock phase control.
 */
typedef union {
	uint8_t ClkPhaseCtrlReg;
	struct{
		uint8_t RESERVED0 				: 5;
		uint8_t ODD_EVEN_OUT_EN 		: 1;
		uint8_t RESERVED1 				: 1;
		uint8_t INVERT_DCO_CLK 			: 1;
	};

} AD9613_ClkPhaseCtrl_t;

/*
 * @brief DCO output delay control.
 */
typedef union {
	uint8_t DcoOutDelayReg;
	struct{
		uint8_t DCO_CLK_DELAY 			: 5;
		uint8_t RESERVED 				: 2;
		uint8_t EN_DCO_CLK_DELAY 		: 1;
	};

} AD9613_DcoOutDelay_t;

/*
 * @brief Input span select.
 */
typedef union {
	uint8_t InputSpanSelReg;
	struct{
		uint8_t IN_SPAN 				: 4;
		uint8_t SPAN_SIGN 			: 1;
		uint8_t RESERVED 				: 3;
	};

} AD9613_InSpanSel_t;

/*
 * @brief Sync control.
 */
typedef union {
	uint8_t SyncControlReg;
	struct{
		uint8_t MASTER_SYNC_BUFF_EN 	: 1;
		uint8_t CLK_DIV_SYNC_EN 		: 1;
		uint8_t CLK_DIV_NEXT_SYNC_EN 	: 1;
		uint8_t RESERVED 				: 5;
	};

} AD9613_SyncCtrl_t;

/*
 * @brief Instruction byte.
 */
typedef struct {
	union {
		uint16_t InstrByte;
		struct {
			uint16_t REG_ADDRESS 			: 13;
			uint16_t DATA_LENGTH_W0W1 		: 2;
			/* 1 - read, 0 - write. */
			uint16_t READ_WRITE 			: 1;
		};
	};
	uint8_t Data[8];

} AD9613_RxTxData_t;

/*
 * @brief General struct.
 */
typedef struct {

	uint8_t ChipId;
	uint8_t ChipGrade;
	AD9613_RxTxData_t RxTxData;

	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr spi_rx_fp;
	rxtx_fptr spi_tx_fp;

} AD9613_GStr_t;


/* Public function prototypes. */
uint8_t AD9613_GetChipId(void);
uint8_t AD9613_GetChipGrade(void);
uint8_t AD9613_GetRstBitState(void);
uint8_t AD9613_GetSoftTxBitState(void);

void AD9613_StartSoftTx(void);
void AD9613_OutputAdj(uint8_t OutAdj);
void AD9613_EnDisDcs(uint8_t EnDisDcs);
void AD9613_ChSelect(uint8_t ChannelNumber);
void AD9613_OffsetAdj(int8_t OffsetAdjInLsb);
void AD9613_InVoltageSel(int8_t InVoltageSel);
void AD9613_SpiPortCfg(uint8_t LsbFirst, uint8_t SoftReset);
void AD9613_PwrModes(uint8_t IntPwrDown, uint8_t ExtPwrDownPinf);
void AD9613_ClkPhaseCtrl(uint8_t OddEvenMode, uint8_t InvertDcoClk);
void AD9613_DcoOutDelay(uint8_t EnDcoClkDelay, uint8_t DcoClkDelay);
void AD9613_ClockDivide(uint8_t ClkDivRatio, uint8_t InClkDivPhaseAdj);
void AD9613_OutputMode(uint8_t OutFormat, uint8_t OutInvert, uint8_t OutEnBar);
void AD9613_SyncCtrl(uint8_t MasterSyncBuffEn, uint8_t ClkDivSyncEn, uint8_t ClkDivNextSyncOnly);
void AD9613_TestMode(uint8_t OutTestMode, uint8_t RstPnShortGen, uint8_t RstPnLongGen, uint8_t UserTestModeCtrl);
void AD9613_SetUserTestPattern(uint16_t UserPattern1, uint16_t UserPattern2, uint16_t UserPattern3, uint8_t UserPattern4);

void AD9613_SpiRxData(uint8_t *pData, uint8_t Size);
void AD9613_SpiTxData(uint8_t *pData, uint8_t Size);

#endif /* AD9613_H_ */
