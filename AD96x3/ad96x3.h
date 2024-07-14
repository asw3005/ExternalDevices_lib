/*
 * AD96x3.h header file.
 *
 * Created on: Aug 23, 2023
 * Author: asw3005
 */

#ifndef AD96x3_H_
#define AD96x3_H_

#include "stm32f4xx.h"

/* Chip ID. */
#define AD96x3_CHIPID 	0x83
#define AD9643_CHIPID 	0x82

/* Define chip select port and pin. */
#define AD96x3_CS_PORT 	GPIOB
#define AD96x3_CS_PIN 	GPIO_PIN_4

/* Ports' selection. */
#define AD96x3_NSS_PIN 					GPIO_PIN_4
#define AD96x3_CLK_PIN 					GPIO_PIN_3
#define AD96x3_DATA_INOUT_PIN			GPIO_PIN_5
#define AD96x3_NSS_PORT					GPIOB
#define AD96x3_CLK_PORT 				GPIOB
#define AD96x3_DATA_INOUT_PORT			GPIOB

/* Significant data bits. */
#define AD96x3_CMD_WORD_SIZE			2
#define AD96x3_BIT_NUMBER				8
#define AD96x3_BIT_MASK					0x80


/*
 * @brief Register maps.
 */
typedef enum {
	/* Chip configuration registers. */
	AD96x3_SPI_PORT_CFG,
	AD96x3_CHIP_ID,
	AD96x3_CHIP_GRADE,
	/* Channel index and transfer registers. */
	AD96x3_CHANNEL_INDEX 				= 0x05,
	AD96x3_TRANSFER 					= 0xFF,
	/* ADC functions. */
	AD96x3_POWER_MODES 					= 0x08,
	AD96x3_GLOBAL_CLOCK,
	AD96x3_CLOCK_DIVIDE 				= 0x0B,
	AD96x3_TEST_MODE 					= 0x0D,
	AD96x3_OFFSET_ADJUST 				= 0x10,
	AD96x3_OUTPUT_MODE 					= 0x14,
	AD96x3_OUTPUT_ADJUST,
	AD96x3_CLOCK_PHASE_CTRL,
	AD96x3_DCO_OUTPUT_DELAY,
	AD96x3_INPUT_SPAN_SEL,
	AD96x3_USER_TEST_PATTERN1_LSB,
	AD96x3_USER_TEST_PATTERN1_MSB,
	AD96x3_USER_TEST_PATTERN2_LSB,
	AD96x3_USER_TEST_PATTERN2_MSB,
	AD96x3_USER_TEST_PATTERN3_LSB,
	AD96x3_USER_TEST_PATTERN3_MSB,
	AD96x3_USER_TEST_PATTERN4_LSB,
	AD96x3_USER_TEST_PATTERN4_MSB,
	AD96x3_SYNC_CTRL 					= 0x3A,
	AD96x3_UPDATE_REG					= 0xFF

} AD96x3_REG_MAPS_t;

/*
 * @brief Read/write command.
 */
typedef enum {
	AD96x3_WRITE_CMD,
	AD96x3_READ_CMD,

	AD96x3_ONE_BYTE 					= 0x00,
	AD96x3_TWO_BYTES,
	AD96x3_THREE_BYTES,
	AD96x3_STREAM_MODE

} AD96x3_RW_CMD_t;

typedef enum {
	AD96x3_250MSPS,
	AD96x3_210MSPS,
	AD96x3_170MSPS 						= 0x03

} AD96x3_SPEED_GRADE_t;

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

} AD96x3_SpiPortCfg_t;

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

} AD96x3_PwrModes_t;

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

} AD96x3_ClockDivide_t;

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

} AD96x3_TestMode_t;

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

} AD96x3_OffsetAdj_t;

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

} AD96x3_OutMode_t;

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

} AD96x3_ClkPhaseCtrl_t;

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

} AD96x3_DcoOutDelay_t;

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

} AD96x3_InSpanSel_t;

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

} AD96x3_SyncCtrl_t;

/*
 * @brief Instruction byte.
 */
typedef struct __attribute__((aligned(1), packed)) {
	union {
		uint16_t InstrByte;
		struct {
			uint16_t REG_ADDRESS 			: 13;
			uint16_t DATA_LENGTH_W0W1 		: 2;
			/* 1 - read, 0 - write. */
			uint16_t READ_WRITE 			: 1;
		};
	};
	uint8_t Data[9];

} AD96x3_RxTxData_t;

/*
 * @brief General struct.
 */
typedef struct {

	uint8_t ChipId;
	uint8_t ChipGrade;
	AD96x3_RxTxData_t RxTxData;

	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr spi_rx_fp;
	rxtx_fptr spi_tx_fp;

} AD96x3_GStr_t;


/* Public function prototypes. */
void AD96x3_Init(void);

uint8_t AD96x3_GetChipId(void);
uint8_t AD96x3_GetChipGrade(void);
uint8_t AD96x3_GetRstBitState(void);
uint8_t AD96x3_GetSoftTxBitState(void);

void AD96x3_StartSoftTx(void);
void AD96x3_OutputAdj(uint8_t OutAdj);
void AD96x3_EnDisDcs(uint8_t EnDisDcs);
void AD96x3_ChSelect(uint8_t ChannelNumber);
void AD96x3_OffsetAdj(int8_t OffsetAdjInLsb);
void AD96x3_InVoltageSel(int8_t InVoltageSel);
void AD96x3_SpiPortCfg(uint8_t LsbFirst, uint8_t SoftReset);
void AD96x3_PwrModes(uint8_t IntPwrDown, uint8_t ExtPwrDownPinf);
void AD96x3_ClkPhaseCtrl(uint8_t OddEvenMode, uint8_t InvertDcoClk);
void AD96x3_DcoOutDelay(uint8_t EnDcoClkDelay, uint8_t DcoClkDelay);
void AD96x3_ClockDivide(uint8_t ClkDivRatio, uint8_t InClkDivPhaseAdj);
void AD96x3_OutputMode(uint8_t OutFormat, uint8_t OutInvert, uint8_t OutEnBar);
void AD96x3_SyncCtrl(uint8_t MasterSyncBuffEn, uint8_t ClkDivSyncEn, uint8_t ClkDivNextSyncOnly);
void AD96x3_TestMode(uint8_t OutTestMode, uint8_t RstPnShortGen, uint8_t RstPnLongGen, uint8_t UserTestModeCtrl);
void AD96x3_SetUserTestPattern(uint16_t UserPattern1, uint16_t UserPattern2, uint16_t UserPattern3, uint8_t UserPattern4);

#endif /* AD96x3_H_ */
