/*
 * si57x.h header file.
 *
 * Created on: Aug 28, 2023
 * Author: asw3005
 */

#ifndef SI57X_H_
#define SI57X_H_

#include "stm32f1xx.h"

/* I2C bus addresses. */
#define SI57x_ADDRESS				0x55
#define SI57x_ADDRESS_SHIFTED 		(SI57x_ADDRESS << 1)
#define SI57x_CURRENT_ADDR 			SI57x_ADDRESS_SHIFTED

/* SI57x frequency constants (MHz). */
#define INTERNAL_CRYSTAL_FREQ 		(double)114.285
#define INTERNAL_MIN_OSC_FREQ 		4850
#define INTERNAL_MAX_OSC_FREQ 		5670
#define FREQ_WINDOW_IN_PPM			3500
#define FREQ_FRACT_MULTIPLIER		(double)268435456.0
#define FREQ_COEFF_MHZ_TO_HZ		1000000
#define N1_DIV_MIN_VALUE			1
#define N1_DIV_MAX_VALUE			128
#define HS_DIV_MIN_VALUE			4
#define HS_DIV_MAX_VALUE			11
#define TRY_CNT_MAX_VALUE			(uint16_t)1408

/*
 * @brief Register maps.
 */
typedef enum {
	SI57x_HIGH_SPEED_N1_DIV 		= 7,
	SI57x_REF_FREQ_B3237,
	SI57x_REF_FREQ_B2431_MSB,
	SI57x_REF_FREQ_B1623_LSB,
	SI57x_REF_FREQ_B815_MSB,
	SI57x_REF_FREQ_B07_LSB,
	SI57x_HIGH_SPEED_N1_DIV7PPM,
	SI57x_REF_FREQ_7PPM_B3237,
	SI57x_REF_FREQ_7PPM_B2431_MSB,
	SI57x_REF_FREQ_7PPM_B1623_LSB,
	SI57x_REF_FREQ_7PPM_B815_MSB,
	SI57x_REF_FREQ_7PPM_B07_LSB,
	SI57x_RST_FREEZE_MEM_CTRL 		= 135,
	SI57x_FREEZE_DCO				= 137

} SI57x_REG_MAP_t;

/*
 * @brief Temperature stability of SI57x.
 */
typedef enum {
	SI570_7PPM,
	SI570_20PPM,
	SI570_50PPM,
	SI571_ALL

} SI57x_TempStab_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t MemAddr, uint8_t* pData, uint8_t Size);

/*
 * @brief High speed/N1 dividers.
 */
typedef union {
	uint8_t HiSpeedN1DivReg;
	struct {
		uint8_t N1_DIV_BIT6_2 		: 5;
		uint8_t HS_DIV 				: 3;
	};
} SI57x_HighSpeedN1Div_t;

/*
 * @brief Reference frequency and N1 divider.
 */
typedef struct {
	union {
		uint8_t RefFreqBit3732N1DivReg;
		struct {
			uint8_t REF_FREQ_BIT37_32	: 6;
			uint8_t N1_DIV_BIT1_0		: 2;
		};
	};

	uint8_t RefFreqBit31_24;
	uint8_t RefFreqBit23_16;
	uint8_t RefFreqBit15_8;
	uint8_t RefFreqBit7_0;

} SI57x_RefFreqN1Div_t;

/*
 * @brief Reset, freeze, memory control.
 */
typedef union {
	uint8_t RstFreezeMemCtrlReg;
	struct {
		uint8_t RECALL 				: 1;
		uint8_t RESERVED 			: 3;
		uint8_t FREEZE_VCADC 		: 1;
		uint8_t FREEZE_M 			: 1;
		uint8_t NEW_FREQ 			: 1;
		uint8_t RST_REG 			: 1;
	};

} SI57x_RstFreezeMemCtrl_t;

/*
 * @brief Internal register struct.
 */
typedef struct __attribute__((aligned(1), packed)) {

	SI57x_HighSpeedN1Div_t 		HighSpeedN1Div;
	SI57x_RefFreqN1Div_t 		RefFreqN1Div;

} SI57x_RawRegData_t;

/*
 * @brief Frequencies and divider table.
 */
typedef struct {
	/* Current frequency. */
	double CurrentFreq;

	/* Usual data. */
	uint8_t HighSpeedDivider;
	union {
		uint8_t N1Divider;
		struct {
			uint8_t N1_DIV_BIT1_0 	: 2;
			uint8_t N1_DIV_BIT6_2 	: 5;
			uint8_t ZERO 			: 1;
		};
	};
	union {
		uint64_t RefFrequency;
		struct {
			uint64_t ZERO_BIT63_38 			: 26;
			uint64_t RFREQ_BIT37_32 		: 6;
			uint64_t RFREQ_BIT31_24 		: 8;
			uint64_t RFREQ_BIT23_16 		: 8;
			uint64_t RFREQ_BIT15_8 			: 8;
			uint64_t RFREQ_BIT7_0 			: 8;
		};
	};

} SI57x_FreqDivTable_t;

/*
 * @brief Instruction byte.
 */
typedef struct {

	uint8_t Data[6];

} SI57x_RxTxData_t;

/*
 * @brief General struct.
 */
typedef struct {

	SI57x_RxTxData_t RxTxData;

	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr i2c_rx_fp;
	rxtx_fptr i2c_tx_fp;

} SI57x_GStr_t;


/* Public function prototypes. */
void SI57x_SetFreq(double NewFreq, uint8_t DevPpmCoeff);
void SI57x_FreezeDco(uint8_t FreezeDco);
void SI57x_RstFreezeMemCtrl(uint8_t Recall, uint8_t FreezeVcadc, uint8_t FreezeM, uint8_t NewFreq, uint8_t RstReg);

#endif /* SI57X_H_ */
