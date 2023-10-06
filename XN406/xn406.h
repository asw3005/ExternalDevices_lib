/*
 * xn406.h header file.
 *
 * Created on: Oct 2, 2023
 * Author: asw3005
 */

#ifndef XN406_H_
#define XN406_H_

#include "stm32f1xx.h"

/* Select hardware or software SPI. */
//#define XN406_SPI_HARD
#ifndef XN406_SPI_HARD
#define XN406_SPI_SOFT
#endif /* XN406_SPI_HARD */


/* XN406 some constants. */
#define XN406_CHIP_ID 				0x534744
#define XN406_VCO_ID				0

/* Pin definitions serial port. */
#define XN406_CS_PIN 				GPIO_PIN_0
#define XN406_CS_PORT 				GPIOA
#ifdef XN406_SPI_SOFT
#define XN406_CLK_PIN 				GPIO_PIN_0
#define XN406_CLK_PORT 				GPIOA
#define XN406_SDI_PIN 				GPIO_PIN_0
#define XN406_SDI_PORT 				GPIOA
#define XN406_SDO_PIN 				GPIO_PIN_0
#define XN406_SDO_PORT 				GPIOA
/* Rx Tx constants. */
#define XN406_BIT_NUMBER			8
#define XN406_W_BIT_NUMBER			7
#define XN406_BIT_MASK				0x80
#endif /* XN406_SPI_SOFT */

/*
 * @brief Registers' map.
 */
typedef enum {
	/* PLL register map. */
	XN406_ID 						= 0,
	XN406_CS_CTRL,
	XN406_REFDIV,
	XN406_FREQ_INTEGER_PART,
	XN406_FREQ_FRACTIONAL_PART,
	XN406_VCO_SPI,
	XN406_SD_CFG,
	XN406_LOCK_DETECT,
	XN406_ANALOG_EN,
	XN406_CHARGE_PUMP,
	XN406_AFC_CTRL,
	XN406_OFFSET_EN,
	XN406_EXACT_FREQ_MODE,
	XN406_GPO_SPI_RDIV 				= 0x0F,
	XN406_VCO_TUNE,
	XN406_INVALID_007FFF,
	XN406_LOCKED,
	XN406_INVALID_000000,
	/* VCO subsystem. */
	XN406_VCO_SUBSYS_ID				= 0,
	XN406_VCO_TUNING 				= 0,
	XN406_VCO_ENABLES,
	XN406_VCO_GAIN_DIV,
	XN406_VCO_CONFIG,
	/* Support only settings on the page 26 of the datasheet. */
	XN406_VCO_BIAS_CAL,
	XN406_VCO_CF_CAL,
	XN406_VCO_MSB_CAL

} XN406_REG_MAP_t;

/*
 * @brief Read/write bit state.
 */
typedef enum {
	XN406_WRITE,
	XN406_READ

} XN406_RW_STATE_t;


/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);
typedef void(*hrxtx_fptr)(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);

/* PLL REGISTER MAP */

/*
 * @brief RST register (default 0x02).
 */
typedef union {
	uint32_t RstReg;
	struct {
		uint32_t RST_CHIPEN_PIN_SEL 		: 1;
		uint32_t RST_CHIPEN_FROM_SPI 		: 1;
		uint32_t RESERVED23_2				: 22;
		/* Program to 0. */
		uint32_t RESERVED31_24				: 8;
	};
} XN406_RstReg_t;

/*
 * @brief VCO subsystem register (default 0x00).
 * Use if VcoAutocalReg[NO_VSPI_TRIG] = 0.
 */
typedef union {

	uint16_t SubsysVcoDataReg;
	struct {
		uint16_t VCO_SUBSYS_ID 				: 3;
		uint16_t VCO_SUBSYS_REG_ADDR 		: 4;
		uint16_t VCO_SUBSYS_DATA			: 9;
	};

} XN406_SubsysVco_t;

/*
 * @brief VCO SPI Register (default 0x00).
 * Use if VcoAutocalReg[NO_VSPI_TRIG] = 1.
 */
typedef union {

	uint16_t VcoSpiDataReg;
	struct {
		uint16_t VCO_SUBSYS_ID 				: 3;
		uint16_t VCO_SUBSYS_REG_ADDR 		: 4;
		uint16_t VCO_LOOP_CTRL				: 1;
		uint16_t VCO_CAPS					: 5;
		/* Program to 0. */
		uint16_t RESERVED13					: 1;
		uint16_t VCO_SELECT					: 2;
	};

} XN406_VcoSpi_t;

/*
 * @brief SD CFG register (default 0x00200B4A).
 */
typedef union {
	uint32_t SdSfgReg;
	struct {
		uint32_t SEED 						: 2;
		uint32_t ORDER 						: 2;
		/* Program to 4. */
		uint32_t RESERVED6_4				: 3;
		uint32_t FRAC_BYPASS 				: 1;
		uint32_t AUTO_SEED 					: 1;
		/* Program to 1. */
		uint32_t RESERVED9 					: 1;
		/* Program to 0. */
		uint32_t RESERVED10					: 1;
		uint32_t FRAC_EN					: 1;
		/* Program to 0. */
		uint32_t RESERVED15_12				: 4;
		/* Program to 3. */
		uint32_t RESERVED17_16				: 2;
		uint32_t RESERVED20_18				: 3;
		/* Program  to 0. */
		uint32_t AUTO_CLK_CONFIG 			: 1;
		/* Program to 0. */
		uint32_t RESERVED31_22				: 10;
	};
} XN406_SdCfg_t;

/*
 * @brief Lock detect register (default 0x0000014D).
 */
typedef union {
	uint32_t LockDetectReg;
	struct {
		uint32_t LKD_WINCNT_MAX 			: 3;
		uint32_t EN_INT_LOCK_DETECT 		: 1;
		/* Program to 0. */
		uint32_t RESERVED5_4 				: 2;
		/* Program to 1. */
		uint32_t RESERVED6			 		: 1;
		uint32_t LD_DIG_WIN_DUR 			: 3;
		uint32_t LD_DIG_TIM_FREQ_CTRL 		: 2;
		/* Program to 0. */
		uint32_t RESERVED12		 			: 1;
		uint32_t AUTORELOCK_ONE_TRY 		: 1;
		uint32_t LOCK_DETECT_WIN_TYPE 		: 1;
		uint32_t RESERVED31_15				: 17;
	};

} XN406_LockDetect_t;

/*
 * @brief Analog enable register (default 0x00C1BEFF).
 */
typedef union {
	uint32_t AnalogEnableReg;
	struct {
		/* Program to 1. */
		uint32_t BIAS_EN 					: 1;
		/* Program to 1. */
		uint32_t CP_EN 						: 1;
		/* Program to 1. */
		uint32_t PD_EN 						: 1;
		/* Program to 1. */
		uint32_t REFBUF_EN 					: 1;
		/* Program to 1. */
		uint32_t VCOBUF_EN 					: 1;
		uint32_t GPO_PAD_EN 				: 1;
		/* Program to 1. */
		uint32_t RESERVED6 					: 1;
		/* Program to 1. */
		uint32_t VCO_DIV_CLK_TO_DIG_EN 		: 1;
		/* Program to 0. */
		uint32_t RESERVED8 					: 1;
		/* Program to 1. */
		uint32_t PRESC_CLOCK_EN 			: 1;
		/* Program to 1. */
		uint32_t VCO_BUFF_PRESC_BIAS_EN 	: 1;
		/* Program to 1. */
		uint32_t CHARGE_PUMP_INT_OAMP_EN 	: 1;
		/* Program to 3. */
		uint32_t ETO_BIAS_EN 				: 3;
		/* Program to 3. */
		uint32_t DIV_RESYNC_BIAS_EN			: 3;
		/* Program to 0. */
		uint32_t RESERVED18					: 1;
		/* Program to 0. */
		uint32_t EIGHT_GHZ_DIV_BY_2			: 1;
		/* Program to 0. */
		uint32_t REF_OUT_LIMITER			: 1;
		uint32_t HIGH_FREQ_REF 				: 1;
		uint32_t RDIV_EN					: 1;
		/* Program to 3. */
		uint32_t RESERVED23 				: 1;
		uint32_t RESERVED31_24 				: 8;
	};

} XN406_AnalogEn_t;

/*
 * @brief Charge pump register (default 0x00403264).
 */
typedef union {
	uint32_t ChargePumpReg;
	struct {
		uint32_t CP_DN_GAIN 				: 7;
		uint32_t CP_UP_GAIN 				: 7;
		uint32_t OFFSET_CURRENT 			: 7;
		uint32_t OFFSET_UP_EN 				: 1;
		uint32_t OFFSET_DN_EN 				: 1;
		uint32_t HI_KCP 					: 1;
		uint32_t RESERVED31_24				: 8;
	};

} XN406_ChargePump_t;

/*
 * @brief VCO autocal configuration register (default 0x00002205).
 */
typedef union {
	uint32_t VcoAutocalReg;
	struct {
		uint32_t VTUNE_RESOLUTION 			: 3;
		uint32_t VCO_CURVE_ADJ 				: 3;
		/* Program to 1. */
		uint32_t WAIT_STATE_SET_UP 			: 2;
		/* Program to 2. */
		uint32_t RESERVED9_8			 	: 2;
		/* Program to 1. */
		uint32_t RESERVED10					: 1;
		/* Program to 0. */
		uint32_t BYPASS_VCO_TUNING 			: 1;
		uint32_t NO_VSPI_TRG 				: 1;
		uint32_t FSM_VSPI_CLK_SEL 			: 2;
		uint32_t XTAL_FALLING_EDGE_FOR_FSM 	: 1;
		/* Program to 0.*/
		uint32_t FORCE_RDIV_BYPASS 			: 1;
		uint32_t CNT_DELAY					: 1;
		uint32_t RESERVED31_18				: 14;
	};

} XN406_VcoAutocal_t;

/*
 * @brief PD register (default 0x000F8061).
 */
typedef union {
	uint32_t PhaseDetectorReg;
	struct {
		uint32_t PD_DEL_SEL 				: 2;
		/* Program to 0. */
		uint32_t RESERVED2					: 1;
		uint32_t SHORT_PD_IN 				: 1;
		uint32_t PD_PHASE_SEL 				: 1;
		uint32_t PD_UP_EN 					: 1;
		uint32_t PD_DN_EN 					: 1;
		uint32_t CSP_MODE 					: 2;
		uint32_t FORCE_CP_UP 				: 1;
		uint32_t FORCE_CP_DN 				: 1;
		uint32_t FORCE_CP_MID_RAIL 			: 1;
		uint32_t PS_BIAS	 				: 3;
		/* Program to 3. */
		uint32_t CP_INT_OAMP_BIAS 			: 2;
		/* Program to 3. */
		uint32_t MCNT_CLK_GATING 			: 2;
		/* Program to 1. */
		uint32_t RESERVED19					: 1;
		uint32_t PULSE_WIDTH_DIV_PULSE		: 2;
		uint32_t RESET_DELAY_DIV			: 1;
		uint32_t RESERVED31_23 				: 9;
	};

} XN406_PhaseDetector_t;

/*
 * @brief Precise frequency control register (default 0x00000000).
 */
typedef union {
	uint32_t ExactFreqCtrlReg;
	struct {
		uint32_t EXACT_MODE 				: 12;
		uint32_t EXACT_IN 					: 12;
		uint32_t RESERVED31_24 				: 8;
	};

} XN406_ExactFreqCtrl_t;

/*
 * @brief GPO SPI RDIV register (default 0x00000001).
 */
typedef union {
	uint32_t GpoSpiRdivReg;
	struct {
		uint32_t GPO_SELECT 				: 5;
		uint32_t GPO_TEST_DATA 				: 1;
		uint32_t PREVENT_AUTOMUX_SDO 		: 1;
		uint32_t LDO_DRV_ALWAYS_ON 			: 1;
		/* Program to 0. */
		uint32_t RESERVED10_8				: 3;
		uint32_t SD_RANDOM_EN				: 1;
		/* Program to 0. */
		uint32_t RESERVED14_12				: 3;
		/* Program to 0. */
		uint32_t DBUFF_EN 					: 1;
		uint32_t RESERVED31_16 				: 16;
	};

} XN406_GpoSpiRdiv_t;

/*
 * @brief VCO tune register (default 0x00000020).
 */
typedef union {
	uint32_t VcoTuneReg;
	struct {
		uint32_t AUTOCAL_BUSY 				: 1;
		uint32_t VCO_SWITCH_SETTING 		: 8;
		uint32_t RESERVED31_9 				: 23;
	};

} XN406_VcoTune_t;

/*
 * @brief Locked register (default 0x00000000).
 */
typedef union {
	uint32_t LockedReg;
	struct {
		uint32_t RESERVED0 					: 1;
		uint32_t LOCK_DETECT 				: 1;
		uint32_t RESERVED31_2 				: 30;
	};

} XN406_Locked_t;

/* VCO SUBSYSTEM MAP */

/*
 * @brief Tuning register 0 (default 0x0020).
 */
typedef union {
	uint16_t VcoTuningReg;
	struct {
		uint16_t TUNE_SEL					: 1;
		uint16_t CS 						: 5;
		uint16_t SPARE 						: 1;
		uint16_t VCO_SEL 					: 2;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoTuning_t;

/*
 * @brief Enables register 1 (default 0x001F).
 */
typedef union {
	uint16_t VcoEnablesReg;
	struct {
		uint16_t EN_VCO_SUBSYS 				: 1;
		uint16_t EN_PLL_BUFF_EN 			: 1;
		uint16_t EN_RF_BUFF_EN 				: 1;
		uint16_t EN_VCO_BUFF 				: 1;
		uint16_t EN_DIV_EN 					: 1;
		/* Program to 0. */
		uint16_t SPARE 						: 4;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoEnables_t;

/*
 * @brief Biases register 2 (default 0x00C1).
 */
typedef union {
	uint16_t VcoBiasesReg;
	struct {
		uint16_t RF_DIV_RATIO 				: 6;
		uint16_t RF_BUFF_GAIN_CTRL 			: 2;
		uint16_t DIV_OUT_GAIN_CTRL 			: 1;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoBiases_t;

/*
 * @brief Config register 3 (default 0x0051).
 */
typedef union {
	uint16_t VcoConfigReg;
	struct {
		uint16_t RF_OUT_MODE		 		: 1;
		uint16_t PD_RF_BUFF_CORE			: 1;
		uint16_t MANUAL_RFO_MODE 			: 1;
		uint16_t RF_BUFF_BIAS 				: 2;
		uint16_t CAL_VOL_SLOPE 				: 2;
		/* Don't care. */
		uint16_t SPARE 						: 2;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoConfig_t;

/*
 * @brief Cal/Bias register 4 (default 0x00C9).
 */
typedef union {
	uint16_t VcoCalBiasReg;
	struct {
		/* Program to 1. */
		uint16_t VCO_BIAS 					: 3;
		/* Program to 1. */
		uint16_t PLL_BUFF_BIAS 				: 2;
		/* Program to 2. */
		uint16_t VCO_BUFF_BIAS 				: 2;
		/* Program to 1. */
		uint16_t CAL_VOL	 				: 2;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoCalBias_t;

/*
 * @brief CF cal register 5 (default 0x00AA).
 */
typedef union {
	uint16_t VcoCfCalReg;
	struct {
		/* Program to 2. */
		uint16_t CF_L 						: 2;
		/* Program to 2. */
		uint16_t CF_ML 						: 2;
		/* Program to 2. */
		uint16_t CF_MH 						: 2;
		/* Program to 2. */
		uint16_t CF_H 						: 2;
		/* Program to 0. */
		uint16_t SPARE 						: 1;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoCfCal_t;

/*
 * @brief MSB cal register 6 (default 0x00FF).
 */
typedef union {
	uint16_t VcoMsbCalReg;
	struct {
		/* Program to 3. */
		uint16_t MSB_L 						: 2;
		/* Program to 3. */
		uint16_t MSB_ML 					: 2;
		/* Program to 3. */
		uint16_t MSB_MH 					: 2;
		/* Program to 3. */
		uint16_t MSB_H 						: 2;
		/* Don't care. */
		uint16_t SPARE 						: 1;
		uint16_t RESERVED16_10				: 7;
	};

} XN406_VcoMsbCal_t;

/*
 * @brief Instruction byte + data (legacy mode).
 */
typedef struct {
	/* Write data. */
	union {
		uint32_t W_InstrData;
		struct {
			uint8_t W_InstrData_L_LSB;
			uint8_t W_InstrData_L_MSB;
			uint8_t W_InstrData_H_LSB;
			uint8_t W_InstrData_H_MSB;
		};
		struct {
			uint32_t W_DONTCARE 			: 1;
			uint32_t W_REG_DATA 			: 24;
			uint32_t W_REG_ADDRESS 			: 6;
			/* 1 - read, 0 - write. */
			uint32_t W_READ_WRITE 			: 1;
		};
	};
	uint8_t TxBuff[4];

} XN406_TxData_t;

/*
 * @brief Reading data tx type.
 */
typedef struct {

	/* Write part. */
	union {
		uint8_t W_InstrData;
		struct {
			uint8_t W_DONTCARE				: 1;
			uint8_t W_REG_ADDR 				: 6;
			uint8_t W_READ_WRITE			: 1;
		};
	};
	/* Read part. */
	union {
		uint32_t R_RegData;
		struct {
			uint8_t R_InstrData_L_LSB;
			uint8_t R_InstrData_L_MSB;
			uint8_t R_InstrData_H_LSB;
			uint8_t R_InstrData_H_MSB;
		};
		struct {
			uint32_t R_LOCK_DETECT0			: 1;
			uint32_t R_REG_DATA 			: 24;
			uint32_t R_LOCK_DETECT1			: 7;
		};
	};
	uint8_t RxBuff[4];
	uint8_t TxBuff[4];

} XN406_RxData_t;

/*
 * @brief General struct.
 */
typedef struct {

	/* Reading part. */
	XN406_RxData_t RxData;
	/* Writing part. */
	XN406_TxData_t TxData;

	/* Function pointers. */
	delay_fptr delay_fp;
	hrxtx_fptr rx_fp;
	rxtx_fptr tx_fp;

} XN406_GStr_t;

/* Public function prototypes. */
void XN406_Init(void);
uint32_t XN406_GetChipId(void);
XN406_Locked_t* XN406_GetLocked(void);
XN406_VcoTune_t* XN406_GetVcoTune(void);

void XN406_RefDivCfg(uint32_t RefDiv);
void XN406_SetFreqIntPart(uint32_t IntValue);
void XN406_SetFreqFracPart(uint32_t FracValue);
void XN406_RstRegCfg(uint8_t RstChipenPinsel, uint8_t RstChipenFromSpi);
void XN406_VcoSubsys(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint16_t VcoSubsysData);
void XN406_SdCfg(uint8_t Seed, uint8_t Order, uint8_t FracBypass, uint8_t AutoSeed, uint8_t FracEn);
void XN406_LockDetect(uint8_t LdWinCnt, uint8_t EnIntLd, uint8_t LdDigWinDuration, uint8_t LdDigTimFreqCtrl,uint8_t AutoRelockOneTry,
					  uint8_t LdWinType);
void XN406_ChargePump(uint8_t CpDnGain, uint8_t CpUpGain, uint8_t OffsetCurrent, uint8_t OffsetUpEn, uint8_t OffsetDnEn, uint8_t HiKcp);
void XN406_VcoAutocal(uint8_t VtuneRes, uint8_t WaitStateSetUp, uint8_t BypassVco, uint8_t NoVspiTrg, uint8_t FsmVspiClkSel,
					  uint8_t XtalFallEdgeFsm, uint8_t ForceRdivBypass, uint8_t CntDelay);
void XN406_OffseEnCtrl(uint8_t PdDelSel, uint8_t ShortPdInputs, uint8_t PdPhaseSel, uint8_t PdUpEn, uint8_t PdDnEn, uint8_t ForceCpUp,
		uint8_t ForceCpDown, uint8_t ForceCpMidRail, uint8_t PsBias, uint8_t MCntClkGating, uint8_t PulseWidth, uint8_t ResetDelay);
void XN406_ExactFreqchCtrl(uint16_t Denominator, uint16_t Numerator);
void XN406_GpoSpiRdiv(uint8_t GpoSel, uint8_t GpoTestData, uint8_t PreventAutomuxSdo, uint8_t LdoDrvAlwaysOn, uint8_t SdRandomEn,
					  uint8_t DbuffEn);


void XN406_VcoTuning(uint8_t TuneSel, uint8_t Cs, uint8_t VcoSel);
void XN406_VcoEnables(uint8_t EnVcoSub, uint8_t EnPllBuff, uint8_t EnRfBuff, uint8_t EnVcoBuff, uint8_t EnDivn);
void XN406_VcoBiases(uint8_t RfDivRatio, uint8_t RfOutBuffGainCtrl, uint8_t DivOutStageGainCtrl);
void XN406_VcoConfig(uint8_t RfOutMode, uint8_t PdRfBuffCore, uint8_t ManualRfoMode, uint8_t RfBuffBias, uint8_t CalVolSlope);

#endif /* XN406_H_ */
