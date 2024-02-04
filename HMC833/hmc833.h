/*
 * hmc833.h header file.
 *
 * Created on: Sep 7, 2023
 * Author: asw3005
 */

#ifndef HMC833_H_
#define HMC833_H_

#include "stm32f4xx.h"

/* Select hardware or software SPI. */
//#define HMC833_SPI_HARD
#ifndef HMC833_SPI_HARD
#define HMC833_SPI_SOFT
#endif /* HMC833_SPI_HARD */


/* HMC833 some constants. */
#define HMC833_CHIP_ID 				0xA7975
#define HMC833_VCO_ID				0

/* Pin definitions serial port. */
#define HMC833_CE_PIN 				GPIO_PIN_10
#define HMC833_CE_PORT 				GPIOG
#define HMC833_CS_PIN 				GPIO_PIN_11
#define HMC833_CS_PORT 				GPIOG
#ifdef HMC833_SPI_SOFT
#define HMC833_CLK_PIN 				GPIO_PIN_13
#define HMC833_CLK_PORT 			GPIOG
#define HMC833_SDI_PIN 				GPIO_PIN_14
#define HMC833_SDI_PORT 			GPIOG
#define HMC833_SDO_PIN 				GPIO_PIN_12
#define HMC833_SDO_PORT 			GPIOG
/* Rx Tx constants. */
#define HMC833_BIT_NUMBER			8
#define HMC833_W_BIT_NUMBER			7
#define HMC833_BIT_MASK				0x80
#endif /* HMC833_SPI_SOFT */

/*
 * @brief Registers' map.
 */
typedef enum {
	/* PLL register map. */
	HMC833_ID 						= 0,
	HMC833_READ_ADDR_RST_STROBE 	= 0,
	HMC833_RST,
	HMC833_REFDIV,
	HMC833_FREQ_INTEGER_PART,
	HMC833_FREQ_FRACTIONAL_PART,
	HMC833_VCO_SPI,
	HMC833_SD_CFG,
	HMC833_LOCK_DETECT,
	HMC833_ANALOG_EN,
	HMC833_CHARGE_PUMP,
	HMC833_VCO_AUTOCAL,
	HMC833_PD,
	HMC833_EXACT_FREQ_MODE,
	HMC833_GPO_SPI_RDIV 			= 0x0F,
	HMC833_VCO_TUNE,
	HMC833_SAR,
	HMC833_GPO2,
	HMC833_BIST,
	/* VCO subsystem. */
	HMC833_VCO_SUBSYS_ID			= 0,
	HMC833_VCO_TUNING 				= 0,
	HMC833_VCO_ENABLES,
	HMC833_VCO_BIASES,
	HMC833_VCO_CONFIG,
	/* Support only settings on the page 60 of the datasheet. */
	HMC833_VCO_BIAS_CAL,
	HMC833_VCO_CF_CAL,
	HMC833_VCO_MSB_CAL

} HMC833_REG_MAP_t;

/*
 * @brief Read/write bit state.
 */
typedef enum {
	HMC833_WRITE,
	HMC833_READ

} HMC833_RW_STATE_t;


/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);
typedef void(*hrxtx_fptr)(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);

/* PLL REGISTER MAP */

/*
 * @brief Open mode read address/RST strobe register.
 */
typedef union {
	uint32_t OpenModeReg;
	struct {
		uint32_t READ_ADDR 					: 5;
		uint32_t SOFT_RST 					: 1;
		uint32_t MUST_BE_ZERO 				: 18;
		uint32_t DONT_CARE 					: 8;
	};

} HMC833_OmReadAddrRst_t;

/*
 * @brief RST register (default 0x02).
 */
typedef union {
	uint32_t RstReg;
	struct {
		uint32_t RST_CHIPEN_PIN_SEL 		: 1;
		uint32_t RST_CHIPEN_FROM_SPI 		: 1;
		uint32_t KEEP_BIAS_ON 				: 1;
		uint32_t KEEP_PD_ON 				: 1;
		uint32_t KEEP_CP_ON 				: 1;
		uint32_t KEEP_REF_BUFF_ON 			: 1;
		uint32_t KEEP_VCO_ON 				: 1;
		uint32_t KEEP_GPO_DRIVER_ON 		: 1;
		/* Program to 0. */
		uint32_t RESERVED31_8				: 24;
	};
} HMC833_RstReg_t;

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

} HMC833_SubsysVco_t;

/*
 * @brief VCO SPI Register (default 0x00).
 * Use if VcoAutocalReg[NO_VSPI_TRIG] = 1.
 */
typedef union {

	uint16_t VcoSpiDataReg;
	struct {
		uint16_t VCO_SUBSYS_ID 				: 3;
		uint16_t VCO_SUBSYS_REG_ADDR 		: 4;
		uint16_t CALIB_TUNE_VOLTAGE_OFF		: 1;
		uint16_t VCO_CAPS					: 5;
		/* Program to 0. */
		uint16_t RESERVED13					: 1;
		uint16_t VCO_SELECT					: 2;
	};

} HMC833_VcoSpi_t;

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
		uint32_t CLKRQ_REFDIV_SEL 			: 1;
		/* Program to 1. */
		uint32_t SD_MODULATOR_CLK_SEL 		: 1;
		uint32_t SD_ENABLE 					: 1;
		/* Program to 0. */
		uint32_t RESERVED15_12				: 4;
		/* Program to 3. */
		uint32_t RESERVED17_16				: 2;
		uint32_t BIST_EN 					: 1;
		uint32_t RDIV_BIST_CYCLES 			: 2;
		/* Program  to 0. */
		uint32_t AUTO_CLK_CONFIG 			: 1;
		/* Program to 0. */
		uint32_t RESERVED22					: 1;
		uint32_t RESERVED31_23				: 9;
	};
} HMC833_SdCfg_t;

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
		uint32_t LOCK_DETECT_WIN_TYPE 		: 1;
		uint32_t LD_DIG_WIN_DUR 			: 3;
		uint32_t LD_DIG_TIM_FREQ_CTRL 		: 2;
		uint32_t LD_TIM_TEST_MODE 			: 1;
		uint32_t AUTORELOCK_ONE_TRY 		: 1;
		uint32_t RESERVED31_14				: 18;
	};

} HMC833_LockDetect_t;

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
		uint32_t RESERVED14_12 				: 3;
		/* Program to 3. */
		uint32_t RESERVED17_15 				: 3;
		uint32_t SPARE 						: 1;
		/* Program to 0. */
		uint32_t RESERVED20_19 				: 2;
		uint32_t HIGH_FREQ_REF 				: 1;
		/* Program to 3. */
		uint32_t RESERVED23_22 				: 2;
		uint32_t RESERVED31_24 				: 8;
	};

} HMC833_AnalogEn_t;

/*
 * @brief Charge pump register (default 0x00403264).
 */
typedef union {
	uint32_t ChargePumpReg;
	struct {
		uint32_t CP_DN_GAIN 				: 7;
		uint32_t CP_UP_GAIN 				: 7;
		uint32_t OFFSET_MAGNITUDE 			: 7;
		uint32_t OFFSET_UP_EN 				: 1;
		uint32_t OFFSET_DN_EN 				: 1;
		uint32_t HI_KCP 					: 1;
		uint32_t RESERVED31_24				: 8;
	};

} HMC833_ChargePump_t;

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
		uint32_t NUM_OF_SAR_BITS_IN_VCO 	: 2;
		/* Program to 0. */
		uint32_t FORCE_CURVE 				: 1;
		uint32_t BYPASS_VCO_TUNING 			: 1;
		uint32_t NO_VSPI_TRG 				: 1;
		uint32_t FSM_VSPI_CLK_SEL 			: 2;
		uint32_t XTAL_FALLING_EDGE_FOR_FSM 	: 1;
		/* Program to 0.*/
		uint32_t FORCE_RDIV_BYPASS 			: 1;
		uint32_t RESERVED31_17				: 15;
	};

} HMC833_VcoAutocal_t;

/*
 * @brief PD register (default 0x000F8061).
 */
typedef union {
	uint32_t PhaseDetectorReg;
	struct {
		uint32_t PD_DEL_SEL 				: 3;
		uint32_t SHORT_PD_IN 				: 1;
		uint32_t PD_PHASE_SEL 				: 1;
		uint32_t PD_UP_EN 					: 1;
		uint32_t PD_DN_EN 					: 1;
		uint32_t CSP_MODE 					: 2;
		uint32_t FORCE_CP_UP 				: 1;
		uint32_t FORCE_CP_DN 				: 1;
		uint32_t FORCE_CP_MID_RAIL 			: 1;
		/* Program to 4. */
		uint32_t RESERVED14_12 				: 3;
		/* Program to 3. */
		uint32_t CP_INT_OAMP_BIAS 			: 2;
		/* Program to 3. */
		uint32_t MCNT_CLK_GATING 			: 2;
		/* Don't care. */
		uint32_t SPARE 						: 1;
		/* Program to 0. */
		uint32_t RESERVED23_20 				: 4;
		uint32_t RESERVED31_25 				: 8;
	};

} HMC833_PhaseDetector_t;

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
		uint32_t DIS_PFET 					: 1;
		/* Program to 0. */
		uint32_t DIS_NFET 					: 1;
		uint32_t RESERVED31_10 				: 22;
	};

} HMC833_GpoSpiRdiv_t;

/*
 * @brief VCO tune register (default 0x00000020).
 */
typedef union {
	uint32_t VcoTuneReg;
	struct {
		uint32_t VCO_SWITCH_SETTING 		: 8;
		uint32_t AUTOCAL_BUSY 				: 1;
		uint32_t RESERVED31_9 				: 23;
	};

} HMC833_VcoTune_t;

/*
 * @brief SAR register (default 0x0007FFFF).
 */
typedef union {
	uint32_t SarReg;
	struct {
		uint32_t SAR_ERR_MAG_CNTS 			: 19;
		uint32_t SAR_ERR_SIGN 				: 1;
		uint32_t RESERVED31_20 				: 12;
	};

} HMC833_Sar_t;

/*
 * @brief GPO2 register (default 0x00000000).
 */
typedef union {
	uint32_t Gpo2Reg;
	struct {
		uint32_t GPO_STATE 					: 1;
		uint32_t LOCK_DETECT 				: 1;
		uint32_t RESERVED31_2 				: 30;
	};

} HMC833_Gpo2_t;

/*
 * @brief BIST register (default 0x00001259).
 */
typedef union {
	uint32_t BistReg;
	struct {
		uint32_t BIST_SIGNATURE 			: 16;
		uint32_t BIST_BUSY 					: 1;
		uint32_t RESERVED31_17 				: 15;
	};

} HMC833_Bist_t;

/* VCO SUBSYSTEM MAP */

/*
 * @brief Tuning register.
 */
typedef union {
	uint16_t VcoTuningReg;
	struct {
		uint16_t CAL 						: 1;
		uint16_t CAPS 						: 8;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoTuning_t;

/*
 * @brief Enables register.
 */
typedef union {
	uint16_t VcoEnablesReg;
	struct {
		uint16_t MASTER_EN_VCO_SUBSYS 		: 1;
		uint16_t MANUAL_MODE_PLL_BUFF_EN 	: 1;
		uint16_t MANUAL_MODE_RF_BUFF_EN 	: 1;
		uint16_t MANUAL_MODE_DIVBY1_EN 		: 1;
		uint16_t MANUAL_MODE_RF_DIV_EN 		: 1;
		uint16_t DONT_CARE 					: 4;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoEnables_t;

/*
 * @brief Biases register.
 */
typedef union {
	uint16_t VcoBiasesReg;
	struct {
		uint16_t RF_DIV_RATIO 				: 6;
		uint16_t RF_OUT_BUFF_GAIN_CTRL 		: 2;
		uint16_t DIV_OUT_STAGE_GAIN_CTRL 	: 1;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoBiases_t;

/*
 * @brief Config register.
 */
typedef union {
	uint16_t VcoConfigReg;
	struct {
		uint16_t FUND_DOUBLER_MODE_SEL 		: 1;
		uint16_t RESERVED1 					: 1;
		uint16_t MANUAL_RFO_MODE 			: 1;
		uint16_t RF_BUFF_BIAS 				: 2;
		/* Don't care. */
		uint16_t SPARE 						: 4;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoConfig_t;

/*
 * @brief Cal/Bias register.
 */
typedef union {
	uint16_t VcoCalBiasReg;
	struct {
		/* Program to 1. */
		uint16_t VCO_BIAS 					: 3;
		/* Program to 0. */
		uint16_t PLL_BUFF_BIAS 				: 2;
		/* Program to 2. */
		uint16_t FND_LMTR_BIAS 				: 2;
		/* Program to 1. */
		uint16_t PRESET_CAL0 				: 2;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoCalBias_t;

/*
 * @brief CF cal register.
 */
typedef union {
	uint16_t VcoCfCalReg;
	struct {
		/* Program to 0. */
		uint16_t CF_L 						: 2;
		/* Program to 3. */
		uint16_t CF_ML 						: 2;
		/* Program to 2. */
		uint16_t CF_MH 						: 2;
		/* Program to 0. */
		uint16_t CF_H 						: 2;
		/* Program to 0. */
		uint16_t SPARE 						: 1;
		uint16_t RESERVED16_10				: 7;
	};

} HMC833_VcoCfCal_t;

/*
 * @brief MSB cal register.
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

} HMC833_VcoMsbCal_t;

/*
 * @brief Instruction byte + data (HMC mode).
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

} HMC833_TxData_t;

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
			//uint32_t R_LOCK_DETECT0			: 1;
			uint32_t R_REG_DATA 			: 24;
			uint32_t R_LOCK_DETECT1			: 8;
		};
	};
	uint8_t RxBuff[4];
	uint8_t TxBuff[4];

} HMC833_RxData_t;

/*
 * @brief General struct.
 */
typedef struct {

	/* Reading part. */
	HMC833_RxData_t RxData;
	/* Writing part. */
	HMC833_TxData_t TxData;

	/* Function pointers. */
	delay_fptr delay_fp;
	hrxtx_fptr rx_fp;
	rxtx_fptr tx_fp;

} HMC833_GStr_t;

/* Public function prototypes. */
void HMC833_Init(void);
uint32_t HMC833_GetChipId(void);
HMC833_Sar_t* HMC833_GetSar(void);
HMC833_Gpo2_t* HMC833_GetGpo2(void);
HMC833_Bist_t* HMC833_GetBist(void);
HMC833_VcoTune_t* HMC833_GetVcoTune(void);

void HMC833_RefDivCfg(uint32_t RefDiv);
void HMC833_SetFreqIntPart(uint32_t IntValue);
void HMC833_SetFreqFracPart(uint32_t FracValue);
void HMC833_ExactFreqchPerFpd(uint16_t NumberOfChannelsPerFpd);
void HMC833_VcoSubsys(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint16_t VcoSubsysData);
void HMC833_GpoSpiRdiv(uint8_t GpoSel, uint8_t GpoTestData, uint8_t PreventAutomuxSdo, uint8_t LdoDrvAlwaysOn);
void HMC833_VcoSpi(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint8_t CalibTuneVoltageOff, uint8_t VcoCaps, uint8_t SubVcoSel);
void HMC833_ChargePump(uint8_t CpDnGain, uint8_t CpUpGain, uint8_t OffsetMag, uint8_t OffsetUpEn, uint8_t OffsetDnEn, uint8_t HiKcp);
void HMC833_SdCfg(uint8_t Seed, uint8_t Order, uint8_t FracBypass, uint8_t AutoSeed, uint8_t ClkrqRefdivSel, uint8_t SdEn, uint8_t BistEn,
			  	  uint8_t RdivBistCycles);
void HMC833_RstRegCfg(uint8_t RstChipenPinsel, uint8_t RstChipenFromSpi, uint8_t KeepBiasOn, uint8_t KeepPdOn, uint8_t KeepCpOn,
					  uint8_t KeepRefBuffOn, uint8_t KeepVcoOn, uint8_t KeepGpoDrvOn);
void HMC833_LockDetect(uint8_t LdWinCnt, uint8_t EnIntLd, uint8_t LdWinType, uint8_t LdDigWinDuration, uint8_t LdDigTimFreqCtrl,
					   uint8_t LdTimTestMode, uint8_t AutoRelockOneTry);
void HMC833_VcoAutocal(uint8_t VtuneRes, uint8_t VcoCurveAdj, uint8_t WaitStateSetUp, uint8_t NumOfSarBits, uint8_t BypassVco,
					   uint8_t NoVspiTrg, uint8_t FsmVspiClkSel, uint8_t XtalFallEdgeFsm);
void HMC833_PhaseDetector(uint8_t PdDelSel, uint8_t ShortPdInputs, uint8_t PdPhaseSel, uint8_t PdUpEn, uint8_t PdDnEn, uint8_t CspMode,
						  uint8_t ForceCpUp, uint8_t ForceCpDown, uint8_t ForceCpMidRail, uint8_t MCntClkGating);


void HMC833_VcoTuning(uint8_t Cal, uint8_t Caps);
void HMC833_VcoConfig(uint8_t FundDoublerModeSel, uint8_t ManRfoMode, uint8_t RfBuffBias);
void HMC833_VcoBiases(uint8_t RfDivRatio, uint8_t RfOutBuffGainCtrl, uint8_t DivOutStageGainCtrl);
void HMC833_VcoEnables(uint8_t MEnVcoSubsys, uint8_t ManModePllBuffEn, uint8_t ManModeRfBuffEn, uint8_t ManModeDivBy1En,
					   uint8_t ManModeRfDivEn);


#endif /* HMC833_H_ */
