/*
 *  @brief ad9959.h header file.
 *
 * Created on: Aug 21, 2023
 * Author: asw3005
 */
#ifndef AD9959_H_
#define AD9959_H_

#include "stm32f1xx.h"

#define AD9959_SYSTEM_CLOCK_RATE 	500000000.0f
#define AD9959_BIT_DEPTH 			1023
//#define AD9959_VOLTAGE_RATIO		1.708984375f
#define AD9959_VOLTAGE_RATIO		1.28f

/* Ports' selection. */
#define AD9959_RST_PIN 					GPIO_PIN_6
#define AD9959_NSS_PIN 					GPIO_PIN_4
#define AD9959_CLK_PIN 					GPIO_PIN_5
#define AD9959_DATA_IN_PIN				GPIO_PIN_7
#define AD9959_DATA_INOUT_PIN			GPIO_PIN_7
#define AD9959_RST_PORT 				GPIOA
#define AD9959_NSS_PORT					GPIOA
#define AD9959_CLK_PORT 				GPIOA
#define AD9959_DATA_IN_PORT				GPIOA
#define AD9959_DATA_INOUT_PORT			GPIOA

/* Significant data bits. */
#define AD9959_CMD_WORD_SIZE			1
#define AD9959_BIT_NUMBER				8
#define AD9959_BIT_MASK					0x80

/*
 * @brief Register maps.
 */
typedef enum {
	/* Control registers. */
	AD9959_CH_SEL,
	AD9959_FUNCTION1,
	AD9959_FUNCTION2,
	/* Channel registers. */
	AD9959_CH_FUNCTION,
	AD9959_CH_FREQUENCY_TUNING,
	AD9959_CH_PHASE_OFFSET,
	AD9959_CH_AMPLITUDE_CTRL,
	AD9959_LINEAR_SWEEP_RAMP_RATE,
	AD9959_LSR_RISING_DELTA_WORD,
	AD9959_LSR_FALLING_DELTA_WORD,
	AD9959_CH_WORD1,
	AD9959_CH_WORD2,
	AD9959_CH_WORD3,
	AD9959_CH_WORD4,
	AD9959_CH_WORD5,
	AD9959_CH_WORD6,
	AD9959_CH_WORD7,
	AD9959_CH_WORD8,
	AD9959_CH_WORD9,
	AD9959_CH_WORD10,
	AD9959_CH_WORD11,
	AD9959_CH_WORD12,
	AD9959_CH_WORD13,
	AD9959_CH_WORD14,
	AD9959_CH_WORD15


} AD9959_REG_MAPS_t;

/*
 * @brief Read/write command.
 */
typedef enum {
	AD9959_WRITE_CMD,
	AD9959_READ_CMD

} AD9959_RW_CMD_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);



/*
 * @brief
 */
typedef union {
	uint8_t ChSelReg;
	struct {
		uint8_t LSB_FIRST 		: 1;
		uint8_t SERIAL_IO_MODE 	: 2;
		/* Must be zero. */
		uint8_t MUST_BE_ZERO	: 1;
		uint8_t CH_ENABLE 		: 4;
	};

} AD9959_ChSel_t;

/*
 * @brief Function register one.
 */
typedef struct {
	union {
		uint8_t FunctReg1_LOW_LSB;
		struct {
			uint8_t MANUAL_SOFT_SYNC 	: 1;
			uint8_t MANUAL_HARD_SYNC 	: 1;
			uint8_t RESERVED0			: 2;
			uint8_t DAC_REF_PWR_DOWN 	: 1;
			uint8_t SYNC_CLK_DIS 		: 1;
			uint8_t EXT_PWR_DOWN_MODE 	: 1;
			uint8_t REF_CLK_IN_PWR_DOWN : 1;
		};
	};

	union {
		uint8_t FunctReg1_LOW_MSB;
		struct {
			uint8_t MODULATION_LEVEL 	: 2;
			uint8_t RAMP_UP_DOWN 		: 2;
			uint8_t PROFILE_PIN_CFG 	: 3;
			uint8_t RESERVED1 			: 1;
		};
	};

	union {
		uint8_t FunctReg1_HIGH_LSB;
		struct {
			uint8_t CHARGE_PUMP_CTRL 	: 2;
			uint8_t PLL_DIV_RATIO 		: 5;
			uint8_t VCO_GAIN_CTRL 		: 1;
		};
	};

} AD9959_Funct1_t;

/*
 * @brief Function register two.
 */
typedef struct {
	union {
		uint8_t FunctReg2_LOW_LSB;
		struct {
			uint8_t SYSTEM_CLK_OFFSET 		: 2;
			uint8_t RESERVED0 				: 2;
			uint8_t SYNC_MASK 				: 1;
			uint8_t SYNC_STATUS 			: 1;
			uint8_t SYNC_MASTER_EN 			: 1;
			uint8_t AUTOSYNC_EN 			: 1;
		};
	};

	union {
		uint8_t FunctReg2_LOW_MSB;
		struct {
			uint8_t RESERVED1 				: 2;
			uint8_t RESERVED2 				: 2;
			uint8_t ALLCH_CLR_PHASE_ACC 	: 1;
			uint8_t ALLCH_AUTOCLR_PHASE_ACC : 1;
			uint8_t ALLCH_CLR_SWEEP_ACC 	: 1;
			uint8_t ALLCH_AUTOCLR_SWEEP_ACC : 1;
		};
	};

} AD9959_Funct2_t;

/*
 * @brief channel function register.
 */
typedef struct {
	union {
		uint8_t ChFunctReg_LOW_LSB;
		struct {
			uint8_t SINE_WAVE_OUT_EN 			: 1;
			uint8_t CLR_PHASE_ACC 				: 1;
			uint8_t AUTOCLR_PHASE_ACC 			: 1;
			uint8_t CLR_SWEEP_ACC 				: 1;
			uint8_t AUTOCLR_SWEEP_ACC 			: 1;
			uint8_t MATCHED_PIPE_DELAYS_ACTIVE 	: 1;
			uint8_t DAC_PWR_DOWN 				: 1;
			uint8_t DIGITAL_PWR_DOWN 			: 1;
		};
	};

	union {
		uint8_t ChFunctReg_LOW_MSB;
		struct {
			uint8_t DAC_FULL_SCALE_CURRENT_CTRL : 2;
			/* Must be zero. */
			uint8_t MUST_BE_ZERO				: 1;
			uint8_t RESERVED1 					: 2;
			uint8_t LOAD_SRR_AT_IO_UPDATE 		: 1;
			uint8_t LINEAR_SWEEP_EN 			: 1;
			uint8_t LINEAR_SWEEP_NO_DWELL 		: 1;
		};
	};

	union {
		uint8_t ChFunctReg_HIGH_LSB;
		struct {
			uint8_t RESERVED2 					: 6;
			uint8_t AFP_SEL 					: 2;
		};
	};

} AD9959_ChFunct_t;

/*
 * @brief Channel frequency tuning word 0.
 */
typedef struct {
	union {
		uint32_t FreqTunWord0_32;
		struct {
			uint8_t FreqTunWord0_LOW_LSB;
			uint8_t FreqTunWord0_LOW_MSB;
			uint8_t FreqTunWord0_HIGH_LSB;
			uint8_t FreqTunWord0_HIGH_MSB;
		};
	};

} AD9959_FreqTuneWord0_t;

/*
 * @brief Channel phase offset word 0.
 */
typedef union {
	uint16_t PhaseOffsetWord016;
	struct {
		uint16_t PHASE_OFFSET_WORD0 		: 14;
		uint16_t RESERVED 					: 2;
	};

} AD9959_PhaseOffsetWord0_t;

/*
 * @brief Amplitude control register.
 */
typedef struct {
	union {
		uint8_t AmplCtrl_LOW_LSB;
		struct {
			uint8_t AMPL_SCALE_FACTOR_LOW 	: 8;
		};
	};

	union {
		uint8_t AmplCtrl_LOW_MSB;
		struct {
			uint8_t AMPL_SCALE_FACTOR_HIGH 	: 2;
			uint8_t LOAD_ARR_AT_IO_UPDATE 	: 1;
			uint8_t RAMP_UP_DOWN_EN 		: 1;
			uint8_t AMPL_MULTIPLIER_EN 		: 1;
			uint8_t RESERVED0 				: 1;
			uint8_t INC_DEC_STEP_SIZE 		: 2;
		};
	};

	union {
		uint8_t AmplCtrl_HIGH_LSB;
		struct {
			uint8_t AMPL_RAMP_RATE : 8;
		};
	};

} AD9959_AmplCtrl_t;

/*
 * @brief Linear sweep ramp rate.
 */
typedef struct {
	uint8_t RisingSweepRampRate_LSB;
	uint8_t FallingSweepRampRate_MSB;

} AD9959_LinearSweepRampRate_t;

/*
 * @brief LSR rising delta word.
 */
typedef struct {
	union {
		uint32_t RisingDeltaWord_32;
		struct {
			uint8_t RisingDeltaWord_LOW_LSB;
			uint8_t RisingDeltaWord_LOW_MSB;
			uint8_t RisingDeltaWord_HIGH_LSB;
			uint8_t RisingDeltaWord_HIGH_MSB;
		};
	};

} AD9959_RisingDeltaWord_t;

/*
 * @brief LSR falling delta word.
 */
typedef struct {
	union {
		uint32_t FallingDeltaWord_32;
		struct {
			uint8_t FallingDeltaWord_LOW_LSB;
			uint8_t FallingDeltaWord_LOW_MSB;
			uint8_t FallingDeltaWord_HIGH_LSB;
			uint8_t FallingDeltaWord_HIGH_MSB;
		};
	};

} AD9959_FallingDeltaWord_t;

/*
 * @brief Profile register.
 */
typedef union __attribute__((aligned(1), packed)) {

	uint32_t ProfileReg_32;
	struct {
		uint32_t FREQ_TUNE_WORD 	: 32;
	};

	struct {
		uint32_t RESERVED0 			: 18;
		uint32_t PHASE_TUNE_WORD 	: 14;
	};

	struct {
		uint32_t RESERVED1 			: 22;
		uint32_t AMPL_TUNE_WORD 	: 10;
	};

	struct {
		uint8_t ChannelWord_LOW_LSB;
		uint8_t ChannelWord_LOW_MSB;
		uint8_t ChannelWord_HIGH_LSB;
		uint8_t ChannelWord_HIGH_MSB;
	};

} AD9959_Profile_t;

/*
 * @brief Instruction byte.
 */
typedef struct __attribute__((aligned(1), packed)) {
	union {
		uint8_t InstrByte;
		struct {
			uint8_t REG_ADDRESS 	: 5;
			uint8_t RESERVED 		: 2;
			/* 1 - read, 0 - write. */
			uint8_t READ_WRITE 		: 1;
		};
	};

	uint8_t Data[4];


} AD9959_RxTxData_t;


/*
 * @brief Reading data.
 */
typedef struct {

	uint8_t RegData[4];

} AD9959_ReadData_t;

/*
 * @brief
 */
typedef struct {

	AD9959_RxTxData_t RxTxData;
	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr spi_rx_fp;
	rxtx_fptr spi_tx_fp;

} AD9959_GStr_t;


/*
 * @brief General struct.
 */


/* Public function prototypes. */
void AD9959_Init(void);
uint16_t AD9959_SetPhase(float Phase);
uint32_t AD9959_SetFrequency(float Frequency);
void AD9959_SetRisingDeltaWord(uint32_t RisingDeltaWord);
void AD9959_SetFallingDeltaWord(uint32_t FallingDeltaWord);
void AD9959_SetSweepRampRate(uint8_t RisingSweepRampRate, uint8_t FallingSweepRampRate);
void AD9959_SetChannelWord(AD9959_REG_MAPS_t NumberOfChWord, uint8_t FreqPhaseAmplSel, uint32_t FreqPhaseAmplWord);
void AD9959_SetAmplitude(float Amplitude, uint8_t LoadArrAtIoUpdate, uint8_t RumpUpDownEn, uint8_t AmplMultiplierEn, uint8_t IncDecStepSize, uint8_t AmplRumpRate);



uint8_t AD9959_ReadSyncStatus(void);
void AD9959_ChSelectReg(uint8_t LsbFirst, uint8_t SerialIoMode, uint8_t ChannelEn);
void AD9959_FunctReg1(uint8_t ManSoftSync, uint8_t ManHardSync, uint8_t DacRefPwrDown, uint8_t SyncClkDis, uint8_t ExtPwrDownMode, uint8_t RefClkInPwrDown,
					uint8_t ModulationLvl, uint8_t RampUpDown, uint8_t ProfilePinCfg, uint8_t ChargePumpCtrl, uint8_t PllDivRatio, uint8_t VcoGainCtrl);
void AD9959_FunctReg2(uint8_t SysClkOffset, uint8_t MultiDevSyncMask, uint8_t MultiDevSyncMasterEn, uint8_t AutoSyncEn,
					uint8_t AllChClrPhaseAcc, uint8_t AllChAutoClrPhaseAcc, uint8_t AllChClrSweepAcc, uint8_t AllChAutoClrSweepAcc);
void AD9959_ChannelFunctReg(uint8_t SineWaveOutEn, uint8_t ClrPhaseAcc, uint8_t AutoClrPhaseAcc, uint8_t ClrSweepAcc, uint8_t AutoClrSweepAcc,
							uint8_t MatchedPipeDelaysActive, uint8_t DacPwrDown, uint8_t DigitalPwrDown, uint8_t DacFullScaleCurrentCtrl,
							uint8_t LoadSrrIoUpdate, uint8_t LinearSweepEn, uint8_t LinearSweepNoDwell, uint8_t AfpSelect);
void AD9959_FreqTuneWord0(uint32_t FreqTuneWord);
void AD9959_ChPhaseOffsetWord0(uint16_t ChPhaseOffsetWord);
AD9959_ReadData_t* AD9959_ReadReg(uint8_t RegAddress, uint8_t Size);


#endif /* AD9959_H_ */
