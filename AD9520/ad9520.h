/*
 * ad9520.h header file.
 *
 * Created on: Aug 5, 2023
 * Author: asw3005
 */

#ifndef AD9520_H_
#define AD9520_H_

/* Part IDs. */
#define AD95200_PART_ID					0x20
#define AD95201_PART_ID					0x60
#define AD95202_PART_ID					0xA0
#define AD95203_PART_ID					0x61
#define AD95204_PART_ID					0xE1
#define AD95205_PART_ID					0xE0
#define AD9520x_NO_DEVICE				0xFF
/* Customer parameters. */
#define AD9520x_TYPE					AD95203_PART_ID
#define AD9520x_CUSTOMER_ID				0x0A0B

/* Ports' selection. */
#define AD9520_NSS_PIN 					GPIO_PIN_11
#define AD9520_CLK_PIN 					GPIO_PIN_13
#define AD9520_DATA_IN_PIN				GPIO_PIN_12
#define AD9520_DATA_INOUT_PIN			GPIO_PIN_14
#define AD9520_NSS_PORT					GPIOG
#define AD9520_CLK_PORT 				GPIOG
#define AD9520_DATA_IN_PORT				GPIOG
#define AD9520_DATA_INOUT_PORT			GPIOG

/* Significant data bits. */
#define AD9520_CMD_WORD_SIZE			2
#define AD9520_BIT_NUMBER				8
#define AD9520_BIT_MASK					0x80

/*
 * @brief AD9520 register map.
 */
typedef enum {
	/* Serial port configuration. */
	AD9520_SERIAL_PORT_CFG,
	/* Read only, default is 0x20. */
	AD9520_PART_ID 							= 0x03,
	AD9520_READBACK_CTRL,

	/* EEPROM ID. */
	AD9520_CUSTOMER_VERSION_ID_LSB,
	AD9520_CUSTOMER_VERSION_ID_MSB,

	/* PLL. */
	/* Default is 0x7D. */
	AD9520_PFD_CHARGE_PUMP 					= 0x10,
	/* Default is 0x01. */
	AD9520_R_COUNTER_LSB,
	AD9520_R_COUNTER_MSB,
	AD9520_A_COUNTER,
	/* Default is 0x03. */
	AD9520_B_COUNTER_LSB,
	AD9520_B_COUNTER_MSB,
	/* Default is 0x06. */
	AD9520_PLL_CTRL_1,
	AD9520_PLL_CTRL_2,
	/* Default is 0x06. */
	AD9520_PLL_CTRL_3,
	AD9520_PLL_CTRL_4,
	AD9520_PLL_CTRL_5,
	AD9520_PLL_CTRL_6,
	AD9520_PLL_CTRL_7,
	/* Default is 0x80. */
	AD9520_PLL_CTRL_8,
	AD9520_PLL_CTRL_9,
	/* Read only. */
	AD9520_PLL_READBACK,

	/* Output driver control. */
	/* Default value of all OUTx_CTRL is 0x64. */
	AD9520_OUT0_CTRL 						= 0xF0,
	AD9520_OUT1_CTRL,
	AD9520_OUT2_CTRL,
	AD9520_OUT3_CTRL,
	AD9520_OUT4_CTRL,
	AD9520_OUT5_CTRL,
	AD9520_OUT6_CTRL,
	AD9520_OUT7_CTRL,
	AD9520_OUT8_CTRL,
	AD9520_OUT9_CTRL,
	AD9520_OUT10_CTRL,
	AD9520_OUT11_CTRL,
	AD9520_CSDLD_OUT_EN_LSB,
	AD9520_CSDLD_OUT_EN_MSB,

	/* LVPECL channel dividers. */
	/* Default is 0x77. */
	AD9520_DIVIDER0_BYTE0 					= 0x0190,
	AD9520_DIVIDER0_BYTE1,
	AD9520_DIVIDER0_BYTE2,
	/* Default is 0x33. */
	AD9520_DIVIDER1_BYTE0,
	AD9520_DIVIDER1_BYTE1,
	AD9520_DIVIDER1_BYTE2,
	/* Default is 0x11. */
	AD9520_DIVIDER2_BYTE0,
	AD9520_DIVIDER2_BYTE1,
	AD9520_DIVIDER2_BYTE2,
	/* Default is 0x00. */
	AD9520_DIVIDER3_BYTE0,
	AD9520_DIVIDER3_BYTE1,
	AD9520_DIVIDER3_BYTE2,

	/* VCO divider and clock input. */
	AD9520_VCO_DIVIDER 						= 0x01E0,
	/* Default is 0x20. */
	AD9520_INPUT_CLOCKS,

	/* System. */
	AD9520_POWER_DOWN_SYNC 					= 0x0230,

	/* Update all registers. */
	AD9520_IO_UPDATE 						= 0x0232,

	/* EEPROM buffer segment. */
	AD9520_EEP_SPCFG_BYTE_NUMBERS			= 0x0A00,
	AD9520_EEP_SPCFG_ADDR_MSB,
	AD9520_EEP_SPCFG_ADDR_LSB,
	AD9520_EEP_CUST_ID_BYTE_NUMBERS,
	AD9520_EEP_CUST_ID_ADDR_MSB,
	AD9520_EEP_CUST_ID_ADDR_LSB,
	AD9520_EEP_PLL_SETS_BYTE_NUMBERS,
	AD9520_EEP_PLL_SETS_ADDR_MSB,
	AD9520_EEP_PLL_SETS_ADDR_LSB,
	AD9520_EEP_OUT_DRV_BYTE_NUMBERS,
	AD9520_EEP_OUT_DRV_ADDR_MSB,
	AD9520_EEP_OUT_DRV_ADDR_LSB,
	AD9520_EEP_CH_DRV_BYTE_NUMBERS,
	AD9520_EEP_CH_DRV_ADDR_MSB,
	AD9520_EEP_CH_DRV_ADDR_LSB,
	AD9520_EEP_VCO_CLK_BYTE_NUMBERS,
	AD9520_EEP_VCO_CLK_ADDR_MSB,
	AD9520_EEP_VCO_CLK_ADDR_LSB,
	AD9520_EEP_PWRDOWN_SYNC_BYTE_NUMBERS,
	AD9520_EEP_PWRDOWN_SYNC_ADDR_MSB,
	AD9520_EEP_PWRDOWN_SYNC_ADDR_LSB,
	AD9520_EEP_IO_UPDATE,
	AD9520_EEP_END_OF_DATA,

	/* EEPROM control. */
	/* Read only. */
	AD9520_EEP_STATUS 						= 0x0B00,
	/* Read only. */
	AD9520_EEP_ERR_CHECKING,
	AD9520_EEP_CTRL1,
	AD9520_EEP_CTRL2

}  AD9520_REG_MAP_t;

/*
 * @brief read/write operation states.
 */
typedef enum {

	AD9520_WRITE,
	AD9520_READ,

	AD9520_ONE_BYTE 						= 0x00,
	AD9520_TWO_BYTES,
	AD9520_THREE_BYTES,
	AD9520_STREAM_MODE

} AD9520_RW_t;

/*
 * @brief EEPROM operation states.
 */
typedef enum {

	AD9520_EEP_COMPLETED,
	AD9520_EEP_IN_PROCESS

} AD9520_EEPROM_STATUS_t;

/*
 * @brief Data exchange function typedefs.
 */
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_rxtx_fptr)(uint8_t* pData, uint8_t size);

/*
 * @brief SPI control.
 */
typedef union {
	uint8_t SpiCtrl;
	struct {
		/* Four mirrored bits. */
		uint8_t SDO_ACTIVE_M 	: 1;
		uint8_t MSB_LSB_CTRL_M 	: 1;
		uint8_t SOFT_RESET_M 	: 1;
		uint8_t RESERVED_M 		: 1;
		uint8_t RESERVED 		: 1;
		/* 1 - soft reset is active. */
		uint8_t SOFT_RESET 		: 1;
		/* 0 - data-oriented MSB first; the addressing decrements (default).
		 * 1 - data-oriented LSB first; the addressing increments.
		 */
		uint8_t MSB_LSB_CTRL 	: 1;
		/* 0 - SDO pin is high impedance (default).
		 * 1 - SDO pin is used for read.
		 */
		uint8_t SDO_ACTIVE 		: 1;
	};

} AD9520_SpiCtrl_t;

/*
 * @brief PFD charge pump control.
 */
typedef union {
	uint8_t PfdChargePumpCfg;
	struct {
		/* 0 - normal operation, this mode must be selected to use the PLL.
		 * 1 - asynchronous power-down (default).
		 * 2 - unused.
		 * 3 - synchronous power-down.
		 */
		uint8_t PLL_POWER_DOWN 	: 2;
		/* 0 - high impedance state.
		 * 1 - forces source current (pump up).
		 * 2 - forces sink current (pump down).
		 * 3 - normal operation (default).
		 */
		uint8_t CP_MODE 		: 2;
		/* From 0 to 7 current is from 0.6ma to 4.8ma with CPset = 5.1kOhm. */
		uint8_t CP_CURRENT 		: 3;
		/* 0 - positive (higher control voltage produces higher frequency) (default).
		 * 1 - negative (higher control voltage produces lower frequency).
		 */
		uint8_t PFD_POLARITY 	: 1;
	};

} AD9520_PfdChargePumpCfg_t;

/*
 * @brief PLL control 1.
 */
typedef union {
	uint8_t PllCtrl1;
	struct {
		uint8_t PRESCALER_P 	: 3;
		uint8_t B_CNT_BYPASS 	: 1;
		uint8_t RST_ALL_CNT 	: 1;
		uint8_t RST_AB_CNT 		: 1;
		uint8_t RST_R_CNT 		: 1;
		uint8_t SET_CP_PIN 		: 1;
	};

} AD9520_PllCtrl1_t;

/*
 * @brief PLL control 2.
 */
typedef union {
	uint8_t PllCtrl2;
	struct {
		uint8_t ANTIBACKLASH_PULSE_WIDTH : 2;
		uint8_t STATUS_PIN_CTRL : 6;
	};

} AD9520_PllCtrl2_t;

/*
 * @brief PLL control 3.
 */
typedef union {
	uint8_t PllCtrl3;
	struct {
		uint8_t VCO_CALIB_NOW 		: 1;
		uint8_t VCO_CALIB_DIV 		: 2;
		uint8_t DIS_DLD 			: 1;
		uint8_t DIS_LDW 			: 1;
		uint8_t LOCK_DETECT_CNT 	: 2;
		uint8_t CMOS_REFIN_OFFSET 	: 1;
	};

} AD9520_PllCtrl3_t;

/*
 * @brief PLL control 4.
 */
typedef union {
	uint8_t PllCtrl4;
	struct {
		uint8_t NPATH_DELAY 		: 3;
		uint8_t RPATH_DELAY 		: 3;
		uint8_t RAB_CNT_SYNCPIN_RST : 2;
	};

} AD9520_PllCtrl4_t;

/*
 * @brief PLL control 5.
 */
typedef union {
	uint8_t PllCtrl5;
	struct {
		uint8_t LD_PIN_CTRL 		: 6;
		uint8_t REF_FREQ_MON_THR 	: 1;
		uint8_t EN_STATUS_PIN_DIV 	: 1;
	};

} AD9520_PllCtrl5_t;

/*
 * @brief PLL control 6.
 */
typedef union {
	uint8_t PllCtrl6;
	struct {
		uint8_t REFMON_PIN_CTRL 	: 5;
		uint8_t EN_REF1_FREQ_MON 	: 1;
		uint8_t EN_REF2_FREQ_MON 	: 1;
		uint8_t EN_VCO_FREQ_MON 	: 1;
	};

} AD9520_PllCtrl6_t;

/*
 * @brief PLL control 7.
 */
typedef union {
	uint8_t PllCtrl7;
	struct {
		uint8_t EN_DIFF_REF 		: 1;
		uint8_t EN_REF1 			: 1;
		uint8_t EN_REF2 			: 1;
		uint8_t STAY_ON_REF2 		: 1;
		uint8_t EN_AUTO_REF_SW 		: 1;
		uint8_t USE_REF_SEL_PIN 	: 1;
		uint8_t SEL_REF2 			: 1;
		uint8_t DIS_SW_DEGLITCH 	: 1;
	};

} AD9520_PllCtrl7_t;

/*
 * @brief PLL control 8.
 */
typedef union {
	uint8_t PllCtrl8;
	struct {
		uint8_t EN_HOLDOVER 		: 1;
		uint8_t EN_EXT_HOLDOVER 	: 1;
		uint8_t RESERVED 			: 1;
		uint8_t EN_LD_PIN_CMP 		: 1;
		uint8_t DIS_PLL_STAT_REG 	: 1;
		uint8_t EN_CLK_DOUBLER 		: 1;
		uint8_t EN_XTAL_OSC 		: 1;
		uint8_t EN_STEEP_ON_STPIN 	: 1;
	};

} AD9520_PllCtrl8_t;

/*
 * @brief PLL control 9.
 */
typedef union {
	uint8_t PllCtrl9;
	struct {
		uint8_t RESERVED0 			: 1;
		uint8_t EN_ZDELAY 			: 1;
		uint8_t EN_EXT_ZDELAY 		: 1;
		uint8_t EXT_ZDELAY_CHDIV 	: 2;
		uint8_t RESERVED1 			: 3;
	};

} AD9520_PllCtrl9_t;

/*
 * @brief PLL readback (status register, read only).
 */
typedef union {
	uint8_t PllReadBack;
	struct {
		uint8_t DLD 				: 1;
		uint8_t REF1_FREQ_THROVF 	: 1;
		uint8_t REF2_FREQ_THROVF 	: 1;
		uint8_t VCO_FREQ_THROVF 	: 1;
		uint8_t REF2_SELECTED 		: 1;
		uint8_t HOLDOVER_ACTIVE 	: 1;
		uint8_t VCO_CALIB_FINISHED 	: 1;
		uint8_t RESERVED 			: 1;
	};

} AD9520_PllReadBack_t;

/*
 * @brief Output control.
 */
typedef union {
	uint8_t OutCtrl;
	struct {
		uint8_t LVPECL_POWER_DOWN	: 1;
		uint8_t LVPECL_DIFF_VOLTAGE	: 2;
		uint8_t POLARITY 			: 2;
		uint8_t CMOS_CFG 			: 2;
		uint8_t OUT_FORMAT 			: 1;
	};

} AD9520_OutCtrl_t;

/*
 * @brief Divider control registers.
 */
typedef struct {
	union {
		uint8_t DivLowHighCycles;
		struct {
			uint8_t DIV_HIGH_CYCLES		: 4;
			uint8_t DIV_LOW_CYCLES		: 4;
		};
	};

	union {
		uint8_t DivCtrl;
		struct {
			uint8_t PHASE_OFFSET		: 4;
			uint8_t DIV_CLK_START		: 1;
			uint8_t DIV_FORCE			: 1;
			uint8_t DIV_CHIP_LEVEL_SYNC	: 1;
			uint8_t DIV_BYPASS			: 1;
		};
	};

	union {
		uint8_t ChannelGroupCtrl;
		struct {
			uint8_t DIV_DCC_EN_DIS		: 1;
			uint8_t CH_DIRECT_TO_OUT	: 1;
			uint8_t CH_PWR_DOWN			: 1;
			uint8_t RESERVED			: 5;
		};
	};


} AD9520_DivCtrl_t;

/*
 * @brief Input clocks control.
 */
typedef union {
	uint8_t InClkCtrl;
	struct {
		uint8_t BYPASS_VCO_DIV			: 1;
		uint8_t SEL_VCO_OR_CLK			: 1;
		uint8_t PWRDOWN_VCO_AND_CLK		: 1;
		uint8_t PWRDOWN_VCO_INTERFACE	: 1;
		uint8_t PWRDOWN_CLK_IN_SECTION	: 1;
		/* Default 0x01. */
		uint8_t RESERVED0				: 2;
		uint8_t RESERVED1				: 1;
	};

} AD9520_InClkCtrl_t;

/*
 * @brief Power down and sync control.
 */
typedef union {
	uint8_t PwrDownAndSyncCtrl;
	struct {
		uint8_t SOFT_SYNC				: 1;
		uint8_t PWRDOWN_DISTRIB_REF		: 1;
		uint8_t PWRDOWN_SYNC			: 1;
		uint8_t DIS_PWRON_SYNC			: 1;
		uint8_t RESERVED				: 4;
	};

} AD9520_PwrDownAndSyncCtrl_t;

/*
 * @brief EEPROM control.
 */
typedef union {
		uint8_t EepromCtrl1;
		struct {
			uint8_t EN_EEPROM_WRITE		: 1;
			uint8_t SOFT_EEPROM			: 1;
			uint8_t RESERVED			: 6;
		};

} AD9520_EepromCtrl_t;

/*
 * @brief Instruction word + data buffer.
 */
typedef struct __attribute__((aligned(1), packed)) {
	union {
		uint16_t InstrHeader;

		struct {
			uint8_t InstrHeader_LSB;
			uint8_t InstrHeader_MSB;
		};

		struct {
			/* The address within the register map that is written to or read from. */
			uint16_t ADDR 				: 13;
			/* Length of the transfer in bytes:
			 *	0 - that is one byte,
			 * 	1 - that is two bytes,
			 * 	2 - that is three bytes,
			 *	3 - streaming mode.
			 */
			uint16_t BYTE_RXTX_COUNT 	: 2;
			/* 0x01 - to read, 0x00 - to write. */
			uint16_t READ_WRITE 		: 1;
		};
	};
	uint8_t RxTxData[5];

}  AD9520_RxTxData_t;


/*
 * @brief
 */
typedef struct {

	/* Part ID */
	uint8_t PartId;
	/* Customer version ID. */

	uint16_t CustId;
	/* Buffer data struct to read/write */
	AD9520_RxTxData_t Data;
	/* Function pointers. */
	delay_fptr delay_fp;
	spi_rxtx_fptr spi_rx_fp;
	spi_rxtx_fptr spi_tx_fp;

}  AD9520_GStr_t;

/* Public function prototypes. */
uint8_t AD9520_Init(void);
void AD9520_ExtDistribModeSelect(void);
void AD9520_IntVcoModeSelect(void);
void AD9520_ResetCtrl(uint8_t RstCtrl);

uint8_t AD9520_GetPartID(void);
uint8_t AD9520_GetEepromStatus(void);
uint8_t AD9520_GetEepromError(void);
AD9520_PllReadBack_t* AD9520_GetPllReadbackCtrl(void);
uint16_t AD9520_EepCustIdRW(uint8_t ReadWrite, uint16_t CustId);

void AD9520_ReadBackCtrl(uint8_t ReadBackActive);
void AD9520_SpiCtrl(uint8_t SdoActive, uint8_t MsbLsbCtrl, uint8_t SoftReset);

void AD9520_IoUpdate(void);
void AD9520_ACounterCfg(uint8_t CntVal);
void AD9520_RCounterCfg(uint16_t CntVal);
void AD9520_BCounterCfg(uint16_t CntVal);
void AD9520_PfdChargePumpCtrl(uint8_t PllPowerDown, uint8_t CpMode, uint8_t CpCurrent, uint8_t PfdPolarity);
void AD9520_PllCtrl1(uint8_t PrescP, uint8_t BCntBypass, uint8_t RstAllCnt, uint8_t RstABCnt, uint8_t RstRCnt, uint8_t SetCPPin);
void AD9520_PllCtrl2(uint8_t AntibacklashPulseWidth, uint8_t StatusPinCtrl);
void AD9520_PllCtrl3(uint8_t VcoCalibNow, uint8_t VcoCalibDiv, uint8_t  DisDLD, uint8_t  DisLDW, uint8_t LockDetectCnt, uint8_t CmosRefinOffset);
void AD9520_PllCtrl4(uint8_t NPathDelay, uint8_t RPathDelay, uint8_t RABCntSyncPinRst);
void AD9520_PllCtrl5(uint8_t LedPinCtrl, uint8_t RefFreqMonThr, uint8_t EnStatusMonPinDiv);
void AD9520_PllCtrl6(uint8_t RefMonPinCtrl, uint8_t EnRef1FreqMon, uint8_t EnRef2FreqMon, uint8_t EnVcoFreqMon);
void AD9520_PllCtrl7(uint8_t EnDiffRef, uint8_t EnRef1, uint8_t  EnRef2, uint8_t StayOnRef2, uint8_t EnAutoRefSw, uint8_t UseRefSelPin, uint8_t SelRef2, uint8_t DisSwDeglitch);
void AD9520_PllCtrl8(uint8_t EnHoldover, uint8_t EnExtHoldover, uint8_t EnLdPinCmp, uint8_t DisPllStatus, uint8_t EnClkDoubler, uint8_t EnXtalOsc, uint8_t EnStatEepAtStatPin);
void AD9520_PllCtrl9(uint8_t EnZeroDelay, uint8_t EnExtZeroDelay, uint8_t ExtZeroDelayChDivSel);
void AD9520_OutCtrl(uint8_t OutNumber, uint8_t OutPowerDownLVPECL, uint8_t OutDiffVoltageLVPECL, uint8_t OutPolarity, uint8_t OutCfgCMOS, uint8_t OutFormat);
void AD9520_EnDisCSDLDToOut(uint8_t RegNumber, uint8_t State);
void AD9520_ChDividersCtrl(uint8_t DivNumber, uint8_t DivHighCycles, uint8_t DivLowCycles, uint8_t PhaseOffset, uint8_t ClkOutStart,
							uint8_t DivOutForce, uint8_t IgnoreSync, uint8_t DivBypass, uint8_t DivDccEnDis, uint8_t ChXDirect,  uint8_t ChXPwrDown);
void AD9520_VcoDivCtrl(uint8_t VcoDiv);
void AD9520_InClkCtrl(uint8_t VcoBypass, uint8_t VcoOrClkAsIn, uint8_t PwrDownVcoAndClk, uint8_t PwrDownVcoClkInterface, uint8_t PwrDownClkInSection);
void AD9520_PwrDownSyncCtrl(uint8_t SoftSync, uint8_t PwrDownDistrRef, uint8_t PwrDownSync, uint8_t DisPowerOnSync);
void AD9520_EepromCtrl(uint8_t EnEepromWrite, uint8_t SoftEeprom);
void AD9520_EepromWrite(void);

#endif  /* AD9520_H_ */
