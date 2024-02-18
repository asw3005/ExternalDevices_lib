/*
 * hmc833.c source file.
 *
 * Created on: Sep 7, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "hmc833.h"
#include "math.h"
#include "Common.h"

/* External variables. */
#ifdef HMC833_SPI_HARD
extern SPI_HandleTypeDef hspi1;
#endif /* HMC833_SPI_HARD */


/* Private function prototypes. */
static uint32_t HMC833_ReadReg(uint8_t RegAddress);
static void HMC833_WriteReg(uint8_t RegAddress, uint32_t Data);
#ifdef HMC833_SPI_HARD
static void HMC833_SpiTxData(uint8_t *pData, uint8_t Size);
static void HMC833_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
#endif /* HMC833_SPI_HARD */
#ifdef HMC833_SPI_SOFT
static void HMC833_ESpiClk(void);
static void HMC833_ESpiTxData(uint8_t *pData, uint8_t Size);
static void HMC833_ESpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
#endif /* HMC833_SPI_SOFT */

/* Private variables. */
#ifdef HMC833_SPI_HARD
static SPI_HandleTypeDef* HMC833Spi = &hspi1;
#endif /* HMC833_SPI_HARD */
static HMC833_GStr_t hmc833 = {
		.delay_fp = HAL_Delay,
		#ifdef HMC833_SPI_HARD
		.rx_fp = HMC833_SpiRxData,
		.tx_fp = HMC833_SpiTxData
		#endif /* HMC833_SPI_HARD */
		#ifdef HMC833_SPI_SOFT
		.rx_fp = HMC833_ESpiRxData,
		.tx_fp = HMC833_ESpiTxData
		#endif /* HMC833_SPI_SOFT */
};

/*
 * @brief Init device.
 */
void HMC833_Init(void) {

	uint32_t ChipID = 0;

	/* Clock select. */
	float fvco, Nint, Nfrac, R, fxtal, fpd, ffrac, fn/*, fvcocheck, foutcheck*/;
	uint16_t ExactReg = 0;
	/* Pin init. */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOG_CLK_ENABLE();

	GPIO_InitStruct.Pin = HMC833_CE_PIN | HMC833_CS_PIN | HMC833_CLK_PIN | HMC833_SDI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HMC833_CS_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = HMC833_SDO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HMC833_SDO_PORT, &GPIO_InitStruct);

	/* Enable chip. Select HMC mode. */
	HAL_GPIO_WritePin(HMC833_CE_PORT, HMC833_CE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HMC833_CLK_PORT, HMC833_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);

	/* Check is device available? */
	ChipID = HMC833_GetChipId();
	if (ChipID != HMC833_CHIP_ID) {
		__NOP();
		return;
	}

	/* Calculate the frequency you need. */
	fxtal 	= 60.0f;
	fvco 	= 2805.0f;
	R 		= 1.0f;
	Nfrac 	= 0.0f;

	fpd 	= fxtal / R;
	Nint 	= floorf(fvco / fpd);
	//Nfrac   = (fvco / fpd) - Nint;
	//ffrac = roundf(Nfrac * 16777216.0f);

	fn = Nint * fpd;
	ffrac = ceilf((16777216.0f * (fvco - fn)) / fpd);

	ExactReg = fpd / (gcd(fvco, fpd));


	/* Check parameters. */
//	fvcocheck = (fpd * 1000000) * (Nint + (ffrac / 16777216.0f));
//	foutcheck = fvcocheck / 1;


	/* Configuring fractional or integer modes. */
	HMC833_SdCfg(2, 2, 0, 1, 1, 1, 0, 0);
	/* Select reference divider. */
	HMC833_RefDivCfg(1);
	/* Select analog enable configuration. */
	HMC833_AnalogEnable(1, 0);

	/* VCO subsystem. */
	HMC833_VcoBiases(1, 3, 0);

	HAL_Delay(100);
	/* Auto calibration. Set default address for the VCO subsystem. */
	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_TUNING, HMC833_GetVcoTune()->VcoTuneReg);

	/* Set integer part. */
	HMC833_SetFreqIntPart(Nint);

	HMC833_ExactFreqchPerFpd(ExactReg);
	/* Set fractional part. */
	HMC833_SetFreqFracPart(ffrac);

	__NOP();

}

void HMC833_SetFreq(float Frequency) {

	/* Clock select. */
	float fvco, Nint, Nfrac, R, fxtal, fpd, ffrac, fn;
	uint16_t ExactReg = 0;

//	/* Calculate the frequency you need. */
//	fxtal 	= 60.0f;
//	fvco 	= Frequency;
//	R 		= 1.0f;
//	Nfrac 	= 0.0f;
//
//	fpd 	= fxtal / R;
//	Nint 	= floorf(fvco / fpd);
//	Nfrac   = (fvco / fpd) - Nint;
//	ffrac = roundf(Nfrac * 16777216.0f);
//
//	HAL_Delay(100);
//	/* Auto calibration. Set default address for the VCO subsystem. */
//	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_TUNING, HMC833_GetVcoTune()->VcoTuneReg);
//
//	/* Set integer part. */
//	HMC833_SetFreqIntPart(Nint);
//	/* Set fractional part. */
//	HMC833_SetFreqFracPart(ffrac);

	/* Calculate the frequency you need. */
	fxtal 	= 60.0f;
	fvco 	= Frequency;
	R 		= 1.0f;
	Nfrac 	= 0.0f;

	fpd 	= fxtal / R;
	Nint 	= floorf(Frequency / fpd);
	//Nfrac   = (fvco / fpd) - Nint;
	//ffrac = roundf(Nfrac * 16777216.0f);

	fn = Nint * fpd;
	ffrac = ceilf((16777216.0f * (Frequency - fn)) / fpd);

	ExactReg = fpd / (gcd(Frequency, fpd));

	HAL_Delay(100);
	/* Auto calibration. Set default address for the VCO subsystem. */
	//HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_TUNING, HMC833_GetVcoTune()->VcoTuneReg);

	/* Set integer part. */
	HMC833_SetFreqIntPart(Nint);

	HMC833_ExactFreqchPerFpd(ExactReg);
	/* Set fractional part. */
	HMC833_SetFreqFracPart(ffrac);
}

/* PLL SUBSYSTEM FUNCTIONS */

/*
 * @brief Reading the chip ID.
 */
uint32_t HMC833_GetChipId(void) {

	return HMC833_ReadReg(HMC833_ID);
}

/*
 * @brief Reset register configuration (default 000002h).
 *
 * @param RstChipenPinsel 	: 0 - take PLL enable via SPI (rst_chipen_from_spi) Reg01[1] (default),
 * 							  1 - take PLL enable via CEN pin, see power down mode description.
 * @param RstChipenFromSpi 	: SPI’s PLL enable bit (default 1).
 * @param KeepBiasOn 		: When PLL is disabled, keeps internal bias generators on, ignores chip enable control (default 0).
 * @param KeepPdOn 			: When PLL is disabled, keeps PD circuit on, ignores chip enable control (default 0).
 * @param KeepCpOn 			: When PLL is disabled, keeps Charge Pump on, ignores chip enable control (default 0).
 * @param KeepRefBuffOn 	: When PLL is disabled, keeps Reference buffer block on, ignores Chip enable control (default 0).
 * @param KeepVcoOn 		: When PLL is disabled, keeps VCO divider buffer on, ignores Chip enable control (default 0).
 * @param KeepGpoDrvOn 		: When PLL is disabled, keeps GPO output Driver On, ignores Chip enable control (default 0).
 */
void HMC833_RstRegCfg(uint8_t RstChipenPinsel, uint8_t RstChipenFromSpi, uint8_t KeepBiasOn, uint8_t KeepPdOn,
					  uint8_t KeepCpOn, uint8_t KeepRefBuffOn, uint8_t KeepVcoOn, uint8_t KeepGpoDrvOn) {

	HMC833_RstReg_t Rst = { 0 };

	Rst.RST_CHIPEN_PIN_SEL = RstChipenPinsel;
	Rst.RST_CHIPEN_FROM_SPI = RstChipenFromSpi;
	Rst.KEEP_BIAS_ON = KeepBiasOn;
	Rst.KEEP_PD_ON = KeepPdOn;
	Rst.KEEP_CP_ON = KeepCpOn;
	Rst.KEEP_REF_BUFF_ON = KeepRefBuffOn;
	Rst.KEEP_VCO_ON = KeepVcoOn;
	Rst.KEEP_GPO_DRIVER_ON = KeepGpoDrvOn;
	Rst.RESERVED31_8 = 0;

	HMC833_WriteReg(HMC833_RST, Rst.RstReg);
}

/*
 * @brief Reference divider register configuration (default 000001h).
 *
 * @param RefDiv : reference Divider ’R’ Value, divider use also requires refBufEn Reg08[3] = 1 and divider min 1 max 16383 (default 1).
 */
void HMC833_RefDivCfg(uint32_t RefDiv) {

	HMC833_WriteReg(HMC833_REFDIV, RefDiv);
}

/*
 * @brief Frequency register, integer part configuration (default 000019h).
 *
 * @param IntValue : VCO Divider Integer part, used in all modes.
 * 					 Fractional Mode min 20 max 2^19-4 = 7FFFCh = 524,284.
 * 					 Integer mode min 16 max 2^19-1 = 7FFFFh = 524,287.
 */
void HMC833_SetFreqIntPart(uint32_t IntValue) {

	HMC833_WriteReg(HMC833_FREQ_INTEGER_PART, IntValue);
}

/*
 * @brief Frequency register, fractional part configuration (default 000000h).
 *
 * @param FracValue : VCO Divider Fractional part (24-bit unsigned).
 * 					  Used in fractional mode only.
 * 					  Nfrac = Reg04h / 2^24, min 0 max 2^24-1.
 */
void HMC833_SetFreqFracPart(uint32_t FracValue) {

	HMC833_WriteReg(HMC833_FREQ_FRACTIONAL_PART, FracValue);
}

/*
 * @brief VCO subsystem register configuration (default 000000h).
 * Reg05h is a special register used for indirect addressing of the VCO subsystem. Writes to Reg05h are automatically forwarded to
 * the VCO subsystem by the VCO SPI state machine controller.
 * Reg05h is a Read-Write register. However, Reg05h only holds the contents of the last transfer to the VCO subsystem. Hence it is
 * not possible to read the full contents of the VCO subsystem. Only the content of the last transfer to the VCO subsystem can be read.
 * Please take note special considerations for AutoCal related to Reg05h.
 * VCO subsystem ID and register address are not modified by the AutoCal state machine. Hence, if a manual access is done to a VCO
 * Subsystem register the user must reset the register address to zero before a change of frequency which will re-run AutoCal.
 *
 * @param VcoSubsysId 		: Internal VCO Subsystem ID (VCO subsystem ID 0).
 * @param VcoSubsysRegAddr 	: VCO subsystem register address.
 * @param VcoSubsysData 	: VCO subsystem data.
 */
void HMC833_VcoSubsys(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint16_t VcoSubsysData) {

	HMC833_SubsysVco_t VcoSubsys = { 0 };

	VcoSubsys.VCO_SUBSYS_ID = VcoSubsysId;
	VcoSubsys.VCO_SUBSYS_REG_ADDR = VcoSubsysRegAddr;
	VcoSubsys.VCO_SUBSYS_DATA = VcoSubsysData;

	HMC833_WriteReg(HMC833_VCO_SPI, VcoSubsys.SubsysVcoDataReg);
}

/*
 * @brief VCO SPI register configuration (default 000000h).
 * Reg05h is a special register used for indirect addressing of the VCO subsystem. Writes to Reg05h are automatically forwarded to
 * the VCO subsystem by the VCO SPI state machine controller.
 * Reg05h is a Read-Write register. However, Reg05h only holds the contents of the last transfer to the VCO subsystem. Hence it is
 * not possible to read the full contents of the VCO subsystem. Only the content of the last transfer to the VCO subsystem can be read.
 * Please take note special considerations for AutoCal related to Reg05h.
 * VCO subsystem ID and register address are not modified by the AutoCal state machine. Hence, if a manual access is done to a VCO
 * Subsystem register the user must reset the register address to zero before a change of frequency which will re-run AutoCal.
 *
 * @param VcoSubsysId 			: Internal VCO Subsystem ID (VCO subsystem ID 0).
 * @param VcoSubsysRegAddr 		: VCO subsystem register address.
 * @param CalibTuneVoltageOff 	: VCO subsystem data.
 * @param VcoCaps 				: VCO caps.
 * @param SubVcoSel 			: VCO select.
 *
 */
void HMC833_VcoSpi(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint8_t CalibTuneVoltageOff, uint8_t VcoCaps, uint8_t SubVcoSel) {

	HMC833_VcoSpi_t VcoSubsys = { 0 };

	VcoSubsys.VCO_SUBSYS_ID = VcoSubsysId;
	VcoSubsys.VCO_SUBSYS_REG_ADDR = VcoSubsysRegAddr;
	VcoSubsys.CALIB_TUNE_VOLTAGE_OFF = CalibTuneVoltageOff;
	VcoSubsys.VCO_CAPS = VcoCaps;
	VcoSubsys.VCO_SELECT = SubVcoSel;

	HMC833_WriteReg(HMC833_VCO_SPI, VcoSubsys.VcoSpiDataReg);
}

/*
 * @brief SD CFG register configuration (default 200B4Ah).
 *
 * @param Seed 				: Selects the seed in fractional mode. Writes to this register are stored in the HMC833LP6GE and are only
 * 							  loaded into the modulator when a frequency change is executed and if AutoSeed Reg06h[8] = 1.
 * 								0 - 0 seed,
 * 								1 - lsb seed,
 * 								2 - B29D08h seed (default),
 * 								3 - 50F1CDh seed.
 * @param Order 			: Select the modulator type.
 * 								0 - 1st order,
 * 								1 - 2nd order,
 * 								2 - mode B, recommended (default),
 * 								3 - mode A.
 * @param FracBypass 		: Bypass fractional part. In bypass fractional modulator output is ignored, but fractional modulator continues
 * 							  to be clocked if frac_rstb = 1, can be used to test the isolation of the digital fractional modulator from
 * 							  the VCO output in integer mode.
 * 							  	0 - use modulator, required for fractional mode (default),
 * 							  	1 - bypass modulator, required for integer mode.
 * @param AutoSeed 			:  Enables or disables seed loading from the seed field of the register.
 * 								0 - when fractional register write changes frequency, modulator starts with previous contents,
 * 								1 - loads the seed whenever the fractional register is written (default).
 * @param ClkrqRefdivSel 	: Selects the modulator clock source, for test only.
 * 								0 - ref divider clock ignored if bits [10] or [21] are set,
 * 								1 - VCO divider clock (recommended for normal operation) (default).
 * @param SdEn 				: This register controls whether autocal starts on an integer or a fractional write.
 * 								0 - disable fractional core, use for integer mode or integer mode with CSP,
 * 								1 - enable fractional core, required for fractional mode, or integer isolation testing (default).
 * @param BistEn 			: Enable built in self test (default 0).
 * @param RdivBistCycles 	: RDiv BIST cycles.
 * 								0 - 1032 (default),
 * 								1 - 2047,
 * 								2 - 3071
 * 								3 - 4095
 */
void HMC833_SdCfg(uint8_t Seed, uint8_t Order, uint8_t FracBypass, uint8_t AutoSeed, uint8_t ClkrqRefdivSel, uint8_t SdEn, uint8_t BistEn,
			  uint8_t RdivBistCycles) {

	HMC833_SdCfg_t SdCfg = { 0 };

	SdCfg.SEED = Seed;
	SdCfg.ORDER = Order;
	SdCfg.FRAC_BYPASS = FracBypass;
	SdCfg.AUTO_SEED = AutoSeed;
	SdCfg.CLKRQ_REFDIV_SEL = ClkrqRefdivSel;
	SdCfg.SD_ENABLE = SdEn;
	SdCfg.BIST_EN = BistEn;
	SdCfg.RDIV_BIST_CYCLES = RdivBistCycles;
	SdCfg.RESERVED15_12 = 0;
	SdCfg.RESERVED17_16 = 3;
	SdCfg.RESERVED22 = 0;

	HMC833_WriteReg(HMC833_SD_CFG, SdCfg.SdSfgReg);
}



/*
 * @brief Reset register configuration (default 00014Dh).
 *
 * @param LdWinCnt 			: Lock detect window. Sets the number of consecutive counts of divided VCO that must land inside the Lock
 * 							  Detect Window to declare LOCK (default 5).
 * 							  	0 - 5,   1 - 32,   2 - 96,   3 - 256,
 * 							  	4 - 512, 5 - 2048, 6 - 8192, 7 - 65535.
 * @param EnIntLd 			: Enable internal lock detect (default 1).
 * @param LdWinType 		: Lock detection window timer selection.
 * 								0 - analog one shot, nominal 10 ns window,
 * 								1 - digital programmable timer (default).
 * @param LdDigWinDuration 	: Lock detection digital window duration (default 2).
 *								0 - 1/2 cycles, 1 - 1 cycles,  2 - 2 cycles,  3 - 4 cycles,
 *								4 - 8 cycles,   5 - 16 cycles, 6 - 32 cycles, 7 - 64 cycles.
 * @param LdDigTimFreqCtrl 	: Lock detection digital timer frequency control.
 * 								0 - fastest (default),
 * 								3 - slowest.
 * @param LdTimTestMode 	: Lock detect timer test mode.
 * 								0 - normal timer operation, one shot (default),
 * 								1 - force timer clock on continuously, for test only.
 * @param AutoRelockOneTry 	: Attempts to relock if Lock Detect fails for any reason. Only tries once (default 0).
 */
void HMC833_LockDetect(uint8_t LdWinCnt, uint8_t EnIntLd, uint8_t LdWinType, uint8_t LdDigWinDuration, uint8_t LdDigTimFreqCtrl,
					   uint8_t LdTimTestMode, uint8_t AutoRelockOneTry) {

	HMC833_LockDetect_t LockDetect = { 0 };

	LockDetect.LKD_WINCNT_MAX = LdWinCnt;
	LockDetect.EN_INT_LOCK_DETECT = EnIntLd;
	LockDetect.LOCK_DETECT_WIN_TYPE = LdWinType;
	LockDetect.LD_DIG_WIN_DUR = LdDigWinDuration;
	LockDetect.LD_DIG_TIM_FREQ_CTRL = LdDigTimFreqCtrl;
	LockDetect.LD_TIM_TEST_MODE = LdTimTestMode;
	LockDetect.AUTORELOCK_ONE_TRY = AutoRelockOneTry;
	LockDetect.RESERVED5_4 = 0;

	HMC833_WriteReg(HMC833_LOCK_DETECT, LockDetect.LockDetectReg);
}

/*
 * @brief Analog EN register configuration (default C1BEFFh).
 *
 * @param GpoPadEn 		: LD_SDO pin control.
 * 							0 - pin LD_SDO disabled,
 * 							1 - and RegFh[LDO_DRV_ALWAYS_ON] = 1 , pin LD_SDO is always on required for use of GPO port,
 * 							    and RegFh[LDO_DRV_ALWAYS_ON] = 0 SPI LDO_SPI is off if unmatched chip address is seen on the SPI, allowing
 * 							    a shared SPI with other compatible parts (default).
 * @param HighFreqRef 	: High frequency reference select.
 * 							0 - for ref < 200 MHz (default),
 * 							1 - for ref >= 200 MHz.
 */
void HMC833_AnalogEnable(uint8_t GpoPadEn, uint8_t HighFreqRef) {

	HMC833_AnalogEn_t AnalogEn = { 0 };

	AnalogEn.GPO_PAD_EN = GpoPadEn;
	AnalogEn.HIGH_FREQ_REF = HighFreqRef;
	AnalogEn.BIAS_EN = 1;
	AnalogEn.CP_EN = 1;
	AnalogEn.PD_EN = 1;
	AnalogEn.REFBUF_EN = 1;
	AnalogEn.VCOBUF_EN = 1;
	AnalogEn.RESERVED6 = 1;
	AnalogEn.VCO_DIV_CLK_TO_DIG_EN = 1;
	AnalogEn.RESERVED8 = 0;
	AnalogEn.PRESC_CLOCK_EN = 1;
	AnalogEn.VCO_BUFF_PRESC_BIAS_EN = 1;
	AnalogEn.CHARGE_PUMP_INT_OAMP_EN = 1;
	AnalogEn.RESERVED14_12 = 3;
	AnalogEn.RESERVED17_15 = 3;
	AnalogEn.RESERVED20_19 = 0;
	AnalogEn.RESERVED23_22 = 3;

	HMC833_WriteReg(HMC833_ANALOG_EN, AnalogEn.AnalogEnableReg);
}

/*
 * @brief Charge pump register configuration (default 403264h).
 *
 * @param CpDnGain 		: Charge Pump DN Gain Control 20 μA per step. Affects fractional phase noise and lock detect settings (default 100).
 * 							0 - 0 uA,
 * 							1 - 20 uA,
 * 							2 - 40 uA,
 * 							...
 * 							127 - 2.54mA.
 * @param CpUpGain 		: Charge Pump UP Gain Control 20 μA per step. Affects fractional phase noise and lock detect settings (default 100).
 * 							0 - 0 uA,
 * 							1 - 20 uA,
 * 							2 - 40 uA,
 * 							...
 * 							127 - 2.54mA.
 * @param OffsetMag 	: Charge Pump Offset Control 5 μA per step. Affects fractional phase noise and lock detect settings (default 0).
 * 							0 - 0 uA,
 * 							1 - 5 uA,
 * 							2 - 10 uA,
 * 							...
 * 							127 -  635 uA.
 * @param OffsetUpEn 	: Recommended setting 0.
 * @param OffsetDnEn 	: Recommended setting 1 in fractional mode, otherwise 0.
 * @param HiKcp 		: HiKcp high current charge pump.
 */
void HMC833_ChargePump(uint8_t CpDnGain, uint8_t CpUpGain, uint8_t OffsetMag, uint8_t OffsetUpEn, uint8_t OffsetDnEn, uint8_t HiKcp) {

	HMC833_ChargePump_t ChargePump = { 0 };

	ChargePump.CP_DN_GAIN = CpDnGain;
	ChargePump.CP_UP_GAIN = CpUpGain;
	ChargePump.OFFSET_MAGNITUDE = OffsetMag;
	ChargePump.OFFSET_UP_EN = OffsetUpEn;
	ChargePump.OFFSET_DN_EN = OffsetDnEn;
	ChargePump.HI_KCP = HiKcp;

	HMC833_WriteReg(HMC833_CHARGE_PUMP, ChargePump.ChargePumpReg);
}

/*
 * @brief VCO autocal register configuration (default 002205h).
 *
 * @param VtuneRes 			: R divider cycles (default 5).
 * 								0 - 1,  1 - 2,  2 - 4,   3 - 8,
 * 								4 - 32, 5 - 64, 6 - 128, 7 - 256.
 * @param VcoCurveAdj 		: Program 0. VCO curve adjustment vs temperature for AutoCal (default 0).
 * 								0 - disabled,  1 - +1 curves, 2 - +2 curves, 3 - +3 curves,
 * 								4 - -4 curves, 5 - -3 curves, 6 - -2 curves, 7 - -1 curve.
 * @param WaitStateSetUp 	: Program 1. Wait State Setup 100 TFSM see section 1.2.4. Tmmt = 1 measurement cycle of AutoCal (default 0).
 * 								0 - wait only at startup,
 * 								1 - wait on startup and after first Tmmt cycle,
 * 								2 - wait on startup and after first two Tmmt cycles,
 * 								3 - wait on startup and after first three Tmmt cycles.
 * @param NumOfSarBits 		: Number of SAR bits in VCO (default 2).
 * 								0 - 8 recommended,
 * 								1 - 7 do not use,
 * 								2 - 6 do not use,
 * 								3 - 5 do not use.
 * @param BypassVco 		: Program 0 for normal operation using auto calibration (default 0).
 * @param NoVspiTrg 		: Program 0 for normal operation. If 1, serial transfers to VCO sub-system (via Reg05h) are disabled (default 0).
 * @param FsmVspiClkSel 	: Set the AutoCal FSM and VSPI clock (50 MHz maximum) (default 1).
 * 								0 - input crystal reference,
 * 								1 - input crystal reference/4,
 * 								2 - input crystal reference/16,
 * 								3 - input crystal reference/32.
 * @param XtalFallEdgeFsm 	: Program 0 for normal operation. Program 1 only for BIST use (default 0).
 */
void HMC833_VcoAutocal(uint8_t VtuneRes, uint8_t VcoCurveAdj, uint8_t WaitStateSetUp, uint8_t NumOfSarBits, uint8_t BypassVco,
					   uint8_t NoVspiTrg, uint8_t FsmVspiClkSel, uint8_t XtalFallEdgeFsm) {

	HMC833_VcoAutocal_t VcoAutocal = { 0 };

	VcoAutocal.VTUNE_RESOLUTION = VtuneRes;
	VcoAutocal.VCO_CURVE_ADJ = VcoCurveAdj;
	VcoAutocal.WAIT_STATE_SET_UP = WaitStateSetUp;
	VcoAutocal.NUM_OF_SAR_BITS_IN_VCO = NumOfSarBits;
	VcoAutocal.BYPASS_VCO_TUNING = BypassVco;
	VcoAutocal.NO_VSPI_TRG = NoVspiTrg;
	VcoAutocal.FSM_VSPI_CLK_SEL = FsmVspiClkSel;
	VcoAutocal.XTAL_FALLING_EDGE_FOR_FSM = XtalFallEdgeFsm;
	VcoAutocal.FORCE_CURVE = 0;
	VcoAutocal.FORCE_RDIV_BYPASS = 0;

	HMC833_WriteReg(HMC833_VCO_AUTOCAL, VcoAutocal.VcoAutocalReg);
}

/*
 * @brief Phase detector register configuration (default F8061h).
 *
 * @param PdDelSel 			: Sets PD reset path delay (recommended setting 1, default).
 * @param ShortPdInputs 	: Program 0 for normal operation. Shorts the inputs of the phase frequency detector - test only (default 0).
 * @param PdPhaseSel 		: Program 0 for normal operation (default 0). Inverts PD polarity when 1.
 * @param PdUpEn 			: Enables the PD UP output (default 1).
 * @param PdDnEn 			: Enables the PD DN output (default 1).
 * @param CspMode 			: Cycle slip prevention mode. This delay varies by +- 10% with temperature, and +- 12% with process. Extra
 * 							  current is driven into the loop filter when the phase error is larger than:
 * 							  	0 - disabled (default),
 * 							  	1 - 5.4 ns,
 * 							  	2 - 14.4 ns,
 * 							  	3 - 24.1 ns.
 * @param ForceCpUp 		: Forces CP UP output on - use for test only (default 0).
 * @param ForceCpDown 		: Forces CP DN output on - use for test only (default 0).
 * @param ForceCpMidRail 	: Force CP MId Rail - use for test only (default 0).
 * @param MCntClkGating 	: Program 11. MCounter clock gating.
 * 								0 - MCounter Off,
 * 								1 - N < 128,
 * 								2 - N < 1023,
 * 								3 - all clocks on (recommended setting 11, default).
 */
void HMC833_PhaseDetector(uint8_t PdDelSel, uint8_t ShortPdInputs, uint8_t PdPhaseSel, uint8_t PdUpEn, uint8_t PdDnEn, uint8_t CspMode,
		uint8_t ForceCpUp, uint8_t ForceCpDown, uint8_t ForceCpMidRail, uint8_t MCntClkGating) {

	HMC833_PhaseDetector_t PhaseDetect;

	PhaseDetect.PD_DEL_SEL = PdDelSel;
	PhaseDetect.SHORT_PD_IN = ShortPdInputs;
	PhaseDetect.PD_PHASE_SEL = PdPhaseSel;
	PhaseDetect.PD_UP_EN = PdUpEn;
	PhaseDetect.PD_DN_EN = PdDnEn;
	PhaseDetect.CSP_MODE = CspMode;
	PhaseDetect.FORCE_CP_UP = ForceCpUp;
	PhaseDetect.FORCE_CP_DN = ForceCpDown;
	PhaseDetect.FORCE_CP_MID_RAIL = ForceCpMidRail;
	PhaseDetect.MCNT_CLK_GATING = MCntClkGating;
	PhaseDetect.RESERVED14_12 = 4;
	PhaseDetect.CP_INT_OAMP_BIAS = 3;
	PhaseDetect.SPARE = 0;
	PhaseDetect.RESERVED23_20 = 0;

	HMC833_WriteReg(HMC833_PD, PhaseDetect.PhaseDetectorReg);
}

/*
 * @brief Exact frequency mode register configuration (default 000000h).
 *
 * @param NumberOfChannelsPerFpd : Comparison Frequency divided by the Correction Rate, Must be an integer. Frequencies at exactly the
 * 								   correction rate will have zero frequency error.
 * 								   		0 - disabled (default),
 * 								   		1 - disabled,
 * 								   		2 - 16383 (3FFFh).
 */
void HMC833_ExactFreqchPerFpd(uint16_t NumberOfChannelsPerFpd) {

	HMC833_WriteReg(HMC833_EXACT_FREQ_MODE, NumberOfChannelsPerFpd);
}

/*
 * @brief GPO SPI RDIV register configuration (default 000001h).
 *
 * @param GpoSel 			: Signal selected here is output to SDO pin when enabled.
 * 								0 - data from Reg0F[5],
 *								1 - lock detect output (default),
 *								2 - lock detect Trigger,
 *								3 - lock detect window output,
 *								4 - ring osc Test,
 *								5 - pullup hard from CSP,
 *								6 - pullDN hard from CSP,
 *								7 - reserved,
 *								8 - reference buffer output,
 *								9 - ref divider output,
 *								10 - VCO divider output,
 *								11 - modulator clock from VCO divider,
 *								12 - auxiliary clock,
 *								13 - aux SPI clock,
 *								14 - aux SPI enable,
 *								15 - aux SPI data out,
 *								16 - PD DN,
 *								17 - PD UP,
 *								18 - SD3 clock delay,
 *								19 - SD3 core clock,
 *								20 - autoStrobe integer write,
 *								21 - autostrobe frac write,
 *								22 - autostrobe aux SPI,
 *								23 - SPI latch enable,
 *								24 - VCO divider sync reset,
 *								25 - seed load strobe,
 *								26 - 29 not used,
 *								30 - SPI output buffer en,
 *								31 - soft RSTB.
 * @param GpoTestData 		: 1 - GPO test data (default 0).
 * @param PreventAutomuxSdo : Prevent automux SDO pin.
 * 								0 - automuxes between SDO and GPO data (default),
 * 								1 - outputs GPO data only.
 * @param LdoDrvAlwaysOn 	: LDO driver always on.
 * 								0 - LD_SDO pin driver only on during SPI read cycle (default),
 * 								1 - LD_SDO pin driver always on.
 */
void HMC833_GpoSpiRdiv(uint8_t GpoSel, uint8_t GpoTestData, uint8_t PreventAutomuxSdo, uint8_t LdoDrvAlwaysOn) {

	HMC833_GpoSpiRdiv_t GpoSpiRdiv;

	GpoSpiRdiv.GPO_SELECT = GpoSel;
	GpoSpiRdiv.GPO_TEST_DATA = GpoTestData;
	GpoSpiRdiv.PREVENT_AUTOMUX_SDO = PreventAutomuxSdo;
	GpoSpiRdiv.LDO_DRV_ALWAYS_ON = LdoDrvAlwaysOn;
	GpoSpiRdiv.DIS_PFET = 0;
	GpoSpiRdiv.DIS_NFET = 0;

	HMC833_WriteReg(HMC833_GPO_SPI_RDIV, GpoSpiRdiv.GpoSpiRdivReg);
}

/*
 * @brief VCO tune register (default 000020h).
 *
 * @return VCO_SWITCH_SETTING 	: Read Only Register. Indicates the VCO switch setting selected by the AutoCal state machine to yield the
 * 								  nearest free running VCO frequency to the desired operating frequency. Not valid when Reg10h[8] = 1,
 * 								  AutoCal Busy. Note if a manual change is done to the VCO switch settings this register will not indicate
 * 								  the current VCO switch position. Note: VCO subsystems may not use all the MSBs, in which case the unused
 * 								  bits are don’t care (default 32).
 * 								  	0 - highest frequency,
 * 								  	1 - 2nd highest,
 * 								  	...
 * 								  	256 - lowest frequency.
 * @return AUTOCAL_BUSY 		: Busy when AutoCal state machine is searching for the nearest switch setting to the requested frequency
 * 								  (default 0).
 */
HMC833_VcoTune_t* HMC833_GetVcoTune(void) {

	static HMC833_VcoTune_t VcoTune;

	VcoTune.VcoTuneReg = HMC833_ReadReg(HMC833_VCO_TUNE);
	return &VcoTune;
}

/*
 * @brief SAR register (default 7FFFFh).
 *
 * @return SAR_ERR_MAG_CNTS : SAR error magnitude counts.
 * @return SAR_ERR_SIGN 	: SAR error sign.
 * 								0 - +value (default),
 * 								1 - -value.
 */
HMC833_Sar_t* HMC833_GetSar(void) {

	static HMC833_Sar_t Sar;

	Sar.SarReg = HMC833_ReadReg(HMC833_SAR);
	return &Sar;
}

/*
 * @brief GPO2 register (default 000000h).
 *
 * @return GPO_STATE 	: GPO State.
 * @return LOCK_DETECT 	: Lock detect status.
 * 							0 - unlocked (default 0),
 * 							1 - locked (default 0).
 */
HMC833_Gpo2_t* HMC833_GetGpo2(void) {

	static HMC833_Gpo2_t Gpo2;

	Gpo2.Gpo2Reg = HMC833_ReadReg(HMC833_GPO2);
	return &Gpo2;
}

/*
 * @brief BIST register configuration (default 1259h).
 *
 * @return BIST_SIGNATURE 	: BIST signature (default 4697).
 * @return BIST_BUSY 		: BIST busy (default 0).
 */
HMC833_Bist_t* HMC833_GetBist(void) {

	static HMC833_Bist_t Bist;

	Bist.BistReg = HMC833_ReadReg(HMC833_BIST);
	return &Bist;
}

/* VCO SUBSYSTEM FUNCTIONS */

/*
 * @brief VCO tuning register configuration.
 *
 * @param Cal 	: VCO tune voltage is redirected to a temperature compensated calibration voltage (default 0).
 * @param Caps 	: VCO sub-band selection. Not all sub-bands are used on the various products (default 16).
 * 					0 - max frequency,
 * 					255 - min frequency.
 */
void HMC833_VcoTuning(uint8_t Cal, uint8_t Caps) {

	HMC833_VcoTuning_t VcoTuning;

	VcoTuning.CAL = Cal;
	VcoTuning.CAPS = Caps;

	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_TUNING, VcoTuning.VcoTuningReg);
}

/*
 * @brief VCO enables  register configuration.
 *
 * @param MEnVcoSubsys		: Master enable VCO subsystem (default 1).
 * 								0 - all VCO subsystem blocks Off,
 * 							  	1 - ANDed with local enables only (Manual mode (VCO_Reg03h[2] = 1),
 * 							  	1 - master enable ignores local enables (Auto Mode (VCO_Reg03h[2] = 0).
 * @param ManModePllBuffEn 	: Enables PLL buffer in manual mode only (default 1).
 * @param ManModeRrfBuffEn 	: Enables RF buffer to Output in manual mode only (default 1).
 * @param ManModeDivBy1En 	: Enables RF divide by 1 in manual mode only (default 1).
 * @param ManModeRfDivEn 	: Enables RF divider in manual mode only (default 1).
 */
void HMC833_VcoEnables(uint8_t MEnVcoSubsys, uint8_t ManModePllBuffEn, uint8_t ManModeRfBuffEn, uint8_t ManModeDivBy1En,
					   uint8_t ManModeRfDivEn) {

	HMC833_VcoEnables_t VcoEnables;

	VcoEnables.MASTER_EN_VCO_SUBSYS = MEnVcoSubsys;
	VcoEnables.MANUAL_MODE_PLL_BUFF_EN = ManModePllBuffEn;
	VcoEnables.MANUAL_MODE_RF_BUFF_EN = ManModeRfBuffEn;
	VcoEnables.MANUAL_MODE_DIVBY1_EN = ManModeDivBy1En;
	VcoEnables.MANUAL_MODE_RF_DIV_EN = ManModeRfDivEn;
	VcoEnables.DONT_CARE = 0;

	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_ENABLES, VcoEnables.VcoEnablesReg);
}

/*
 * @brief  VCO biases register configuration.
 *
 * @param RfDivRatio 			: RF divide ratio. This register automatically controls the enables to the, RF output buffer, RF divider,
 * 								  RF divide by 1 path, and requires Master Enable (VCO_Reg01h[0] = 1) and AutoRFO mode (VCO_Reg03h [2] = 0)
 * 								  Note: bit[0] is a don’t care in ManualRFO mode  (default 1).
 * 									0 - mute, VCO and PLL buffer On, RF output stages off,
 *									1 - Fo (default),
 *									2 - Fo/2,
 *									3 - invalid, defaults to 2,
 *									4 - Fo/4,
 *									5 - invalid, defaults to 4,
 *									6 - Fo/6,
 *									...
 *									60 - Fo/60,
 *									61 - invalid, defaults to 60,
 *									62 - Fo/62,
 *									> 62 - invalid, defaults to 62.
 * @param RfOutBuffGainCtrl 	: RF output buffer gain control.
 * 									0 - Max Gain -9 dB,
 * 									1 - Max Gain -6 dB,
 * 									2 - Max Gain -3 dB,
 * 									3 - Max Gain  (default).
 * @param DivOutStageGainCtrl 	: Divider output stage gain control.
 * 								  Used to flatten the output power level across frequency
 * 								  For divide-by 1 or divide-by 2 it is recommended to set this bit to 1. 0 will reduce output power and
 * 								  degrade noise floor performance.
 * 								  For divide-by 4 or higher, it is recommended to set this bit to 0 to maintain flat output power across
 * 								  divider settings. Setting this bit to 1, with divide-by 4 or higher provides higher output power compared
 * 								  to the divide by 1 or 2 case.
 * 									0 - max gain -3 dB,
 * 									1 - max gain.
 */
void HMC833_VcoBiases(uint8_t RfDivRatio, uint8_t RfOutBuffGainCtrl, uint8_t DivOutStageGainCtrl) {

	HMC833_VcoBiases_t VcoBiases;

	VcoBiases.RF_DIV_RATIO = RfDivRatio;
	VcoBiases.RF_OUT_BUFF_GAIN_CTRL = RfOutBuffGainCtrl;
	VcoBiases.DIV_OUT_STAGE_GAIN_CTRL = DivOutStageGainCtrl;

	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_BIASES, VcoBiases.VcoBiasesReg);
}

/*
 * @brief VCO config register configuration.
 *
 * @param FundDoublerModeSel 	: Fundamental/Doubler mode selection.
 * 									0 - enable the frequency doubler mode of operation,
 * 									1 - enable fundamental mode of operation (default).
 * @param ManRfoMode 			: Manual RFO mode. AutoRFO mode controls output buffers and RF divider enables according to RF divider
 * 								  setting in “VCO_Reg02h Biases”[5:0] ManualRFO mode requires manual enables of individual blocks via
 * 								  VCO_Reg01h.
 * 								  	0 - AutoRFO mode (recommended, default),
 * 								  	1 - ManualRFO mode.
 * @param RfBuffBias 			: RF buffer bias.
 * 									Program to 0 for output frequencies >3000MHz (when VCO_Reg 03h[0]=0),
 * 									Program to 2 for output frequencies <=3000MHz (when VCO_Reg 03h[0]=1) (default).
 */
void HMC833_VcoConfig(uint8_t FundDoublerModeSel, uint8_t ManRfoMode, uint8_t RfBuffBias) {

	HMC833_VcoConfig_t VcoConfig;

	VcoConfig.FUND_DOUBLER_MODE_SEL = FundDoublerModeSel;
	VcoConfig.MANUAL_RFO_MODE = ManRfoMode;
	VcoConfig.RF_BUFF_BIAS = RfBuffBias;
	VcoConfig.RESERVED1 = 0;
	VcoConfig.SPARE = 2;

	HMC833_VcoSubsys(HMC833_VCO_SUBSYS_ID, HMC833_VCO_CONFIG, VcoConfig.VcoConfigReg);
}

/* END OF THE VCO AND PLL SUBSYSTEMS' FUNCTIONS */

/*
 * @brief Reading register from the device.
 *
 * @param RegAddress : Address of register to read.
 */
static uint32_t HMC833_ReadReg(uint8_t RegAddress) {

	static uint32_t RegData;

	hmc833.RxData.W_DONTCARE = 0;
	hmc833.RxData.W_REG_ADDR = RegAddress;
	hmc833.RxData.W_READ_WRITE = HMC833_READ;
	hmc833.RxData.TxBuff[0] = hmc833.RxData.W_InstrData;
	hmc833.RxData.TxBuff[1] = 0;
	hmc833.RxData.TxBuff[2] = 0;
	hmc833.RxData.TxBuff[3] = 0;
	hmc833.rx_fp(&hmc833.RxData.TxBuff[0], &hmc833.RxData.RxBuff[0], 3);

	hmc833.RxData.R_InstrData_H_MSB = hmc833.RxData.RxBuff[0];
	hmc833.RxData.R_InstrData_H_LSB = hmc833.RxData.RxBuff[1];
	hmc833.RxData.R_InstrData_L_MSB = hmc833.RxData.RxBuff[2];
	hmc833.RxData.R_InstrData_L_LSB = hmc833.RxData.RxBuff[3];

	return RegData = hmc833.RxData.R_REG_DATA;
}

/*
 * @brief Writing device' register.
 *
 * @param RegAddress : Address of register to write.
 */
static void HMC833_WriteReg(uint8_t RegAddress, uint32_t Data) {

	hmc833.TxData.W_DONTCARE = 0;
	hmc833.TxData.W_REG_DATA = Data;
	hmc833.TxData.W_REG_ADDRESS = RegAddress;
	hmc833.TxData.W_READ_WRITE = HMC833_WRITE;
	hmc833.TxData.TxBuff[0] = hmc833.TxData.W_InstrData_H_MSB;
	hmc833.TxData.TxBuff[1] = hmc833.TxData.W_InstrData_H_LSB;
	hmc833.TxData.TxBuff[2] = hmc833.TxData.W_InstrData_L_MSB;
	hmc833.TxData.TxBuff[3] = hmc833.TxData.W_InstrData_L_LSB;
	hmc833.tx_fp(&hmc833.TxData.TxBuff[0], 4);
}


/* Hardware dependent functions. */

#ifdef HMC833_SPI_SOFT
/*
 * @brief Receive data from the chip, software version.
 */
static void HMC833_ESpiClk(void) {

	HAL_GPIO_WritePin(HMC833_CLK_PORT, HMC833_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(HMC833_CLK_PORT, HMC833_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Receive data from the chip, software version.
 */
static void HMC833_ESpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size) {

	uint8_t TxByte = 0, PinSate;
	uint16_t RxByte = 0;

	/* Assertion serial enable. */
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
	/* Write instruction byte. */
	TxByte = *pTxData;
	for (uint8_t TxBits = HMC833_W_BIT_NUMBER; TxBits > 0; TxBits--) {
		PinSate = (TxByte & HMC833_BIT_MASK) ? 1 : 0;
		HAL_GPIO_WritePin(HMC833_SDI_PORT, HMC833_SDI_PIN, PinSate);
		HMC833_ESpiClk();
		TxByte <<= 1;
	}
	/* Read the data from the chip. */
	pRxData++;
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t RxBits = HMC833_BIT_NUMBER; RxBits > 0; RxBits--) {
			HAL_GPIO_WritePin(HMC833_CLK_PORT, HMC833_CLK_PIN, GPIO_PIN_SET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			RxByte |= HAL_GPIO_ReadPin(HMC833_SDO_PORT, HMC833_SDO_PIN);
			HAL_GPIO_WritePin(HMC833_CLK_PORT, HMC833_CLK_PIN, GPIO_PIN_RESET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			RxByte <<= 1;
		}
		RxByte >>= 1;
		*pRxData = RxByte;
		RxByte = 0;
		pRxData++;
	}
	/* 32th clock. */
	HMC833_ESpiClk();
	/* De-assertion serial enable. */
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
}

/*
 * @brief Transmit data to the chip, software version.
 */
static void HMC833_ESpiTxData(uint8_t *pData, uint8_t Size) {

	uint8_t TxByte = 0, PinSate;

	/* Assertion serial enable. */
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
	for (uint8_t i = Size; i > 0; i--) {
		TxByte = *pData;
		for (uint8_t TxBits = HMC833_BIT_NUMBER; TxBits > 0; TxBits--) {
			PinSate = (TxByte & HMC833_BIT_MASK) ? 1 : 0;
			HAL_GPIO_WritePin(HMC833_SDI_PORT, HMC833_SDI_PIN, PinSate);
			HMC833_ESpiClk();
			TxByte <<= 1;
		}
		pData++;
	}
	/* De-assertion serial enable. */
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
}
#endif /* HMC833_SPI_SOFT */

#ifdef HMC833_SPI_HARD
/*
 * @brief Receive data from the chip.
 */
static void HMC833_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size) {

	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
	HAL_SPI_TransmitReceive(HMC833Spi, pTxData, pRxData, Size, 25);
	//HAL_SPI_Receive(HMC833Spi, pData, Size, 25);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
}

/*
 * @brief Transmit data to the chip.
 */
static void HMC833_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_SET);
	HAL_SPI_Transmit(HMC833Spi, pData, Size, 25);
	HAL_GPIO_WritePin(HMC833_CS_PORT, HMC833_CS_PIN, GPIO_PIN_RESET);
}
#endif /* HMC833_SPI_HARD */
