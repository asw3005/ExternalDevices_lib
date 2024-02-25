/*
 * xn406.c source file.
 *
 * Created on: Oct 2, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "xn406.h"
#include "math.h"
#include "Common.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;


/* Private function prototypes. */
static uint32_t XN406_ReadReg(uint8_t RegAddress);
static void XN406_WriteReg(uint8_t RegAddress, uint32_t Data);
#ifdef XN406_SPI_HARD
static void XN406_SpiTxData(uint8_t *pData, uint8_t Size);
static void XN406_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
#endif /* XN406_SPI_HARD */
#ifdef XN406_SPI_SOFT
static void XN406_ESpiClk(void);
static void XN406_ESpiTxData(uint8_t *pData, uint8_t Size);
static void XN406_ESpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);
#endif /* XN406_SPI_SOFT */

/* Private variables. */
#ifdef XN406_SPI_HARD
static SPI_HandleTypeDef* XN406Spi = &hspi1;
#endif /* XN406_SPI_HARD */
static XN406_GStr_t xn406 = {
		.delay_fp = HAL_Delay,
		#ifdef XN406_SPI_HARD
		.rx_fp = XN406_SpiRxData,
		.tx_fp = XN406_SpiTxData
		#endif /* XN406_SPI_HARD */
		#ifdef XN406_SPI_SOFT
		.rx_fp = XN406_ESpiRxData,
		.tx_fp = XN406_ESpiTxData
		#endif /* XN406_SPI_SOFT */
};

/*
 * @brief Init device.
 */
void XN406_Init(void) {

	uint32_t ChipID = 0;

	/* Clock select. */
	float fvco, Nint, R, fxtal, fpd, ffrac, fn;
	//uint16_t ExactReg = 0;
	/* Pin init. */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOG_CLK_ENABLE();

	GPIO_InitStruct.Pin = XN406_CE_PIN | XN406_CS_PIN | XN406_CLK_PIN | XN406_SDI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XN406_CE_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = XN406_SDO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XN406_SDO_PORT, &GPIO_InitStruct);

	/* Enable chip. Select HMC mode. */
	HAL_GPIO_WritePin(XN406_CE_PORT, XN406_CE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XN406_CLK_PORT, XN406_CLK_PIN, GPIO_PIN_RESET);

	/* Check is device available? */
	ChipID = XN406_GetChipId();
	if (ChipID != XN406_CHIP_ID) {
		__NOP();
		return;
	}

	/* Calculate the frequency you need. */
	fxtal 	= 60.0f;
	fvco 	= 2800.2f;
	R 		= 1.0f;

	fpd 	= fxtal / R;
	Nint 	= floorf(fvco / fpd);

	fn = Nint * fpd;
	ffrac = ceilf((16777216.0f * (fvco - fn)) / fpd);
	//ExactReg = fpd / (gcd(fvco, fpd));

	/* Chip enable control. */
	XN406_RstRegCfg(1, 1);
	/* Select reference divider. */
	XN406_RefDivCfg(1);
	/* VCO subsystem control.. */
	XN406_VcoSubsys(0, 1, 0x001F);
	XN406_VcoSubsys(0, 2, 0x01C1);
	XN406_VcoSubsys(0, 3, 0x0050);
	XN406_VcoSubsys(0, 4, 0x0141);
	XN406_VcoSubsys(0, 5, 0x00AA);
	XN406_VcoSubsys(0, 6, 0x00FF);
	/* */
	XN406_SdCfg(2, 2, 0, 1, 1);
	XN406_LockDetect(5, 1, 2, 0, 0, 0);
	XN406_AnalogEnable(1, 0 ,0);
	/* 100, 100, 107 Eliminating spurs by offset correction (3thd position). */
	XN406_ChargePump(70, 70, 35, 0, 1, 0);
	/* Dis AFC SPI trigger. */
	//XN406_VcoAutocal(1, 0, 0, 1, 1, 0, 0, 0);
	XN406_VcoAutocal(1, 0, 0, 0, 1, 0, 0, 0);
	XN406_GpoSpiRdiv(1, 0, 0, 0, 0, 1);
	//XN406_OffseEnCtrl(1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 3, 3, 1, 0, 0);

	HAL_Delay(100);
	/* Auto calibration. Set default address for the VCO subsystem. */




	XN406_ExactFreqchCtrl(0, 0);
	/* Set integer part. */
	XN406_SetFreqIntPart(Nint);
	/* En AFC SPI trigger. */
	//XN406_VcoAutocal(1, 0, 0, 0, 1, 0, 0, 0);
	/* Set fractional part. */
	XN406_SetFreqFracPart(ffrac);
}

void XN406_SetFreq(float Frequency) {

	/* Clock select. */
	float Nint, R, fxtal, fpd, ffrac, fn;
	//uint16_t ExactReg = 0;

	/* Calculate the frequency you need. */
	fxtal 	= 60.0f;
	R 		= 1.0f;

	fpd 	= fxtal / R;
	Nint 	= floorf(Frequency / fpd);
	fn = Nint * fpd;
	ffrac = ceilf((16777216.0f * (Frequency - fn)) / fpd);

	//ExactReg = fpd / (gcd(Frequency, fpd));

	HAL_Delay(100);
	/* Auto calibration. Set default address for the VCO subsystem. */

	/* Set integer part. */
	XN406_SetFreqIntPart(Nint);

	XN406_ExactFreqchCtrl(0, 0);
	/* Set fractional part. */
	XN406_SetFreqFracPart(ffrac);
}

/* PLL SUBSYSTEM FUNCTIONS */

/*
 * @brief Reading the chip ID.
 */
uint32_t XN406_GetChipId(void) {

	return XN406_ReadReg(XN406_ID);
}

/*
 * @brief Reset register configuration (default 000002h).
 *
 * @param RstChipenPinsel 	: 0 - take PLL enable via SPI (rst_chipen_from_spi) Reg01[1] (default),
 * 							  1 - take PLL enable via CEN pin, see power down mode description.
 */
void XN406_RstRegCfg(uint8_t RstChipenPinsel, uint8_t RstChipenFromSpi) {

	XN406_RstReg_t Rst = { 0 };

	Rst.RST_CHIPEN_PIN_SEL = RstChipenPinsel;
	Rst.RST_CHIPEN_FROM_SPI = RstChipenFromSpi;
	Rst.RESERVED31_24 = 0;

	XN406_WriteReg(XN406_CS_CTRL, Rst.RstReg);
}

/*
 * @brief Reference divider register configuration (default 000001h).
 *
 * @param RefDiv : reference Divider ’R’ Value, divider use also requires refBufEn Reg08[3] = 1 and divider min 1 max 16383 (default 1).
 */
void XN406_RefDivCfg(uint32_t RefDiv) {

	XN406_WriteReg(XN406_REFDIV, RefDiv);
}

/*
 * @brief Frequency register, integer part configuration (default 0000C8h).
 *
 * @param IntValue : VCO Divider Integer part, used in all modes.
 * 					 Integer mode min 16 max 524,287.
 * 					 Integer mode min 32 max 1,048,574 and the integer frequency dividing ratio can only be an even number.
 * 					 Fractional mode min 20 max 524,283.
 * 					 Fractional mode min 40 max 1048566 for the mode of the prescale divided by 2.
 */
void XN406_SetFreqIntPart(uint32_t IntValue) {

	XN406_WriteReg(XN406_FREQ_INTEGER_PART, IntValue);
}

/*
 * @brief Frequency register, fractional part configuration (default 000000h).
 *
 * @param FracValue : VCO Divider Fractional part (24-bit unsigned).
 * 					  Used in fractional mode only.
 * 					  Nfrac = Reg04h / 2^24, min 0 max 2^24-1.
 */
void XN406_SetFreqFracPart(uint32_t FracValue) {

	XN406_WriteReg(XN406_FREQ_FRACTIONAL_PART, FracValue);
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
 * @param VcoSubsysId 		: Internal VCO Subsystem ID (VCO subsystem ID 0).
 * @param VcoSubsysRegAddr 	: VCO subsystem register address.
 * @param VcoSubsysData 	: VCO subsystem data.
 */
void XN406_VcoSubsys(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint16_t VcoSubsysData) {

	XN406_SubsysVco_t VcoSubsys = { 0 };

	VcoSubsys.VCO_SUBSYS_ID = VcoSubsysId;
	VcoSubsys.VCO_SUBSYS_REG_ADDR = VcoSubsysRegAddr;
	VcoSubsys.VCO_SUBSYS_DATA = VcoSubsysData;

	XN406_WriteReg(XN406_VCO_SPI, VcoSubsys.SubsysVcoDataReg);
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
 * @param VcoSubsysId 		: Internal VCO Subsystem ID (VCO subsystem ID 0).
 * @param VcoSubsysRegAddr 	: VCO subsystem register address.
 * @param LoopCtrl 			: VCO tuning open and closed loop control positions.
 *								0 - closed loop,
 *								1 - open loop.
 * @param VcoCaps 			: VCO caps.
 * 								0 - the lowest,
 * 								...
 * 								31 - the highest.
 * @param VcoSel 			: VCO select.
 * 								3 - maximum frequency of VCO.
 *
 */
void XN406_VcoSpi(uint8_t VcoSubsysId, uint8_t VcoSubsysRegAddr, uint8_t LoopCtrl, uint8_t VcoCaps, uint8_t VcoSel) {

	XN406_VcoSpi_t VcoSubsys = { 0 };

	VcoSubsys.VCO_SUBSYS_ID = VcoSubsysId;
	VcoSubsys.VCO_SUBSYS_REG_ADDR = VcoSubsysRegAddr;
	VcoSubsys.VCO_LOOP_CTRL = LoopCtrl;
	VcoSubsys.VCO_CAPS = VcoCaps;
	VcoSubsys.VCO_SELECT = VcoSel;

	XN406_WriteReg(XN406_VCO_SPI, VcoSubsys.VcoSpiDataReg);
}

/*
 * @brief SD CFG register configuration (default 200B4Ah).
 *
 * @param Seed 				: Selects the seed in fractional mode. Writes to this register are stored in the XN406LP6GE and are only
 * 							  loaded into the modulator when a frequency change is executed and if AutoSeed Reg06h[8] = 1.
 * 								0 - 0 seed,
 * 								1 - 1 seed,
 * 								2 - B29D08h seed (default),
 * 								3 - 50F1CDh seed.
 * @param Order 			: Fractional frequency division algorithm selection.
 * 								0 - 1st order,
 * 								1 - 2nd order,
 * 								2 - mash (default).
 * @param FracBypass 		: Bypass fractional part. In bypass fractional modulator output is ignored, but fractional modulator continues
 * 							  to be clocked if frac_rstb = 1, can be used to test the isolation of the digital fractional modulator from
 * 							  the VCO output in integer mode.
 * 							  	0 - use modulator, required for fractional mode (default),
 * 							  	1 - bypass modulator, required for integer mode.
 * @param AutoSeed 			:  Enables or disables seed loading from the seed field of the register.
 * 								0 - when fractional register write changes frequency, modulator starts with previous contents,
 * 								1 - loads the seed whenever the fractional register is written (default).
 * @param FracEn 			: Selects the modulator clock source, for test only.
 * 								0 - turn off fractional frequency division function,
 * 								1 - turn on fractional frequency division function (recommended for normal operation) (default).
 */
void XN406_SdCfg(uint8_t Seed, uint8_t Order, uint8_t FracBypass, uint8_t AutoSeed, uint8_t FracEn) {

	XN406_SdCfg_t SdCfg = { 0 };

	SdCfg.SEED = Seed;
	SdCfg.ORDER = Order;
	SdCfg.FRAC_BYPASS = FracBypass;
	SdCfg.AUTO_SEED = AutoSeed;
	SdCfg.FRAC_EN = FracEn;
	SdCfg.RESERVED6_4 = 4;
	SdCfg.RESERVED9 = 1;
	SdCfg.RESERVED10 = 0;
	SdCfg.AUTO_CLK_CONFIG = 1;
	SdCfg.RESERVED15_12 = 0;
	SdCfg.RESERVED17_16 = 3;
	SdCfg.RESERVED20_18 = 0;
	SdCfg.RESERVED31_22 = 0;

	XN406_WriteReg(XN406_SD_CFG, SdCfg.SdSfgReg);
}



/*
 * @brief Reset register configuration (default 00014Dh).
 *
 * @param LdWinCnt 			: Lock detect window. Sets the number of consecutive counts of divided VCO that must land inside the Lock
 * 							  Detect Window to declare LOCK (default 5).
 * 							  	0 - 5,   1 - 32,   2 - 96,   3 - 256,
 * 							  	4 - 512, 5 - 2048, 6 - 8192, 7 - 65535 (65520 for counting mode).
 * @param EnIntLd 			: The phase-locked detection unit is enabled.
 * 								 0 - off,
 * 								 1 - operating (default 1).
 * @param LdDigWinDuration 	: Lock detection digital window duration (default 2). Condition: Toffset < Twin < TPFD.
 *								0 - 5.5 ns,    1 - 5.5 ns,  2 - 8.7 ns,   3 - 15.4 ns,
 *								4 - 27.8 ns,   5 - 52.2 ns, 6 - 100.7 ns, 7 - 197 ns.
 * @param LdDigTimFreqCtrl 	: Lock detection digital timer frequency control.
 * 								0 - fastest (default),
 * 								3 - slowest.
 * @param AutoRelockOneTry 	: Attempts to relock if Lock Detect fails for any reason.
 * 								0 - do not restart the AFC after unlocking (default 0),
 * 								1 - trying to re-lock if phase lock fails.
 * @param LdWinType 		: Selection of phase-locked detection mode.
 * 								0 - counting mode (default),
 * 								1 - window mode.
 */
void XN406_LockDetect(uint8_t LdWinCnt, uint8_t EnIntLd, uint8_t LdDigWinDuration, uint8_t LdDigTimFreqCtrl,uint8_t AutoRelockOneTry,
					  uint8_t LdWinType) {

	XN406_LockDetect_t LockDetect = { 0 };

	LockDetect.LKD_WINCNT_MAX = LdWinCnt;
	LockDetect.EN_INT_LOCK_DETECT = EnIntLd;
	LockDetect.LD_DIG_WIN_DUR = LdDigWinDuration;
	LockDetect.LD_DIG_TIM_FREQ_CTRL = LdDigTimFreqCtrl;
	LockDetect.AUTORELOCK_ONE_TRY = AutoRelockOneTry;
	LockDetect.LOCK_DETECT_WIN_TYPE = LdWinType;
	LockDetect.RESERVED5_4 = 0;
	LockDetect.RESERVED6 = 1;

	XN406_WriteReg(XN406_LOCK_DETECT, LockDetect.LockDetectReg);
}

/*
 * @brief Analog EN register configuration (default C1BEFFh).
 *
 * @param GpoPadEn 			: LD_SDO pin control.
 * 								0 - pin LD_SDO disabled,
 * 								1 - and RegFh[LDO_DRV_ALWAYS_ON] = 1 , pin LD_SDO is always on required for use of GPO port,
 * 							    	and RegFh[LDO_DRV_ALWAYS_ON] = 0 SPI LDO_SPI is off if unmatched chip address is seen on the SPI, allowing
 * 							    	a shared SPI with other compatible parts (default).
 * @param HighFreqRef 		: High frequency reference select.
 * 								0 - for ref < 200 MHz (default),
 * 								1 - for ref >= 200 MHz.
 * @param EightGhzDivBy2 	: Prescaler divided by 2 of RF divider is enabled.
 * 								0 - off (default),
 * 								1 - on.
 */
void XN406_AnalogEnable(uint8_t GpoPadEn, uint8_t HighFreqRef, uint8_t EightGhzDivBy2) {

	XN406_AnalogEn_t AnalogEn = { 0 };

	AnalogEn.GPO_PAD_EN = GpoPadEn;
	AnalogEn.HIGH_FREQ_REF = HighFreqRef;
	AnalogEn.EIGHT_GHZ_DIV_BY_2 = EightGhzDivBy2;
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
	AnalogEn.ETO_BIAS_EN = 3;
	AnalogEn.DIV_RESYNC_BIAS_EN = 3;
	AnalogEn.RESERVED18 = 0;
	AnalogEn.REF_OUT_LIMITER = 0;
	AnalogEn.RDIV_EN = 1;
	AnalogEn.RESERVED23 = 0;

	XN406_WriteReg(XN406_ANALOG_EN, AnalogEn.AnalogEnableReg);
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
 * @param OffsetCurrent	: Charge Pump Offset Control 5 μA per step. Affects fractional phase noise and lock detect settings (default 0).
 * 							0 - 0 uA,
 * 							1 - 5 uA,
 * 							2 - 10 uA,
 * 							...
 * 							127 -  635 uA.
 * @param OffsetUpEn 	: Recommended setting 0.
 * @param OffsetDnEn 	: Recommended setting 1 in fractional mode, otherwise 0.
 * @param HiKcp 		: HiKcp high current charge pump.
 */
void XN406_ChargePump(uint8_t CpDnGain, uint8_t CpUpGain, uint8_t OffsetCurrent, uint8_t OffsetUpEn, uint8_t OffsetDnEn, uint8_t HiKcp) {

	XN406_ChargePump_t ChargePump = { 0 };

	ChargePump.CP_DN_GAIN = CpDnGain;
	ChargePump.CP_UP_GAIN = CpUpGain;
	ChargePump.OFFSET_CURRENT = OffsetCurrent;
	ChargePump.OFFSET_UP_EN = OffsetUpEn;
	ChargePump.OFFSET_DN_EN = OffsetDnEn;
	ChargePump.HI_KCP = HiKcp;

	XN406_WriteReg(XN406_CHARGE_PUMP, ChargePump.ChargePumpReg);
}

/*
 * @brief VCO autocal register configuration (default 002205h).
 *
 * @param VtuneRes 			: AFC counting cycle (default 5).
 * 								0 - 512,  1 - 1000,  2 - 8,   3 - 16,
 * 								4 - 32,   5 - 64,    6 - 128, 7 - 256.
 * @param WaitStateSetUp 	: Waiting for 100 VSPI clock cycles after switching the VCO and its child segment (this clock is determined by Reg0A<14:13>).
 * 								0 - waiting after the first switch (default 0),
 * 								1 - waiting after the first two switches,
 * 								2 - waiting after the first three switches,
 * 								3 - waiting after the first four switches.
 * @param BypassVco 		: Select the VCO and its child segment sources.
 * 								0 - configuration through SPI (Reg05) and AFC (default 0),
 * 								1 - configuration of VCO only through SPI (Reg05).
 * @param NoVspiTrg 		: VSPI sending is not triggered after writing the 05 register (VCO chip is configured).
 * 								0 - trigger sending (default),
 * 								1 - do not trigger sending.
 * @param FsmVspiClkSel 	: Set the AutoCal FSM and VSPI clock (50 MHz maximum).
 * 								0 - input reference,
 * 								1 - input reference/4 (default),
 * 								2 - input reference/16,
 * 								3 - input reference/32.
 * @param XtalFallEdgeFsm 	: The falling edge of the clock is used to drive the AFC. (default 0).
 * @param ForceRdivBypass 	: Forced bypass reference divider.
 * 								0 - no bypass (default0,
 * 								1 - bypass.
 * @param CntDelay 			:
 * 								0 - waiting for 8 reference cycles after switching VCO,
 * 								1 -	no waiting after switching VCO.
 */
void XN406_VcoAutocal(uint8_t VtuneRes, uint8_t WaitStateSetUp, uint8_t BypassVco, uint8_t NoVspiTrg, uint8_t FsmVspiClkSel,
					  uint8_t XtalFallEdgeFsm, uint8_t ForceRdivBypass, uint8_t CntDelay) {

	XN406_VcoAutocal_t VcoAutocal = { 0 };

	VcoAutocal.VTUNE_RESOLUTION = VtuneRes;
	VcoAutocal.WAIT_STATE_SET_UP = WaitStateSetUp;
	VcoAutocal.BYPASS_VCO_TUNING = BypassVco;
	VcoAutocal.NO_VSPI_TRG = NoVspiTrg;
	VcoAutocal.FSM_VSPI_CLK_SEL = FsmVspiClkSel;
	VcoAutocal.XTAL_FALLING_EDGE_FOR_FSM = XtalFallEdgeFsm;
	VcoAutocal.FORCE_RDIV_BYPASS = ForceRdivBypass;
	VcoAutocal.CNT_DELAY = CntDelay;
	VcoAutocal.VCO_CURVE_ADJ = 0;
	VcoAutocal.RESERVED9_8 = 2;
	VcoAutocal.RESERVED10 = 0;

	XN406_WriteReg(XN406_AFC_CTRL, VcoAutocal.VcoAutocalReg);
}

/*
 * @brief Offset and enable control register configuration (default F8061h).
 *
 * @param PdDelSel 			: Delayed control bit of PFD deadband. (recommended setting 1, default).
 * @param ShortPdInputs 	: Effective control bit of PFD input signal. (default 0).
 * @param PdPhaseSel 		: PFD polarity selection.
 * 								0 - VCO and loop filter with positive polarity (default 0),
 * 								1 - VCO and loop filter with negative polarity.
 * @param PdUpEn 			: Enables the PD UP output (default 1).
 * @param PdDnEn 			: Enables the PD DN output (default 1).
 * @param ForceCpUp 		: The charge pump is forced to set in a charging mode. (default 0).
 * @param ForceCpDown 		: The charge pump is forced to set in a discharging mode. (default 0).
 * @param ForceCpMidRail 	: The charge pump is forced to output to the intermediate voltage. (default 0).
 * @param PsBias 			: Current control of RF divider (default 0).
 * @param MCntClkGating 	: Program 11. MCounter clock gating.
 * 								0 - MCounter Off,
 * 								1 - N < 128,
 * 								2 - N < 1023,
 * 								3 - all clocks on (recommended setting 11, default).
 * @param PulseWidth 		: Output pulse width control of the RF divider (default 0).
 * @param ResetDelay 		: Delayed reset of the RF divider (default 0).
 */
void XN406_OffseEnCtrl(uint8_t PdDelSel, uint8_t ShortPdInputs, uint8_t PdPhaseSel, uint8_t PdUpEn, uint8_t PdDnEn, uint8_t ForceCpUp,
		uint8_t ForceCpDown, uint8_t ForceCpMidRail, uint8_t PsBias, uint8_t MCntClkGating, uint8_t PulseWidth, uint8_t ResetDelay) {

	XN406_PhaseDetector_t PhaseDetect;

	PhaseDetect.PD_DEL_SEL = PdDelSel;
	PhaseDetect.SHORT_PD_IN = ShortPdInputs;
	PhaseDetect.PD_PHASE_SEL = PdPhaseSel;
	PhaseDetect.PD_UP_EN = PdUpEn;
	PhaseDetect.PD_DN_EN = PdDnEn;
	PhaseDetect.FORCE_CP_UP = ForceCpUp;
	PhaseDetect.FORCE_CP_DN = ForceCpDown;
	PhaseDetect.FORCE_CP_MID_RAIL = ForceCpMidRail;
	PhaseDetect.PS_BIAS = PsBias;

	PhaseDetect.MCNT_CLK_GATING = MCntClkGating;
	PhaseDetect.CP_INT_OAMP_BIAS = 3;
	PhaseDetect.RESERVED2 = 0;
	PhaseDetect.RESERVED19 = 1;
	PhaseDetect.RESERVED31_23 = 0;

	XN406_WriteReg(XN406_OFFSET_EN, PhaseDetect.PhaseDetectorReg);
}

/*
 * @brief Exact frequency mode register configuration (default 000000h).
 *
 * @param Denominator 	: Accurate frequency division mode, denominator.
 * @param Numerator 	: Accurate frequency division mode, numerator.
 */
void XN406_ExactFreqchCtrl(uint16_t Denominator, uint16_t Numerator) {

	XN406_ExactFreqCtrl_t ExactFreqCtrlReg;

	ExactFreqCtrlReg.EXACT_MODE = Denominator;
	ExactFreqCtrlReg.EXACT_IN = Numerator;

	XN406_WriteReg(XN406_EXACT_FREQ_MODE, ExactFreqCtrlReg.ExactFreqCtrlReg);
}

/*
 * @brief GPO SPI RDIV register configuration (default 000001h).
 *
 * @param GpoSel 			: Signal selected here is output to SDO pin when enabled.
 * 								0 - data from Reg0F[5],
 *								1 - lock detect output (default),
 *								3 - lock detect window output,
 *								4 - ring osc frequency output under window mode of phase-locked detection,
 *								8 - reference buffer output,
 *								9 - reference frequency division output,
 *								10 - RF divider output,
 *								11 - clock of sigma-delta modulator,
 *								13 - aux SPI clock,
 *								14 - aux SPI enable,
 *								15 - aux SPI data out,
 *								16 - PD DN,
 *								17 - PD UP,
 *								24 - reset signal of RF divider,
 *								2, 7, 12, 25 - 0,
 *								5, 6, 18-23, 26-31 - NC.
 * @param GpoTestData 		: 1 - GPO test data (default 0).
 * @param PreventAutomuxSdo : Prevent automux SDO pin.
 * 								0 - automuxes between SDO and GPO data (default),
 * 								1 - outputs GPO data only.
 * @param LdoDrvAlwaysOn 	: LDO driver always on.
 * 								0 - LD_SDO pin driver only on during SPI read cycle (default),
 * 								1 - LD_SDO pin driver always on.
 * @param SdRandomEn 		:
 * 								0 - random number closed (default),
 * 								1 - random number opened.
 * @param DbuffEn 			: Dual buffer enable bit of frequency dividing value is highly effective.
 * 								0 - The integer frequency dividing value is directly updated to the divider after writing Reg03,
 * 								1 - In integer mode (Reg06<7> =1), the integer frequency dividing value is updated to the divider
 * 								 	immediately after writing Reg03 to make the integer frequency division effective directly; in
 * 								 	fractional mode (Reg06<7> =0), the integer frequency dividing value and the fractional frequency
 * 								 	dividing value are updated to the divider only after writing Reg04 (fractional frequency dividing
 * 								 	value).
 */
void XN406_GpoSpiRdiv(uint8_t GpoSel, uint8_t GpoTestData, uint8_t PreventAutomuxSdo, uint8_t LdoDrvAlwaysOn, uint8_t SdRandomEn,
					  uint8_t DbuffEn) {

	XN406_GpoSpiRdiv_t GpoSpiRdiv;

	GpoSpiRdiv.GPO_SELECT = GpoSel;
	GpoSpiRdiv.GPO_TEST_DATA = GpoTestData;
	GpoSpiRdiv.PREVENT_AUTOMUX_SDO = PreventAutomuxSdo;
	GpoSpiRdiv.LDO_DRV_ALWAYS_ON = LdoDrvAlwaysOn;
	GpoSpiRdiv.SD_RANDOM_EN = SdRandomEn;
	GpoSpiRdiv.DBUFF_EN = DbuffEn;
	GpoSpiRdiv.RESERVED10_8 = 0;
	GpoSpiRdiv.RESERVED14_12 = 0;

	XN406_WriteReg(XN406_GPO_SPI_RDIV, GpoSpiRdiv.GpoSpiRdivReg);
}

/*
 * @brief VCO tune register (default 000020h).
 *
 * @return VCO_SWITCH_SETTING 	: VCO segment selection.
 * @return AUTOCAL_BUSY 		: Automatic segment selection in progress.
 */
XN406_VcoTune_t* XN406_GetVcoTune(void) {

	static XN406_VcoTune_t VcoTune;

	VcoTune.VcoTuneReg = XN406_ReadReg(XN406_VCO_TUNE);
	return &VcoTune;
}

/*
 * @brief Lock detect register.
 *
 * @return LOCK_DETECT : Lock detection.
 */
XN406_Locked_t* XN406_GetLocked(void) {

	static XN406_Locked_t Locked;

	Locked.LockedReg = XN406_ReadReg(XN406_LOCKED);
	return &Locked;
}

/* VCO SUBSYSTEM FUNCTIONS */

/*
 * @brief VCO tuning register configuration (default 0020h).
 *
 * @param TuneSel 	: VCO tune voltage selection.
 * 						0 - tune voltage is selected as the output of the loop filter, close loop (default),
 * 						1 - tune voltage is selected as the internal positive temperature voltage for VCO tuning segment selection, open loop.
 * @param Cs 		: Selection of VCO segment code (default 16).
 * 						0 - highest frequency segment,
 * 						1 - lowest frequency segment.
 * @param VcoSel 	: VCO selection.
 * 						0 - VCO with the highest frequency (default),
 * 						1 - VCO with the lowest frequency.
 */
void XN406_VcoTuning(uint8_t TuneSel, uint8_t Cs, uint8_t VcoSel) {

	XN406_VcoTuning_t VcoTuning;

	VcoTuning.TUNE_SEL = TuneSel;
	VcoTuning.CS = Cs;
	VcoTuning.VCO_SEL = VcoSel;
	VcoTuning.SPARE = 0;

	XN406_VcoSubsys(XN406_VCO_SUBSYS_ID, XN406_VCO_TUNING, VcoTuning.VcoTuningReg);
}

/*
 * @brief VCO enables  register configuration.
 *
 * @param EnVcoSub 	: Vco_sub is enabled (default 1).
 * @param EnPllBuff : Pll_buf is enabled (default 1).
 * @param EnRfBuff 	: Rf_buf is enabled (default 1).
 * @param EnVcoBuff : Vco_buf is enabled (default 1).
 * @param EnDivn 	: Div is enabled (default 1).
 */
void XN406_VcoEnables(uint8_t EnVcoSub, uint8_t EnPllBuff, uint8_t EnRfBuff, uint8_t EnVcoBuff, uint8_t EnDivn) {

	XN406_VcoEnables_t VcoEnables;

	VcoEnables.EN_VCO_SUBSYS = EnVcoSub;
	VcoEnables.EN_PLL_BUFF_EN = EnPllBuff;
	VcoEnables.EN_RF_BUFF_EN = EnRfBuff;
	VcoEnables.EN_VCO_BUFF = EnVcoBuff;
	VcoEnables.EN_DIV_EN = EnDivn;
	VcoEnables.SPARE = 0;

	XN406_VcoSubsys(XN406_VCO_SUBSYS_ID, XN406_VCO_ENABLES, VcoEnables.VcoEnablesReg);
}

/*
 * @brief  VCO biases register configuration.
 *
 * @param RfDivRatio 			: Frequency dividing ratio control of divider (default 1).
 * 									0    - mute,
 *									1 	 - Fo (default),
 *									2, 3 - Fo/2,
 *									4, 5 - Fo/4,
 *									6, 7 - Fo/6,
 *									...
 *									60, 61 - Fo/60,
 *									62, 63 - Fo/62,
 * @param RfOutBuffGainCtrl 	: RF output buffer gain control.
 * 									0 - Min Gain -9 dB,
 * 									1 - Gain -6 dB,
 * 									2 - Gain -3 dB,
 * 									3 - Max Gain  (default).
 * @param DivOutStageGainCtrl 	: Divider output gain control.
 * 								  Used to flatten the output power level across frequency
 * 								  For divide-by 1 or divide-by 2 it is recommended to set this bit to 1. 0 will reduce output power and
 * 								  degrade noise floor performance. For divide-by 4 or higher, it is recommended to set this bit to 0 to
 * 								  maintain flat output power across divider settings. Setting this bit to 1, with divide-by 4 or higher
 * 								  provides higher output power compared to the divide by 1 or 2 case.
 * 									0 - max gain -3 dB,
 * 									1 - max gain.
 */
void XN406_VcoBiases(uint8_t RfDivRatio, uint8_t RfOutBuffGainCtrl, uint8_t DivOutStageGainCtrl) {

	XN406_VcoBiases_t VcoBiases;

	VcoBiases.RF_DIV_RATIO = RfDivRatio;
	VcoBiases.RF_BUFF_GAIN_CTRL = RfOutBuffGainCtrl;
	VcoBiases.DIV_OUT_GAIN_CTRL = DivOutStageGainCtrl;

	XN406_VcoSubsys(XN406_VCO_SUBSYS_ID, XN406_VCO_GAIN_DIV, VcoBiases.VcoBiasesReg);
}

/*
 * @brief VCO config register configuration.
 *
 * @param RfOutMode 	: Fundamental/Doubler mode selection.
 * 							0 - enable fundamental mode of operation,
 * 							1 - reserved (default).
 * @param PdRfBuffCore 	: Rf_buf_core operating control.
 * 							0 - operating (default),
 * 							1 - off.
 * @param ManualRfoMode : Output control mode.
 * 							0 - automatic control (default),
 * 							1 - manual control.
 * @param RfBuffBias 	: Rf buffer bias current control (default 2).
 * @param CalVolSlope 	: VCO segment voltage slope control (default 2).
 */
void XN406_VcoConfig(uint8_t RfOutMode, uint8_t PdRfBuffCore, uint8_t ManualRfoMode, uint8_t RfBuffBias, uint8_t CalVolSlope) {

	XN406_VcoConfig_t VcoConfig;

	VcoConfig.RF_OUT_MODE = RfOutMode;
	VcoConfig.PD_RF_BUFF_CORE = PdRfBuffCore;
	VcoConfig.MANUAL_RFO_MODE = ManualRfoMode;
	VcoConfig.RF_BUFF_BIAS = RfBuffBias;
	VcoConfig.CAL_VOL_SLOPE = CalVolSlope;
	VcoConfig.SPARE = 0;

	XN406_VcoSubsys(XN406_VCO_SUBSYS_ID, XN406_VCO_CONFIG, VcoConfig.VcoConfigReg);
}

/* END OF THE VCO AND PLL SUBSYSTEMS' FUNCTIONS */

/*
 * @brief Reading register from the device.
 *
 * @param RegAddress : Address of register to read.
 */
static uint32_t XN406_ReadReg(uint8_t RegAddress) {

	static uint32_t RegData;

	xn406.RxData.W_DONTCARE = 0;
	xn406.RxData.W_REG_ADDR = RegAddress;
	xn406.RxData.W_READ_WRITE = XN406_READ;
	xn406.RxData.TxBuff[0] = xn406.RxData.W_InstrData;
	xn406.RxData.TxBuff[1] = 0;
	xn406.RxData.TxBuff[2] = 0;
	xn406.RxData.TxBuff[3] = 0;
	xn406.rx_fp(&xn406.RxData.TxBuff[0], &xn406.RxData.RxBuff[0], 3);

	xn406.RxData.R_InstrData_H_MSB = xn406.RxData.RxBuff[0];
	xn406.RxData.R_InstrData_H_LSB = xn406.RxData.RxBuff[1];
	xn406.RxData.R_InstrData_L_MSB = xn406.RxData.RxBuff[2];
	xn406.RxData.R_InstrData_L_LSB = xn406.RxData.RxBuff[3];

	return RegData = xn406.RxData.R_REG_DATA;
}

/*
 * @brief Writing device' register.
 *
 * @param RegAddress : Address of register to write.
 */
static void XN406_WriteReg(uint8_t RegAddress, uint32_t Data) {

	xn406.TxData.W_DONTCARE = 0;
	xn406.TxData.W_REG_DATA = Data;
	xn406.TxData.W_REG_ADDRESS = RegAddress;
	xn406.TxData.W_READ_WRITE = XN406_WRITE;
	xn406.TxData.TxBuff[0] = xn406.TxData.W_InstrData_H_MSB;
	xn406.TxData.TxBuff[1] = xn406.TxData.W_InstrData_H_LSB;
	xn406.TxData.TxBuff[2] = xn406.TxData.W_InstrData_L_MSB;
	xn406.TxData.TxBuff[3] = xn406.TxData.W_InstrData_L_LSB;
	xn406.tx_fp(&xn406.TxData.TxBuff[0], 4);
}



/* Hardware dependent functions. */

#ifdef XN406_SPI_SOFT
/*
 * @brief Receive data from the chip, software version.
 */
static void XN406_ESpiClk(void) {

	HAL_GPIO_WritePin(XN406_CLK_PORT, XN406_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(XN406_CLK_PORT, XN406_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Receive data from the chip, software version.
 */
static void XN406_ESpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size) {

	uint8_t TxByte = 0, PinSate;
	uint16_t RxByte = 0;

	/* Assertion serial enable. */
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_SET);
	/* Write instruction byte. */
	TxByte = *pTxData;
	for (uint8_t TxBits = XN406_W_BIT_NUMBER; TxBits > 0; TxBits--) {
		PinSate = (TxByte & XN406_BIT_MASK) ? 1 : 0;
		HAL_GPIO_WritePin(XN406_SDI_PORT, XN406_SDI_PIN, PinSate);
		XN406_ESpiClk();
		TxByte <<= 1;
	}
	/* Read the data from the chip. */
	pRxData++;
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t RxBits = XN406_BIT_NUMBER; RxBits > 0; RxBits--) {
			HAL_GPIO_WritePin(XN406_CLK_PORT, XN406_CLK_PIN, GPIO_PIN_SET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			RxByte |= HAL_GPIO_ReadPin(XN406_SDO_PORT, XN406_SDO_PIN);
			HAL_GPIO_WritePin(XN406_CLK_PORT, XN406_CLK_PIN, GPIO_PIN_RESET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
			RxByte <<= 1;
		}
		RxByte >>= 1;
		*pRxData = RxByte;
		RxByte = 0;
		pRxData++;
	}
	/* 32th clock. */
	XN406_ESpiClk();
	/* De-assertion serial enable. */
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_RESET);
}

/*
 * @brief Transmit data to the chip, software version.
 */
static void XN406_ESpiTxData(uint8_t *pData, uint8_t Size) {

	uint8_t TxByte = 0, PinSate;

	/* Assertion serial enable. */
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_SET);
	for (uint8_t i = Size; i > 0; i--) {
		TxByte = *pData;
		for (uint8_t TxBits = XN406_BIT_NUMBER; TxBits > 0; TxBits--) {
			PinSate = (TxByte & XN406_BIT_MASK) ? 1 : 0;
			HAL_GPIO_WritePin(XN406_SDI_PORT, XN406_SDI_PIN, PinSate);
			XN406_ESpiClk();
			TxByte <<= 1;
		}
		pData++;
	}
	/* De-assertion serial enable. */
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_RESET);
}
#endif /* XN406_SPI_SOFT */

#ifdef XN406_SPI_HARD
/*
 * @brief Receive data from the chip.
 */
static void XN406_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size) {

	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_SET);
	HAL_SPI_TransmitReceive(XN406Spi, pTxData, pRxData, Size, 25);
	//HAL_SPI_Receive(XN406Spi, pData, Size, 25);
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_RESET);
}

/*
 * @brief Transmit data to the chip.
 */
static void XN406_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_SET);
	HAL_SPI_Transmit(XN406Spi, pData, Size, 25);
	HAL_GPIO_WritePin(XN406_CS_PORT, XN406_CS_PIN, GPIO_PIN_RESET);
}
#endif /* XN406_SPI_HARD */


