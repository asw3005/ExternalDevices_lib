/*
 *  @brief ad9959.c source file.
 *
 * Created on: Aug 21, 2023
 * Author: asw3005
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "ad9959.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;

/* Private variables. */
static SPI_HandleTypeDef* AD9959Spi = &hspi1;

/* Private function prototypes. */
//static uint8_t AD9959_ReadByte(uint8_t Address);
//static void AD9959_WriteByte(uint8_t Address, uint32_t Value, uint8_t Size);
static void AD9959_ClockData(void);
static void AD9959_ESpiRxData(uint8_t* pData, uint8_t Size);
static void AD9959_ESpiTxData(uint8_t* pData, uint8_t Size);
static void AD9959_SpiTxData(uint8_t *pData, uint8_t Size);
static void AD9959_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size);

/* Init general struct. */
static AD9959_GStr_t ad9959 = {
		.delay_fp = HAL_Delay,
		.spi_rx_fp = AD9959_ESpiRxData,
		.spi_tx_fp = AD9959_ESpiTxData
};

/*
 * @brief AD9959 DDS registers initialization.
 */
void AD9959_Init(void) {

	//uint8_t Temp[10];

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

//	/* Init pins. */
//	GPIO_InitStruct.Pin = AD9959_NSS_PIN | AD9959_RST_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_SET);

//	/* Reset after power up . */
//	HAL_GPIO_WritePin(AD9959_RST_PORT, AD9959_RST_PIN, GPIO_PIN_SET);
//	HAL_Delay(500);
//	HAL_GPIO_WritePin(AD9959_RST_PORT, AD9959_RST_PIN, GPIO_PIN_RESET);
//	HAL_Delay(100);

	/* Enable the channels. */
	AD9959_ChSelectReg(0, 0, 0x0F);
	/* Initial configuration. */
	AD9959_FunctReg1(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
	/* Set amplitude. */
	AD9959_SetFrequency(81250000);
	AD9959_SetAmplitude(800, 0, 0, 1, 0, 0);

	/* Enable the channels. */
//	AD9959_ChSelectReg(0, 0, 0x0F);
//	AD9959_SetPhase(0);


//	Temp[0] = AD9959_ReadReg(AD9959_CH_SEL, 1)->RegData[0];
//	Temp[1] = AD9959_ReadReg(AD9959_CH_FUNCTION, 3)->RegData[1];
//	Temp[2] = AD9959_ReadReg(AD9959_FUNCTION1, 3)->RegData[0];
//	Temp[3] = AD9959_ReadReg(AD9959_CH_AMPLITUDE_CTRL, 3)->RegData[2];


}

/*
 * @brief Frequency setting.
 *
 * @param Frequency : Frequency that you desire.
 * @return : FreqTuneWord.
 */
uint32_t AD9959_SetFrequency(float Frequency) {

	uint32_t FreqTuneWord = (uint32_t)((4294967296.0f * Frequency) / AD9959_SYSTEM_CLOCK_RATE);

	AD9959_FreqTuneWord0(FreqTuneWord);
	return FreqTuneWord;
}

/*
 * @brief Phase setting.
 *
 * @param Phase : Phase that you desire, from 0 to 360 degree.
 * @return : ChPhaseOffsetWord.
 */
uint16_t AD9959_SetPhase(float Phase) {

	uint16_t ChPhaseOffsetWord = (uint16_t)((16384.0f * Phase) / 360.0f);

	AD9959_ChPhaseOffsetWord0(ChPhaseOffsetWord);
	return ChPhaseOffsetWord;
}

/*
 * @brief Amplitude control register.
 *
 * @param Amplitude 			: Amplitude in millivolts from 0 to 800mV.
 * @param LoadArrAtIoUpdate 	: Load ARR at I/O_UPDATE.
 * 									0 - the amplitude ramp rate timer is loaded only upon timeout (timer = 1) and is not loaded due to an I/O_UPDATE input
 * 										signal (default),
 * 									1 - the amplitude ramp rate timer is loaded upon timeout (timer = 1) or at the time of an I/O_UPDATE input signal.
 * @param RumpUpDownEn 			: Ramp-up/ramp-down enable. This bit is valid only when ACR[AMPL_MULTIPLIER_EN] is active high.
 * 									0 - when ACR[AMPL_MULTIPLIER_EN] is active, Logic 0 on ACR[RAMP_UP_DOWN_EN] enables the manual RU/RD operation (default),
 * 									1 - if ACR[AMPL_MULTIPLIER_EN] is active, a Logic 1 on ACR[RAMP_UP_DOWN_EN] enables the auto RU/RD operation.
 * @param AmplMultiplierEn 		: Amplitude multiplier enable.
 * 									0 - amplitude multiplier is disabled. The clocks to this scaling function (auto RU/RD) are stopped for power saving, and
 * 										the data from the DDS core is routed around the multipliers (default),
 * 									1 - amplitude multiplier is enabled.
 * @param IncDecStepSize 		: Amplitude increment/decrement step size.
 * 									0 - step size is 1,
 * 									1 - step size is 2,
 * 									2 - step size is 4,
 * 									3 - step size is 8.
 * @param AmplRumpRate 			: Amplitude ramp rate value.
 */
void AD9959_SetAmplitude(float Amplitude, uint8_t LoadArrAtIoUpdate, uint8_t RampUpDownEn, uint8_t AmplMultiplierEn, uint8_t IncDecStepSize, uint8_t AmplRampRate) {

	uint16_t AmplitudeScaleFactor = (uint16_t)((Amplitude * AD9959_VOLTAGE_RATIO));
	AD9959_AmplCtrl_t AmplitudeCtrl;

	if(AmplitudeScaleFactor > AD9959_BIT_DEPTH) { AmplitudeScaleFactor = AD9959_BIT_DEPTH; }

	AmplitudeCtrl.AMPL_SCALE_FACTOR_LOW = AmplitudeScaleFactor;
	AmplitudeCtrl.AMPL_SCALE_FACTOR_HIGH = AmplitudeScaleFactor >> 8;
	AmplitudeCtrl.LOAD_ARR_AT_IO_UPDATE = LoadArrAtIoUpdate;
	AmplitudeCtrl.RAMP_UP_DOWN_EN = RampUpDownEn;
	AmplitudeCtrl.AMPL_MULTIPLIER_EN = AmplMultiplierEn;
	AmplitudeCtrl.INC_DEC_STEP_SIZE = IncDecStepSize;
	AmplitudeCtrl.AMPL_RAMP_RATE = AmplRampRate;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_CH_AMPLITUDE_CTRL;
	ad9959.RxTxData.Data[0] = AmplitudeCtrl.AmplCtrl_HIGH_LSB;
	ad9959.RxTxData.Data[1] = AmplitudeCtrl.AmplCtrl_LOW_MSB;
	ad9959.RxTxData.Data[2] = AmplitudeCtrl.AmplCtrl_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 4);
	ad9959.delay_fp(1);
}

/*
 * @brief Linear sweep ramp rate.
 *
 * @param RisingSweepRampRate 	: Linear rising sweep ramp rate.
 * @param FallingSweepRampRate 	: Linear falling sweep ramp rate.
 */
void AD9959_SetSweepRampRate(uint8_t RisingSweepRampRate, uint8_t FallingSweepRampRate) {

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_LINEAR_SWEEP_RAMP_RATE;
	ad9959.RxTxData.Data[0] = FallingSweepRampRate;
	ad9959.RxTxData.Data[1] = RisingSweepRampRate;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 3);
	ad9959.delay_fp(1);
}

/*
 * @brief Rising delta word.
 *
 * @param RisingDeltaWord : 32-bit rising delta-tuning word.
 */
void AD9959_SetRisingDeltaWord(uint32_t RisingDeltaWord) {

	AD9959_RisingDeltaWord_t RisingDeltaWordReg;

	RisingDeltaWordReg.RisingDeltaWord_32 = RisingDeltaWord;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_LSR_RISING_DELTA_WORD;
	ad9959.RxTxData.Data[0] = RisingDeltaWordReg.RisingDeltaWord_HIGH_MSB;
	ad9959.RxTxData.Data[1] = RisingDeltaWordReg.RisingDeltaWord_HIGH_LSB;
	ad9959.RxTxData.Data[2] = RisingDeltaWordReg.RisingDeltaWord_LOW_MSB;
	ad9959.RxTxData.Data[3] = RisingDeltaWordReg.RisingDeltaWord_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 5);
	ad9959.delay_fp(1);
}

/*
 * @brief Falling delta word.
 *
 * @param FallingDeltaWord : 32-bit falling delta-tuning word.
 */
void AD9959_SetFallingDeltaWord(uint32_t FallingDeltaWord) {

	AD9959_FallingDeltaWord_t FallingDeltaWordReg;

	FallingDeltaWordReg.FallingDeltaWord_32 = FallingDeltaWord;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_LSR_FALLING_DELTA_WORD;
	ad9959.RxTxData.Data[0] = FallingDeltaWordReg.FallingDeltaWord_HIGH_MSB;
	ad9959.RxTxData.Data[1] = FallingDeltaWordReg.FallingDeltaWord_HIGH_LSB;
	ad9959.RxTxData.Data[2] = FallingDeltaWordReg.FallingDeltaWord_LOW_MSB;
	ad9959.RxTxData.Data[3] = FallingDeltaWordReg.FallingDeltaWord_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 5);
	ad9959.delay_fp(1);
}

/*
 * @brief Channel word X.
 *
 * @param NumberOfChWord 	: Number of word to be set. It should be from AD9959_CH_WORD1 to AD9959_CH_WORD15 (see the AD9959_REG_MAPS_t).
 * @param FreqPhaseAmplSel 	: 0 - frequency tuning word selected,
 * 							  1 - phase tuning word selected,
 * 							  2 - amplitude tuning word selected.
 * @param FreqPhaseAmplWord : 32-bit frequency or 14-bit phase or 10-bit amplitude word.
 */
void AD9959_SetChannelWord(AD9959_REG_MAPS_t NumberOfChWord, uint8_t FreqPhaseAmplSel, uint32_t FreqPhaseAmplWord) {

	AD9959_Profile_t ProfileData;

	if (NumberOfChWord < AD9959_CH_WORD1 || NumberOfChWord > AD9959_CH_WORD15) { return; }
	switch(FreqPhaseAmplWord) {
		case 0:
			ProfileData.FREQ_TUNE_WORD = FreqPhaseAmplWord;
			break;
		case 1:
			ProfileData.PHASE_TUNE_WORD = FreqPhaseAmplWord;
			break;
		case 2:
			ProfileData.AMPL_TUNE_WORD = FreqPhaseAmplWord;
			break;
		default:
			return;
		//break;
	}

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = NumberOfChWord;
	ad9959.RxTxData.Data[0] = ProfileData.ChannelWord_HIGH_MSB;
	ad9959.RxTxData.Data[1] = ProfileData.ChannelWord_HIGH_LSB;
	ad9959.RxTxData.Data[2] = ProfileData.ChannelWord_LOW_MSB;
	ad9959.RxTxData.Data[3] = ProfileData.ChannelWord_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 5);
	ad9959.delay_fp(1);
}

/*
 * @brief Channel select register.
 *
 * @param LsbFirst 		: See the Serial I/O Modes of Operation section for more details.
 * 							0 - the serial interface accepts serial data in MSB first format (default),
 * 							1 - the serial interface accepts serial data in LSB first format.
 * @param SerialIoMode 	: Serial IO mode select.
 * 							0 - single-bit serial (2-wire mode)(default),
 * 							1 - single-bit serial (3-wire mode),
 * 							2 - 2-bit serial mode,
 * 							3 - 4-bit serial mode.
 * @param ChannelEn 	: Number of active channels. Bits are active immediately after being written. They do not require an I/O update to take effect.
 * 						  There are four sets of channel registers and profile (channel word) registers, one per channel. This is not shown in the channel
 * 						  register map or the profile register map. The addresses of all channel registers and profile registers are the same for each channel.
 * 						  Therefore, the channel enable bits distinguish the channel registers and profile registers values of each channel. For example,
 * 						  1001 = only Channel 3 and Channel 0 receive commands from the channel registers and profile registers.
 */
void AD9959_ChSelectReg(uint8_t LsbFirst, uint8_t SerialIoMode, uint8_t ChannelEn) {

	AD9959_ChSel_t ChSelReg;

	ChSelReg.LSB_FIRST = LsbFirst;
	ChSelReg.SERIAL_IO_MODE = SerialIoMode;
	ChSelReg.CH_ENABLE = ChannelEn;
	ChSelReg.MUST_BE_ZERO = 0;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_CH_SEL;
	ad9959.RxTxData.Data[0] = ChSelReg.ChSelReg;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 2);
	ad9959.delay_fp(1);
}

/*
 * @brief Function register 1.
 *
 * @param ManSoftSync 		: Manual software sync.
 * 								0 - the manual software synchronization feature of multiple devices is inactive (default),
 * 								1 - the manual software synchronization feature of multiple devices is active.
 * @param ManHardSync 		: Manual hardware sync.
 * 								0 - the manual hardware synchronization feature of multiple devices is inactive (default),
 * 								1 - the manual hardware synchronization feature of multiple devices is active.
 * @param DacRefPwrDown 	: DAC reference power-down.
 *  							0 - DAC reference is enabled (default),
 * 								1 - DAC reference is powered down.
 * @param SyncClkDis 		: SYNC_CLK disable.
 * 								0 - the SYNC_CLK pin is active (default),
 * 								1 - the SYNC_CLK pin assumes a static Logic 0 state (disabled). In this state, the pin drive
 * 									logic is shut down. However, the synchronization circuitry remains active internally to maintain
 * 									normal device operation.
 * @param ExtPwrDownMode 	: External power-down mode.
 * 								0 - the external power-down mode is in fast recovery power-down mode (default). In this mode, when
 * 									the PWR_DWN_CTL input pin is high, the digital logic and the DAC digital logic are powered down.
 * 									The DAC bias circuitry, PLL, oscillator, and clock input circuitry are not powered down.
 * 								1 - the external power-down mode is in full power-down mode. In this mode, when the PWR_DWN_CTL
 * 									input pin is high, all functions are powered down. This includes the DAC and PLL, which take a significant
 * 									amount of time to power up.
 * @param RefClkInPwrDown 	: Reference clock input power-down.
 * 								0 - the clock input circuitry is enabled for operation (default),
 * 								1 - the clock input circuitry is disabled and is in a low power dissipation state.
 * @param ModulationLvl 	: Modulation level. The modulation (FSK, PSK, and ASK) level bits control the level (2/4/8/16) of modulation
 * 							  to be performed for a channel.
 * 								0 - 2-level,
 * 								1 - 4-level,
 * 								2 - 8-level,
 * 								3 - 16-level.
 * @param RampUpDown 		: Ramp-up/ramp-down (RU/RD). The RU/RD bits control the amplitude ramp-up/ramp-down time of a channel.
 * 								0 - RU/RD disabled,
 * 								1 - Only Profile Pin P2 and Profile Pin P3 available for RU/RD operation,
 * 								2 - Only Profile Pin P3 available for RU/RD operation,
 * 								3 - Only SDIO_1, SDIO_2, and SDIO_3 pins available for RU/RD operation; this forces the serial I/O to be used
 * 									only in 1-bit mode
 * @param ProfilePinCfg 	: Profile pin configuration (PPC). The profile pin configuration bits control the configuration of the
 * 							  data and SDIO_x pins for the different modulation modes. See the Modulation Mode section in this document for details.
 * @param ChargePumpCtrl 	: Charge pump control.
 * 								0 - the charge pump current is 75 μA (default),
 * 								1 - charge pump current is 100 μA,
 * 								2 - charge pump current is 125 μA,
 * 								3 - charge pump current is 150 μA.
 * @param PllDivRatio 		: PLL divider ratio. If the value is 4 or 20 (decimal) or between 4 and 20, the PLL is enabled and the value sets the
 * 							  multiplication factor. If the value is outside of 4 and 20 (decimal), the PLL is disabled (default is 0, the PLL is disabled).
 * @param VcoGainCtrl 		: VCO gain control.
 * 								0 - the low range (system clock below 160 MHz) (default),
 * 								1 - the high range (system clock above 255 MHz).
 */
void AD9959_FunctReg1(uint8_t ManSoftSync, uint8_t ManHardSync, uint8_t DacRefPwrDown, uint8_t SyncClkDis, uint8_t ExtPwrDownMode, uint8_t RefClkInPwrDown,
			uint8_t ModulationLvl, uint8_t RampUpDown, uint8_t ProfilePinCfg, uint8_t ChargePumpCtrl, uint8_t PllDivRatio, uint8_t VcoGainCtrl) {

	AD9959_Funct1_t Function1reg;

	Function1reg.MANUAL_SOFT_SYNC = ManSoftSync;
	Function1reg.MANUAL_HARD_SYNC = ManHardSync;
	Function1reg.DAC_REF_PWR_DOWN = DacRefPwrDown;
	Function1reg.SYNC_CLK_DIS = SyncClkDis;
	Function1reg.EXT_PWR_DOWN_MODE = ExtPwrDownMode;
	Function1reg.REF_CLK_IN_PWR_DOWN = RefClkInPwrDown;
	Function1reg.MODULATION_LEVEL = ModulationLvl;
	Function1reg.RAMP_UP_DOWN = RampUpDown;
	Function1reg.PROFILE_PIN_CFG = ProfilePinCfg;
	Function1reg.CHARGE_PUMP_CTRL = ChargePumpCtrl;
	Function1reg.PLL_DIV_RATIO = PllDivRatio;
	Function1reg.VCO_GAIN_CTRL = VcoGainCtrl;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_FUNCTION1;
	ad9959.RxTxData.Data[0] = Function1reg.FunctReg1_HIGH_LSB;
	ad9959.RxTxData.Data[1] = Function1reg.FunctReg1_LOW_MSB;
	ad9959.RxTxData.Data[2] = Function1reg.FunctReg1_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 4);
	ad9959.delay_fp(1);
}

/*
 * @brief Function register 2.
 *
 * @param SysClkOffset 			: System clock offset.
 * 									0 - 0 ≤ delay ≤ 1,
 * 									1 - 1 ≤ delay ≤ 2,
 * 									2 - 2 ≤ delay ≤ 3,
 * 									3 - 3 ≤ delay ≤ 4.
 * @param MultiDevSyncMask 		: The synchronization routine continues to operate regardless of the state of FR2[SYNC_STATUS_BIT].
 * 							 	  FR2[SYNC_STATUS_BIT] can be masked by writing Logic 1 to the multidevice sync mask bit (FR2[SYNC_MASK_BIT]).
 * 							 	  If FR2[SYNC_STATUS_BIT] is masked, it is held low.
 * @param MultiDevSyncMasterEn 	: Enabling device as master or as slave.
 * 									0 - enabling device as slave,
 * 									1 - enabling device as master.
 * @param AutoSyncEn 			: The first steps are to program the master and slave devices for their respective roles and then write the auto
 * 								  sync enable bit.
 * 									0 - sync disable,
 * 									1 - sync enable.
 * @param AllChClrPhaseAcc 		: All channels clear phase accumulator.
 * 									0 - the phase accumulator functions as normal (default),
 * 									1 - the phase accumulator memory elements for all four channels are asynchronously cleared.
 * @param AllChAutoClrPhaseAcc 	: All channels autoclear phase accumulator.
 * 									0 - a new frequency tuning word is applied to the inputs of the phase accumulator, but not loaded into the accumulator
 * 									 	(default),
 * 									1 - this bit automatically and synchronously clears (loads 0s into) the phase accumulator for one cycle upon receipt of
 * 									 	the I/O update sequence indicator on all four channels.
 * @param AllChClrSweepAcc 		: All channels clear sweep accumulator.
 * 									0 - the sweep accumulator functions as normal (default),
 * 									1 - the sweep accumulator memory elements for all four channels are asynchronously cleared.
 * @param AllChAutoClrSweepAcc 	: All channels autoclear sweep accumulator.
 * 									0 - a new delta word is applied to the input, as in normal operation, but not loaded into the accumulator (default),
 * 									1 - this bit automatically and synchronously clears (loads 0s into) the sweep accumulator for one cycle upon reception
 * 										of the I/O_UPDATE sequence indicator on all four channels.
 */
void AD9959_FunctReg2(uint8_t SysClkOffset, uint8_t MultiDevSyncMask, uint8_t MultiDevSyncMasterEn, uint8_t AutoSyncEn,
					uint8_t AllChClrPhaseAcc, uint8_t AllChAutoClrPhaseAcc, uint8_t AllChClrSweepAcc, uint8_t AllChAutoClrSweepAcc) {

	AD9959_Funct2_t Function2reg;

	Function2reg.SYSTEM_CLK_OFFSET = SysClkOffset;
	Function2reg.SYNC_MASK = MultiDevSyncMask;
	Function2reg.SYNC_MASTER_EN = MultiDevSyncMasterEn;
	Function2reg.AUTOSYNC_EN = AutoSyncEn;
	Function2reg.ALLCH_CLR_PHASE_ACC = AllChClrPhaseAcc;
	Function2reg.ALLCH_AUTOCLR_PHASE_ACC = AllChAutoClrPhaseAcc;
	Function2reg.ALLCH_CLR_SWEEP_ACC = AllChClrSweepAcc;
	Function2reg.ALLCH_AUTOCLR_SWEEP_ACC = AllChAutoClrSweepAcc;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_FUNCTION2;
	ad9959.RxTxData.Data[0] = Function2reg.FunctReg2_LOW_MSB;
	ad9959.RxTxData.Data[1] = Function2reg.FunctReg2_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 3);
	ad9959.delay_fp(1);
}

/*
 * @brief Read sync status bit.
 *
 * @return MultiDevSyncStatus 	: Automatic synchronization status bits. If a slave device falls out of sync, the sync status bit is set high.
 * 							  The multidevice sync status bit (FR2[SYNC_STATUS_BIT]) can be read through the serial port. It is automatically
 * 							  cleared when read.
 */
uint8_t AD9959_ReadSyncStatus(void) {

	static uint16_t SyncStatus;

	ad9959.RxTxData.READ_WRITE = AD9959_READ_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_FUNCTION2;
	ad9959.spi_rx_fp((uint8_t*)&SyncStatus, 2);
	ad9959.delay_fp(1);

	SyncStatus = (SyncStatus >> 8 ) & 0x20;
	return SyncStatus;
}

/*
 * @brief Channel function register.
 *
 * @param SineWaveOutEn 			: Sine wave output enable.
 * 										0 - the angle-to-amplitude conversion logic employs a cosine function (default),
 * 										1 - the angle-to-amplitude conversion logic employs a sine function.
 * @param ClrPhaseAcc 				: Clear phase accumulator.
 * 										0 - the phase accumulator functions as normal (default),
 * 										1 - the phase accumulator memory elements are asynchronously cleared.
 * @param AutoClrPhaseAcc 			: Autoclear phase accumulator.
 *  									0 - the current state of the phase accumulator is not impacted by receipt of an I/O_UPDATE signal (default),
 * 										1 - the phase accumulator is automatically and synchronously cleared for one cycle upon receipt of an I/O_UPDATE signal.
 * @param ClrSweepAcc 				: Clear sweep accumulator.
 * 										0 - the sweep accumulator functions as normal (default),
 * 										1 - the sweep accumulator memory elements are asynchronously cleared.
 * @param AutoClrSweepAcc 			: Autoclear sweep accumulator.
 * 										0 - the current state of the sweep accumulator is not impacted by receipt of an I/O_UPDATE signal (default),
 * 										1 - the sweep accumulator is automatically and synchronously cleared for one cycle upon receipt of an I/O_UPDATE signal.
 * @param MatchedPipeDelaysActive 	: Matched pipe delays active.
 * 										0 - matched pipe delay mode is inactive (default),
 * 										1 - matched pipe delay mode is active. See the Single-Tone Mode—Matched Pipeline Delay section for details.
 * @param DacPwrDown 				: DAC power-down.
 * 										0 - the DAC is enabled for operation (default),
 * 										1 - the DAC is disabled and is in its lowest power dissipation state.
 * @param DigitalPwrDown 			: Digital power-down.
 * 										0 - the digital core is enabled for operation (default),
 * 										1 - the digital core is disabled and is in its lowest power dissipation state.
 * @param DacFullScaleCurrentCtrl 	: DAC full-scale current control.
 * 										0 - eighth scale,
 * 										1 - half scale,
 * 										2 - quarter scale,
 * 										3 - full scale.
 * @param LoadSrrIoUpdate 			: Load SRR at I/O_UPDATE.
 * 										0 - the linear sweep ramp rate timer is loaded only upon timeout (timer = 1) and is not loaded because of an I/O_UPDATE
 * 											input signal (default),
 * 										1 - the linear sweep ramp rate timer is loaded upon timeout (timer = 1) or at the time of an I/O_UPDATE input signal.
 * @param LinearSweepEn 			: Linear sweep enable.
 * 										0 - the linear sweep capability is inactive (default),
 * 										1 - the linear sweep capability is enabled. When enabled, the delta frequency tuning word is applied to the frequency
 * 											accumulator at the programmed ramp rate.
 * @param LinearSweepNoDwell 		: Linear sweep no-dwell.
 * 										0 - the linear sweep no-dwell function is inactive (default),
 * 										1 - the linear sweep no-dwell function is active. If CFR[LINEAR_SWEEP_NO_DWELL] is active, the linear sweep no-dwell function
 * 											is activated. See the Linear Sweep Mode section for details. If CFR[LINEAR_SWEEP_ENABLE] is clear, this bit is don’t care.
 * @param AfpSelect 				: Amplitude frequency phase (AFP) select. Controls what type of modulation is to be performed for that channel. See the Modulation
 * 									  Mode section for details.
 * 									  	0 - modulation disabled,
 * 									  	1 - amplitude modulation,
 * 									  	2 - frequency modulation,
 * 									  	3 - phase modulation.
 */
void AD9959_ChannelFunctReg(uint8_t SineWaveOutEn, uint8_t ClrPhaseAcc, uint8_t AutoClrPhaseAcc, uint8_t ClrSweepAcc, uint8_t AutoClrSweepAcc,
							uint8_t MatchedPipeDelaysActive, uint8_t DacPwrDown, uint8_t DigitalPwrDown, uint8_t DacFullScaleCurrentCtrl,
							uint8_t LoadSrrIoUpdate, uint8_t LinearSweepEn, uint8_t LinearSweepNoDwell, uint8_t AfpSelect) {

	AD9959_ChFunct_t ChFunctReg;

	ChFunctReg.SINE_WAVE_OUT_EN = SineWaveOutEn;
	ChFunctReg.CLR_PHASE_ACC = ClrPhaseAcc;
	ChFunctReg.AUTOCLR_PHASE_ACC = AutoClrPhaseAcc;
	ChFunctReg.CLR_SWEEP_ACC = ClrSweepAcc;
	ChFunctReg.AUTOCLR_SWEEP_ACC = AutoClrSweepAcc;
	ChFunctReg.MATCHED_PIPE_DELAYS_ACTIVE = MatchedPipeDelaysActive;
	ChFunctReg.DAC_PWR_DOWN = DacPwrDown;
	ChFunctReg.DIGITAL_PWR_DOWN = DigitalPwrDown;
	ChFunctReg.DAC_FULL_SCALE_CURRENT_CTRL = DacFullScaleCurrentCtrl;
	ChFunctReg.LOAD_SRR_AT_IO_UPDATE = LoadSrrIoUpdate;
	ChFunctReg.LINEAR_SWEEP_EN = LinearSweepEn;
	ChFunctReg.LINEAR_SWEEP_NO_DWELL = LinearSweepNoDwell;
	ChFunctReg.AFP_SEL = AfpSelect;
	ChFunctReg.MUST_BE_ZERO = 0;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_CH_FUNCTION;
	ad9959.RxTxData.Data[0] = ChFunctReg.ChFunctReg_HIGH_LSB;
	ad9959.RxTxData.Data[1] = ChFunctReg.ChFunctReg_LOW_MSB;
	ad9959.RxTxData.Data[2] = ChFunctReg.ChFunctReg_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 4);
	ad9959.delay_fp(1);
}

/*
 * @brief Frequency tuning word 0.
 *
 * @param FreqTuneWord0 : Frequency Tuning Word 0 for each channel.
 */
void AD9959_FreqTuneWord0(uint32_t FreqTuneWord) {

	AD9959_FreqTuneWord0_t FreqTuneWord0;

	FreqTuneWord0.FreqTunWord0_HIGH_MSB = FreqTuneWord >> 24;
	FreqTuneWord0.FreqTunWord0_HIGH_LSB = FreqTuneWord >> 16;
	FreqTuneWord0.FreqTunWord0_LOW_MSB = FreqTuneWord >> 8;
	FreqTuneWord0.FreqTunWord0_LOW_LSB = FreqTuneWord;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_CH_FREQUENCY_TUNING;
	ad9959.RxTxData.Data[0] = FreqTuneWord0.FreqTunWord0_HIGH_MSB;
	ad9959.RxTxData.Data[1] = FreqTuneWord0.FreqTunWord0_HIGH_LSB;
	ad9959.RxTxData.Data[2] = FreqTuneWord0.FreqTunWord0_LOW_MSB;
	ad9959.RxTxData.Data[3] = FreqTuneWord0.FreqTunWord0_LOW_LSB;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 5);
	ad9959.delay_fp(1);
}

/*
 * @brief Channel phase offset word 0.
 *
 * @param ChPhaseOffsetWord : Phase offset word 0 for each channel. Only first 14-bit are valid.
 */
void AD9959_ChPhaseOffsetWord0(uint16_t ChPhaseOffsetWord) {

	AD9959_PhaseOffsetWord0_t ChPhaseOffsetWord0;

	ChPhaseOffsetWord0.PHASE_OFFSET_WORD0 = ChPhaseOffsetWord;

	ad9959.RxTxData.READ_WRITE = AD9959_WRITE_CMD;
	ad9959.RxTxData.REG_ADDRESS = AD9959_CH_PHASE_OFFSET;
	ad9959.RxTxData.Data[0] = ChPhaseOffsetWord0.PhaseOffsetWord016 >> 8;
	ad9959.RxTxData.Data[1] = ChPhaseOffsetWord0.PhaseOffsetWord016;
	ad9959.spi_tx_fp(&ad9959.RxTxData.InstrByte, 3);
	ad9959.delay_fp(1);
}

/*
 * @brief Read specific register.
 *
 * @return RegAddress 	:
 */
AD9959_ReadData_t* AD9959_ReadReg(uint8_t RegAddress, uint8_t Size) {

	static AD9959_ReadData_t Data;

	ad9959.RxTxData.READ_WRITE = AD9959_READ_CMD;
	ad9959.RxTxData.REG_ADDRESS = RegAddress;
	ad9959.spi_rx_fp(&Data.RegData[0], Size);
	ad9959.delay_fp(1);

	__NOP();

	return &Data;
}

/* Private functions. */

/*
 * @brief Read byte from the specific address.
 *
 * @param Address : Specific address of the ad9959 register map.
 */
//static uint8_t AD9959_ReadByte(uint8_t Address) {
//
//	return 0;
//}

/*
 * @brief Write byte to the specific address.
 *
 * @param Address 	: Specific address of the ad9959 register map.
 * @param Value 	: Desired value of register.
 * @param Size 		: Data transfer size.
 */
//static void AD9959_WriteByte(uint8_t Address, uint32_t Value, uint8_t Size) {
//
//}

/* Hardware dependent functions. */

/*
 * @brief Clocking data.
 */
static void AD9959_ClockData(void) {

	HAL_GPIO_WritePin(AD9959_CLK_PORT, AD9959_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(AD9959_CLK_PORT, AD9959_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Transmit data to the chip (MSB first).
 */
static void AD9959_ESpiTxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Transmitting the data to the shift register. */
	TxData = *pData;

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = AD9959_NSS_PIN | AD9959_CLK_PIN | AD9959_DATA_INOUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(AD9959_NSS_PORT, &GPIO_InitStruct);
	/* Activate NSS pin. */
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_RESET);
	/* Write command word. */
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t j = AD9959_BIT_NUMBER; j > 0; j--) {
			/* MSB first. */
			PinState = (TxData & AD9959_BIT_MASK) ? (PinState = 1) : (PinState = 0);
			HAL_GPIO_WritePin(AD9959_DATA_INOUT_PORT, AD9959_DATA_INOUT_PIN, PinState);
			/* Clocking data. */
			AD9959_ClockData();
			/* Getting next bit.*/
			TxData <<= 1;
		}
		pData++;
		TxData = *pData;
	}
	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Receive data from the chip (MSB first).
 */
static void AD9959_ESpiRxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;
	uint16_t RxData;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Transmitting the data to the shift register. */
	TxData = ad9959.RxTxData.InstrByte;

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = AD9959_NSS_PIN | AD9959_CLK_PIN | AD9959_DATA_INOUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(AD9959_NSS_PORT, &GPIO_InitStruct);
	/* Activate NSS pin. */
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_RESET);
	/* Write command word. */
	for (uint8_t i = AD9959_BIT_NUMBER; i > 0; i--) {
		/* MSB first. */
		PinState = (TxData & AD9959_BIT_MASK) ? (PinState = 1) : (PinState = 0);
		HAL_GPIO_WritePin(AD9959_DATA_INOUT_PORT, AD9959_DATA_INOUT_PIN, PinState);
		/* Clocking data. */
		AD9959_ClockData();
		/* Getting next bit.*/
		TxData <<= 1;
	}
	/* Configurate MCU pin to input. */
	GPIO_InitStruct.Pin = AD9959_DATA_INOUT_PIN | AD9959_DATA_IN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(AD9959_DATA_INOUT_PORT, &GPIO_InitStruct);
	/* Read data byte. */
	RxData = 0;
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t j = AD9959_BIT_NUMBER; j > 0; j--) {
			/* MSB first. */
			RxData |= HAL_GPIO_ReadPin(AD9959_DATA_INOUT_PORT, AD9959_DATA_INOUT_PIN);
			RxData <<= 1;
			/* Clocking data. */
			AD9959_ClockData();
		}
		RxData >>= 1;
		*pData = RxData;
		RxData = 0;
		pData++;
	}

	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Receive data from the chip.
 */
static void AD9959_SpiRxData(uint8_t *pTxData, uint8_t *pRxData, uint8_t Size) {

	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(AD9959Spi, pTxData, 1, 25);
	HAL_SPI_Receive(AD9959Spi, pRxData, Size, 25);
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Transmit data to the chip.
 */
static void AD9959_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(AD9959Spi, pData, Size, 25);
	HAL_GPIO_WritePin(AD9959_NSS_PORT, AD9959_NSS_PIN, GPIO_PIN_SET);
}
















