/*
 * ad9613.c source file.
 * Created on: Aug 23, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "ad9613.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;

/* Private variables. */
static SPI_HandleTypeDef* AD9613Spi = &hspi1;

/* Private function prototypes. */
static uint8_t AD9613_ReadByte(uint8_t Address);
static void AD9613_WriteByte(uint8_t Address, uint8_t Value);

static void AD9613_ClockData(void);
static void AD9613_ESpiTxData(uint8_t* pData, uint8_t Size);
static void AD9613_ESpiRxData(uint8_t* pData, uint8_t Size);

static void AD9613_SpiRxData(uint8_t *pData, uint8_t Size);
static void AD9613_SpiTxData(uint8_t *pData, uint8_t Size);

/* Init general struct. */
static AD9613_GStr_t ad9613 = {
		.delay_fp = HAL_Delay,
		.spi_rx_fp = AD9613_ESpiRxData,
		.spi_tx_fp = AD9613_ESpiTxData
};

/*
 * @brief Initialization the chip.
 */
void AD9613_Init(void) {


	uint16_t pattern;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = AD9613_NSS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(AD9613_NSS_PORT, &GPIO_InitStruct);
	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(AD9613_NSS_PORT, AD9613_NSS_PIN, GPIO_PIN_SET);

	/* Reset the chip. */
	AD9613_SpiPortCfg(0, 1);
	HAL_Delay(100);

	if (AD9613_GetChipId() != AD9643_CHIID || AD9613_GetChipId() != AD9613_CHIID) {


		AD9613_SetUserTestPattern(0xAA55, 0xBB55, 0xAABB, 0xAB);
		AD9613_EnDisDcs(0);
		AD9613_TestMode(7, 0, 0, 0);

		pattern = AD9613_ReadByte(AD9613_SPI_PORT_CFG);
		pattern = AD9613_ReadByte(AD9613_CHIP_ID);
		pattern = AD9613_ReadByte(AD9613_CHIP_GRADE);
		pattern = AD9613_ReadByte(AD9613_CHANNEL_INDEX);

		pattern = AD9613_ReadByte(AD9613_TEST_MODE);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN1_LSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN1_MSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN2_LSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN2_MSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN3_LSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN3_MSB);
		pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN4_LSB);

		pattern = AD9613_ReadByte(AD9613_GLOBAL_CLOCK);
		pattern = AD9613_ReadByte(AD9613_CLOCK_DIVIDE);
		__NOP();
		return;
	}





	AD9613_SetUserTestPattern(0xAA55, 0xBB55, 0xAABB, 0xAB);
	AD9613_EnDisDcs(0);
	AD9613_TestMode(4, 0, 0, 0);

	pattern = AD9613_ReadByte(AD9613_SPI_PORT_CFG);
	pattern = AD9613_ReadByte(AD9613_CHIP_ID);
	pattern = AD9613_ReadByte(AD9613_CHIP_GRADE);
	pattern = AD9613_ReadByte(AD9613_CHANNEL_INDEX);

	pattern = AD9613_ReadByte(AD9613_TEST_MODE);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN1_LSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN1_MSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN2_LSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN2_MSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN3_LSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN3_MSB);
	pattern = AD9613_ReadByte(AD9613_USER_TEST_PATTERN4_LSB);

	pattern = AD9613_ReadByte(AD9613_GLOBAL_CLOCK);
	pattern = AD9613_ReadByte(AD9613_CLOCK_DIVIDE);

	__NOP();

}

/*
 * @brief Read chip ID (global).
 *
 */
uint8_t AD9613_GetChipId(void) {

	static uint8_t ChipId;

	ChipId = AD9613_ReadByte(AD9613_CHIP_ID);
	return ChipId;
}

/*
 * @brief Read chip grade (global).
 *
 */
uint8_t AD9613_GetChipGrade(void) {

	static uint8_t ChipGrade;

	ChipGrade = AD9613_ReadByte(AD9613_CHIP_GRADE);
	ChipGrade = (ChipGrade & 0x30) >> 4;

	return ChipGrade;
}

/*
 * @brief Read RST bit state.
 *
 */
uint8_t AD9613_GetRstBitState(void) {

	static uint8_t RstBitState;

	RstBitState = AD9613_ReadByte(AD9613_SPI_PORT_CFG);
	RstBitState = (RstBitState & 0x04) >> 2;

	return RstBitState;
}

/*
 * @brief Read software transfer bit state.
 *
 */
uint8_t AD9613_GetSoftTxBitState(void) {

	static uint8_t SoftTxBitState;

	SoftTxBitState = AD9613_ReadByte(AD9613_TRANSFER);
	SoftTxBitState &= 0x01;

	return SoftTxBitState;
}

/*
 * @brief SPI port configuration control (global).
 *
 * @param LsbFirst 	: Change output/input data order and addressing.
 * 						0 - MSB first and decrementing addressing,
 * 						1 - LSB first and incrementing addressing.
 * @param SoftReset : Software reset the chip.
 * 						0 - on-chip power up, any registers with a default set,
 * 						1 - restoring any default values to internal registers. Registers with no default are not changed.
 * 							Ones this is complete, the state machine clears this bit.
 */
void AD9613_SpiPortCfg(uint8_t LsbFirst, uint8_t SoftReset) {

	AD9613_SpiPortCfg_t SpiPortCfg;

	SpiPortCfg.MUST_BE_ZERO = 0;
	SpiPortCfg.LSB_FIRST = LsbFirst;
	SpiPortCfg.SOFT_RST = SoftReset;
	SpiPortCfg.MUST_BE_11 = 3;
	SpiPortCfg.SOFT_RSTM = SoftReset;
	SpiPortCfg.LSB_FIRSTM = LsbFirst;
	SpiPortCfg.MUST_BE_ZEROM = 0;

	AD9613_WriteByte(AD9613_SPI_PORT_CFG, SpiPortCfg.SpiPortCfgReg);
}

/*
 * @brief Selecting active channel, ADC A or ADC B. Bits are set to determine which device on the chip receives the
 * next write command, applies to local registers only (global).
 *
 * @param ChannelNumber	: Set active channel's number (default 0x03).
 * 							0 - there is no active channel,
 * 							1 - ADC A is active,
 * 							2 - ADC B is active,
 * 							3 - ADC A and ADC B are active.
 *
 */
void AD9613_ChSelect(uint8_t ChannelNumber) {

	AD9613_WriteByte(AD9613_CHANNEL_INDEX, ChannelNumber & 0x03);
}

/*
 * @brief Synchronously transfers data from the master shift register to the slave (global).
 *
 * @param InitSoftTransfer 	: 1 generates an internal transfer signal.
 *
 */
void AD9613_StartSoftTx(void) {

	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Power modes control. Determines various generic modes of chip operation (local).
 *
 * @param IntPwrDown 		: Internal power-down mode.
 * 								0 - normal operation (default),
 * 								1 - full power-down,
 * 								2 - standby,
 * 								3 - reserved.
 * @param ExtPwrDownPinf 	: External power-down pin function.
 * 								0 - power-down (default),
 * 								1 - standby.
 */
void AD9613_PwrModes(uint8_t IntPwrDown, uint8_t ExtPwrDownPinf) {

	AD9613_PwrModes_t PwrModes;

	PwrModes.INT_PWR_DOWN = IntPwrDown;
	PwrModes.EXT_PWR_DOWN_PINF = ExtPwrDownPinf;

	AD9613_WriteByte(AD9613_POWER_MODES, PwrModes.PwrModesReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Duty cycle stabilizer control (global).
 *
 * @param EnDisDcs : Disable or enable internal duty cycle stabilizer (DCS).
 * 						0 - DCS is disabled,
 * 						1 - DCS is enabled (default).
 */
void AD9613_EnDisDcs(uint8_t EnDisDcs) {

	AD9613_WriteByte(AD9613_GLOBAL_CLOCK, EnDisDcs & 0x01);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Used to divide the applied clock to a lower rate for the encode. Clock divide values other than 000 auto-matically cause
 * the duty cycle stabilizer to become active (global).
 *
 * @param ClkDivRatio 		: Clock divide ratio. Values from 0 to 7 are dividers from 1 to 8 (default is 0).
 * @param InClkDivPhaseAdj 	: Input clock divider phase adjust. Values from 0 to 7 are delays from 1 to 7 input clock cycles (default is 0).
 */
void AD9613_ClockDivide(uint8_t ClkDivRatio, uint8_t InClkDivPhaseAdj) {

	AD9613_ClockDivide_t ClkDivide;

	ClkDivide.CLK_DIV_RATIO = ClkDivRatio;
	ClkDivide.IN_CLK_DIV_PHASE_ADJ = InClkDivPhaseAdj;

	AD9613_WriteByte(AD9613_CLOCK_DIVIDE, ClkDivide.ClkDivideReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief User test mode sequences. When this register is set, the test data is placed on the output pins in place of normal data (local).
 *
 * @param OutTestMode 		: Output test mode.
 * 								0 - off (default),
 * 								1 - midscale short,
 * 								2 - positive FS,
 * 								3 - negative FS,
 * 								4 - alternating checkerboard,
 * 								5 - PN long sequence (ITU 0.150 x^23 + x^18 + 1),
 * 								6 - PN short sequence (ITU 0.150 x^9 + x^5 + 1),
 * 								7 - one/zero word toggle,
 * 								8 - user test mode,
 * 								9 - unused,
 * 								10 - ramp output.
 * @param RstPnShortGen 	: Controls the reset short PN sequence.
 * 								0 - the PN sequence resumes from the seed value (seed value is 0x000092) (default),
 * 								1 - the PN sequence is held in reset.
 * @param RstPnLongGen 		: Controls the reset long PN sequence.
 * 								0 - the PN sequence resumes from the seed value (seed value is 0x003AFF) (default),
 * 								1 - the PN sequence is held in reset.
 * @param UserTestModeCtrl 	: These bits are used in conjunction with teas mode 8 defined by bit 3 to bit 0 (OutTestMode parameter).
 * 								0 - continuous/repeat pattern stored in user pattern x registers (default),
 * 								1 - single pattern, then 0s.
 */
void AD9613_TestMode(uint8_t OutTestMode, uint8_t RstPnShortGen, uint8_t RstPnLongGen, uint8_t UserTestModeCtrl) {

	AD9613_TestMode_t TestMode;

	TestMode.OUT_TEST_MODE = OutTestMode;
	TestMode.RST_PN_SHORT_GEN = RstPnShortGen;
	TestMode.RST_PN_LONG_GEN = RstPnLongGen;
	TestMode.USER_TEST_MODE_CTRL = UserTestModeCtrl;

	AD9613_WriteByte(AD9613_TEST_MODE, TestMode.TestModeReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Allows the offset of the device to be tweaked. The purpose of this register is to provide sufficient offset to move thermal noise
 * off midscale (local).
 *
 * @param OffsetAdjInLsb : Offset adjust in LSBs from +31 to −32 (twos complement format).
 */
void AD9613_OffsetAdj(int8_t OffsetAdjInLsb) {

	AD9613_OffsetAdj_t OffsetAdj;

	OffsetAdj.OFFSET_ADJ = OffsetAdjInLsb;
	if (OffsetAdjInLsb < 0) { OffsetAdj.OFFSET_SIGN = 1; }
	if (OffsetAdjInLsb >= 0) { OffsetAdj.OFFSET_SIGN = 0; }

	AD9613_WriteByte(AD9613_OFFSET_ADJUST, OffsetAdj.OffsetAdjReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Configures the outputs and the format of the data.
 *
 * @param OutFormat	: Set output format.
 * 						0 - offset binary,
 * 						1 - twos complement (default),
 * 						2 - gray code,
 * 						3 - reserved.
 * @param OutInvert	: Inverts output.
 * 						0 - inverted,
 * 						1 - normal (default).
 * @param OutEnBar	: Enables the output of the chip.
 * 						0 - the output is enabled (default),
 * 						1 - the output is disabled.
 */
void AD9613_OutputMode(uint8_t OutFormat, uint8_t OutInvert, uint8_t OutEnBar) {

	AD9613_OutMode_t OutMode;

	OutMode.OUT_FORMAT = OutFormat;
	OutMode.OUT_INVERT = OutInvert;
	OutMode.OUT_EN_BAR = OutEnBar;

	AD9613_WriteByte(AD9613_OUTPUT_MODE, OutMode.OutModeReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Output current adjust (global).
 *
 * @param OutAdj : LVDS output drive current adjust.
 *					0 - 3.72mA,
 *					1 - 3.5mA,
 *					2 - 3.3mA,
 *					3 - 2.96mA,
 *					4 - 2.82mA,
 *					5 - 2.57mA,
 *					6 - 2.27mA,
 *					7 - 2.0mA,
 *					8 - 16 are reserved.
 */
void AD9613_OutputAdj(uint8_t OutAdj) {

	AD9613_WriteByte(AD9613_OUTPUT_ADJUST, OutAdj & 0x07);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Clock phase control (global).
 *
 * @param OutFormat	: Invert DCO clock.
 * 						0 - phase is not inverted (default),
 * 						1 - phase is inverted.
 * @param OutInvert	: Odd/Even mode output enable.
 * 						0 - disabled (default),
 * 						1 - enabled.
 */
void AD9613_ClkPhaseCtrl(uint8_t OddEvenMode, uint8_t InvertDcoClk) {

	AD9613_ClkPhaseCtrl_t ClkPhase;

	ClkPhase.ODD_EVEN_OUT_EN = OddEvenMode;
	ClkPhase.INVERT_DCO_CLK = InvertDcoClk;

	AD9613_WriteByte(AD9613_CLOCK_PHASE_CTRL, ClkPhase.ClkPhaseCtrlReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief DCO output delay control (global).
 *
 * @param EnDcoClkDelay	: Enable DCO clock delay.
 * 							0 - disabled (default),
 * 							1 - enabled.
 * @param DcoClkDelay	: DCO clock delay [delay = (3100 ps × register value/31 +100)].
 * 							0 - 100ps (default),
 * 							...
 * 							31 - 3200ps.
 */
void AD9613_DcoOutDelay(uint8_t EnDcoClkDelay, uint8_t DcoClkDelay) {

	AD9613_DcoOutDelay_t DcoOutDelay;

	DcoOutDelay.EN_DCO_CLK_DELAY = EnDcoClkDelay;
	DcoOutDelay.EN_DCO_CLK_DELAY = DcoClkDelay;

	AD9613_WriteByte(AD9613_DCO_OUTPUT_DELAY, DcoOutDelay.DcoOutDelayReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Input span select (global).
 *
 * @param EnDcoClkDelay	: Full-scale input voltage selection. Input adjustment is in 0.022V steps, from 15 to -15.
 * 							01111 - 2.087Vp-p,
 * 							...
 * 							00001 - 1.772Vp-p,
 * 							00000 - 1.75Vp-p (default),
 * 							11111 - 1.727Vp-p,
 * 							...
 * 							10000 - 1.383Vp-p.
 */
void AD9613_InVoltageSel(int8_t InVoltageSel) {

	AD9613_InSpanSel_t InSpanSel;

	InSpanSel.IN_SPAN = InVoltageSel;
	if (InVoltageSel < 0) { InSpanSel.SPAN_SIGN = 1; }
	if (InVoltageSel >= 0) { InSpanSel.SPAN_SIGN = 0; }

	AD9613_WriteByte(AD9613_INPUT_SPAN_SEL, InSpanSel.InputSpanSelReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Set user test pattern X.
 *
 * @param UserPattern 1	: 16-bit user pattern value.
 * @param UserPattern 2	: 16-bit user pattern value.
 * @param UserPattern 3	: 16-bit user pattern value.
 * @param UserPattern 4	: 8-bit user pattern value.
 */
void AD9613_SetUserTestPattern(uint16_t UserPattern1, uint16_t UserPattern2, uint16_t UserPattern3, uint8_t UserPattern4) {

	ad9613.RxTxData.READ_WRITE = AD9613_WRITE_CMD;
	ad9613.RxTxData.REG_ADDRESS = AD9613_USER_TEST_PATTERN4_LSB;
	ad9613.RxTxData.DATA_LENGTH_W0W1 = AD9613_STREAM_MODE;
	/* Reverse instruction byte (MSB byte). */
	ad9613.RxTxData.Data[0] = (ad9613.RxTxData.InstrByte & 0xFF00) >> 8;
	ad9613.RxTxData.Data[1] = ad9613.RxTxData.InstrByte;
	/* data to transfer. */
	ad9613.RxTxData.Data[8] = UserPattern1;
	ad9613.RxTxData.Data[7] = UserPattern1 >> 8;
	ad9613.RxTxData.Data[6] = UserPattern2;
	ad9613.RxTxData.Data[5] = UserPattern2 >> 8;
	ad9613.RxTxData.Data[4] = UserPattern3;
	ad9613.RxTxData.Data[3] = UserPattern3 >> 8;
	ad9613.RxTxData.Data[2] = UserPattern4;
	ad9613.spi_tx_fp(&ad9613.RxTxData.Data[0], 9);
	ad9613.delay_fp(1);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/*
 * @brief Sync control.
 *
 * @param MasterSyncBuffEn		: Must be set high to enable any of the sync functions. If the sync capability is not used,
 * 								  this bit should remain low to conserve power..
 * 									0 - disabled (default),
 * 									1 - enabled.
 * @param ClkDivSyncEn 			: Gates the sync pulse to the clock divider. The sync signal is enabled when Bit 1 is high
 * 								  and Bit 0 is high. This is continuous sync mode.
 * 									0 - disabled (default),
 * 									1 - enabled.
 * @param ClkDivNextSyncOnly 	: If the master sync buffer enable bit and the clock divider sync enable bit are high,
 * 								  Bit 2 allows the clock divider to sync to the first sync pulse that it receives and to ignore
 * 								  the rest. The clock divider sync enable bit resets after it syncs.
 */
void AD9613_SyncCtrl(uint8_t MasterSyncBuffEn, uint8_t ClkDivSyncEn, uint8_t ClkDivNextSyncOnly) {

	AD9613_SyncCtrl_t SyncCtrl;

	SyncCtrl.MASTER_SYNC_BUFF_EN = MasterSyncBuffEn;
	SyncCtrl.CLK_DIV_SYNC_EN = ClkDivSyncEn;
	SyncCtrl.CLK_DIV_NEXT_SYNC_EN = ClkDivNextSyncOnly;

	AD9613_WriteByte(AD9613_SYNC_CTRL, SyncCtrl.SyncControlReg);
	AD9613_WriteByte(AD9613_TRANSFER, 0x01);
}

/* Private functions. */

/*
 * @brief Read byte from the specific address.
 *
 * @param Address : Specific address of the ad9959 register map.
 */
static uint8_t AD9613_ReadByte(uint8_t Address) {

	uint8_t ReadBack;

	ad9613.RxTxData.READ_WRITE = AD9613_READ_CMD;
	ad9613.RxTxData.REG_ADDRESS = Address;
	ad9613.RxTxData.DATA_LENGTH_W0W1 = AD9613_ONE_BYTE;

	AD9613_ESpiRxData(&ReadBack, 1);
	ad9613.delay_fp(1);

//	ad9613.spi_tx_fp((uint8_t*)&ad9613.RxTxData.InstrByte, 2);
//	ad9613.delay_fp(1);
//	ad9613.spi_rx_fp((uint8_t*)&ReadBack, 1);
//	ad9613.delay_fp(1);
//	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_RESET);
//	//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&ad9613.RxTxData.InstrByte, (uint8_t*)&ReadBack, 3, 10);
//	HAL_SPI_Transmit(AD9613Spi, (uint8_t*)&ad9613.RxTxData.InstrByte, 2, 25);
//	HAL_SPI_Receive(AD9613Spi, (uint8_t*)&ReadBack, 1, 25);
//
//	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_SET);
//	ad9613.delay_fp(1);

	return ReadBack;
}

/*
 * @brief Write byte to the specific address.
 *
 * @param Address 	: Specific address of the ad9959 register map.
 * @param Value 	: Desired value of register.
 */
static void AD9613_WriteByte(uint8_t Address, uint8_t Value) {

	ad9613.RxTxData.READ_WRITE = AD9613_WRITE_CMD;
	ad9613.RxTxData.REG_ADDRESS = Address;
	ad9613.RxTxData.DATA_LENGTH_W0W1 = AD9613_ONE_BYTE;

	/* Reverse instruction byte (MSB byte). */
	ad9613.RxTxData.Data[0] = (ad9613.RxTxData.InstrByte & 0xFF00) >> 8;
	ad9613.RxTxData.Data[1] = ad9613.RxTxData.InstrByte;
	/* Data to transfer. */
	ad9613.RxTxData.Data[2] = Value;
	ad9613.spi_tx_fp(&ad9613.RxTxData.Data[0], 3);
	ad9613.delay_fp(1);

}


/* Hardware dependent functions. */

/*
 * @brief Clocking data.
 */
static void AD9613_ClockData(void) {

	HAL_GPIO_WritePin(AD9613_CLK_PORT, AD9613_CLK_PIN, GPIO_PIN_SET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	HAL_GPIO_WritePin(AD9613_CLK_PORT, AD9613_CLK_PIN, GPIO_PIN_RESET);
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}

/*
 * @brief Transmit data to the chip (MSB first).
 */
static void AD9613_ESpiTxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Transmitting the data to the shift register. */
	TxData = *pData;

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = AD9613_NSS_PIN | AD9613_CLK_PIN | AD9613_DATA_INOUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AD9613_NSS_PORT, &GPIO_InitStruct);
	/* Activate NSS pin. */
	HAL_GPIO_WritePin(AD9613_NSS_PORT, AD9613_NSS_PIN, GPIO_PIN_RESET);
	/* Write command word. */
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t j = AD9613_BIT_NUMBER; j > 0; j--) {
			/* MSB first. */
			PinState = (TxData & AD9613_BIT_MASK) ? (PinState = 1) : (PinState = 0);
			HAL_GPIO_WritePin(AD9613_DATA_INOUT_PORT, AD9613_DATA_INOUT_PIN, PinState);
			/* Clocking data. */
			AD9613_ClockData();
			/* Getting next bit.*/
			TxData <<= 1;
		}
		pData++;
		TxData = *pData;
	}
	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(AD9613_NSS_PORT, AD9613_NSS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Receive data from the chip (MSB first).
 */
static void AD9613_ESpiRxData(uint8_t* pData, uint8_t Size) {

	uint8_t TxData, PinState;
	uint16_t RxData;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Transmitting instruction byte to the shift register (MSB byte). */
	TxData = (ad9613.RxTxData.InstrByte & 0xFF00) >> 8;

	/* Configurate MCU pin to output. */
	GPIO_InitStruct.Pin = AD9613_NSS_PIN | AD9613_CLK_PIN | AD9613_DATA_INOUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AD9613_NSS_PORT, &GPIO_InitStruct);
	/* Activate NSS pin. */
	HAL_GPIO_WritePin(AD9613_NSS_PORT, AD9613_NSS_PIN, GPIO_PIN_RESET);
	/* Write command word. */
	for (uint8_t i = AD9613_CMD_WORD_SIZE; i > 0; i--) {
		for (uint8_t j = AD9613_BIT_NUMBER; j > 0; j--) {
			/* MSB first. */
			PinState = (TxData & AD9613_BIT_MASK) ? (PinState = 1) : (PinState = 0);
			HAL_GPIO_WritePin(AD9613_DATA_INOUT_PORT, AD9613_DATA_INOUT_PIN, PinState);
			/* Clocking data. */
			AD9613_ClockData();
			/* Getting next bit.*/
			TxData <<= 1;
		}
		/* LSB byte. */
		TxData = ad9613.RxTxData.InstrByte;
	}
	/* Configurate MCU pin to input. */
	GPIO_InitStruct.Pin = AD9613_DATA_INOUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(AD9613_DATA_INOUT_PORT, &GPIO_InitStruct);
	/* Read data byte. */
	RxData = 0;
	for (uint8_t i = Size; i > 0; i--) {
		for (uint8_t j = AD9613_BIT_NUMBER; j > 0; j--) {
			/* Clocking data. */
			HAL_GPIO_WritePin(AD9613_CLK_PORT, AD9613_CLK_PIN, GPIO_PIN_SET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

			/* MSB first. */
			RxData |= HAL_GPIO_ReadPin(AD9613_DATA_INOUT_PORT, AD9613_DATA_INOUT_PIN);
			RxData <<= 1;
			/* Clocking data. */
			HAL_GPIO_WritePin(AD9613_CLK_PORT, AD9613_CLK_PIN, GPIO_PIN_RESET);
			__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		}
	}
	RxData >>= 1;
	/* Deactivate NSS pin. */
	HAL_GPIO_WritePin(AD9613_NSS_PORT, AD9613_NSS_PIN, GPIO_PIN_SET);

	*pData = RxData;
}

/*
 * @brief Receive data from the chip.
 */
static void AD9613_SpiRxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Receive(AD9613Spi, pData, Size, 25);
	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_SET);
}

/*
 * @brief Transmit data to the chip.
 */
static void AD9613_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(AD9613Spi, pData, Size, 25);
	HAL_GPIO_WritePin(AD9613_CS_PORT, AD9613_CS_PIN, GPIO_PIN_SET);
}

