/*
 * ad9520.c
 *
 * Created on: Aug 5, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "ad9520.h"


/* External variables. */
extern SPI_HandleTypeDef hspi6;

/* Private variables. */
static SPI_HandleTypeDef* AD9520Spi = &hspi6;
static AD9520_GStr_t ad9520 = {

		.delay_fp = HAL_Delay,
		.spi_rx_fp = Ad9520_SpiRxData,
		.spi_tx_fp = Ad9520_SpiTxData
};


/*
 * @brief Init AD9520.
 */
uint8_t AD9520_Init(void) {

	/* SDO disable, LSB first, no soft reset. */
	AD9520_SpiCtrl(0, 1, 0);

	/* Reinit AD9520Spi */
	AD9520Spi->Instance 				= SPI6;
	AD9520Spi->Init.Mode 				= SPI_MODE_MASTER;
	AD9520Spi->Init.Direction 			= SPI_DIRECTION_1LINE;
	AD9520Spi->Init.DataSize 			= SPI_DATASIZE_8BIT;
	AD9520Spi->Init.CLKPolarity 		= SPI_POLARITY_LOW;
	AD9520Spi->Init.CLKPhase 			= SPI_PHASE_1EDGE;
	AD9520Spi->Init.NSS 				= SPI_NSS_SOFT;
	AD9520Spi->Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_2;
	AD9520Spi->Init.FirstBit 			= SPI_FIRSTBIT_LSB;
	AD9520Spi->Init.TIMode 				= SPI_TIMODE_DISABLE;
	AD9520Spi->Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
	AD9520Spi->Init.CRCPolynomial 		= 10;
	if (HAL_SPI_Init(AD9520Spi) != HAL_OK) {
	Error_Handler();
	}

	/* Is the chip available? */
	if (AD9520_GetPartID() != AD9520x_TYPE) {
		return AD9520x_NO_DEVICE;
	}
	/* PFD charge pump. */
	AD9520_PfdChargePumpCtrl(0, 3, 7, 0);
	AD9520_RCounterCfg(0x0001);
	AD9520_ACounterCfg(0x00);
	AD9520_BCounterCfg(0x003);
	AD9520_PllCtrl1(6, 0, 0, 0, 0, 0);
	AD9520_PllCtrl2(0, 0);
	AD9520_PllCtrl3(0, 3, 0, 0, 0, 0);
	AD9520_PllCtrl4(0, 0, 0);
	AD9520_PllCtrl5(0, 0, 0);
	AD9520_PllCtrl6(0, 0, 0, 0);
	AD9520_PllCtrl7(0, 1, 0, 0, 0, 0, 0, 0);
	AD9520_PllCtrl8(0, 0, 0, 0, 0, 0, 1);
	AD9520_PllCtrl9(0, 0, 0);
	/* Left enabled only outputs from 3 to 8. Remaining outputs set to safe power-down mode. */
	AD9520_OutCtrl(0, 1, 2, 0, 3, 0);
	AD9520_OutCtrl(1, 1, 2, 0, 3, 0);
	AD9520_OutCtrl(2, 1, 2, 0, 3, 0);
	AD9520_OutCtrl(9, 1, 2, 0, 3, 0);
	AD9520_OutCtrl(10, 1, 2, 0, 3, 0);
	AD9520_OutCtrl(11, 1, 2, 0, 3, 0);
	/* Current source digital lock detect pin control. */
	AD9520_EnDisCSDLDToOut(AD9520_CSDLD_OUT_EN_LSB, 0x00);
	AD9520_EnDisCSDLDToOut(AD9520_CSDLD_OUT_EN_MSB, 0x00);
	/* LVPECL channel dividers setting up. */
	AD9520_ChDividersCtrl(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
	AD9520_ChDividersCtrl(1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
	AD9520_ChDividersCtrl(2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
	AD9520_ChDividersCtrl(3, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
	/* Clock divider. */
	AD9520_VcoDivCtrl(4);
	/* Input clocks control. */
	AD9520_InClkCtrl(0, 1, 0, 0, 0);
	/* Power down and sync. */
	AD9520_PwrDownSyncCtrl(0, 0, 0, 0);
	/* IO update. */
	AD9520_IoUpdate(1);

	return ad9520.PartId;
}

/*
 * @brief Get chip's part ID.
 */
uint8_t AD9520_GetPartID(void) {

	ad9520.PartId = 0x00;

	ad9520.Data.ADDR = AD9520_PART_ID;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_READ;

	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 2);
	ad9520.delay_fp(1);
	ad9520.spi_rx_fp(&ad9520.PartId, 1);
	ad9520.delay_fp(1);
	return ad9520.PartId;
}

/*
 * @brief Set SPI mode to LSB first.
 *
 * @param SdoActive 	: 0 - SDO pin is high impedance (default), 1 - SDO pin is used for read.
 * @param MsbLsbCtrl 	: 0 - MSB first, the addressing decrements (default), 1 - LSB first, the addressing increments.
 * @param SoftReset 	: 1 - reset (self-clearing). If the EEPROM pin is high, soft reset loads the register values from
 * 			the EEPROM. If the EEPROM pin is low, soft reset loads the register values to the on-chip defaults.
 *
 */
void AD9520_SpiCtrl(uint8_t SdoActive, uint8_t MsbLsbCtrl, uint8_t SoftReset) {

	AD9520_SpiCtrl_t SpiCtrl;

	SpiCtrl.SDO_ACTIVE = SdoActive;
	SpiCtrl.MSB_LSB_CTRL = MsbLsbCtrl;
	SpiCtrl.SOFT_RESET = SoftReset;
	SpiCtrl.SOFT_RESET_M = SoftReset;
	SpiCtrl.MSB_LSB_CTRL_M = MsbLsbCtrl;
	SpiCtrl.SDO_ACTIVE_M = SdoActive;

	ad9520.Data.ADDR = AD9520_SERIAL_PORT_CFG;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = SpiCtrl.SpiCtrl;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief Read back control.
 *
 * @param ReadBackActive : 0 - reads back buffer registers (default), 1 - reads back active registers.
 */
void AD9520_ReadBackCtrl(uint8_t ReadBackActive) {

	ad9520.Data.ADDR = AD9520_READBACK_CTRL;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = ReadBackActive & 0x01;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief EEPROM customer version ID control.
 *
 * @param ReadWrite : may be AD9520_READ or AD9520_WRITE.
 * @param CustId : may be AD9520x_CUSTOMER_ID or any.
 */
uint16_t AD9520_EepCustIdRW(uint8_t ReadWrite, uint16_t CustId) {

	ad9520.Data.ADDR = AD9520_CUSTOMER_VERSION_ID_LSB;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_TWO_BYTES;
	if (ReadWrite) {
		ad9520.Data.READ_WRITE = AD9520_READ;
		ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 2);
		ad9520.delay_fp(1);
		ad9520.spi_rx_fp((uint8_t*)&ad9520.CustId, 2);
		ad9520.delay_fp(1);
	} else {
		ad9520.CustId = CustId;
		ad9520.Data.READ_WRITE = AD9520_WRITE;
		ad9520.Data.RxTxData[0] = CustId;
		ad9520.Data.RxTxData[1] = CustId >> 8;
		ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 4);
		ad9520.delay_fp(1);
	}
	return ad9520.CustId;
}

/*
 * @brief PFD charge pump configuration.
 *
 * @param PllPowerDown : 0 - normal operation, this mode must be selected to use the PLL. 1 - asynchronous power-down (default).
 * 						 2 - unused. 3 - synchronous power-down.
 * @param CpMode : 		 0 - high impedance state. 1 - forces source current (pump up).
 * 				   		 2 - forces sink current (pump down). 3 - normal operation (default).
 * @param CpCurrent : From 0 to 7 current is from 0.6ma to 4.8ma with CPset = 5.1kOhm.
 * @param PfdPolarity :  0 - positive (higher control voltage produces higher frequency) (default).
 * 				 		 1 - negative (higher control voltage produces lower frequency).
 */
void AD9520_PfdChargePumpCtrl(uint8_t PllPowerDown, uint8_t CpMode, uint8_t CpCurrent, uint8_t PfdPolarity) {

	AD9520_PfdChargePumpCfg_t PfdChargePump;

	PfdChargePump.PLL_POWER_DOWN = PllPowerDown;
	PfdChargePump.CP_MODE = CpMode;
	PfdChargePump.CP_CURRENT = CpCurrent;
	PfdChargePump.PFD_POLARITY = PfdPolarity;

	ad9520.Data.ADDR = AD9520_PFD_CHARGE_PUMP;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PfdChargePump.PfdChargePumpCfg;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief R counter value configuration.
 *
 * @param CntValue : it is 14-bit value of R counter (default: 0x0001).
 */
void AD9520_RCounterCfg(uint16_t CntVal) {

	ad9520.Data.ADDR = AD9520_R_COUNTER_LSB;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_TWO_BYTES;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = CntVal;
	ad9520.Data.RxTxData[1] = CntVal >> 8;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 4);
	ad9520.delay_fp(1);
}

/*
 * @brief A counter value configuration.
 *
 * @param CntValue : it is 6-bit value of A counter.
 */
void AD9520_ACounterCfg(uint8_t CntVal) {

	ad9520.Data.ADDR = AD9520_A_COUNTER;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = CntVal;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief B counter value configuration.
 *
 * @param CntValue : it is 13-bit value of B counter (default: 0x0003).
 */
void AD9520_BCounterCfg(uint16_t CntVal) {

	ad9520.Data.ADDR = AD9520_B_COUNTER_LSB;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_TWO_BYTES;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = CntVal;
	ad9520.Data.RxTxData[1] = CntVal >> 8;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 4);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 1 register.
 *
 * @param PrescP :  	Prescaler: DM = dual modulus; FD = fixed divide.
 * 						Values from 0 to 7,
 * 						divide by 1, FD mode,
 * 						divide by 2, FD mode,
 * 						divide by 2, and divide by 3 when A ≠ 0; divide by 2 when A = 0. DM mode,
 * 						divide by 4, and divide by 5 when A ≠ 0; divide by 4 when A = 0. DM mode,
 * 						divide by 8, and divide by 9 when A ≠ 0; divide by 8 when A = 0. DM mode,
 * 						divide by 16, and divide by 17 when A ≠ 0; divide by 16 when A = 0. DM mode,
 * 						divide by 32, and divide by 33 when A ≠ 0; divide by 32 when A = 0 (default). DM mode,
 * 						divide by 3, FD mode.
 * @param BCntBypass : 	0 - normal (default),
 * 					   	1 - B counter is set to divide-by-1. This allows the prescaler setting to determine the divide for the N divider.
 * @param RstAllCnt :   0 - normal (default),
 * 						1 - holds R, A, and B counters in reset.
 * @param RstABCnt :	0 - normal (default),
 * 						1 - holds A and B counters in reset.
 * @param RstRCnt :		0 - normal (default),
 * 						1 - holds R counter in reset.
 * @param SetCPPin :	0 - CP normal operation mode (default),
 * 						1 - CP pin set to VCP/2.
 */
void AD9520_PllCtrl1(uint8_t PrescP, uint8_t BCntBypass, uint8_t RstAllCnt, uint8_t RstABCnt, uint8_t RstRCnt, uint8_t SetCPPin) {

	AD9520_PllCtrl1_t PllCtrl1;

	PllCtrl1.PRESCALER_P = PrescP;
	PllCtrl1.B_CNT_BYPASS = BCntBypass;
	PllCtrl1.RST_ALL_CNT = RstAllCnt;
	PllCtrl1.RST_AB_CNT = RstABCnt;
	PllCtrl1.RST_R_CNT = RstRCnt;
	PllCtrl1.SET_CP_PIN = SetCPPin;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_1;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl1.PllCtrl1;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 2 register.
 *
 * @param AntibacklashPulseWidth : 	0 - 2.9nS (default),
 * 									1 - 1.3nS,
 * 									2 - 6.0nS,
 * 									3 - 2.9nS.
 * @param StatusPinCtrl : 	Selects the signal that appears at the STATUS pin. Register 0x01D[7] must be 0b to reprogram the STATUS pin.
 * 							Values from 0 to 6,
 * 							0 - Ground (dc) (default),
 * 							1 - N divider output (after the delay),
 * 							2 - R divider output (after the delay),
 * 							3 - A divider output,
 * 							4 - Prescaler output,
 * 							5 - PFD up pulse,
 * 							6 - PFD down pulse.
 *							Values from 32 to 63
 *							32 - Ground (dc),
 *							33 - REF1 clock (differential reference when in differential mode),
 *							34 - REF2 clock (N/A in differential mode),
 *							35 - Selected reference to PLL (differential reference when in differential mode),
 *							36 - Unselected reference to PLL (not available in differential mode),
 *							37 - Status of selected reference (status of differential reference); active high,
 *							38 - Status of unselected reference (not available in differential mode); active high,
 *							39 - Status of REF1 frequency; active high,
 *							40 - Status of REF2 frequency; active high,
 *							41 - (Status of REF1 frequency) AND (status of REF2 frequency),
 *							42 - (DLD) AND(status of selected reference) AND(status of VCO),
 *							43 - Status of VCO frequency; active high,
 *							44 - Selected reference (low = REF1, high = REF2),
 *							45 - DLD; active high,
 *							46 - Holdover active; active high,
 *							47 - N/A. Do not use,
 *							48 - VS (PLL power supply),
 *							49 - #REF1 clock (differential reference when in differential mode),
 *							50 - #REF2 clock (not available in differential mode),
 *							51 - #Selected reference to PLL (differential reference when in differential mode),
 *							52 - #Unselected reference to PLL (not available when in differential mode),
 *							53 - Status of selected reference (status of differential reference); active low,
 *							54 - Status of unselected reference (not available in differential mode); active low,
 *							55 - Status of REF1 frequency; active low,
 *							56 - Status of REF2 frequency; active low,
 *							57 - #(Status of REF1 frequency) AND (status of REF2 frequency),
 *							58 - #(DLD)AND (status of selected reference)AND (status of VCO),
 *							59 - Status of VCO frequency; active low,
 *							60 - Selected reference (low = REF2, high = REF1),
 *							61 - DLD; active low,
 *							62 - Holdover active; active low,
 *							63 - N/A. Do not use.
 */
void AD9520_PllCtrl2(uint8_t AntibacklashPulseWidth, uint8_t StatusPinCtrl) {

	AD9520_PllCtrl2_t PllCtrl2;

	PllCtrl2.ANTIBACKLASH_PULSE_WIDTH = AntibacklashPulseWidth;
	PllCtrl2.STATUS_PIN_CTRL = StatusPinCtrl;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_2;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl2.PllCtrl2;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 3 register.
 *
 * @param VcoCalibNow : 	Initiates VCO calibration. This bit must be toggled from 0b to 1b in the active registers. The sequence to initiate a calibration is
 * 							as follows: program to 0b, followed by an IO_UPDATE (Register 0x232[0]); then program to 1b, followed by another IO_UPDATE (Register 0x232[0]).
 * 					 		This sequence gives complete control over when the VCO calibration occurs relative to the programming of other registers that can impact the
 * 					 		calibration (default = 0b). Note that the VCO divider (Register 0x1E0[2:0]) must not be static during VCO calibration.
 * @param VcoCalibDiv : 	Divider used to generate the VCO calibration clock from the PLL reference clock (see the VCO Calibration section for the recommended setting
 * 							of the VCO calibration divider based on the PFD rate).
 * 							0 - 2. This setting is fine for PFD frequencies < 12.5 MHz. The PFD frequency is fREF/R,
 * 							1 - 4. This setting is fine for PFD frequencies < 12.5 MHz. The PFD frequency is fREF/R,
 * 							2 - 8. This setting is fine for PFD frequencies < 50 MHz,
 * 							3 - 16 (default). This setting is fine for any PFD frequency, but it also results in the longest VCO calibration time.
 * @param DisDLD :			Digital lock detect operation.
 * 							0 - normal lock detect operation (default),
 * 							1 - disables lock detect.
 * @param DisLDW :			If the time difference of the rising edges at the inputs to the PFD is less than the lock detect window time, the digital lock detect flag is set.
 * 							The flag remains set until the time difference is greater than the loss-of-lock threshold.
 * 							0 - high range (default). The default setting is 3.5 ns,
 * 							1 - low range
 * @param LockDetectCnt : 	Required consecutive number of PFD cycles with edges inside lock detect window before the DLD indicates a locked condition.
 * 							0 - 5 (default),
 * 							1 - 16,
 * 							2 - 64,
 * 							3 - 255.
 * @param CmosRefinOffset :	Enables dc offset in single-ended CMOS input mode to prevent chattering when ac-coupled and input is lost.
 * 							0 - disables dc offset (default),
 * 							1 - enables dc offset.
 */
void AD9520_PllCtrl3(uint8_t VcoCalibNow, uint8_t VcoCalibDiv, uint8_t  DisDLD, uint8_t  DisLDW, uint8_t LockDetectCnt, uint8_t CmosRefinOffset) {

	AD9520_PllCtrl3_t PllCtrl3;

	PllCtrl3.VCO_CALIB_NOW = VcoCalibNow;
	PllCtrl3.VCO_CALIB_DIV = VcoCalibDiv;
	PllCtrl3.DIS_DLD = DisDLD;
	PllCtrl3.DIS_LDW = DisLDW;
	PllCtrl3.LOCK_DETECT_CNT = LockDetectCnt;
	PllCtrl3.CMOS_REFIN_OFFSET = CmosRefinOffset;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_3;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl3.PllCtrl3;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 4 register.
 *
 * @param NPathDelay :
 * @param RPathDelay :
 * @param RABCntSyncPinRst :
 */
void AD9520_PllCtrl4(uint8_t NPathDelay, uint8_t RPathDelay, uint8_t RABCntSyncPinRst) {

	AD9520_PllCtrl4_t PllCtrl4;

	PllCtrl4.NPATH_DELAY = NPathDelay;
	PllCtrl4.RPATH_DELAY = RPathDelay;
	PllCtrl4.RAB_CNT_SYNCPIN_RST = RABCntSyncPinRst;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_4;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl4.PllCtrl4;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 5 register.
 *
 * @param LedPinCtrl : Selects the signal that is connected to the LD pin.
 * 								Values from 0 to 4,
 * 								0 - Digital lock detect (high = lock; low = unlock, default),
 * 								1 - P-channel, open-drain lock detect (analog lock detect),
 * 								2 - N-channel, open-drain lock detect (analog lock detect),
 * 								3 - Tristate (high-Z) LD pin,
 * 								4 - Current source lock detect (110 µA when DLD is true),
 *								Values from 32 to 63
 *								32 - Ground (dc),
 *								33 - REF1 clock (differential reference when in differential mode),
 *								34 - REF2 clock (N/A in differential mode),
 *								35 - Selected reference to PLL (differential reference when in differential mode),
 *								36 - Unselected reference to PLL (not available in differential mode),
 *								37 - Status of selected reference (status of differential reference); active high,
 *								38 - Status of unselected reference (not available in differential mode); active high,
 *								39 - Status of REF1 frequency; active high,
 *								40 - Status of REF2 frequency; active high,
 *								41 - (Status of REF1 frequency) AND (status of REF2 frequency),
 *								42 - (DLD) AND(status of selected reference) AND(status of VCO),
 *								43 - Status of VCO frequency; active high,
 *								44 - Selected reference (low = REF1, high = REF2),
 *								45 - DLD; active high,
 *								46 - Holdover active; active high,
 *								47 - N/A. Do not use,
 *								48 - VS (PLL power supply),
 *								49 - #REF1 clock (differential reference when in differential mode),
 *								50 - #REF2 clock (not available in differential mode),
 *								51 - #Selected reference to PLL (differential reference when in differential mode),
 *								52 - #Unselected reference to PLL (not available when in differential mode),
 *								53 - Status of selected reference (status of differential reference); active low,
 *								54 - Status of unselected reference (not available in differential mode); active low,
 *								55 - Status of REF1 frequency; active low,
 *								56 - Status of REF2 frequency; active low,
 *								57 - #(Status of REF1 frequency) AND (status of REF2 frequency),
 *								58 - #(DLD)AND (status of selected reference)AND (status of VCO),
 *								59 - Status of VCO frequency; active low,
 *								60 - Selected reference (low = REF2, high = REF1),
 *								61 - DLD; active low,
 *								62 - Holdover active; active low,
 *								63 - N/A. Do not use.
 * @param RefFreqMonThr : Sets the reference (REF1/REF2) frequency monitor’s detection threshold frequency. This does not affective VCO frequency monitor’s
 *								detection threshold (see Table 17: REF1, REF2, and VCO frequency status monitor parameter).
 *								0 - frequency valid if frequency is above 1.02 MHz (default),
 *								1 - frequency valid if frequency is above 6 kHz.
 * @param EnStatusMonPinDiv : Enables a divide-by-4 on the STATUS pin. This makes it easier to look at low duty-cycle signals out of the R and N dividers.
 * 								0 - divide-by-4 disabled on STATUS pin (default),
 * 								1 - divide-by-4 enabled on STATUS pin.
 */
void AD9520_PllCtrl5(uint8_t LedPinCtrl, uint8_t RefFreqMonThr, uint8_t EnStatusMonPinDiv) {

	AD9520_PllCtrl5_t PllCtrl5;

	PllCtrl5.LD_PIN_CTRL = LedPinCtrl;
	PllCtrl5.REF_FREQ_MON_THR = RefFreqMonThr;
	PllCtrl5.EN_STATUS_PIN_DIV = EnStatusMonPinDiv;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_5;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl5.PllCtrl5;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 6 register.
 *
 * @param RefMonPinCtrl : Selects the signal that is connected to the REFMON pin.
 *								Values from 0 to 31
 *								0 - Ground (dc),
 *								1 - REF1 clock (differential reference when in differential mode),
 *								2 - REF2 clock (N/A in differential mode),
 *								3 - Selected reference to PLL (differential reference when in differential mode),
 *								4 - Unselected reference to PLL (not available in differential mode),
 *								5 - Status of selected reference (status of differential reference); active high,
 *								6 - Status of unselected reference (not available in differential mode); active high,
 *								7 - Status of REF1 frequency; active high,
 *								8 - Status of REF2 frequency; active high,
 *								9 - (Status of REF1 frequency) AND (status of REF2 frequency),
 *								10 - (DLD) AND(status of selected reference) AND(status of VCO),
 *								11 - Status of VCO frequency; active high,
 *								12 - Selected reference (low = REF1, high = REF2),
 *								13 - DLD; active high,
 *								14 - Holdover active; active high,
 *								15 - N/A. Do not use,
 *								16 - VS (PLL power supply),
 *								17 - #REF1 clock (differential reference when in differential mode),
 *								18 - #REF2 clock (not available in differential mode),
 *								19 - #Selected reference to PLL (differential reference when in differential mode),
 *								20 - #Unselected reference to PLL (not available when in differential mode),
 *								21 - Status of selected reference (status of differential reference); active low,
 *								22 - Status of unselected reference (not available in differential mode); active low,
 *								23 - Status of REF1 frequency; active low,
 *								24 - Status of REF2 frequency; active low,
 *								25 - #(Status of REF1 frequency) AND (status of REF2 frequency),
 *								26 - #(DLD)AND (status of selected reference)AND (status of VCO),
 *								27 - Status of VCO frequency; active low,
 *								28 - Selected reference (low = REF2, high = REF1),
 *								29 - DLD; active low,
 *								30 - Holdover active; active low,
 *								31 - N/A. Do not use.
 * @param EnRef1FreqMon : REF1 (REFIN) frequency monitor enabled; this is for both REF1 (single-ended) and REFIN (differential) inputs (as selected by
 *								differential reference mode).
 *								0 -  disables the REF1 (REFIN) frequency monitor (default),
 *								1 - enables the REF1 (REFIN) frequency monitor.
 * @param EnRef2FreqMon : Enables or disables the REF2 frequency monitor.
 * 								0 - disables the REF2 frequency monitor (default),
 * 								1 - enables the REF2 frequency monitor.
 * @param EnVcoFreqMon :  Enables or disables the VCO frequency monitor.
 * 								0 - disables the VCO frequency monitor (default),
 * 								1 - enables the VCO frequency monitor.
 */
void AD9520_PllCtrl6(uint8_t RefMonPinCtrl, uint8_t EnRef1FreqMon, uint8_t EnRef2FreqMon, uint8_t EnVcoFreqMon) {

	AD9520_PllCtrl6_t PllCtrl6;

	PllCtrl6.REFMON_PIN_CTRL = RefMonPinCtrl;
	PllCtrl6.EN_REF1_FREQ_MON = EnRef1FreqMon;
	PllCtrl6.EN_REF2_FREQ_MON = EnRef2FreqMon;
	PllCtrl6.EN_VCO_FREQ_MON = EnVcoFreqMon;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_6;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl6.PllCtrl6;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 7 register.
 *
 * @param EnDiffRef : 		Selects the PLL reference mode, differential or single-ended. Register 0x01C[2:1] should be cleared when this bit is set.
 * 								0 - single-ended reference mode (default),
 * 								1 - differential reference mode.
 * @param EnRef1 : 			This bit turns the REF1 power on. This bit is overridden when automatic reference switchover is enabled.
 * 								0 - REF1 power off (default),
 * 								1 - REF1 power on.
 * @param EnRef2 : 			This bit turns the REF2 power on. This bit is overridden when automatic reference switchover is enabled.
 * 								0 - REF2 power off (default),
 * 								1 - REF2 power on.
 * @param StayOnRef2 : 		Stays on REF2 after switchover.
 * 								0 - returns to REF1 automatically when REF1 status is good again (default),
 * 								1 - stays on REF2 after switchover. Does not automatically return to REF1.
 * @param EnAutoRefSw : 	Automatic or manual reference switchover. Single-ended reference mode must be selected by Register 0x01C[0] = 0b.
 * 								0 - manual reference switchover (default),
 * 								1 - automatic reference switchover. Setting this bit also powers on REF1 and REF2 and overrides the settings in Register 0x01C[2:1].
 * @param UseRefSelPin : 	If Register 0x01C[4] = 0b (manual), sets the method of PLL reference selection.
 * 								0 - uses Register 0x01C[6] (default),
 * 								1 - uses REF_SEL pin.
 * @param SelRef2 : 		If Register 0x01C[5] = 0b, selects the reference for PLL when in manual; register selected reference control.
 * 								0 - selects REF1 (default),
 * 								1 - selects REF2.
 * @param DisSwDeglitch :	 Disables or enables the switchover deglitch circuit.
 * 								0 - enables the switchover deglitch circuit (default),
 * 								1 - disables the switchover deglitch circuit.
 */
void AD9520_PllCtrl7(uint8_t EnDiffRef, uint8_t EnRef1, uint8_t  EnRef2, uint8_t StayOnRef2, uint8_t EnAutoRefSw, uint8_t UseRefSelPin, uint8_t SelRef2, uint8_t DisSwDeglitch) {

	AD9520_PllCtrl7_t PllCtrl7;

	PllCtrl7.EN_DIFF_REF = EnDiffRef;
	PllCtrl7.EN_REF1 = EnRef1;
	PllCtrl7.EN_REF2 = EnRef2;
	PllCtrl7.STAY_ON_REF2 = StayOnRef2;
	PllCtrl7.EN_AUTO_REF_SW = EnAutoRefSw;
	PllCtrl7.USE_REF_SEL_PIN = UseRefSelPin;
	PllCtrl7.SEL_REF2 = SelRef2;
	PllCtrl7.DIS_SW_DEGLITCH = DisSwDeglitch;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_7;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl7.PllCtrl7;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 8 register.
 *
 * @param EnHoldover : 			Enables the internally controlled holdover function.
 * 									0 - holdover disabled (default),
 * 									1 - holdover enabled.
 * @param EnExtHoldover : 		Enables the external hold control through the SYNC pin. (This disables the internal holdover mode).
 * 									0 - automatic holdover mode, holdover controlled by automatic holdover circuit (default),
 * 									1 - external holdover mode, holdover controlled by SYNC pin.
 * @param EnLdPinCmp : 			Enables the LD pin voltage comparator. Used with the LD pin current source lock detect mode. When the AD9520-3 is in internal
 * 						 		(automatic) holdover mode, this bit enables the use of the voltage on the LD pin to determine if the PLL was previously in
 * 						 		a locked state (see Figure 47). Otherwise, this setting can be used with the REFMON and STATUS pins to monitor the voltage
 * 						 		on the LD pin.
 * 									0 - disables LD pin comparator and ignore the LD pin voltage; internal/automatic holdover controller treats the LD pin as true
 * 									(high, default),
 * 									1 - enables LD pin comparator (use LD pin voltage to determine if the PLL was previously locked).
 * @param DisPllStatus : 		Disables the PLL status register readback.
 * 									0 - PLL status register enabled (default),
 * 									1 - PLL status register disabled. If this bit is set, Register 01F is not automatically updated.
 * @param EnClkDoubler : 		Enables PLL reference input clock doubler.
 * 									0 - doubler disabled (default),
 * 									1 - doubler enabled.
 * @param EnXtalOsc : 			Enables the maintaining amplifier needed by a crystal oscillator at the PLL reference input.
 * 									0 - crystal oscillator maintaining amplifier disabled (default),
 * 									1 - crystal oscillator maintaining amplifier enabled.
 * @param EnStatEepAtStatPin : 	Enables the Status_EEPROM signal at the STATUS pin.
 * 									0 - the STATUS pin is controlled by Register 0x017[7:2] selection,
 * 									1 - select STATUS_EEPROM signal at STATUS pin. This bit overrides Register 0x017[7:2] (default).
 */
void AD9520_PllCtrl8(uint8_t EnHoldover, uint8_t EnExtHoldover, uint8_t EnLdPinCmp, uint8_t DisPllStatus, uint8_t EnClkDoubler, uint8_t EnXtalOsc, uint8_t EnStatEepAtStatPin) {

	AD9520_PllCtrl8_t PllCtrl8;

	PllCtrl8.EN_HOLDOVER = EnHoldover;
	PllCtrl8.EN_EXT_HOLDOVER = EnExtHoldover;
	PllCtrl8.EN_LD_PIN_CMP = EnLdPinCmp;
	PllCtrl8.DIS_PLL_STAT_REG = DisPllStatus;
	PllCtrl8.EN_CLK_DOUBLER = EnClkDoubler;
	PllCtrl8.EN_XTAL_OSC = EnXtalOsc;
	PllCtrl8.EN_STEEP_ON_STPIN = EnStatEepAtStatPin;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_8;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl8.PllCtrl8;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL control 9 register.
 *
 * @param EnZeroDelay : 		 Enables zero delay function.
 * 									0 - disables zero delay function (default),
 * 									1 - enables zero delay function.
 * @param EnExtZeroDelay : 		 Selects which zero delay mode to use.
 * 									0 - enables internal zero delay mode if Register 0x01E[1] = 1 (default),
 * 									1 - enables external zero delay mode if Register 0x01E[1] = 1.
 * @param ExtZeroDelayChDivSel : Selection of Channel Divider for Use in the External Zero-Delay Path.
 * 									0 - selects Channel Divider 0 (default),
 * 									1 - selects Channel Divider 1,
 * 									2 - selects Channel Divider 2,
 * 									3 - selects Channel Divider 3.
 */
void AD9520_PllCtrl9(uint8_t EnZeroDelay, uint8_t EnExtZeroDelay, uint8_t ExtZeroDelayChDivSel) {

	AD9520_PllCtrl9_t PllCtrl9;

	PllCtrl9.EN_ZDELAY = EnZeroDelay;
	PllCtrl9.EN_EXT_ZDELAY = EnExtZeroDelay;
	PllCtrl9.EXT_ZDELAY_CHDIV = ExtZeroDelayChDivSel;

	ad9520.Data.ADDR = AD9520_PLL_CTRL_9;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PllCtrl9.PllCtrl9;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief PLL readback control register.
 *
 * @ret bit 0 : Digital lock detect.
 * 					0 - PLL is not locked,
 * 					1 - PLL is locked.
 * @ret bit 1 : Indicates if the frequency of the signal at REF1 is greater than the threshold frequency set by Register 0x01A[6].
 * 					0 - REF1 frequency is less than the threshold frequency,
 * 					1 - REF1 frequency is greater than the threshold frequency.
 * @ret bit 2 : Indicates if the frequency of the signal at REF2 is greater than the threshold frequency set by Register 0x01A[6].
 * 					0 - REF2 frequency is less than the threshold frequency,
 * 					1 - REF2 frequency is less than the threshold frequency.
 * @ret bit 3 : Indicates if the VCO frequency is greater than the threshold (see Table 17: REF1, REF2, and VCO frequency status monitor parameter).
 * 					0 - VCO frequency is less than the threshold,
 * 					1 - VCO frequency is greater than the threshold.
 * @ret bit 4 : Indicates which PLL reference is selected as the input to the PLL.
 * 					0 - REF1 selected (or differential reference if in differential mode),
 * 					1 - REF2 selected.
 * @ret bit 5 : Indicates if the part is in the holdover state (see Figure 47). Note that this is not the same as holdover enabled.
 * 					0 - not in holdover state,
 * 					1 - holdover state active.
 * @ret bit 6 : Indicates the status of the VCO calibration.
 * 					0 - VCO calibration not finished,
 * 					1 - VCO calibration finished.
 * @ret bit 7 : Unused.
 */
AD9520_PllReadBack_t* AD9520_PllReadbackCtrl(void) {

	static AD9520_PllReadBack_t PllReadbackCtrl;

	ad9520.Data.ADDR = AD9520_PLL_READBACK;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_READ;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 2);
	ad9520.delay_fp(1);
	ad9520.spi_rx_fp(&PllReadbackCtrl.PllReadBack, 1);
	ad9520.delay_fp(1);
	return &PllReadbackCtrl;
}

/*
 * @brief Output control.
 *
 * @param OutNumber : 			Number of controlled channel. Values from 0 to 11.
 * @param OutPowerDownLVPECL : 	LVPECL power-down.
 * 									0 - normal operation (default),
 * 									1 - safe power-down.
 * @param OutDiffVoltageLVPECL : Sets the LVPECL output differential voltage (VOD).
 * 									0 - 400mV,
 * 									1 - 600mV,
 * 									2 - 780mV (default),
 * 									3 - 960mV.
 * @param OutPolarity : 		Sets the output polarity for output X.
 * 									0 - LVPECL, OUTA noninverting, OUTB inverting (default),
 * 									1 - LVPECL, OUTA inverting, OUTB noninverting,
 * 									0 - CMOS, OUTA noninverting, OUTB inverting,
 * 									1 - CMOS, OUTA inverting, OUTB inverting,
 * 									2 - CMOS, OUTA noninverting, OUTB noninverting,
 * 									3 - CMOS, OUTA inverting, OUTB noninverting.
 * @param OutCfgCMOS : 			Sets the CMOS output configuration for OUT0 when Register 0x0F0[7] = 1b.
 * 									0 - OUTA tristate, OUTB tristate,
 * 									1 - OUTA on, OUTB tristate,
 * 									2 - OUTA tristate, OUTB on,
 * 									3 - OUTA on, OUTB on (default).
 * @param OutFormat : 			Selects the output type for OUT0.
 * 									0 - LVPECL (default),
 * 									1 - CMOS.
 */
void AD9520_OutCtrl(uint8_t OutNumber, uint8_t OutPowerDownLVPECL, uint8_t OutDiffVoltageLVPECL, uint8_t OutPolarity, uint8_t OutCfgCMOS, uint8_t OutFormat) {

	AD9520_OutCtrl_t OutControl;

	if (OutNumber > 11) { return; }

	OutControl.LVPECL_POWER_DOWN = OutPowerDownLVPECL;
	OutControl.LVPECL_DIFF_VOLTAGE = OutDiffVoltageLVPECL;
	OutControl.POLARITY = OutPolarity;
	OutControl.CMOS_CFG = OutCfgCMOS;
	OutControl.OUT_FORMAT = OutFormat;

	ad9520.Data.ADDR = AD9520_OUT0_CTRL + OutNumber;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = OutControl.OutCtrl;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief Current source digital lock detect pin control.
 *
 * @param RegNumber : Register of controlled channels. May be AD9520_CSDLD_OUT_EN_LSB or AD9520_CSDLD_OUT_EN_MSB.
 * @param State 	: bits from 0 to 7 for LSB register (OUT0 to OUT7) and 0 to 3 for MSB register (OUT8 to OUT11).
 * 						BitX	CSDLD signal	OUTx enable status
 * 					 	 0 		  0					 not affected by CSDLD signal (default),
 * 					 	 1 		  0 				 asynchronous power-down,
 *					 	 1 		  1 				 asynchronously enables OUT7 if not powered down by other settings. For this feature, use current source digital lock detect and set the
 *												 	 enable LD pin comparator bit (Register AD9520_PLL_CTRL_8[EN_LD_PIN_CMP])
 */
void AD9520_EnDisCSDLDToOut(uint8_t RegNumber, uint8_t State) {

	if (RegNumber > 11) { return; }
	if (RegNumber != AD9520_CSDLD_OUT_EN_LSB || RegNumber != AD9520_CSDLD_OUT_EN_MSB) { return; }

	ad9520.Data.ADDR = RegNumber;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = State;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief LVPECL channel dividers.
 *
 * @param DivNumber 	: Number of divider (from 0 to 3).
 * 						  Divider 0 controls OUT0, OUT1, OUT2.
 * 						  Divider 1 controls OUT3, OUT4, OUT5.
 * 						  Divider 2 controls OUT6, OUT7, OUT8.
 * 						  Divider 3 controls OUT9, OUT10, OUT11.
 * @param DivHighCycles : Number of clock cycles (minus 1) of the divider input during which the divider output stays low (from 0 to 15).
 * 						  A value of 0x7 means that the divider is low for eight input clock cycles (default: 0x7).
 * @param DivLowCycles 	: Number of clock cycles (minus 1) of the divider input during which the divider output stays low (from 0 to 15).
 * 						  A value of 0x7 means that the divider is low for eight input clock cycles (default: 0x7).
 * @param PhaseOffset 	: Phase offset (from 0 to 15, default: 0x0).
 * @param ClkOutStart 	: Selects clock output to start high or start low.
 * 							0 - starts low (default),
 * 							1 - starts high.
 * @param DivOutForce 	: Forces divider output to a specific state. This requires that ignore SYNC also be set. Note that this bit has no
 * 						  effect if the channel divider is bypassed, but the driver polarity can still be reversed.
 * 							0 - divider output is forced to low (default),
 * 							1 - divider output is forced to the setting stored in Bit 4 of this register.
 * @param IgnoreSync 	: Ignores SYNC.
 * 							0 - obeys chip-level SYNC signal (default),
 * 							1 - ignores chip-level SYNC signal.
 * @param DivBypass 	: Bypasses and powers down the divider; routes input to divider output.
 * 							0 - uses divider (default),
 * 							1 - bypasses divider.
 * @param DivDccEnDis 	: Duty-cycle correction function.
 * 							0 - enables duty-cycle correction (default),
 * 							1 - disables duty-cycle correction.
 * @param ChXDirect 	: Connects three OUTx to Divider X or directly to VCO or CLK.
 * 							0 - OUTx are connected to Divider X (default).,
 * 							1 - if Register AD9520_INPUT_CLOCKS[1:0] = 10b, the VCO is routed directly to OUTx. If Register AD9520_INPUT_CLOCKS[1:0] = 00b,
 * 								the CLK is routed directly to OUTx. If Register AD9520_INPUT_CLOCKS[1:0] = 01b, there is no effect.
 * @param ChXPwrDown 	: Channel x powers down.
 * 							0 - normal operation (default),
 * 							1 - powered down. (Setting this bit puts three pairs of OUTx/#OUTx into safe power-down mode.)
 */
void AD9520_ChDividersCtrl(uint8_t DivNumber, uint8_t DivHighCycles, uint8_t DivLowCycles, uint8_t PhaseOffset, uint8_t ClkOutStart,
							uint8_t DivOutForce, uint8_t IgnoreSync, uint8_t DivBypass, uint8_t DivDccEnDis, uint8_t ChXDirect,  uint8_t ChXPwrDown) {

	AD9520_DivCtrl_t DivControl;

	if (DivNumber > 3) { return; }

	DivControl.DIV_HIGH_CYCLES = DivHighCycles;
	DivControl.DIV_LOW_CYCLES = DivLowCycles;
	ad9520.Data.RxTxData[0] = DivControl.DivLowHighCycles;


	DivControl.PHASE_OFFSET = PhaseOffset;
	DivControl.DIV_CLK_START = ClkOutStart;
	DivControl.DIV_FORCE = DivOutForce;
	DivControl.DIV_CHIP_LEVEL_SYNC = IgnoreSync;
	DivControl.DIV_BYPASS = DivBypass;
	ad9520.Data.RxTxData[1] = DivControl.DivCtrl;

	DivControl.DIV_DCC_EN_DIS = DivDccEnDis;
	DivControl.CH_DIRECT_TO_OUT = ChXDirect;
	DivControl.CH_PWR_DOWN = ChXPwrDown;
	ad9520.Data.RxTxData[2] = DivControl.ChannelGroupCtrl;

	ad9520.Data.ADDR = AD9520_DIVIDER0_BYTE0 + (DivNumber * 3);
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_THREE_BYTES;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 5);
	ad9520.delay_fp(5);
}

/*
 * @brief VCO divider control.
 *
 * @param OutPolarity : VCO divider coefficient.
 * 							0 - divide 2 (default),
 * 							1 - divide 3,
 * 							2 - divide 4,
 * 							3 - divide 5,
 * 							4 - divide 6,
 * 							5 - output static.
 * 							6 - divide 1 (bypass),
 * 							7 - output static.
 */
void AD9520_VcoDivCtrl(uint8_t VcoDiv) {

	ad9520.Data.ADDR = AD9520_VCO_DIVIDER;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = VcoDiv;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief Input clocks control.
 *
 * @param VcoBypass 				: Bypasses or uses the VCO divider.
 * 										0 - uses VCO divider (default),
 * 										1 - bypasses VCO divider; VCO cannot be selected as input when this bit is set.
 * @param VcoOrClkAsIn 				: Selects either the VCO or the CLK as the input to VCO divider.
 * 										0 - selects external CLK as input to VCO divider (default),
 * 										1 - selects VCO as input to VCO divider; VCO divider cannot be bypassed when this bit is set. This bit must
 * 											be set to use the PLL with the internal VCO.
 * @param PwrDownVcoAndClk 			: Powers down both the VCO and the CLK input.
 * 										0 - normal operation (default),
 * 										1 - power-down.
 * @param PwrDownVcoClkInterface 	: Powers down the interface block between VCO and clock distribution.
 * 										0 - normal operation (default),
 * 										1 - power-down.
 * @param PwrDownClkInSection 		: Powers down the clock input section (including CLK buffer, VCO divider, and CLK tree).
 * 										0 - normal operation (default),
 * 										1 - power-down.
 */
void AD9520_InClkCtrl(uint8_t VcoBypass, uint8_t VcoOrClkAsIn, uint8_t PwrDownVcoAndClk, uint8_t PwrDownVcoClkInterface, uint8_t PwrDownClkInSection) {

	AD9520_InClkCtrl_t InClkControl;

	InClkControl.BYPASS_VCO_DIV = VcoBypass;
	InClkControl.SEL_VCO_OR_CLK = VcoOrClkAsIn;
	InClkControl.PWRDOWN_VCO_AND_CLK = PwrDownVcoAndClk;
	InClkControl.PWRDOWN_VCO_INTERFACE = PwrDownVcoClkInterface;
	InClkControl.PWRDOWN_CLK_IN_SECTION = PwrDownClkInSection;

	ad9520.Data.ADDR = AD9520_INPUT_CLOCKS;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = InClkControl.InClkCtrl;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief Power down and sync control.
 *
 * @param SoftSync 			: The soft SYNC bit works in the same way as the SYNC pin, except that the polarity of the bit is reversed. That is,
 * 							  a high level forces selected channels into a predetermined static state, and a 1bto-0b transition triggers a SYNC.
 * 								0 - same as SYNC pin high,
 * 								1 - same as SYNC pin low.
 * @param PwrDownDistrRef 	: Powers down the reference for the distribution section.
 * 								0 - normal operation of the reference for the distribution section (default),
 * 								1 - powers down the reference for the distribution section.
 * @param PwrDownSync 		: Powers down the SYNC function.
 * 								0 - normal operation of the SYNC function (default),
 * 								1 - powers down SYNC circuitry.
 * @param DisPowerOnSync 	: Powers on SYNC mode. Used to disable the antiruntpulse circuitry.
 * 								0 - enables the antiruntpulse circuitry (default),
 * 								1 - disables the antiruntpulse circuitry.
 */
void AD9520_PwrDownSyncCtrl(uint8_t SoftSync, uint8_t PwrDownDistrRef, uint8_t PwrDownSync, uint8_t DisPowerOnSync) {

	AD9520_PwrDownAndSyncCtrl_t PwrDownAndSyncCtrl;

	PwrDownAndSyncCtrl.SOFT_SYNC = SoftSync;
	PwrDownAndSyncCtrl.PWRDOWN_DISTRIB_REF = PwrDownDistrRef;
	PwrDownAndSyncCtrl.PWRDOWN_SYNC = PwrDownSync;
	PwrDownAndSyncCtrl.DIS_PWRON_SYNC = DisPowerOnSync;

	ad9520.Data.ADDR = AD9520_POWER_DOWN_SYNC;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = PwrDownAndSyncCtrl.PwrDownAndSyncCtrl;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief IO update.
 *
 * @param SoftSync : This bit must be set to 1b to transfer the contents of the buffer registers into the active registers. This transfer occurs
 * 					 on the next SCLK rising edge. This bit is self-clearing; that is, it does not have to be set back to 0b.
 * 						1 -  (self-clearing): updates all active registers to the contents of the buffer registers.
 */
void AD9520_IoUpdate(uint8_t IoUpdate) {

	ad9520.Data.ADDR = AD9520_IO_UPDATE;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = IoUpdate;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief EEPROM control 1.
 *
 * @param EnEepromWrite : Enables the user to write to the EEPROM.
 * 							0 - EEPROM write protection is enabled. User cannot write to the EEPROM (default),
 * 							1 - EEPROM write protection is disabled. User can write to the EEPROM. Once an EEPROM save/load transfer is complete,
 * 								the user must wait a minimum of 10µs before starting the next EEPROM save/load transfer.
 * @param SoftEeprom 	: When the EEPROM pin is tied low, setting SOFT_EEPROM resets the AD9520-1 using the settings saved in the EEPROM.
 * 							1 - soft reset with EEPROM settings (self-clearing).
 */
void AD9520_EepromCtrl(uint8_t EnEepromWrite, uint8_t SoftEeprom) {

	AD9520_EepromCtrl_t EepromCtrl;

	EepromCtrl.EN_EEPROM_WRITE = EnEepromWrite;
	EepromCtrl.SOFT_EEPROM = SoftEeprom;

	ad9520.Data.ADDR = AD9520_EEP_CTRL1;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = EepromCtrl.EepromCtrl1;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief EEPROM transfer activation.
 *
 * @param RegToEeprom : Transfers data from the buffer register to the EEPROM (self-clearing).
 * 						1 - setting this bit initiates the data transfer from the buffer register to the EEPROM (writing process); it is reset
 * 							by the I²C master after the data transfer is complete. Once an EEPROM save/load transfer is complete, the user must
 * 							wait a minimum of 10µs before starting the next EEPROM save/load transfer.
 */
void AD9520_EepromWrite(uint8_t RegToEeprom) {

	ad9520.Data.ADDR = AD9520_EEP_CTRL2;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_WRITE;
	ad9520.Data.RxTxData[0] = RegToEeprom;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 3);
	ad9520.delay_fp(1);
}

/*
 * @brief Read EEPROM status.
 *
 * @ret bit 0 : This read-only register indicates the status of the data transfer between the EEPROM and the buffer register bank during the writing
 * 				and reading of the EEPROM. This signal is also available at the STATUS pin when Register 0x01D[7] is set
 * 					0 - data transfer is complete,
 * 					1 - data transfer is not complete.
 */
uint8_t AD9520_EepromStatus(void) {

	static uint8_t Status;

	ad9520.Data.ADDR = AD9520_EEP_STATUS;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_READ;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 2);
	ad9520.delay_fp(1);
	ad9520.spi_rx_fp(&Status, 1);
	ad9520.delay_fp(1);

	return (Status & 0x01);
}

/*
 * @brief Read EEPROM error.
 *
 * @ret bit 0 : This read-only register indicates an error during the data transfer between the EEPROM and the buffer.
 * 					0 - no error. Data is correct,
 * 					1 - incorrect data detected.
 */
uint8_t AD9520_EepromError(void) {

	static uint8_t EepromErr;

	ad9520.Data.ADDR = AD9520_EEP_ERR_CHECKING;
	ad9520.Data.BYTE_RXTX_COUNT = AD9520_ONE_BYTE;
	ad9520.Data.READ_WRITE = AD9520_READ;
	ad9520.spi_tx_fp((uint8_t*)&ad9520.Data.InstrHeader, 2);
	ad9520.delay_fp(1);
	ad9520.spi_rx_fp(&EepromErr, 1);
	ad9520.delay_fp(1);
	return (EepromErr & 0x01);
}

/* Hardware dependent functions. */

/*
 * @brief Receive data from the chip.
 */
void Ad9520_SpiRxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(SPI6SNSS_GPIO_Port, SPI6SNSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(AD9520Spi, pData, Size, 25);
	HAL_GPIO_WritePin(SPI6SNSS_GPIO_Port, SPI6SNSS_Pin, GPIO_PIN_SET);
}

/*
 * @brief Transmit data to the chip.
 */
void Ad9520_SpiTxData(uint8_t *pData, uint8_t Size) {

	HAL_GPIO_WritePin(SPI6SNSS_GPIO_Port, SPI6SNSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(AD9520Spi, pData, Size, 25);
	HAL_GPIO_WritePin(SPI6SNSS_GPIO_Port, SPI6SNSS_Pin, GPIO_PIN_SET);
}























