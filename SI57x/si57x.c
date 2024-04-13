/*
 * si57x.c source file.
 *
 * Created on: Aug 28, 2023
 * Author: asw3005
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "si57x.h"

/* External variables. */
extern I2C_HandleTypeDef hi2c2;

/* Private variables. */
static I2C_HandleTypeDef* SI57xI2C = &hi2c2;

/* Private function prototypes. */
static void SI57x_ReadDivRef(uint8_t Address);
static void SI57x_WriteDivRef(uint8_t Address);
static SI57x_RawRegData_t* SI57x_RawRegData(void);
static SI57x_FreqDivTable_t* SI57x_DevRefData(void);

static void SI57x_OEPin(uint8_t OEEnDis);
static void SI57x_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);
static void SI57x_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size);

/* Init general struct. */
static SI57x_GStr_t si57x = {
		.delay_fp = HAL_Delay,
		.i2c_rx_fp = SI57x_I2CRxData,
		.i2c_tx_fp = SI57x_I2CTxData
};


/*
 * @brief
 */
void SI57x_Init(void) {

	SI57x_OEPin(1);
	HAL_Delay(50);
	//SI57x_RstFreezeMemCtrl(1, 0, 0, 0, 0);
	//SI57x_ReadDivRef(SI57x_HIGH_SPEED_N1_DIV);
	SI57x_SelPresetFreq(SI57x_HIGH_SPEED_N1_DIV7PPM, 4);
	//SI57x_SetFreq(100, SI570_7PPM);
}

/*
 * @brief Set frequency in Hertz.
 *
 * @param NewFreq 		: Desired frequency in MHz.
 * @param DevPpmCoeff 	: Temperature stability of your devices. May be SI570_7PPM, SI570_20PPM, SI570_50PPM, SI571_ALL.
 */
void SI57x_SetFreq(float NewFreq, uint8_t DevPpmCoeff) {

	float FDCO, FreqDifference, PpmCoeff;
	uint8_t CoeffSel;
	uint16_t TryCounter = TRY_CNT_MAX_VALUE;

	uint64_t rawdata[12] = {0};
	float CurrentFreq;
	uint8_t addr = 0;

	/* Reading the current configuration. */
	if (DevPpmCoeff == SI570_20PPM || DevPpmCoeff == SI570_50PPM || DevPpmCoeff == SI571_ALL) {
		CoeffSel = SI57x_HIGH_SPEED_N1_DIV;
	} else if (DevPpmCoeff == SI570_7PPM) {
		CoeffSel = SI57x_HIGH_SPEED_N1_DIV7PPM;
	}
	/* Reset internal registers. */
	SI57x_RstFreezeMemCtrl(0, 0, 0, 0, 1);
	SI57x_RstFreezeMemCtrl(1, 0, 0, 0, 0);
	HAL_Delay(50);
	/* Read and build the dividers and frequency data. */
	SI57x_ReadDivRef(CoeffSel);



	rawdata[0] = SI57x_RawRegData()->HighSpeedN1Div.HS_DIV;
	rawdata[1] = SI57x_RawRegData()->RefFreqN1Div.N1_DIV_BIT1_0;
	rawdata[2] = SI57x_RawRegData()->HighSpeedN1Div.N1_DIV_BIT6_2;
	rawdata[3] = SI57x_RawRegData()->RefFreqN1Div.REF_FREQ_BIT37_32;
	rawdata[4] = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit31_24;
	rawdata[5] = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit23_16;
	rawdata[6] =  SI57x_RawRegData()->RefFreqN1Div.RefFreqBit15_8;
	rawdata[7] = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit7_0;

	rawdata[8] = SI57x_DevRefData()->HighSpeedDivider;
	rawdata[9] = SI57x_DevRefData()->N1DivRef;
	rawdata[10] = SI57x_DevRefData()->RefFrequency;

	/* Calculate the current frequency. */
	SI57x_DevRefData()->CurrentFreq = (INTERNAL_CRYSTAL_FREQ * (SI57x_DevRefData()->RefFrequency / FREQ_FRACT_MULTIPLIER)) /
										(SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef);

	CurrentFreq = SI57x_DevRefData()->CurrentFreq;
	FDCO = CurrentFreq * SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef;
	/* Check is there a small difference between the frequencies. */
	FreqDifference = (NewFreq - SI57x_DevRefData()->CurrentFreq) * FREQ_COEFF_MHZ_TO_HZ;
	if(FreqDifference < 0) { FreqDifference = -FreqDifference; }
	PpmCoeff = SI57x_DevRefData()->CurrentFreq /* / 1000000 */;
	/* Check the frequency window. */
	if ((FreqDifference / PpmCoeff) <= FREQ_WINDOW_IN_PPM) {
		/* Calculate the new value of frequency if output changes are smaller than +-3500 ppm. */
		SI57x_DevRefData()->RefFrequency = SI57x_DevRefData()->RefFrequency * (NewFreq / SI57x_DevRefData()->CurrentFreq);
		/* Write registers. */
		//SI57x_WriteDivRef(CoeffSel);
	} else {
		/* Looking for the appropriate frequency of DCO. */
		FDCO = NewFreq * SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef;
		//rawdata[9] = SI57x_DevRefData()->N1DivRef;
		while (FDCO < INTERNAL_MIN_OSC_FREQ || FDCO > INTERNAL_MAX_OSC_FREQ) {

			/* N1 divider start value check and tuning. */
			if ((SI57x_DevRefData()->N1DivRef == 1) || (SI57x_DevRefData()->N1DivRef % 2)) {
				//SI57x_DevRefData()->N1Divider += 1;
				SI57x_DevRefData()->N1DivRef += 1;
			} else {
				if (FDCO > INTERNAL_MAX_OSC_FREQ) {
					//SI57x_DevRefData()->N1Divider -= 2;
					SI57x_DevRefData()->N1DivRef -=2;
					}
				else {
					//SI57x_DevRefData()->N1Divider += 2;
					SI57x_DevRefData()->N1DivRef +=2;
					}
			}

			/* N1 divider boundaries check. HS divider tuning. */
			if (SI57x_DevRefData()->N1DivRef > N1_DIV_MAX_VALUE) {
				SI57x_DevRefData()->N1DivRef = N1_DIV_MIN_VALUE;
				SI57x_DevRefData()->HighSpeedDivider -= 1;
			}
			if (SI57x_DevRefData()->N1DivRef < N1_DIV_MIN_VALUE) {
				SI57x_DevRefData()->N1DivRef = N1_DIV_MIN_VALUE;
				SI57x_DevRefData()->HighSpeedDivider += 1;
			}
			SI57x_DevRefData()->N1Divider = SI57x_DevRefData()->N1DivRef - 1;

			/* HS divider some values check. */
			if (SI57x_DevRefData()->HighSpeedDivider == 8) {
				if (FDCO > INTERNAL_MAX_OSC_FREQ) {
					SI57x_DevRefData()->HighSpeedDivider = 9;
				} else SI57x_DevRefData()->HighSpeedDivider = 7;
			}
			if (SI57x_DevRefData()->HighSpeedDivider == 10) {
				if (FDCO > INTERNAL_MAX_OSC_FREQ) {
					SI57x_DevRefData()->HighSpeedDivider = 11;
				} else SI57x_DevRefData()->HighSpeedDivider = 9;
			}
			/* HS divider boundaries check. */
			if (SI57x_DevRefData()->HighSpeedDivider < HS_DIV_MIN_VALUE) {
				SI57x_DevRefData()->HighSpeedDivider = HS_DIV_MIN_VALUE;
			}
			if (SI57x_DevRefData()->HighSpeedDivider > HS_DIV_MAX_VALUE) {
				SI57x_DevRefData()->HighSpeedDivider = HS_DIV_MAX_VALUE;
			}
			rawdata[8] = SI57x_DevRefData()->HighSpeedDivider;
			rawdata[9] = SI57x_DevRefData()->N1Divider;
			/* Try to take a new FDCO value. */
			FDCO = NewFreq * SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef;
			/* Try number check. */
			TryCounter -= 1;
			if (TryCounter == 0) {
				//TryCounter = 100;
				break;
			}
		}
		/* Recalculate reference frequency. */
		SI57x_DevRefData()->RefFrequency = ((NewFreq * SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef) / INTERNAL_CRYSTAL_FREQ) * FREQ_FRACT_MULTIPLIER;
		SI57x_DevRefData()->CurrentFreq = (INTERNAL_CRYSTAL_FREQ * (SI57x_DevRefData()->RefFrequency / FREQ_FRACT_MULTIPLIER)) /
											(SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef);
		rawdata[10] = SI57x_DevRefData()->RefFrequency;
		CurrentFreq = SI57x_DevRefData()->CurrentFreq;
	}

	/* Freeze M control word to prevent interim frequency changes when writing RFREQ registers. */
	//SI57x_RstFreezeMemCtrl(0, 0, 1, 0, 0);
	SI57x_FreezeDco(1);
	//si57x.delay_fp(1);
	/* Write registers. */
	SI57x_WriteDivRef(CoeffSel);
	si57x.delay_fp(1);
	/* Notice to update the frequency. */
	SI57x_FreezeDco(0);
	SI57x_RstFreezeMemCtrl(0, 0, 0, 1, 0);
	si57x.delay_fp(50);

	/* Read and build the dividers and frequency data. */
	SI57x_ReadDivRef(CoeffSel);

	/* Calculate the current frequency. */
	SI57x_DevRefData()->CurrentFreq = (INTERNAL_CRYSTAL_FREQ * (SI57x_DevRefData()->RefFrequency / FREQ_FRACT_MULTIPLIER)) /
										(SI57x_DevRefData()->HighSpeedDivider * SI57x_DevRefData()->N1DivRef);


	CurrentFreq = SI57x_DevRefData()->CurrentFreq;
	si57x.delay_fp(50);
}

/*
 * @brief Frequency selector (prepared meanings).
 *
 * @param RegAddress 	: Start address of RFREQ register. May be SI57x_HIGH_SPEED_N1_DIV or SI57x_HIGH_SPEED_N1_DIV7PPM.
 * @param PresetNumber 	: Number of preset frequency.
 *
 */
void SI57x_SelPresetFreq(uint8_t RegAddress, uint8_t PresetNumber) {

	static uint8_t PresetFreqPattern[6][6]  = {

			/* 10 MHz. */
			{ 0xAD, 0x42, 0xA8, 0x67, 0x7D, 0x17 },
			/* 50 MHz. */
			{ 0x63, 0x42, 0xAE, 0x01, 0x18, 0xFC },
			/* 88.125 MHz. */
			{ 0x61, 0xC2, 0xB2, 0xE7, 0x81, 0x64 },
			/* 100 MHz. */
			{ 0x22, 0x42, 0xBC, 0x01, 0x1E, 0xB8 },
			/* 176.25 MHz. */
			{ 0x60, 0xC2, 0xB2, 0xE7, 0x81, 0x64 },
			/* 200 MHz. */
			{ 0x60, 0xC3, 0x10, 0x01, 0x41, 0x20 }

	};

	if (PresetNumber > 6 || PresetNumber < 1) return;


	/* Freeze M control word to prevent interim frequency changes when writing RFREQ registers. */
	SI57x_FreezeDco(1);
	/* Write registers. */
	si57x.i2c_tx_fp(RegAddress, (uint8_t*)&PresetFreqPattern[PresetNumber - 1][0], 6);
	si57x.delay_fp(2);
	/* Notice to update the frequency. */
	SI57x_FreezeDco(0);
	SI57x_RstFreezeMemCtrl(0, 0, 0, 1, 0);
	si57x.delay_fp(50);
}

/*
 * @brief Reset, freeze and memory control.
 *
 * @param Recall 		: Recall NVM into RAM. Asserting RECALL reloads the NVM contents in to the operating registers without interrupting
 * 				   		  the I2C state machine. It is the recommended approach for starting from initial conditions.
 * 							0 - no operation,
 * 							1 - write NVM bits into RAM. Bit is internally reset following completion of operation.
 * @param FreezeVcadc 	: Freezes the VC ADC Output Word. May be used to hold the nominal output frequency of an Si571.
 * 							0 - don't hold the frequency,
 * 							1 - hold the frequency.
 * @param FreezeM 		: Freezes the M Control Word. Prevents interim frequency changes when writing RFREQ registers.
 * 							0 - don't freeze M,
 * 							1 - freeze M.
 * @param NewFreq 		: New Frequency Applied. Alerts the DSPLL that a new frequency configuration has been applied. This bit will clear
 * 						  itself when the new frequency is applied.
 * 							1 - notice the DSPLL (apply the new frequency).
 * @param RstReg 		: Internal Reset. Upon completion of internal logic reset, RST_REG is internally reset to zero. Asserting RST_REG will
 * 						  interrupt the I2C state machine. It is not the recommended approach for starting from initial conditions.
 * 							0 - normal operation,
 * 							1 - reset of all internal logic. Output tristated during reset.
 */
void SI57x_RstFreezeMemCtrl(uint8_t Recall, uint8_t FreezeVcadc, uint8_t FreezeM, uint8_t NewFreq, uint8_t RstReg) {

	static SI57x_RstFreezeMemCtrl_t RegCtrl;
	static uint8_t FreezeDcoReg = 0;
	FreezeDcoReg = 0;


	RegCtrl.RECALL = Recall;
	RegCtrl.FREEZE_VCADC = FreezeVcadc;
	RegCtrl.FREEZE_M = FreezeM;
	RegCtrl.NEW_FREQ = NewFreq;
	RegCtrl.RST_REG = RstReg;
	RegCtrl.RESERVED = 0;

	//si57x.i2c_tx_fp(SI57x_FREEZE_DCO, &FreezeDcoReg, 1);
	si57x.i2c_tx_fp(SI57x_RST_FREEZE_MEM_CTRL, &RegCtrl.RstFreezeMemCtrlReg, 1);
	//si57x.delay_fp(1);
}

/*
 * @brief Freeze DCO control.
 *
 * @param FreezeDco : Freezes the DSPLL so the frequency configuration can be modified.
 * 						0 - don't freeze DCO,
 * 						1 - freeze DCO.
 */
void SI57x_FreezeDco(uint8_t FreezeDco) {

	static uint8_t FreezeDcoReg = 0;
	FreezeDcoReg = FreezeDco << 4;


	si57x.i2c_tx_fp(SI57x_FREEZE_DCO, &FreezeDcoReg, 1);
	//si57x.delay_fp(1);
}

/*
 * @brief Read high speed, N1 dividers and reference clock registers from the device.
 *
 * @param Address : Start address of RFREQ register. May be SI57x_HIGH_SPEED_N1_DIV or SI57x_HIGH_SPEED_N1_DIV7PPM.
 */
static void SI57x_ReadDivRef(uint8_t Address) {

	if (Address != SI57x_HIGH_SPEED_N1_DIV && Address != SI57x_HIGH_SPEED_N1_DIV7PPM) { return; }

	static uint8_t div_coeff[8] = { 4, 5, 6, 7, 7, 9, 9, 11 };

	si57x.i2c_rx_fp(Address, (uint8_t*)&SI57x_RawRegData()->HighSpeedN1Div, 6);
	si57x.delay_fp(10);

	/* Build the dividers and frequency data. */
	SI57x_DevRefData()->HighSpeedDivider = 0;
	SI57x_DevRefData()->N1Divider = 0;
	SI57x_DevRefData()->RefFrequency = 0;
	SI57x_DevRefData()->HighSpeedDivider = div_coeff[SI57x_RawRegData()->HighSpeedN1Div.HS_DIV];
	SI57x_DevRefData()->N1_DIV_BIT1_0 = SI57x_RawRegData()->RefFreqN1Div.N1_DIV_BIT1_0;
	SI57x_DevRefData()->N1_DIV_BIT6_2 = SI57x_RawRegData()->HighSpeedN1Div.N1_DIV_BIT6_2;
	SI57x_DevRefData()->N1DivRef = SI57x_DevRefData()->N1Divider;
	if ((SI57x_DevRefData()->N1Divider != 1) && (SI57x_DevRefData()->N1Divider % 2) > 0) {
		SI57x_DevRefData()->N1DivRef = SI57x_DevRefData()->N1Divider + 1;
	}
	SI57x_DevRefData()->RFREQ_BIT37_32 = SI57x_RawRegData()->RefFreqN1Div.REF_FREQ_BIT37_32;
	SI57x_DevRefData()->RFREQ_BIT31_24 = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit31_24;
	SI57x_DevRefData()->RFREQ_BIT23_16 = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit23_16;
	SI57x_DevRefData()->RFREQ_BIT15_8 = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit15_8;
	SI57x_DevRefData()->RFREQ_BIT7_0 = SI57x_RawRegData()->RefFreqN1Div.RefFreqBit7_0;
}

/*
 * @brief Write high speed, N1 dividers and reference clock registers to the device.
 *
 * @param Address : Start address of RFREQ register. May be SI57x_HIGH_SPEED_N1_DIV or SI57x_HIGH_SPEED_N1_DIV7PPM.
 */
static void SI57x_WriteDivRef(uint8_t Address) {

	static uint8_t div_coeff[12] = { 0, 0, 0, 0, 0, 1, 2, 3, 3, 5, 5, 7 };

	if (Address != SI57x_HIGH_SPEED_N1_DIV && Address != SI57x_HIGH_SPEED_N1_DIV7PPM) { return; }

	/* Build the dividers and frequency data. */
	SI57x_RawRegData()->HighSpeedN1Div.HS_DIV = div_coeff[SI57x_DevRefData()->HighSpeedDivider];
	SI57x_RawRegData()->RefFreqN1Div.N1_DIV_BIT1_0 = SI57x_DevRefData()->N1_DIV_BIT1_0;
	SI57x_RawRegData()->HighSpeedN1Div.N1_DIV_BIT6_2 = SI57x_DevRefData()->N1_DIV_BIT6_2;
	SI57x_RawRegData()->RefFreqN1Div.REF_FREQ_BIT37_32 = SI57x_DevRefData()->RFREQ_BIT37_32;
	SI57x_RawRegData()->RefFreqN1Div.RefFreqBit31_24 = SI57x_DevRefData()->RFREQ_BIT31_24;
	SI57x_RawRegData()->RefFreqN1Div.RefFreqBit23_16 = SI57x_DevRefData()->RFREQ_BIT23_16;
	SI57x_RawRegData()->RefFreqN1Div.RefFreqBit15_8 = SI57x_DevRefData()->RFREQ_BIT15_8;
	SI57x_RawRegData()->RefFreqN1Div.RefFreqBit7_0 = SI57x_DevRefData()->RFREQ_BIT7_0;

	si57x.i2c_tx_fp(Address, (uint8_t*)&SI57x_RawRegData()->HighSpeedN1Div, 6);
	si57x.delay_fp(1);
}

/*
 * @brief Dividers and ref-clocks data struct.
 *
 * @retval SI57x_FreqDivTable_t : Return calculated values of dividers and reference clock.
 */
static SI57x_FreqDivTable_t* SI57x_DevRefData(void) {

	static SI57x_FreqDivTable_t RefData;
	return &RefData;
}

/*
 * @brief Internal RAW data struct object.
 *
 * @retval SI57x_RawRegData_t : Return RAW register values of dividers and reference clock.
 */
static SI57x_RawRegData_t* SI57x_RawRegData(void) {
	static SI57x_RawRegData_t RegData;
	return &RegData;
}

/* Hardware dependent functions. */

/*
 * @brief Output enable pin control.
 *
 * @param OEEnDis : OE pin control.
 * 						0 - chip is not active,
 * 						1 - chip is active.
 */
static void SI57x_OEPin(uint8_t OEEnDis) {

	  if (OEEnDis) { HAL_GPIO_WritePin(SI570_OE_GPIO_Port, SI570_OE_Pin, GPIO_PIN_SET); }
	  else { HAL_GPIO_WritePin(SI570_OE_GPIO_Port, SI570_OE_Pin, GPIO_PIN_RESET); }
}

/*
 * @brief Receive data from the chip.
 */
static void SI57x_I2CRxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Read(SI57xI2C, SI57x_CURRENT_ADDR, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);

}

/*
 * @brief Transmit data to the chip.
 */
static void SI57x_I2CTxData(uint8_t MemAddr, uint8_t *pData, uint8_t Size) {

	HAL_I2C_Mem_Write(SI57xI2C, SI57x_CURRENT_ADDR, MemAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, 25);
}

