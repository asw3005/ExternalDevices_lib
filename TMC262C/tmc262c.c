/*
 * @brief Common C for TMC262C.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "tmc262c.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "spi.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>

//#define HARD_SPI_NSS

 /* External variables. */
extern SPI_HandleTypeDef hspi3;

/* Private variables. */
static SPI_HandleTypeDef* TMC262C_SpiInst = &hspi3;


/* Private function prototypes. */
static uint8_t TMC262C_TxCheck(void);
static uint8_t TMC262C_RxCheck(void);
static void TMC262C_Tx(uint8_t *pData, uint8_t size);
static void TMC262C_Rx(uint8_t *pData, uint8_t size);
static void TMC262C_RxTx(uint8_t *pTxData, uint8_t *pRxData, uint8_t size);
static void TMC262C_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/* General struct. */
static TMC262C_GInst_t tmc262c_inst = {

	.delay = HAL_Delay,
	.spi_rx = TMC262C_Rx,
	.spi_tx = TMC262C_Tx
};

/*
 * @brief 
 *
**/

/*
 * @brief Init control pins.
 *
**/
void TMC262C_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = { 0} ;

    /*Configure GPIO pin : input pin 4 */
    GPIO_InitStruct.Pin = TMC262C_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC262C_CS_GPIO_Port, &GPIO_InitStruct);

    TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
}

/*
 * @brief Driver control register. 
 *
 * @param sdoff_state 		: Select driver control register format.
 * 								0 - STEP/DIR interface is enabled, and DRVCTRL is a configuration register
 *								for the STEP/DIR interface (default).
 *							  	1 - STEP/DIR interface is disabled, and DRVCTRL is the interface for specifying the
 * 								currents through each coil.
 * @param intpol			: Enable STEP interpolation.
 *								0 - disable STEP pulse interpolation,
 *								1 - enable MicroPlyer STEP pulse multiplication by 16.
 * @param dedge				: Enable double edge STEP pulses.
 *								0 - rising STEP pulse edge is active, falling edge is inactive,
 *								1 - both rising and falling STEP pulse edges are active.
 * @param mres				: Microstep resolution for STEP/DIR mode, microsteps per 90 degrees.
 *								0 - 256, 1 - 128, 2 - 64, 3 - 32,
 *								4 - 16,	 5 - 8,   6 - 4, 7 - 2 (halfstep),
 *								8 - 1 (fullstep)
 * @param pha_polarity_a 	: Sign of current flow through coil A.
 *								0 - current flows from OA1 pins to OA2 pins,
 *								1 - current flows from OA2 pins to OA1 pins.
 * @param ca_current_a 		: Magnitude of current flow through coil A. The range is 0 to 248, if hysteresis or offset are used up to their full extent. The 
 *								resulting value after applying hysteresis or offset must not exceed 255.
 * @param phb_polarity_b 	: Sign of current flow through coil B.
 *								0 - current flows from OB1 pins to OB2 pins,
 *								1 - current flows from OB2 pins to OB1 pins.
 * @param cb_current_b 		: Magnitude of current flow through coil B. The range is 0 to 248, if hysteresis or offset are used up to their fullextent. The 
 *								resulting value after applying hysteresis or offset must not exceed 255.
 * @param read_back 		: Read current state of register (it is software only register read, hardware registers are read only).
 *								0 - regular state, write current values to the register,
 *								1 - read back, it'll not write any values to the register.	
 *
**/
TMC262C_DrvCtrlRSet_t TMC262C_DrvCtrl(uint8_t sdoff_state, uint8_t intpol, uint8_t dedge, uint8_t mres, 
										uint8_t pha_polarity_a, uint8_t ca_current_a, uint8_t phb_polarity_b, uint8_t cb_current_b, uint8_t read_back) {

	static TMC262C_DrvCtrlRSet_t DrvCtrlRSet = { 0 };

	if(!read_back) {
		DrvCtrlRSet.DrvCtrlSPI.REG_ADDR = TMC262C_DRVCTRL;

		if(!sdoff_state) {

			DrvCtrlRSet.DrvCtrlSTEPDIR.INTPOL = intpol;
			DrvCtrlRSet.DrvCtrlSTEPDIR.DEDGE = dedge;
			DrvCtrlRSet.DrvCtrlSTEPDIR.MRES3_0 = mres;
			DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED17_16 = 0;
			DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED15_10 = 0;
			DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED7_4 = 0;
			/* Send 3 bytes to the driver. Highest four bit are dummy. */
			TMC262C_Tx(&DrvCtrlRSet.DrvCtrlSTEPDIR.DrvCtrl_LSB_H, 3);

		} else {

			DrvCtrlRSet.DrvCtrlSPI.PHA = pha_polarity_a;
			DrvCtrlRSet.DrvCtrlSPI.PHB = phb_polarity_b;
			DrvCtrlRSet.DrvCtrlSPI.CA6_0 = ca_current_a;
			DrvCtrlRSet.DrvCtrlSPI.CA7 = ca_current_a >> 7;
			/* Send 3 bytes to the driver. Highest four bit are dummy. */
			TMC262C_Tx(&DrvCtrlRSet.DrvCtrlSPI.DrvCtrl_LSB_H, 3);
		}

		/* Check transmit or just wait. */
		TMC262C_TxCheck();
	}

	return DrvCtrlRSet;
}

/*
 * @brief Chopper control register. 
 *
 * @param tbl 		: Blanking time interval, in system clock periods.
 *						0 - 16, 1 - 24, 2 - 36, 3 - 54.
 * @param chm 		: This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below.
 *						0 - standard mode (SpreadCycle),
 *						1 - Constant tOFF with fast decay time. Fast decay time is also terminated when the negative 
 *							nominal current is reached. Fast decay is after on time.
 * @param rndtf 	: Enable randomizing the slow decay phase duration.
 *						0 - chopper off time is fixed as set by bits tOFF,
 *						1 - random mode, tOFF is random modulated by dNCLK= -24 … +6 clocks.
 * @param hdec1,
 * 		  hdec0 	: Hysteresis decrement period setting, in system clock periods:
 *					  CHM = 0
 *						00 - 16, 01 - 32, 10 - 48, 11 - 64.
 *					  CHM = 1
 *						HDEC1 = 0 - current comparator can terminate the fast decay phase before timer expires,
 *						HDEC1 = 1 - only the timer terminates the fast decay phase,
 *						HDEC0 - MSB of fast decay time setting, HDEC0 + HSTRT2_0.
 * @param hend 		: Hysteresis end (low) value or Sine wave offset.
 *					  CHM = 0
 *						0000 - 1111 (0 - 15)
 *						Hysteresis is -3, -2, -1, 0, 1, …, 12 (1/512 of this setting adds to current setting)
 *						This is the hysteresis value which becomes used for the hysteresis chopper.
 *					  CHM = 1
 *						0000 - 1111 (0 -15)
 *						Offset is -3, -2, -1, 0, 1, …, 12 This is the sine wave offset and 1/512 of the
 *						value becomes added to the absolute value of each sine wave entry.
 * @param hstrt 	: Hysteresis start value or Fast decay time setting.
 *					  CHM = 0 
 *						Hysteresis start offset from HEND. Effective: HEND + HSTRT must be ≤ 15.
 *						0 - 1, 1 - 2, 2 - 3, 3 - 4,
 *						4 - 5, 5 - 6, 6 - 7, 7 - 8.
 *					  CHM = 1
 *						Three least-significant bits of the duration of the fast decay phase. The MSB is HDEC0.
 *						Fast decay time is a multiple of system clock periods: NCLK= 32 x (HDEC0+HSTRT). 
 * @param toff 		: Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay
 *				  	  time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF).
 *						0000 - driver disable, all bridges off,
 *						0001 - 1 (use with TBL of minimum 24 clocks),
 *						0010 - 1111 2..15.
 * @param read_back : Read current state of register (it is software only register read, hardware registers are write only).
 *						0 - regular state, write current values to the register,
 *						1 - read back, it'll not write any values to the register.	
 *
**/
TMC262C_ChopConf_t TMC262C_ChopConf(uint8_t tbl, uint8_t chm, uint8_t rndtf, uint8_t hdec1,
										uint8_t hdec0, uint8_t hend, uint8_t hstrt, uint8_t toff, uint8_t read_back) {

	static TMC262C_ChopConf_t ChopConf = { 0 };

	if(!read_back) {
		ChopConf.REG_ADDR = TMC262C_CHOPCONF;
		ChopConf.TBL1 = tbl >> 1;
		ChopConf.TBL0 = tbl;
		ChopConf.CHM = chm;
		ChopConf.RNDTF = rndtf;
		ChopConf.HDEC1_0 = (hdec1 << 1) | hdec0;
		ChopConf.HEND3_1 = hend >> 1;
		ChopConf.HEND0 = hend;
		ChopConf.HSTRT2_0 = hstrt;
		ChopConf.TOFF3_0 = toff;

		/* Send 3 bytes to the driver. Highest four bit are dummy. */
		TMC262C_Tx(&ChopConf.ChopConf_LSB_H, 3);
		/* Check transmit or just wait. */
		TMC262C_TxCheck();
	}

	return ChopConf;
}

/*
 * @brief CoolStep control register. 
 *
 * @param seimin 	: Minimum CoolStep current.
 *						0 - 1/2 CS current setting,
 *						1 - 1/4 CS current setting.
 * @param sedn 		: Current decrement speed. Number of times that the StallGuard2 value must be sampled equal to or above the upper threshold for each
 *				      decrement of the coil current.
 *						0 - 32, 1 - 8, 2 - 2, 3 - 1. 
 * @param semax 	: Upper CoolStep threshold as an offset from the lower threshold. If the StallGuard2 measurement value SG is sampled equal to or above 
 *					  (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented. Range 0..15.
 * @param seup 		: Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold.
 *						0 - 1, 1 - 2, 2 - 4, 3 - 8.
 * @param semin3_0 	: Lower CoolStep threshold. CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls 
 *					  below SEMIN x 32, the CoolStep current scaling factor is increased. Range 0..15.
 * @param read_back : Read current state of register (it is software only register read, hardware registers are read only).
 *						0 - regular state, write current values to the register,
 *						1 - read back, it'll not write any values to the register.						
 *
**/
TMC262C_SmartEn_t TMC262C_SmartEn(uint8_t semin, uint8_t sedn, uint8_t semax, uint8_t seup, 
									uint8_t semin3_0, uint8_t read_back) {

	static TMC262C_SmartEn_t SmartEn = { 0 };

	if(!read_back) {
		SmartEn.REG_ADDR = TMC262C_SMARTEN;
		SmartEn.SEIMIN = semin;
		SmartEn.SEDN1_0 = sedn;
		SmartEn.SEMAX3_0 = semax;
		SmartEn.SEUP1_0 = seup;
		SmartEn.SEMIN3_0 = semin3_0;
		SmartEn.RESERVED16 = 0;
		SmartEn.RESERVED12 = 0;
		SmartEn.RESERVED7 = 0;
		SmartEn.RESERVED4 = 0;

		/* Send 3 bytes to the driver. Highest four bit are dummy. */
		TMC262C_Tx(&SmartEn.SmartEn_LSB_H, 3);
		/* Check transmit or just wait. */
		TMC262C_TxCheck();
	}

	return SmartEn;
}

/*
 * @brief StallGuard2 control register. 
 *
 * @param sfilt 	: StallGuard2 filter enable.
 *						0 - standard mode, fastest response time,
 *						1 - filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
 * @param sgt_sign	: sign of the sgt parameter.
 * 						0 - positive value,
 * 						1 - negative value.
 * @param sgt 		: StallGuard2 threshold value. The StallGuard2 threshold value controls the optimum measurement range for readout and stall indicator
 *				  	  output (SG_TST). A lower value results in a higher sensitivity and less torque is required to indicate a stall. The value is a two’s
 * 				  	  complement signed integer. Range: -64 to +63.
 * @param cs 		: Current scale (scales digital currents A and B). Current scaling for SPI and STEP/DIR operation. This value is biased by 1 and divided
 *				  	  by 32, so the range is 1/32 to 32/32. Example: CS=20 is 21/32 current.
 *						00000 ... 11111 - 1/32, 2/32, 3/32, ...	32/32.
 *						Example: CS=20 is 21/32 current.	
 * @param read_back : Read current state of register (it is software only register read, hardware registers are read only).
 *						0 - regular state, write current values to the register,
 *						1 - read back, it'll not write any values to the register.				
 *
**/
TMC262C_SgcsConf_t TMC262C_StallGuard(uint8_t sfilt, uint8_t sgt_sign, uint8_t sgt, uint8_t cs, uint8_t read_back) {

	static TMC262C_SgcsConf_t StallGuard = { 0 };

	if(!read_back) {
		StallGuard.REG_ADDR = TMC262C_SGCSCONF;
		StallGuard.SFILT = sfilt;
		StallGuard.SGT6_0 = (sgt_sign << 6) | sgt;
		StallGuard.CS4_0 = cs;
		StallGuard.RESERVED15 = 0;
		StallGuard.RESERVED7_5 = 0;

		/* Send 3 bytes to the driver. Highest four bit are dummy. */
		TMC262C_Tx(&StallGuard.SgcsConf_LSB_H, 3);
		/* Check transmit or just wait. */
		TMC262C_TxCheck();
	}

	return StallGuard;
}

/*
 * @brief Driver configuration register (tmc262c prefix - for C revision only, for TMC262 does not have any effect). 
 *
 * @param tst 				: Reserved TEST mode. Must be cleared for normal operation. When set, the SG_TST output 
 *							  exposes digital test values, and the TEST_ANA output exposes analog test values. 
 * @param slph 				: Slope control, high side. Gate driver strength 1 to 7, 7 is maximum current for fastest slopes. 
 *							  Adjust the gate driver strength to the gate charge of the external MOSFETs and check the desired slope.
 *							  In temperature compensated mode (tc), the MOSFET gate driver strength is increased by one count if the 
 *							  overtemperature warning temperature is reached. This compensates for temperature dependency of high-side 
 *							  slope control.
 *								000 - minimum slope, lowest driver strenghth,
 *								111 - maximum slope, highest driver strength. 
 * @param slpl 				: Slope control, low side. Gate driver strength 1 to 7, 7 is maximum current for fastest slopes. 
 *							  Adjust the gate driver strength to the gate charge of the external MOSFETs and check the desired slope. 
 *								000 - minimum slope, lowest driver strenghth,
 *								111 - maximum slope, highest driver strength. 
 * @param tmc262c_slp2 		: Slope control MSB for high and low side.
 * @param dis_s2g 			: Short to GND protection disable.
 *								0 - short to GND protection is enabled,
 *								1 - short to GND protection is disabled.
 * @param ts2g 				: Short detection delay for high-side and low-side FETs.
 *								0 - 3.2us,
 *								1 - 1.6us,
 *								2 - 1.2us,
 *								3 - 0.8us.
 * @param sdoff 			: STEP/DIR interface disables.
 *								0 - enable STEP/DIR operation,
 *								1 - disable STEP/DIR operation, SPI interface is used to move motor.
 * @param vsense 			: Sense resistor voltage-based current scaling (Full-scale refers to a current setting of 31).
 *								0 - full-scale sense resistor voltage is 325mV,
 *								1 - full-scale sense resistor voltage is 173mV.
 * @param rdsel 			: Select value for read out (RD bits).
 *								0 - microstep position read back,
 *								1 - StallGuard2 level read back,
 *								2 - StallGuard2 and CoolStep current level read back,
 *								3 - all status flags and detectors (TMC262C only).
 * @param tmc262c_otsens 	: Overtemperature sensitivity.
 *								0 - shutdown at 150 Celsius degree,
 *								1 - sensitive shutdown at 136 Celsius degree.
 * @param tmc262c_shrtsens 	: Short to GND sensitivity.
 *								0 - low sensitivity,
 *								1 - high sensitivity - better protection for high side FETs.
 * @param tmc262c_en_pfd 	: Enable passive fast decay, 5V undervoltage threshold.
 *								0 - no addition motor dampening,
 *								1 - motor dampening to reduce motor resonance at medium velocity. In addition, this bit reduces the lower
 *									nominal operation voltage limit from 7V to 4.5V.
 * @param tmc262c_en_s2vs 	: Short to VS protection, CLK failsave enable.
 *								0 - short to VS and overload protection disabled,
 *								1 - short to VS, overcurrent protection enabled. In addition, enables protection against clock input CLK fail,
 *									when using an external clock source. 	
 * @param read_back 		: Read current state of register (it is software only register read, hardware registers are read only).
 *								0 - regular state, write current values to the register,
 *								1 - read back, it'll not write any values to the register.		
 *
**/
TMC262C_DrvConf_t TMC262C_DrvConf(uint8_t tst, uint8_t slph, uint8_t slpl, uint8_t tmc262c_slp2, uint8_t dis_s2g, 
									uint8_t ts2g, uint8_t sdoff, uint8_t vsense, uint8_t rdsel, uint8_t tmc262c_otsens, 
									uint8_t tmc262c_shrtsens, uint8_t tmc262c_en_pfd, uint8_t tmc262c_en_s2vs, uint8_t read_back) {

	static TMC262C_DrvConf_t DrvConf = { 0 };

	if(!read_back) {
		DrvConf.REG_ADDR = TMC262C_DRVCONF;
		DrvConf.TST = tst;
		DrvConf.SLPH1_0 = slph;
		DrvConf.SLPL1_0 = slpl;
		DrvConf.SLP2 = tmc262c_slp2;
		DrvConf.DISS2G = dis_s2g;
		DrvConf.TS2G1_0 = ts2g;
		DrvConf.SDOFF = sdoff;
		DrvConf.VSENSE = vsense;
		DrvConf.RDSEL1_0 = rdsel;
		DrvConf.OTSENS = tmc262c_otsens;
		DrvConf.SHRTSENS = tmc262c_shrtsens;
		DrvConf.EN_PFD = tmc262c_en_pfd;
		DrvConf.EN_S2VS = tmc262c_en_s2vs;

		/* Send 3 bytes to the driver. Highest four bit are dummy. */
		TMC262C_Tx(&DrvConf.DrvConf_LSB_H, 3);
		/* Check transmit or just wait. */
		TMC262C_TxCheck();
	}

	return DrvConf;
}


/*
 * @brief Read responce, four formats at a time. 
 *
 * @retval SG 		: StallGuard2 status.
 *						0 - no motor stall detected,
 *						1 - StallGuard2 threshold has been reached, and the SG_TST output is driven high.
 * @retval OT 		: Overtemperature shutdown.
 *						0 - no overtemperature shutdown condition,
 *						1 - overtemperature shutdown has occurred.
 * @retval OTPW 	: Overtemperature warning.
 *						0 - no overtemperature warning condition,
 *						1 - warning threshold is active.
 * @retval SHORTA 	: Short detection status channel A.
 * @retval SHORTB 	: Short detection status channel B.
 *						0 - no short condition,
 *						1 - short condition. The short counter is incremented by each short circuit and the 
 *							chopper cycle is suspended. The counter is decremented for each phase polarity change. 
 *							The MOSFETs are shut off when the counter reaches 3 and remain shut off until the shutdown
 *							condition is cleared by disabling and re-enabling the driver. The shutdown condition becomes
 *							reset by de-asserting the ENN input or clearing the TOFF parameter.
 * @retval OLA 		: Open load indicator, channel A.
 * @retval OLB 		: Open load indicator, channel B.
 *						0 - no open load condition detected,
 *						1 - no chopper event has happened during the last period with constant coil polarity. Only a current
 *							above 1/16 of the maximum setting can clear this bit! Hint: This bit is only a status indicator. 
 *							The chip takes no other action when this bit is set. False indications may occur during fast
 *							motion and at standstill. Check this bit only during slow motion.
 * @retval STST 	: Standstill indicator.
 *						0 - no standstill condition detected,
 *						1 - no active edge occurred on the STEP input during the last 220 system clock cycles.
 * @retval MSTEP9_0 : Microstep counter. Microstep position in sine table for coil A in STEP/DIR mode. 
 *						MSTEP9 is the Polarity bit.
 *						0 - current flows from OA1 pins to OA2 pins,
 *						1 - current flows from OA2 pins to OA1 pins.
 * @retval SG9_0 	: StallGuard2 value SG9:0.
 * @retval SG9_5 	: StallGuard2 value SG9:5.
 * @retval SE4_0 	: Actual CoolStep scaling value SE4:0.
 * @retval OT100 	: Overtemperature 100 Celsius degree.
 * @retval OT120 	: Overtemperature 120 Celsius degree.
 * @retval OT136 	: Overtemperature 136 Celsius degree.
 * @retval OT150 	: Overtemperature 150 Celsius degree.
 * @retval S2GA 	: Short to GND channel A.
 * @retval S2VSA 	: Short to VS channel A.
 * @retval S2GB 	: Short to GND channel B.
 * @retval S2VSB 	: Short to VS channel B.
 * @retval ENN in 	: state of ENN input.
 * @retval UV_7V 	: <7V VS flag.
 * @retval bits9_8 	: 11 response allows to distinguish -C type. Non-C-type delivers %00 in each case.
 *
**/
TMC262C_ReadBack_t* TMC262C_ReadBack(void) {

	static TMC262C_DrvConf_t DrvConf = { 0 };
	static TMC262C_ReadBack_t ReadBack = { 0 };

	/* Read current value of driver configuration. */
	DrvConf = TMC262C_DrvConf(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

	/* Read all four responce formats. */
	DrvConf.RDSEL1_0 = 1;
	TMC262C_RxTx(&DrvConf.DrvConf_LSB_H, &ReadBack.ReadBackRDSEL00.ReadBack_MSB_H, 3);
	tmc262c_inst.delay(1);

	DrvConf.RDSEL1_0 = 2;
	TMC262C_RxTx(&DrvConf.DrvConf_LSB_H, &ReadBack.ReadBackRDSEL01.ReadBack_MSB_H, 3);
	tmc262c_inst.delay(1);

	DrvConf.RDSEL1_0 = 3;
	TMC262C_RxTx(&DrvConf.DrvConf_LSB_H, &ReadBack.ReadBackRDSEL10.ReadBack_MSB_H, 3);
	tmc262c_inst.delay(1);

	DrvConf.RDSEL1_0 = 0;
	TMC262C_RxTx(&DrvConf.DrvConf_LSB_H, &ReadBack.ReadBackRDSEL11.ReadBack_MSB_H, 3);
	tmc262c_inst.delay(1);

	return &ReadBack;
}

 /* Hardware dependent functions. */

 /*
 * @brief Switches off all MOSFETs. Tie low ENN pin for normal operation.
 * 
 * @param state : Enable or disable driver.
 *					0 - driver enabled,
 *					1 - driver disabled.
 *
 **/
void TMC262C_EnableCtrl(uint8_t state) {
	
	if (state > 0) {
		TMC262C_EN_GPIO_Port->BSRR = TMC262C_EN_Pin;
	}
	else {
		TMC262C_EN_GPIO_Port->BSRR = TMC262C_EN_Pin << 16;
	}	
}

/*
* @brief Direction select.
*
* @param direction :0 - forward,
*					1 - backward.
*
**/
void TMC262C_DirectionCtrl(uint8_t direction) {

	if (direction > 0) {
		TMC262C_DIR_GPIO_Port->BSRR = TMC262C_DIR_Pin;
	}
	else {
		TMC262C_DIR_GPIO_Port->BSRR = TMC262C_DIR_Pin << 16;
	}
}

#ifndef HARD_SPI_NSS
/*
 * @brief SPI chip select. You must disable hardware SPI_NSS to use this function.
 * 
 * @param gpio      : TMC262C_CS_GPIO_Port.
 * @param gpio_pin  : TMC262C_CS_Pin.
 * @param state     : GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void TMC262C_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}
#endif 

/*
 * @brief SPI Tx data.
 *
 **/
static void TMC262C_Tx(uint8_t *pData, uint8_t size) {

	#ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_RESET);
    #endif
	HAL_SPI_Transmit_IT(TMC262C_SpiInst, pData, size);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}

/*
 * @brief SPI Rx data.
 * 
 *
 **/
static void TMC262C_Rx(uint8_t *pData, uint8_t size) {
	
	#ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_RESET);
    #endif
	HAL_SPI_Receive_IT(TMC262C_SpiInst, pData, size);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}

/*
 * @brief SPI TxRx data.
 * 
 *
 **/
static void TMC262C_RxTx(uint8_t *pTxData, uint8_t *pRxData, uint8_t size) {
	
	#ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_RESET);
    #endif
	HAL_SPI_TransmitReceive_IT(TMC262C_SpiInst, pTxData, pRxData, size);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}


 /*
 * @brief Transmit complete check.
 *
**/
static uint8_t TMC262C_TxCheck(void) {

	//tmc262c_inst.delay(1);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	return 0;
}

 /*
 * @brief Receive complete check.
 *
**/
static uint8_t TMC262C_RxCheck(void) {

	//tmc262c_inst.delay(1);
	__NOP();__NOP();__NOP();__NOP();__NOP();
	return 0;
}
