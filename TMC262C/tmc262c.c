/*
 * @brief Common C for TMC262C.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "tmc262c.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "spi.h"
#include <complex.h>
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

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : input pin 4 */
    GPIO_InitStruct.Pin = TMC262C_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC262C_CS_GPIO_Port, &GPIO_InitStruct);
	
}

/*
 * @brief Driver control register. 
 *
 * @param sdoff_state 		: STEP/DIR mode - SDOFF, DRVCONF[7], bit is clear, the STEP/DIR interface is enabled, and DRVCTRL is a configuration register 
 *								for the STEP/DIR interface (default).
 *							  SPI mode 	  - SDOFF bit, DRVCONF[7], is set, the STEP/DIR interface is disabled, and DRVCTRL is the interface for specifying the 
 * 								currents through each coil. 
 *
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
 *
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
 *
**/
TMC262C_DrvCtrlRSet_t TMC262C_DrvCtrl(uint8_t sdoff_state, uint8_t intpol, uint8_t dedge, uint8_t mres, 
										uint8_t pha_polarity_a, uint8_t ca_current_a, uint8_t phb_polarity_b, uint8_t cb_current_b) {

	static TMC262C_DrvCtrlRSet_t DrvCtrlRSet;

	DrvCtrlRSet.DrvCtrlSPI.REG_ADDR = TMC262C_DRVCTRL;

	if(!sdoff_state) {

		DrvCtrlRSet.DrvCtrlSTEPDIR.INTPOL = intpol;
		DrvCtrlRSet.DrvCtrlSTEPDIR.DEDGE = dedge;
		DrvCtrlRSet.DrvCtrlSTEPDIR.MRES3_0 = mres;
		DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED17_16 = 0;
		DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED15_10 = 0;
		DrvCtrlRSet.DrvCtrlSTEPDIR.RESERVED7_4 = 0;

	} else {

		DrvCtrlRSet.DrvCtrlSPI.PHA = pha_polarity_a;
		DrvCtrlRSet.DrvCtrlSPI.PHB = phb_polarity_b;
		DrvCtrlRSet.DrvCtrlSPI.CA6_0 = ca_current_a;
		DrvCtrlRSet.DrvCtrlSPI.CA7 = ca_current_a >> 7;
	}

	/* Send 3 bytes to the driver. Highest four bit are dummy. */
	TMC262C_Tx(&DrvCtrlRSet.DrvCtrlSPI.DrvCtrl_LSB_H, 3);
	/* Check transmit or just wait. */
	TMC262C_TxCheck();

	return DrvCtrlRSet;
}

/*
 * @brief Chopper control register. 
 *
 * @param tbl 	: Blanking time interval, in system clock periods.
 *					0 - 16, 1 - 24, 2 - 36, 3 - 54.
 * @param chm 	: This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below.
 *					0 - standard mode (SpreadCycle),
 *					1 - Constant tOFF with fast decay time. Fast decay time is also terminated when the negative 
 *						nominal current is reached. Fast decay is after on time.
 * @param rndtf : Enable randomizing the slow decay phase duration.
 *					0 - chopper off time is fixed as set by bits tOFF,
 *					1 - random mode, tOFF is random modulated by dNCLK= -24 … +6 clocks.
 * @param hdec10: Hysteresis decrement period setting, in system clock periods:
 *					CHM = 0
 *						00 - 16, 01 - 32, 10 - 48, 11 - 64.
 *					CHM = 1
 *						HDEC1 = 0 - current comparator can terminate the fast decay phase before timer expires,
 *						HDEC1 = 1 - only the timer terminates the fast decay phase,
 *						HDEC0 - MSB of fast decay time setting, HDEC0 + HSTRT2_0.
 * @param hend 	: Hysteresis end (low) value or Sine wave offset.
 *					CHM = 0
 *						0000 - 1111 (0 - 15)
 *						Hysteresis is -3, -2, -1, 0, 1, …, 12 (1/512 of this setting adds to current setting)
 *						This is the hysteresis value which becomes used for the hysteresis chopper.
 *					CHM = 1
 *						0000 - 1111 (0 -15)
 *						Offset is -3, -2, -1, 0, 1, …, 12 This is the sine wave offset and 1/512 of the
 *						value becomes added to the absolute value of each sine wave entry.
 * @param hstrt : Hysteresis start value or Fast decay time setting.
 *					CHM = 0 
 *						Hysteresis start offset from HEND. Effective: HEND + HSTRT must be ≤ 15.
 *						0 - 1, 1 - 2, 2 - 3, 3 - 4,
 *						4 - 5, 5 - 6, 6 - 7, 7 - 8.
 *					CHM = 1
 *						Three least-significant bits of the duration of the fast decay phase. The MSB is HDEC0.
 *						Fast decay time is a multiple of system clock periods: NCLK= 32 x (HDEC0+HSTRT). 
 * @param toff 	: Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay
 *				  time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF).
 *						0000 - driver disable, all bridges off,
 *						0001 - 1 (use with TBL of minimum 24 clocks),
 *						0010 - 1111 2..15.
 *
**/
TMC262C_ChopConf_t TMC262C_ChopConf(uint8_t tbl, uint8_t chm, uint8_t rndtf, uint8_t hdec0, 
										uint8_t hdec1, uint8_t hend, uint8_t hstrt, uint8_t toff) {

	static TMC262C_ChopConf_t ChopConf;

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
 *					  (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented.
 * @param seup 		: Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold.
 *						0 - 1, 1 - 2, 2 - 4, 3 - 8.
 * @param semin3_0 	: Lower CoolStep threshold. CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls 
 *					  below SEMIN x 32, the CoolStep current scaling factor is increased.						
 *
**/
TMC262C_SmartEn_t TMC262C_SmartEn(uint8_t semin, uint8_t sedn, uint8_t semax, uint8_t seup, 
									uint8_t semin3_0) {

	static TMC262C_SmartEn_t SmartEn;

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

	return SmartEn;
}

/*
 * @brief StallGuard2 control register. 
 *
 * @param sfilt : StallGuard2 filter enable.
 *					0 - standard mode, fastest response time,
 *					1 - filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
 * @param sgt 	: StallGuard2 threshold value. The StallGuard2 threshold value controls the optimum measurement range for readout and stall indicator
 *				  output (SG_TST). A lower value results in a higher sensitivity and less torque is required to indicate a stall. The value is a two’s
 * 				  complement signed integer. Range: -64 to +63.
 * @param cs 	: Current scale (scales digital currents A and B). Current scaling for SPI and STEP/DIR operation. This value is biased by 1 and divided
 *				  by 32, so the range is 1/32 to 32/32. Example: CS=20 is 21/32 current.
 *					00000 ... 11111 - 1/32, 2/32, 3/32, ...	32/32.				
 *
**/
TMC262C_SgcsConf_t TMC262C_StallGuard(uint8_t sfilt, uint8_t sgt, uint8_t cs) {

	static TMC262C_SgcsConf_t StallGuard;

	StallGuard.REG_ADDR = TMC262C_SFCSCONF;
	StallGuard.SFILT = sfilt;
	StallGuard.SGT6_0 = sgt;
	StallGuard.CS4_0 = cs;
	StallGuard.RESERVED15 = 0;
	StallGuard.RESERVED7_5 = 0;

	/* Send 3 bytes to the driver. Highest four bit are dummy. */
	TMC262C_Tx(&StallGuard.SgcsConf_LSB_H, 3);
	/* Check transmit or just wait. */
	TMC262C_TxCheck();

	return StallGuard;
}

 /* Hardware dependent functions. */

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
	HAL_SPI_Transmit(TMC262C_SpiInst, pData, size, 10);
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
	HAL_SPI_Receive(TMC262C_SpiInst, pData, size, 10);
    #ifndef HARD_SPI_NSS
	TMC262C_SPI_CS(TMC262C_CS_GPIO_Port, TMC262C_CS_Pin, GPIO_PIN_SET);
    #endif
}


 /*
 * @brief Transmit complete check.
 *
**/
static uint8_t TMC262C_TxCheck(void) {

	tmc262c_inst.delay(5);
	return 0;
}

 /*
 * @brief Receive complete check.
 *
**/
static uint8_t TMC262C_RxCheck(void) {

	tmc262c_inst.delay(5);
	return 0;
}