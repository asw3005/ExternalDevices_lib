/*
 * @brief C for MC33CD1020.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "mc33cd1020.h"
#include "cmsis_gcc.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>

//#define HARD_SPI_NSS

 /* External variables. */
extern SPI_HandleTypeDef hspi2;

/* Private variables. */
static SPI_HandleTypeDef* MC33CD1020_SpiInst = &hspi2;


/* Private function prototypes. */
static uint8_t MC33CD1020_TxCheck(void); 
static uint8_t MC33CD1020_RxCheck(void); 
static void MC33CD1020_Tx(uint8_t *pData, uint8_t size);
static void MC33CD1020_Rx(uint8_t *pData, uint8_t size);
static void MC33CD1020_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/* General struct. */
static MC33CD1020_GInst_t mc33cd1020_inst = {

	.delay = HAL_Delay,
	.spi_rx = MC33CD1020_Rx,
	.spi_tx = MC33CD1020_Tx
};

/*
 * @brief 
 *
**/

/*
 * @brief SPI check.
 * @retval uint32_t : 0x00123456. 
 *
**/
uint32_t MC33CD1020_SPICheck(void) {

	static uint32_t SPICheckWData = { 0 };
	
	SPICheckWData = 0;
	mc33cd1020_inst.spi_tx((uint8_t *)&SPICheckWData, sizeof(SPICheckWData));
	mc33cd1020_inst.spi_rx((uint8_t *)&SPICheckWData, sizeof(SPICheckWData));

	SPICheckWData = (SPICheckWData << 24) | ((SPICheckWData & 0x0000FF00) << 8) | ((SPICheckWData & 0x00FF0000) >> 8) | (SPICheckWData >> 24);

	__NOP();
	return SPICheckWData;
}

/*
 * @brief Device configuration register.
 *
 * @param rw_bit 			: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ.
 * @param sp0-sp7 			: 	0 - switch to ground,
 * 								1 - switch to battery (default).
 * @param intb_out 			: 	0 – INT pin stays low when interrupt occurs (default),
 *					  		  	1 – INT pin pulse low and return high.
 * @param wakeb_vddqcheck 	: 	0 – WAKE_B is pulled up to VDDQ (internally and/or externally). WAKE_B is ignored while in LPM if VDDQ is low,
 *							  	1 – WAKE_B is externally pulled up to VBATP or VDDQ and wakes upon a falling edge of the WAKE_B pin regardless of the VDDQ
 *								  	status.(VDDQ is not expected to go low) (default)
 * @param vbat_ovdis 		: VBATP Overvoltage protection
 *								0 - enabled (default),
 *								1 - disabled.
 * @param sbpoll_time 		: Select the polling time for SP channels configured as SB.
 *								0 - set the active polling timer to 1ms (default),
 *								1 - set the active polling timer to 55us.
 *
 * @retval MC33CD1020_DevCfg_t type.
 *
 **/
 MC33CD1020_DevCfg_t MC33CD1020_DevCfg(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7,
											uint8_t intb_out, uint8_t wakeb_vddqcheck,
											uint8_t vbat_ovdis, uint8_t sbpoll_time
											) {

	static MC33CD1020_DevCfg_t DevCfg = { 0 };

	DevCfg.REG_ADDR_RW = (MC33CD1020_DEV_CFG | rw_bit);
	DevCfg.SP0 = sp0;
	DevCfg.SP1 = sp1;
	DevCfg.SP2 = sp2;
	DevCfg.SP3 = sp3;
	DevCfg.SP4 = sp4;
	DevCfg.SP5 = sp5;
	DevCfg.SP6 = sp6;
	DevCfg.SP7 = sp7;
	DevCfg.INTB_OUT = intb_out;
	DevCfg.WAKEB_VDDQCHECK = wakeb_vddqcheck;
	DevCfg.VBATP_OVDIS = vbat_ovdis;
	DevCfg.SBPOLLTIME = sbpoll_time;

	mc33cd1020_inst.spi_tx((uint8_t *)&DevCfg.DevCfgReg, sizeof(DevCfg.DevCfgReg));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&DevCfg.DevCfgReg, sizeof(DevCfg.DevCfgReg));
	}

	return DevCfg;
 }

 /*
 * @brief Tri-state SP register. The tri-state command is use to set the input nodes as high-impedance. The configurable
 *			comparator (4.0 V default) on each input remains active. The MCU may change or update the tri-state register
 *			via software at any time in normal mode. The tri-state register defaults to 1 (inputs are tri-stated). Any inputs in
 *			tristate are still polled in LPM, but the current source is not active during this time. The determination of change
 *			of state occurs at the end of the tACTIVEPOLL and the wake-up decision is made.
 *
 * @param rw_bit 			: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param sp0-sp7 			: 	0 - active state,
 * 								1 - the input is high-impedance regardless of the Wetting current settings (default).
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSP_t MC33CD1020_TriStateSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t TriStateSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&TriStateSP, (MC33CD1020_TRISTATE_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;
	if(!rw_bit) {
		TriStateSP.UniSPReg = ReadBack;
	}

	return TriStateSP;
 }

/*
 * @brief Tri-state SG register. The tri-state command is use to set the input nodes as high-impedance. The configurable
 *			comparator (4.0 V default) on each input remains active. The MCU may change or update the tri-state register
 *			via software at any time in normal mode. The tri-state register defaults to 1 (inputs are tri-stated). Any inputs in
 *			tristate are still polled in LPM, but the current source is not active during this time. The determination of change
 *			of state occurs at the end of the tACTIVEPOLL and the wake-up decision is made.
 *
 * @param rw_bit 			: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param sg0-sg13 			: 	0 - active state,
 * 								1 - the input is high-impedance regardless of the Wetting current settings (default).
 *
 * @retval MC33CD1020_UniSG_t type.
 *
 **/
 MC33CD1020_UniSG_t MC33CD1020_TriStateSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;												
	static MC33CD1020_UniSG_t TriStateSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&TriStateSG, (MC33CD1020_TRISTATE_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		TriStateSG.UniSGReg = ReadBack;
	}

	return TriStateSG;
 }

/*
 * @brief Wetting current levels, SP0 - SP7.
 *
 * @param WettCurrent spx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
 *                      	software at any time in normal mode.
 *                          	0 - 2.0 mA,
 *                          	1 - 8.0 mA,
 *                          	2 - 12.0 mA,
 *                          	3 - 16.0 mA(default).
 *
 * @retval MC33CD1020_WettCurrentSP_t type.
 *
 **/
 MC33CD1020_WettCurrentSP_t MC33CD1020_WettCurrentSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	static MC33CD1020_WettCurrentSP_t WettCurrentSP = { 0 };

	WettCurrentSP.WettCurrentSPReg = 0;
	WettCurrentSP.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SP | rw_bit);
	WettCurrentSP.SP0 = sp0;
	WettCurrentSP.SP1 = sp1;
	WettCurrentSP.SP2 = sp2;
	WettCurrentSP.SP3 = sp3;
	WettCurrentSP.SP4 = sp4;
	WettCurrentSP.SP5 = sp5;
	WettCurrentSP.SP6 = sp6;
	WettCurrentSP.SP7 = sp7;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSP.WettCurrentSPReg, sizeof(WettCurrentSP.WettCurrentSPReg));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSP.WettCurrentSPReg, sizeof(WettCurrentSP.WettCurrentSPReg));
	}

	return WettCurrentSP;
}

/*
 * @brief Wetting current levels, SG0 - SG7.
 *
 * @param WettCurrent sgx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
 *                      	software at any time in normal mode.
 *                          	0 - 2.0 mA,
 *                          	1 - 8.0 mA,
 *                          	2 - 12.0 mA,
 *                          	3 - 16.0 mA (default).
 *
 * @retval MC33CD1020_WettCurrentSGReg0_t type.
 *
 **/
 MC33CD1020_WettCurrentSGReg0_t MC33CD1020_WettCurrentSGReg0(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3,
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7
											) {

	static MC33CD1020_WettCurrentSGReg0_t WettCurrentSG = { 0 };

	WettCurrentSG.WettCurrentSGReg0 = 0;
	WettCurrentSG.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SGR0 | rw_bit);
	WettCurrentSG.SG0 = sg0;
	WettCurrentSG.SG1 = sg1;
	WettCurrentSG.SG2 = sg2;
	WettCurrentSG.SG3 = sg3;
	WettCurrentSG.SG4 = sg4;
	WettCurrentSG.SG5 = sg5;
	WettCurrentSG.SG6 = sg6;
	WettCurrentSG.SG7 = sg7;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSG.WettCurrentSGReg0, sizeof(WettCurrentSG.WettCurrentSGReg0));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSG.WettCurrentSGReg0, sizeof(WettCurrentSG.WettCurrentSGReg0));
	}

	return WettCurrentSG;
}

/*
 * @brief Wetting current levels, SG8 - SG13.
 *
 * @param WettCurrent sgx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
 *                      	software at any time in normal mode.
 *                          	0 - 2.0 mA,
 *                          	1 - 8.0 mA,
 *                          	2 - 12.0 mA,
 *                          	3 - 16.0 mA (default).
 *
 * @retval MC33CD1020_WettCurrentSGReg1_t type.
 *
 **/
 MC33CD1020_WettCurrentSGReg1_t MC33CD1020_WettCurrentSGReg1(uint8_t rw_bit, 
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	static MC33CD1020_WettCurrentSGReg1_t WettCurrentSG = { 0 };

	WettCurrentSG.WettCurrentSGReg1 = 0;
	WettCurrentSG.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SGR1 | rw_bit);
	WettCurrentSG.SG8 = sg8;
	WettCurrentSG.SG9 = sg9;
	WettCurrentSG.SG10 = sg10;
	WettCurrentSG.SG11 = sg11;
	WettCurrentSG.SG12 = sg12;
	WettCurrentSG.SG13 = sg13;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSG.WettCurrentSGReg1, sizeof(WettCurrentSG.WettCurrentSGReg1));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSG.WettCurrentSGReg1, sizeof(WettCurrentSG.WettCurrentSGReg1));
	}

	return WettCurrentSG;
}

/*
* @brief Each switch input has a designated 20 ms timer. The timer starts when the specific switch input crosses the
*			comparator threshold. When the 20 ms timer expires, the contact current is reduced from the configured wetting
*			current (16 mA) to the Sustain current (2mA). The wetting current is defined to be an elevated level that reduces to
*			the lower sustain current level after the timer has expired. With multiple wetting current timers disabled, power
*			dissipation for the IC must be considered.
*			The MCU may change or update the continuous wetting current register via software at any time in normal
*			mode. This allows the MCU to control the amount of time wetting current is applied to the switch contact.
*			Programming the continuous wetting current bit to logic [0] operates normally with a higher wetting current
*			followed by sustain current after 20 ms (pulsed Wetting current operation). Programming to logic [1] enables
*			the continuous wetting current and results in a full time wetting current level. The continuous wetting
*			current register defaults to 0 (pulse wetting current operation).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param contwett_current spx	: 	0 - normal operation with a higher wetting current followed by sustein current after 20 ms (default),
 * 									1 - operation with a full time wetting current.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSP_t MC33CD1020_ContWettCurrentSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {
	
	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t ContWettCurrentSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&ContWettCurrentSP, (MC33CD1020_WETTCURRENT_CONTEN_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		ContWettCurrentSP.UniSPReg = ReadBack;
	}

	return ContWettCurrentSP;
 }

/*
 * @brief Each switch input has a designated 20 ms timer. The timer starts when the specific switch input crosses the
 *			comparator threshold. When the 20 ms timer expires, the contact current is reduced from the configured wetting
 *			current (16 mA) to the Sustain current (2mA). The wetting current is defined to be an elevated level that reduces to
 *			the lower sustain current level after the timer has expired. With multiple wetting current timers disabled, power
 *			dissipation for the IC must be considered.
 *			The MCU may change or update the continuous wetting current register via software at any time in normal
 *			mode. This allows the MCU to control the amount of time wetting current is applied to the switch contact.
 *			Programming the continuous wetting current bit to logic [0] operates normally with a higher wetting current
 *			followed by sustain current after 20 ms (pulsed Wetting current operation). Programming to logic [1] enables
 *			the continuous wetting current and results in a full time wetting current level. The continuous wetting
 *			current register defaults to 0 (pulse wetting current operation).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param contwett_current sgx	: 	0 - normal operation with a higher wetting current followed by sustein current after 20 ms (default),
 * 									1 - operation with a full time wetting current.
 *
 * @retval MC33CD1020_UniSG_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_ContWettCurrentSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3,
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11, 
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;												
	static MC33CD1020_UniSG_t ContWettCurrentSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&ContWettCurrentSG, (MC33CD1020_WETTCURRENT_CONTEN_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		ContWettCurrentSG.UniSGReg = ReadBack;
	}

	return ContWettCurrentSG;
}

 /*
 * @brief The interrupt register defines the inputs that are allowed to Interrupt the CD1020 normal mode. Programming
 *			the interrupt bit to logic [0] disables the specific input from generating an interrupt. Programming the interrupt bit
 *			to logic [1] enables the specific input to generate an interrupt with switch change of state The MCU may change
 * 			or update the interrupt register via software at any time in normal mode. The Interrupt register defaults to logic
 *			[1] (Interrupt enabled).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param interrupt enable spx	: 	0 - disables the specific input from generating an interrupt,
 * 									1 - enables the specific input to generate an interrupt with switch change of state (default).
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSP_t MC33CD1020_IntEnSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {
	
	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t IntEnSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&IntEnSP, (MC33CD1020_INTEN_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		IntEnSP.UniSPReg = ReadBack;
	}

	return IntEnSP;
 }

/*
 * @brief The interrupt register defines the inputs that are allowed to Interrupt the CD1020 normal mode. Programming
 *			the interrupt bit to logic [0] disables the specific input from generating an interrupt. Programming the interrupt bit
 *			to logic [1] enables the specific input to generate an interrupt with switch change of state The MCU may change
 * 			or update the interrupt register via software at any time in normal mode. The Interrupt register defaults to logic
 *			[1] (Interrupt enabled).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param Interrupt enable sgx	: 	0 - disables the specific input from generating an interrupt,
 * 									1 - enables the specific input to generate an interrupt with switch change of state (default).
 *
 * @retval MC33CD1020_UniSG_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_IntEnSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;												
	static MC33CD1020_UniSG_t IntEnSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&IntEnSG, (MC33CD1020_INTEN_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		IntEnSG.UniSGReg = ReadBack;
	}

	return IntEnSG;
 }

/*
 * @brief Low power mode configuration.
 *
 * @param WettCurrent : The device has poll[3-0] to set the normal polling rate for the IC. The polling rate is the time between polling
 *                      events. The current sources become active at this time for a time of tACTIVESGPOLLING or tACTIVESBPOLLING for SG
 *                      or SB channels respectively.
 *
 * @param rw_bit 	: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param poll_rate	:  	0 - 3.0ms,  1 - 6.0ms,  2 - 12ms,  3 - 24ms,
 *                    	4 - 48ms,   5 - 68ms,   6 - 76ms,  7 - 128ms,                      
 *                    	8 - 32ms,   9 - 36ms,   10 - 40ms, 11 - 44ms,
 *                    	12 - 52ms,  13 - 56ms,  14 - 60ms, 15 - 64ms (default).
 *
 * @retval MC33CD1020_LowPwrMode_t type.
 *
 **/
MC33CD1020_LowPwrMode_t MC33CD1020_LowPwrModeCfg(uint8_t rw_bit, uint8_t poll_rate) {

	static MC33CD1020_LowPwrMode_t LowPwrMode = { 0 };

	LowPwrMode.REG_ADDR_RW = (MC33CD1020_LOWPWRMODE_CFG | rw_bit);
	LowPwrMode.POLL3_0 = poll_rate;

	mc33cd1020_inst.spi_tx((uint8_t *)&LowPwrMode.LowPwrModeReg, sizeof(LowPwrMode.LowPwrModeReg));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&LowPwrMode.LowPwrModeReg, sizeof(LowPwrMode.LowPwrModeReg));
	}

	return LowPwrMode;
 }

/*
 * @brief The wake-up register defines the inputs that are allowed to wake the CD1020 from low-power mode.
 *			Programming the wake-up bit to logic [0] disables the specific input from waking the IC (Table 25). Programming
 *			the wake-up bit to logic [1] enables the specific input to wake-up with switch change of state The MCU may
 *			change or update the wake-up register via software at any time in normal mode. The Wake-up register defaults
 *			to logic [1] (wake-up enabled). If all channels (SG and SB) have the Wake-up bit disabled, the device disables
 *			the polling timer to reduce the current consumption during low-power mode.
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param wake up enable spx	: 	0 - disables the specific input from waking the IC,
 * 									1 - enables the specific input to wake-up with switch change of state (default).
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSP_t MC33CD1020_WakeUpEnSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t WakwUpEnSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&WakwUpEnSP, (MC33CD1020_WAKEUPEN_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		WakwUpEnSP.UniSPReg = ReadBack;
	}

	return WakwUpEnSP;
 }

 /*
 * @brief The wake-up register defines the inputs that are allowed to wake the CD1020 from low-power mode.
 *			Programming the wake-up bit to logic [0] disables the specific input from waking the IC (Table 25). Programming
 *			the wake-up bit to logic [1] enables the specific input to wake-up with switch change of state The MCU may
 *			change or update the wake-up register via software at any time in normal mode. The Wake-up register defaults
 *			to logic [1] (wake-up enabled). If all channels (SG and SB) have the Wake-up bit disabled, the device disables
 *			the polling timer to reduce the current consumption during low-power mode.
 *
 * @param rw_bit 		: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param wakeupen sgx	: 	0 - disables the specific input from waking the IC,
 * 							1 - enables the specific input to wake-up with switch change of state (default).
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSG_t MC33CD1020_WakeUpEnSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSG_t WakeUpEnSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&WakeUpEnSG, (MC33CD1020_WAKEUPEN_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		WakeUpEnSG.UniSGReg = ReadBack;
	}

	return WakeUpEnSG;
 }

 /*
 * @brief The comparator only register allows the input comparators to be active during LPM with no polling current. In
 *			this case, the inputs can receive a digital signal on the order of the LPM clock cycle and wake-up on a change
 *			of state. This register is intended to be used for signals that are driven by an external chip and drive to 5.0 V.
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param Comparator only spx	: 	0 - comparators are active during LPM mode with polling current (default),
 * 									1 - comparators are active during LPM mode with no polling current.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSP_t MC33CD1020_CmpOnlySP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t CmpOnlySP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&CmpOnlySP, (MC33CD1020_LPMCMPONLY_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		CmpOnlySP.UniSPReg = ReadBack;
	}

	return CmpOnlySP;
 }

  /*
 * @brief The comparator only register allows the input comparators to be active during LPM with no polling current. In
 *			this case, the inputs can receive a digital signal on the order of the LPM clock cycle and wake-up on a change
 *			of state. This register is intended to be used for signals that are driven by an external chip and drive to 5.0 V.
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param Comparator only sgx	: 	0 - comparators are active during LPM mode with polling current (default),
 * 									1 - comparators are active during LPM mode with no polling current.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_CmpOnlySG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSG_t CmpOnlySP = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&CmpOnlySP, (MC33CD1020_LPMCMPONLY_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		CmpOnlySP.UniSGReg = ReadBack;
	}

	return CmpOnlySP;
 }

/*
 * @brief The CD1020 is able to use different voltage thresholds to wake-up from LPM. When configured as SG, a Logic
 *			[0] means the input will use the LPM delta voltage threshold to determine the state of the switch. A Logic [1]
 *			means the input uses the Normal threshold (VICTHR) to determine the state of the switch. When configured as
 *			an SB, it only uses the 4.0 V threshold regardless of the status of the LPM voltage threshold bit. The user must
 *			ensure that the correct current level is set to allow the crossing of the normal mode threshold (typically 4.0 V).
 *
 * @param rw_bit 					: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param LPM voltage threshold spx	: 	0 - the input will use the LPM delta voltage threshold to determine the state of the switch (SG only) (default),
 * 										1 - the input will use the Normal threshold (VICTHR) to determine the state of the switch (SG only).
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSP_t MC33CD1020_LpmVoltageThrSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t LpmVoltageThrSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&LpmVoltageThrSP, (MC33CD1020_LPMVOLTAGETHR_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		LpmVoltageThrSP.UniSPReg = ReadBack;
	}

	return LpmVoltageThrSP;
 }

/*
 * @brief This means the input uses the LPM delta voltage threshold to determine the state of the switch. A Logic [1]
 *			means the input uses the Normal threshold to determine the state of the switch. The user must ensure that the
 *			correct current level is set to allow the crossing of the normal mode threshold (typically 4.0 V)

 *
 * @param rw_bit 					: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param LPM voltage threshold sgx	: 	0 - the input will use the LPM delta voltage threshold to determine the state of the switch (default),
 * 										1 - the input will use the Normal threshold (VICTHR) to determine the state of the switch.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_LpmVoltageThrSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSG_t LpmVoltageThrSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&LpmVoltageThrSG, (MC33CD1020_LPMVOLTAGETHR_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		LpmVoltageThrSG.UniSGReg = ReadBack;
	}

	return LpmVoltageThrSG;
 }

 /*
 * @brief The normal polling current for LPM is 2.2 mA for SB channels and 1.0 mA for SG channels, A logic [0] selects
 *			the normal polling current for each individual channel. The user may choose to select the IWET current value
 *			as defined in the wetting current level registers by writing a Logic [1] on this bit; this will result in higher LPM
 *			currents but may be used in cases when a higher polling current is needed.
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param Polling current spx	: 	0 - normal polling current for each individual channel (default),
 * 									1 - IWET polling current for each individual channel.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSP_t MC33CD1020_PollCurrentSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSP_t PollCurrentSP = { 0 };

	ReadBack = MC33CD1020_UniRegSP(&PollCurrentSP, (MC33CD1020_POLLCURRENTCFG_SP | rw_bit), sp0, sp1, sp2, sp3, sp4, sp5, sp6, sp7).UniSPReg;

	if(!rw_bit) {
		PollCurrentSP.UniSPReg = ReadBack;
	}

	return PollCurrentSP;
 }

/*
 * @brief A Logic [0] selects the normal polling current for LPM = 1.0 mA. The user may choose to select the IWET current
 *			value as defined in the wetting current registers for LPM by writing a Logic [1] in this bit; this results in higher
 *			LPM currents but may be used in cases when a higher polling current is needed.

 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param Polling current sgx 	: 	0 - normal polling current for each individual channel (default),
 * 									1 - IWET polling current for each individual channel.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_PollCurrentSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	uint32_t ReadBack = 0;
	static MC33CD1020_UniSG_t PollCurrentSG = { 0 };

	ReadBack = MC33CD1020_UniRegSG(&PollCurrentSG, (MC33CD1020_POLLCURRENTCFG_SG | rw_bit), sg0, sg1, sg2, sg3, sg4, sg5, sg6, sg7, sg8,  sg9, sg10, sg11, sg12, sg13).UniSGReg;

	if(!rw_bit) {
		PollCurrentSG.UniSGReg = ReadBack;
	}

	return PollCurrentSG;
 }

 /*
 * @brief Low-power mode (LPM) is used to reduce system quiescent currents. Low-power mode may be entered only by
 *			sending the low-power command. When returning to normal mode, all register settings is maintained.
 *			The Enter Low-power mode register is write only and has the effect of going to LPM and beginning operation as
 *			selected (polling, interrupt timer). When returning form low-power mode, the first SPI transaction will return the
 *			Fault Status and the intflg bit set to high, as well as the actual status of the Input pins.
 *
 **/
void MC33CD1020_EnterLpmMode(void) {

	static MC33CD1020_UniSP_t EnterLpmMode = { 0 };

	EnterLpmMode.REG_ADDR_RW = (MC33CD1020_ENTERLPM | MC33CD1020_WRITE_SEQ);
	EnterLpmMode.SP0 = 1;

	mc33cd1020_inst.spi_tx((uint8_t *)&EnterLpmMode.UniSPReg, sizeof(EnterLpmMode.UniSPReg));
 }

  /*
 * @brief Wakes up the chip by falling edge of WAKA_B pin. 
 *
 **/
void MC33CD1020_WakeUp(void) {

	MC33CD1020_WAKE_B_GPIO_Port->BSRR = MC33CD1020_WAKE_B_Pin << 16;
	__NOP();
	MC33CD1020_WAKE_B_GPIO_Port->BSRR = MC33CD1020_WAKE_B_Pin;
 }

/*
 * @brief The analog voltage on switch inputs may be read by the MCU using the analog command (Table 34). Internal
 *			to the CD1020 is a 22-to-1 analog multiplexer. The voltage present on the selected input pin is buffered
 *          and made available on the AMUX output pin. The AMUX output pin is clamped to a maximum of VDDQ volts
 *          regardless of the higher voltages present on the input pin. After an input has been selected as the analog, the
 *          corresponding bit in the next MISO data stream is logic [0].
 *
 * @param amux_current : 0 - hi Z input impedance (default),
 *						 1 - IWET.
 *	
 * @param amux_channel : 0 - no input selected (default),
 *
 *                       1 - SG0, 2 - SG1, 3 - SG2, 4  - SG3,                    
 *                       5 - SG4, 6 - SG5, 7 - SG6, 8  - SG7, 
 *                       9 - SG8, 10 - SG9, 11 - SG10, 12 - SG11, 
 *                       13 - SG12, 14 - SG13.
 *                    
 *                       5 - SP0,  16 - SP1,  17 - SP2,  18 - SP3,
 *                       19 - SP4,  20 - SP5,  21 - SP6,  22 - SP7,
 *
 * @retval MC33CD1020_AmuxCtrl_t type. 
 *
 **/
MC33CD1020_AmuxCtrl_t MC33CD1020_AMUXCtrl(uint8_t rw_bit, uint8_t amux_current, uint8_t amux_channel) {

	static MC33CD1020_AmuxCtrl_t AmuxControl = { 0 };

	AmuxControl.REG_ADDR_RW = (MC33CD1020_AMUXCHSEL_SPI | rw_bit);
	AmuxControl.ASETT0 = amux_current;
	AmuxControl.ASEL5_0 = amux_channel;

	mc33cd1020_inst.spi_tx((uint8_t *)&AmuxControl.AmuxCtrlReg, sizeof(AmuxControl.AmuxCtrlReg));
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&AmuxControl.AmuxCtrlReg, sizeof(AmuxControl.AmuxCtrlReg));
	}

	return AmuxControl;
 }

 /*
 * @brief The Read switch status register is used to determine the state of each of the inputs and is read only. All of the inputs (SGn and SPn) are returned after the next
 *			command is sent. A Logic [1] means the switch is closed while a Logic [0] is an open switch.
 *			Included in the status register are two more bits, the Fault Status bit and intflg bit. The Fault Status bit is a combination of the extended status bits and the wetting
 *			current fault bits. If any of these bits are set, the Fault Status bit is set. The intflg bit is set when an interrupt occurs on this device.
 *			After POR, both the Fault Status bit and the intflg bit are set high to indicate an interrupt due to a POR occurring. The intflg bit will be cleared upon reading the
 *			Read Switch Status register, and the Fault Status bit will remain high until the Fault status register is read and thus the POR fault bit and all other fault flags are
 *			cleared.
 *			The Fault Status and Intflg bits are semi-global flags, if a fault or an interrupt occurs, these bit will be returned after writing or reading any command, except for the
 *			SPICheck and the Wetting Current configuration registers, which use those bits to set/display the device configuration.
 *
 * @retval MC33CD1020_SwStatusRead_t type. 
 * 				0 - no fault, no change of state, open switch,
 *				1 - indicates a fault has occurred and should be viewed in the fault status registe, change of state detected, closed switch.
 *
 **/
MC33CD1020_SwStatusRead_t MC33CD1020_ReadSWStatus(void) {

	static MC33CD1020_SwStatusRead_t ReadSwStatus = { 0 };

	ReadSwStatus.SwStatusReg = 0;
	ReadSwStatus.REG_ADDR_RW = (MC33CD1020_READSWSTAT | MC33CD1020_READ_SEQ);


	mc33cd1020_inst.spi_tx((uint8_t *)&ReadSwStatus.SwStatusReg, sizeof(ReadSwStatus.SwStatusReg));
	mc33cd1020_inst.spi_rx((uint8_t*)&ReadSwStatus.SwStatusReg, sizeof(ReadSwStatus.SwStatusReg));

	return ReadSwStatus;
 }

  /*
 * @brief To read the fault status bits, the user should first send a message to the IC with the fault status register address
 *			followed by any given second command. The MISO response from the second command will contain the fault
 *			flags information.
 *
 * @retval MC33CD1020_FaultStatus_t type. 
 *			POR 			- Reports a POR event occurred
 *								0 - flag read (SPI),
 *								1 - Voltage at VBATP pin dropped below VBATP(POR) voltage.
 *			SPI_WAKE 		- Part awaken via a SPI message
 *								0 - flag read (SPI),
 *								1 - SPI message wakes the IC from LPM.
 * 			WAKEB_WAKE 		- Part awakens via an external WAKE_B falling edge
 *								0 - flag read (SPI),
 *								1 - External WAKE_B falling edge seen.
 * 			INTB_WAKE 		- Part awakens via an external INT_B falling edge
 *								0 - flag read (SPI),
 *								1 - INT_B Wakes the part from LPM (external falling edge).
 * 			OT 				- Tlim event occurred on the IC
 *								0 - Temperature drops below thermal warning threshold + hysteresis and flag read (SPI),
 *								1 - Tlim warning threshold is passed.
 * 			TEMP_FLG 		- Temperature warning to note elevated IC temperature
 *								0 - Temperature drops below thermal warning threshold + hysteresis and flag read (SPI),
 *								1 - tLIM warning threshold is passed.
 * 			OV 				- Report that the voltage on VBATP was higher than OV threshold
 *								0 - Overvoltage condition is over and flag read (SPI),
 *								1 - Voltage at VBATP rises above overvoltage threshold.
 * 			UV 				- Reports that low VBATP voltage was in undervoltage range
 *								0 - VBATP rises above UV level and flag read (SPI),
 *								1 - Voltage drops below UV level.
 * 			HASH_FAULT 		- SPI register and hash mismatch
 *								0 - No mismatch and SPI flag read,
 *								1 - Mismatch between SPI registers and hash.
 * 			SPI_ERR			- Any SPI error generates a bit (Wrong address, incorrect modulo)
 *								0 - Read fault status register and no SPI errors,
 *								1 - SPI message error.
 * 			INT_FLG 		- Reports that an Interrupt has occurred, user should read the status register to determine cause
 *								0 - Clear of fault or read of Status register,		
*								1 - Various (SGx change of state, SPx change of state, Extended status bits).
 * 			FAULT_STATUS 	- unused. 	
 *
 **/
MC33CD1020_FaultStatus_t MC33CD1020_ReadFaultStatus(void) {

	static MC33CD1020_FaultStatus_t FaultStatus = { 0 };

	FaultStatus.FaultStatusReg = 0;
	FaultStatus.REG_ADDR_RW = (MC33CD1020_FAULTSTAT | MC33CD1020_READ_SEQ);


	mc33cd1020_inst.spi_tx((uint8_t *)&FaultStatus.FaultStatusReg, sizeof(FaultStatus.FaultStatusReg));
	mc33cd1020_inst.spi_rx((uint8_t*)&FaultStatus.FaultStatusReg, sizeof(FaultStatus.FaultStatusReg));

	return FaultStatus;
 }

/*
 * @brief The MCU may request an Interrupt pulse of duration 100 μs by sending the Interrupt request command. After
 *			an Interrupt request command, the CD1020 returns the Interrupt request command word, as well as the Fault
 *			status and INTflg bits set if a fault/interrupt event occurred. Sending an interrupt request command does not set
 * 			the INTflg bit itself.
 *
 **/
void MC33CD1020_InterruptRequest(void) {

	static MC33CD1020_UniSP_t InterruptRequest = { 0 };

	InterruptRequest.REG_ADDR_RW = (MC33CD1020_INT_PULSE_REQ | MC33CD1020_WRITE_SEQ);
	InterruptRequest.SP0 = 1;

	mc33cd1020_inst.spi_tx((uint8_t *)&InterruptRequest.UniSPReg, sizeof(InterruptRequest.UniSPReg));
 }

/*
 * @brief Writing to this register causes all of the SPI registers to reset.
 *
 **/
void MC33CD1020_Reset(void) {

	static MC33CD1020_UniSP_t Reset = { 0 };

	Reset.REG_ADDR_RW = (MC33CD1020_RST | MC33CD1020_WRITE_SEQ);
	Reset.SP0 = 1;

	mc33cd1020_inst.spi_tx((uint8_t *)&Reset.UniSPReg, sizeof(Reset.UniSPReg));
 }


/*
 * @brief Universal SP R/W function.
 *
 * @param rw_bit 		: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param wakeupen_spx	: SP arguments.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSP_t MC33CD1020_UniRegSP(MC33CD1020_UniSP_t* uniregSP, uint8_t addr_rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											) {

	static MC33CD1020_UniSP_t RetValSP = { 0 };

	uniregSP->REG_ADDR_RW = addr_rw_bit;
	uniregSP->SP0 = sp0;
	uniregSP->SP1 = sp1;
	uniregSP->SP2 = sp2;
	uniregSP->SP3 = sp3;
	uniregSP->SP4 = sp4;
	uniregSP->SP5 = sp5;
	uniregSP->SP6 = sp6;
	uniregSP->SP7 = sp7;

	mc33cd1020_inst.spi_tx((uint8_t *)&uniregSP->UniSPReg, sizeof(uniregSP->UniSPReg));
	if(!(addr_rw_bit & 0x01)) {
		mc33cd1020_inst.spi_rx((uint8_t*)&RetValSP.UniSPReg, sizeof(RetValSP.UniSPReg));
	}

	return RetValSP;
 }

 /*
 * @brief Universal SG R/W function.
 *
 * @param rw_bit 		: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param wakeupen_sgx	: SG arguments.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_UniRegSG(MC33CD1020_UniSG_t* uniregSG, uint8_t addr_rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											) {

	static MC33CD1020_UniSG_t RetValSG = { 0 };

	uniregSG->REG_ADDR_RW = addr_rw_bit;
	uniregSG->SG0 = sg0;
	uniregSG->SG1 = sg1;
	uniregSG->SG2 = sg2;
	uniregSG->SG3 = sg3;
	uniregSG->SG4 = sg4;
	uniregSG->SG5 = sg5;
	uniregSG->SG6 = sg6;
	uniregSG->SG7 = sg7;
	uniregSG->SG8 = sg8;
	uniregSG->SG9 = sg9;
	uniregSG->SG10 = sg10;
	uniregSG->SG11 = sg11;
	uniregSG->SG12 = sg12;
	uniregSG->SG13 = sg13;


	mc33cd1020_inst.spi_tx((uint8_t *)&uniregSG->UniSGReg, sizeof(uniregSG->UniSGReg));
	if(!(addr_rw_bit & 0x01)) {
		mc33cd1020_inst.spi_rx((uint8_t *)&RetValSG.UniSGReg, sizeof(RetValSG.UniSGReg));
	}

	return RetValSG;
 }

 /* Hardware dependent functions. */



#ifndef HARD_SPI_NSS
/*
 * @brief SPI chip select. You must disable hardware SPI_NSS to use this function.
 * 
 * @param gpio      : MC33CD1020_CS_GPIO_Port.
 * @param gpio_pin  : MC33CD1020_CS_Pin.
 * @param state     : GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void MC33CD1020_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
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
static void MC33CD1020_Tx(uint8_t *pData, uint8_t size) {

	#ifndef HARD_SPI_NSS
	MC33CD1020_SPI_CS(MC33CD1020_CS_GPIO_Port, MC33CD1020_CS_Pin, 0);
    #endif
	HAL_SPI_Transmit_IT(MC33CD1020_SpiInst, pData, size);
    #ifndef HARD_SPI_NSS
	MC33CD1020_TxCheck();
	MC33CD1020_SPI_CS(MC33CD1020_CS_GPIO_Port, MC33CD1020_CS_Pin, 1);
    #endif
}

/*
 * @brief SPI Rx data.
 * 
 *
 **/
static void MC33CD1020_Rx(uint8_t *pData, uint8_t size) {
	
	#ifndef HARD_SPI_NSS
	MC33CD1020_SPI_CS(MC33CD1020_CS_GPIO_Port, MC33CD1020_CS_Pin, 0);
    #endif
	HAL_SPI_Receive_IT(MC33CD1020_SpiInst, pData, size);
    #ifndef HARD_SPI_NSS
	MC33CD1020_RxCheck();
	MC33CD1020_SPI_CS(MC33CD1020_CS_GPIO_Port, MC33CD1020_CS_Pin, 1);
    #endif
}


 /*
 * @brief Transmit complete check.
 *
**/
static uint8_t MC33CD1020_TxCheck(void) {

	mc33cd1020_inst.delay(5);
	return 0;
}

 /*
 * @brief Receive complete check.
 *
**/
static uint8_t MC33CD1020_RxCheck(void) {

	mc33cd1020_inst.delay(5);
	return 0;
}

