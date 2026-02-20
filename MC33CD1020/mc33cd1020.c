/*
 * @brief C for MC33CD1020.
 * Created 02.12.26 by asw3005. 
 *
 **/

#include "mc33cd1020.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
//#include <stdint.h>


#define HARD_SPI_NSS

 /* External variables. */
extern SPI_HandleTypeDef hspi2;

/* Private variables. */
static SPI_HandleTypeDef* MC33CD1020_SpiInst = &hspi2;


/* Private function prototypes. */
static uint8_t MC33CD1020_TxCheck(void); 
static uint8_t MC33CD1020_RxCheck(void); 
static void MC33CD1020_Tx(uint8_t *pData, uint8_t size);
static void MC33CD1020_Rx(uint8_t *pData, uint8_t size);

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

	static uint32_t SPICheckData = { 0 };

	mc33cd1020_inst.spi_tx((uint8_t *)&SPICheckData, sizeof(SPICheckData));
	MC33CD1020_TxCheck();
	mc33cd1020_inst.spi_rx((uint8_t*)&SPICheckData, sizeof(SPICheckData));
	MC33CD1020_RxCheck();
	return SPICheckData;
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
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&DevCfg.DevCfgReg, sizeof(DevCfg.DevCfgReg));
		MC33CD1020_RxCheck();
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

	static MC33CD1020_UniSP_t TriStateSP = { 0 };

	TriStateSP.REG_ADDR_RW = (MC33CD1020_TRISTATE_SP | rw_bit);
	TriStateSP.SP0 = sp0;
	TriStateSP.SP1 = sp1;
	TriStateSP.SP2 = sp2;
	TriStateSP.SP3 = sp3;
	TriStateSP.SP4 = sp4;
	TriStateSP.SP5 = sp5;
	TriStateSP.SP6 = sp6;
	TriStateSP.SP7 = sp7;

	mc33cd1020_inst.spi_tx((uint8_t *)&TriStateSP.UniSPReg, sizeof(TriStateSP.UniSPReg));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&TriStateSP.UniSPReg, sizeof(TriStateSP.UniSPReg));
		MC33CD1020_RxCheck();
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

	static MC33CD1020_UniSG_t TriStateSG = { 0 };

	TriStateSG.REG_ADDR_RW = (MC33CD1020_TRISTATE_SG | rw_bit);
	TriStateSG.SG0 = sg0;
	TriStateSG.SG1 = sg1;
	TriStateSG.SG2 = sg2;
	TriStateSG.SG3 = sg3;
	TriStateSG.SG4 = sg4;
	TriStateSG.SG5 = sg5;
	TriStateSG.SG6 = sg6;
	TriStateSG.SG7 = sg7;
	TriStateSG.SG8 = sg8;
	TriStateSG.SG9 = sg9;
	TriStateSG.SG10 = sg10;
	TriStateSG.SG11 = sg11;
	TriStateSG.SG12 = sg12;
	TriStateSG.SG13 = sg13;


	mc33cd1020_inst.spi_tx((uint8_t *)&TriStateSG.UniSGReg, sizeof(TriStateSG.UniSGReg));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&TriStateSG.UniSGReg, sizeof(TriStateSG.UniSGReg));
		MC33CD1020_RxCheck();
	}

	return TriStateSG;
 }

/*
 * @brief Wetting current levels, SP0 - SP7.
 *
 * @param WettCurrent_spx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
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
											uint8_t wettcurrent_sp0, uint8_t wettcurrent_sp1, uint8_t wettcurrent_sp2, uint8_t wettcurrent_sp3,
											uint8_t wettcurrent_sp4, uint8_t wettcurrent_sp5, uint8_t wettcurrent_sp6, uint8_t wettcurrent_sp7
											) {

	static MC33CD1020_WettCurrentSP_t WettCurrentSP = { 0 };

	WettCurrentSP.WettCurrentSPReg = 0;
	WettCurrentSP.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SP | rw_bit);
	WettCurrentSP.SP0 = wettcurrent_sp0;
	WettCurrentSP.SP1 = wettcurrent_sp1;
	WettCurrentSP.SP2 = wettcurrent_sp2;
	WettCurrentSP.SP3 = wettcurrent_sp3;
	WettCurrentSP.SP4 = wettcurrent_sp4;
	WettCurrentSP.SP5 = wettcurrent_sp5;
	WettCurrentSP.SP6 = wettcurrent_sp6;
	WettCurrentSP.SP7 = wettcurrent_sp7;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSP.WettCurrentSPReg, sizeof(WettCurrentSP.WettCurrentSPReg));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSP.WettCurrentSPReg, sizeof(WettCurrentSP.WettCurrentSPReg));
		MC33CD1020_RxCheck();
	}

	return WettCurrentSP;
}

/*
 * @brief Wetting current levels, SG0 - SG7.
 *
 * @param WettCurrent_sgx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
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
											uint8_t wettcurrent_sg0, uint8_t wettcurrent_sg1, uint8_t wettcurrent_sg2, uint8_t wettcurrent_sg3,
											uint8_t wettcurrent_sg4, uint8_t wettcurrent_sg5, uint8_t wettcurrent_sg6, uint8_t wettcurrent_sg7
											) {

	static MC33CD1020_WettCurrentSGReg0_t WettCurrentSG = { 0 };

	WettCurrentSG.WettCurrentSGReg0 = 0;
	WettCurrentSG.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SGR0 | rw_bit);
	WettCurrentSG.SG0 = wettcurrent_sg0;
	WettCurrentSG.SG1 = wettcurrent_sg1;
	WettCurrentSG.SG2 = wettcurrent_sg2;
	WettCurrentSG.SG3 = wettcurrent_sg3;
	WettCurrentSG.SG4 = wettcurrent_sg4;
	WettCurrentSG.SG5 = wettcurrent_sg5;
	WettCurrentSG.SG6 = wettcurrent_sg6;
	WettCurrentSG.SG7 = wettcurrent_sg7;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSG.WettCurrentSGReg0, sizeof(WettCurrentSG.WettCurrentSGReg0));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSG.WettCurrentSGReg0, sizeof(WettCurrentSG.WettCurrentSGReg0));
		MC33CD1020_RxCheck();
	}

	return WettCurrentSG;
}

/*
 * @brief Wetting current levels, SG8 - SG13.
 *
 * @param WettCurrent_sgx : The IC contains configurable wetting currents (Default = 16 mA). The MCU may change or update the wetting current register via
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
											uint8_t wettcurrent_sg8, uint8_t wettcurrent_sg9, uint8_t wettcurrent_sg10, uint8_t wettcurrent_sg11,
											uint8_t wettcurrent_sg12, uint8_t wettcurrent_sg13
											) {

	static MC33CD1020_WettCurrentSGReg1_t WettCurrentSG = { 0 };

	WettCurrentSG.WettCurrentSGReg1 = 0;
	WettCurrentSG.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_LVL_SGR1 | rw_bit);
	WettCurrentSG.SG8 = wettcurrent_sg8;
	WettCurrentSG.SG9 = wettcurrent_sg9;
	WettCurrentSG.SG10 = wettcurrent_sg10;
	WettCurrentSG.SG11 = wettcurrent_sg11;
	WettCurrentSG.SG12 = wettcurrent_sg12;
	WettCurrentSG.SG13 = wettcurrent_sg13;

	mc33cd1020_inst.spi_tx((uint8_t *)&WettCurrentSG.WettCurrentSGReg1, sizeof(WettCurrentSG.WettCurrentSGReg1));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&WettCurrentSG.WettCurrentSGReg1, sizeof(WettCurrentSG.WettCurrentSGReg1));
		MC33CD1020_RxCheck();
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
*			the continuous wetting current (Table 19) and results in a full time wetting current level. The continuous wetting
*			current register defaults to 0 (pulse wetting current operation).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param contwett_current_spx	: 	0 - normal operation with a higher wetting current followed by sustein current after 20 ms (default),
 * 									1 - operation with a full time wetting current.
 *
 * @retval MC33CD1020_UniSP_t type.
 *
 **/
 MC33CD1020_UniSP_t MC33CD1020_ContWettCurrentSP(uint8_t rw_bit, 
											uint8_t contwett_current_sp0, uint8_t contwett_current_sp1, uint8_t contwett_current_sp2, uint8_t contwett_current_sp3, 
											uint8_t contwett_current_sp4, uint8_t contwett_current_sp5, uint8_t contwett_current_sp6, uint8_t contwett_current_sp7
											) {

	static MC33CD1020_UniSP_t ContWettCurrentSP = { 0 };

	ContWettCurrentSP.REG_ADDR_RW = (MC33CD1020_WETTCURRENT_CONTEN_SP | rw_bit);
	ContWettCurrentSP.SP0 = contwett_current_sp0;
	ContWettCurrentSP.SP1 = contwett_current_sp1;
	ContWettCurrentSP.SP2 = contwett_current_sp2;
	ContWettCurrentSP.SP3 = contwett_current_sp3;
	ContWettCurrentSP.SP4 = contwett_current_sp4;
	ContWettCurrentSP.SP5 = contwett_current_sp5;
	ContWettCurrentSP.SP6 = contwett_current_sp6;
	ContWettCurrentSP.SP7 = contwett_current_sp7;

	mc33cd1020_inst.spi_tx((uint8_t *)&ContWettCurrentSP.UniSPReg, sizeof(ContWettCurrentSP.UniSPReg));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&ContWettCurrentSP.UniSPReg, sizeof(ContWettCurrentSP.UniSPReg));
		MC33CD1020_RxCheck();
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
*			the continuous wetting current (Table 19) and results in a full time wetting current level. The continuous wetting
*			current register defaults to 0 (pulse wetting current operation).
 *
 * @param rw_bit 				: MC33CD1020_READ_SEQ or MC33CD1020_WRITE_SEQ. 
 * @param contwett_current_sgx	: 	0 - normal operation with a higher wetting current followed by sustein current after 20 ms (default),
 * 									1 - operation with a full time wetting current.
 *
 * @retval MC33CD1020_UniSG_t type.
 *
 **/
MC33CD1020_UniSG_t MC33CD1020_ContWettCurrentSG(uint8_t rw_bit, 
											uint8_t contwett_current_sg0, uint8_t contwett_current_sg1,	uint8_t contwett_current_sg2, uint8_t contwett_current_sg3,
											uint8_t contwett_current_sg4, uint8_t contwett_current_sg5, uint8_t contwett_current_sg6, uint8_t contwett_current_sg7,
											uint8_t contwett_current_sg8, uint8_t contwett_current_sg9, uint8_t contwett_current_sg10, uint8_t contwett_current_sg11, 
											uint8_t contwett_current_sg12, uint8_t contwett_current_sg13
											) {

	static MC33CD1020_UniSG_t ContWettCurrentSG = { 0 };

	ContWettCurrentSG.REG_ADDR_RW = (MC33CD1020_TRISTATE_SG | rw_bit);
	ContWettCurrentSG.SG0 = contwett_current_sg0;
	ContWettCurrentSG.SG1 = contwett_current_sg1;
	ContWettCurrentSG.SG2 = contwett_current_sg2;
	ContWettCurrentSG.SG3 = contwett_current_sg3;
	ContWettCurrentSG.SG4 = contwett_current_sg4;
	ContWettCurrentSG.SG5 = contwett_current_sg5;
	ContWettCurrentSG.SG6 = contwett_current_sg6;
	ContWettCurrentSG.SG7 = contwett_current_sg7;
	ContWettCurrentSG.SG8 = contwett_current_sg8;
	ContWettCurrentSG.SG9 = contwett_current_sg9;
	ContWettCurrentSG.SG10 = contwett_current_sg10;
	ContWettCurrentSG.SG11 = contwett_current_sg11;
	ContWettCurrentSG.SG12 = contwett_current_sg12;
	ContWettCurrentSG.SG13 = contwett_current_sg13;


	mc33cd1020_inst.spi_tx((uint8_t *)&ContWettCurrentSG.UniSGReg, sizeof(ContWettCurrentSG.UniSGReg));
	MC33CD1020_TxCheck();
	if(!rw_bit) {
		mc33cd1020_inst.spi_rx((uint8_t*)&ContWettCurrentSG.UniSGReg, sizeof(ContWettCurrentSG.UniSGReg));
		MC33CD1020_RxCheck();
	}

	return ContWettCurrentSG;
}

/*
 * @brief Low power mode configuration.
 *
 * @param WettCurrent : The device has poll[3-0] to set the normal polling rate for the IC. The polling rate is the time between polling
 *                      events. The current sources become active at this time for a time of tACTIVESGPOLLING or tACTIVESBPOLLING for SG
 *                      or SB channels respectively.
 *
 * @param PollRate :  0 - 3.0ms,  1 - 6.0ms,  2 - 12ms,  3 - 24ms,
 *                    4 - 48ms,   5 - 68ms,   6 - 76ms,  7 - 128ms,                      
 *                    8 - 32ms,   9 - 36ms,   10 - 40ms, 11 - 44ms,
 *                    12 - 52ms,  13 - 56ms,  14 - 60ms, 15 - 64ms.
 *
 **/

/*
 * @brief Low power mode configuration.
 *
 * @param WettCurrent : The analog voltage on switch inputs may be read by the MCU using the analog command (Table 34). Internal
 *                      to the CD1020 is a 22---to-1 analog multiplexer. The voltage present on the selected input pin is buffered
 *                      and made available on the AMUX output pin. The AMUX output pin is clamped to a maximum of VDDQ volts
 *                      regardless of the higher voltages present on the input pin. After an input has been selected as the analog, the
 *                      corresponding bit in the next MISO data stream is logic [0].
 *
 *
 * @param AmuxChannel : 0 - no input selected
 *                      1 - SG0, 2 - SG1, 3 - SG2, 4  - SG3,                    
 *                      5 - SG4, 6 - SG5, 7 - SG6, 8  - SG7, 
 *                      9 - SG8, 10 - SG9, 11 - SG10, 12 - SG11, 
 *                      13 - SG12, 14 - SG13.
 *                    
 *                      15 - SP0,  16 - SP1,  17 - SP2,  18 - SP3,
 *                      19 - SP4,  20 - SP5,  21 - SP6,  22 - SP7,
 **/


 /*
 * @brief Weting current level SP register.
 *
 **/


/*
 * @brief Wetting current level SG register 0.
 *
 **/

  /*
 * @brief Continuous weting current SP register.
 *
 **/

 /*
 * @brief Continuous wetting current SG register.
 *
 **/


/*
 * @brief Interrupt enable SP register.
 *
 **/

  /*
 * @brief Interrupt enable SG register.
 *
 **/

 /*
 * @brief Low power mode register.
 *
 **/


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
	HAL_SPI_Transmit(MC33CD1020_SpiInst, pData, size, 10);
    #ifndef HARD_SPI_NSS
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
	HAL_SPI_Receive(MC33CD1020_SpiInst, pData, size, 10);
    #ifndef HARD_SPI_NSS
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

