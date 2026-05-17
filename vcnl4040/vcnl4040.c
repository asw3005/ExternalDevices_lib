/*
 * @brief Common source for VCNL4040.
 * Created 03.31.26 by asw3005. 
 *
 **/
#include "vcnl4040.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"

 /* External variables. */
extern I2C_HandleTypeDef hi2c3;

 

/* Private variables. */
static I2C_HandleTypeDef* VCNL4040_I2CInst = &hi2c3;

/* Private function prototypes. */
static void VCNL4040_Tx(uint16_t MemAddress, uint8_t *pData, uint16_t Size);
static void VCNL4040_Rx(uint16_t MemAddress, uint8_t *pData, uint16_t Size);

/* General struct. */
static VCNL4040_GInst_t vcnl4040_inst = {

	.i2c_rx = VCNL4040_Tx,
	.i2c_rx = VCNL4040_Rx
};

/*
 * @brief 
 *
**/






/*
 * @brief ALS integration time, persistence, interrupt, and function enable / disable (default 0x0001).
 *
 * @param als_sd 		: 0 - ALS power on, 
 *						  1 - ALS shut down(default).
 * @param als_int_en 	: The VCNL4040 has an interrupt feature for both the PS and ALS channel. The purpose of the interrupt feature is to actively
 *						  inform the host once INT has been triggered. When the interrupt is enabled, the host does not need to continuously read the
 *						  data registers of the sensor, but instead can simply react to the interrupt pin. As long as the host enables ALS interrupt (register:
 *						  ALS_INT_EN) or PS interrupt (register: PS_INT) function, the level of INT pin (pin 6) is pulled low once an interrupt event has
 *						  been triggered. All registers are accessible even if INT is triggered. ALS INT is triggered when ALS value crosses over the value
 *						  set in register: ALS_THDH or below the value set by register: ALS_THDL. PS INT is triggered when the PS value crosses over the value
 * 						  set in register: PS_THDH or falls below the value set in register: PS_THDL. Which of these thresholds to react to, can be set by the 
 *						  PS_INT bits in the register: PS_CONF2.
 *							0 - ALS interrupt disable,
 *						  	1 - ALS interrupt enable.
 * @param als_pers 		: ALS interrupt persistence setting. The ALS INT is triggered once the ALS value is higher or 
 *						  lower than the threshold window. The ALS_PERS (1, 2, 4, 8 times) parameter, sets the amount 
 *						  of consecutive hits needed, in order for an interrupt event to trigger.
 *							0 - 1, 1 - 2, 
 *							2 - 4, 3 - 8.
 * @param als_it 		: ALS integration time setting, longer integration time has higher sensitivity.
 *							0 - 80 ms,  1 - 160 ms, 
 *							2 - 320 ms, 3 - 640 ms. 
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation 
 *
**/
VCNL4040_AlsConf_t VCNL4040_AlsConf(uint8_t als_sd, uint8_t als_int_en, uint8_t als_pers, uint8_t als_it,  uint8_t read_back) {

	static VCNL4040_AlsConf_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_ALS_CONF;
	
	if(read_back) {
		RegData.ALS_SD = als_sd;
		RegData.ALS_INT_EN = als_int_en;
		RegData.ALS_PERS = als_pers;
		RegData.ALS_IT = als_it;
		RegData.RESERVED5_4 = 0;
		RegData.RESERVED15_8 = 0;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData;
}

/*
 * @brief ALS high interrupt threshold (default 0x0000).
 *			To easily define the threshold range, multiply the value of the resolution (lux/step) by the threshold level.
 *			VCNL4040_ALS_CONF[ALS_IT]
 *				0 - 80 ms  - 0.1 lux/step,    range 6553.5 lux,
 *				1 - 160 ms - 0.05 lux/step,   range 3276.8 lux,
 *				2 - 320 ms - 0.025 lux/step,  range 1638.4 lux,
 *				3 - 640 ms - 0.0125 lux/step, range 819.2 lux.
 *
 * @param als_thdh 		: ALS high interrupt threshold.
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation.
 * @retval uint16_t :  ALS high interrupt threshold.
 *
**/
uint16_t VCNL4040_Thdh(uint16_t als_thdh, uint8_t read_back) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_ALS_THDH;
	
	if(read_back) {
		RegData.UniReg = als_thdh;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData.UniReg;
}

/*
 * @brief ALS low interrupt threshold (default 0x0000).
 *			To easily define the threshold range, multiply the value of the resolution (lux/step) by the threshold level.
 *			VCNL4040_ALS_CONF[ALS_IT]
 *				0 - 80 ms  - 0.1 lux/step,    range 6553.5 lux,
 *				1 - 160 ms - 0.05 lux/step,   range 3276.8 lux,
 *				2 - 320 ms - 0.025 lux/step,  range 1638.4 lux,
 *				3 - 640 ms - 0.0125 lux/step, range 819.2 lux.
 *
 * @param als_thdh 		: ALS low interrupt threshold.
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation. 
 * @retval uint16_t :  ALS low interrupt threshold.
 *
**/
uint16_t VCNL4040_Thdl(uint16_t als_thdl, uint8_t read_back) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_ALS_THDL;
	
	if(read_back) {
		RegData.UniReg = als_thdl;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData.UniReg;
}

/*
 * @brief PS duty ratio, integration time, persistence, and PS enable / disable (default 0x0001).
 *
 * @param ps_sd 	: 0 - PS power on,
 *					  1 - PS shut down (default).
 * @param ps_it 	: PS integration time setting.
 *						0 - 1T, 1 - 1.5T, 2 - 2T, 3 - 2.5T,
 *						4 - 3T, 5 - 3.5T, 6 - 4T, 7 - 8T. 
 * @param ps_pers 	: PS interrupt persistence setting. The PS persistence function (PS_PERS, 1, 2, 3, 4) helps to avoid false
 *					  trigger of the PS INT. It defines the amount of consecutive hits needed in order for a PS interrupt event
 *					  to be triggered.
 *						0 - 1, 1 - 2,
 *						2 - 3, 3 - 4.
 * @param ps_duty 	: PS IRED on / off duty ratio setting.  PS_Duty is related to the current consumption and PS response time. 
 *					  The higher the duty ratio, the faster the response time achieved with higher power consumption. For example, 
 *					  PS_Duty = 1/320, peak IRED current = 100 mA, averaged current consumption is 100 mA/320 = 0.3125 mA.
 *						0 - 1/40,  1 - 1/80,
 *						2 - 1/160, 3 - 1/320.
 * @param ps_int 	: The VCNL4040 has an interrupt feature for both the PS and ALS channel. The purpose of the interrupt feature is to actively
 *					  inform the host once INT has been triggered. When the interrupt is enabled, the host does not need to continuously read the
 *					  data registers of the sensor, but instead can simply react to the interrupt pin. As long as the host enables ALS interrupt (register:
 *					  ALS_INT_EN) or PS interrupt (register: PS_INT) function, the level of INT pin (pin 6) is pulled low once an interrupt event has
 *					  been triggered. All registers are accessible even if INT is triggered. ALS INT is triggered when ALS value crosses over the value
 *					  set in register: ALS_THDH or below the value set by register: ALS_THDL. PS INT is triggered when the PS value crosses over the value
 * 					  set in register: PS_THDH or falls below the value set in register: PS_THDL. Which of these thresholds to react to, can be set by the 
 *					  PS_INT bits in the register: PS_CONF2.
 *						0 - interrupt disable,
 *					  	1 - trigger when close,
 *					  	2 - trigger when away,
 *					  	3 - trigger when close or away.
 * @param ps_hd 	: 0 - PS output is 12 bits,
 *					  1 - PS output is 16 bits.   
 * @param read_back : 0 - read operation, 
 *					  1 - write operation  
 *
**/
VCNL4040_PsConf21_t VCNL4040_PsConf21(uint8_t ps_sd, uint8_t ps_it, uint8_t ps_pers, uint8_t ps_duty,  
										uint8_t ps_int, uint8_t ps_hd, uint8_t read_back) {

	static VCNL4040_PsConf21_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_CONF21;

	if(read_back) {
		RegData.PS_SD = ps_sd;
		RegData.PS_IT = ps_it;
		RegData.PS_PERS = ps_pers;
		RegData.PS_DUTY = ps_duty;
		RegData.PS_INT = ps_int;
		RegData.PS_HD = ps_hd;
		RegData.RESERVED10 = 0;
		RegData.RESERVED15_12 = 0;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData;
}

/*
 * @brief PS duty ratio, integration time, persistence, and PS enable / disable (default 0x0000).
 *
 * @param ps_sc_en 		: 0 - PS sunlight cancel enable setting,
 *						  1 - sunlight cancellation function enable.  
 * @param ps_trig 		: VCNL4040 output one cycle data every time host writes in ‘1’ to sensor. The state returns to ‘0’ automatically.
 *							0 - no PS active force mode trigger,
 *							1 - trigger one time cycle.
 * @param ps_af 		: An extreme power saving way to use PS is to apply PS active force (register: PS_CONF3 command: PS_AF = 1) mode.
 *						  Anytime host would like to request one proximity measurement, write a ‘1’ into register: PS_CONF3 command: PS_Trig. 
 *						  This triggers a single PS measurement, which can be read from the PS result registers. VCNL4040 stays in standby mode
 *						  constantly.
 *							0 - active force mode disable (normal mode),
 *						  	1 - active force mode enable. 
 * @param ps_smart_pers : 0 - disable,
 *						  1 - enable PS smart persistence.
 * @param ps_mps 		: Proximity multi pulse numbers.
 *							0 - 1, 1 - 2,
 *							2 - 4, 3 - 8 multi pulses. 
 * @param led_i 		: LED current selection setting.
 *							0 - 50 mA,  1 - 75 mA,  2 - 100 mA, 3 - 120 mA,
 *							4 - 140 mA, 5 - 160 mA, 6 - 180 mA, 7 - 200 mA. 
 * @param ps_ms 		: 0 - proximity normal operation with interrupt function,
 *						  1 - proximity detection logic output mode enable.
 * @param white_en 		: 0 - white channel enabled,
 *						  1 - white channel disabled.
 * @param read_back : 0 - read operation, 
 *					  1 - write operation  
 *
**/
VCNL4040_PsConf3Ms_t VCNL4040_PsConf3Ms(uint8_t ps_sc_en, uint8_t ps_trig, uint8_t ps_af, uint8_t ps_smart_pers,
										uint8_t ps_mps, uint8_t led_i, uint8_t ps_ms, uint8_t white_en, uint8_t read_back) {

	static VCNL4040_PsConf3Ms_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_CONF3_MS;

	if(read_back) {
		RegData.PS_SC_EN = ps_sc_en;
		RegData.PS_TRIG = ps_trig;
		RegData.PS_AF = ps_af;
		RegData.PS_SMART_PERS = ps_smart_pers;
		RegData.PS_MPS = ps_mps;
		RegData.LED_I = led_i;
		RegData.PS_MS = ps_ms;
		RegData.WHITE_EN = white_en;
		RegData.RESERVED1 = 0;
		RegData.RESERVED7 = 0;
		RegData.RESERVED13_11 = 0;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData;
}

/*
 * @brief PS cancellation level setting (default 0x0000).
 *
 * @param ps_canc 		: PS cancellation level.
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation.
 * @retval uint16_t :  PS cancellation level setting. 
 *
**/
uint16_t VCNL4040_PsCanc(uint16_t ps_canc, uint8_t read_back) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_CANC;
	
	if(read_back) {
		RegData.UniReg = ps_canc;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData.UniReg;
}

/*
 * @brief PS low interrupt threshold setting (default 0x0000).
 *
 * @param ps_thdl 		: PS low interrupt threshold.
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation.
 * @retval uint16_t :  PS low interrupt threshold. 
 *
**/
uint16_t VCNL4040_PsThdl(uint16_t ps_thdl, uint8_t read_back) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_THDL;
	
	if(read_back) {
		RegData.UniReg = ps_thdl;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData.UniReg;
}

/*
 * @brief PS high interrupt threshold setting (default 0x0000).
 *
 * @param ps_thdl 		: PS high interrupt threshold.
 * @param read_back 	: 0 - read operation, 
 *						  1 - write operation.
 * @retval uint16_t :  PS high interrupt threshold. 
 *
**/
uint16_t VCNL4040_PsThdh(uint16_t ps_thdh, uint8_t read_back) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_THDH;
	
	if(read_back) {
		RegData.UniReg = ps_thdh;
		vcnl4040_inst.i2c_tx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	} else {
		vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));
	}
	return RegData.UniReg;
}

/*
 * @brief PS output data (default 0x0000).
 *
 * @retval uint16_t : PS output data. 
 *
**/
uint16_t VCNL4040_PsOutData(void) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_PS_DATA;	
	vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));

	return RegData.UniReg;
}

/*
 * @brief ALS output data (default 0x0000).
 *
 * @retval uint16_t : ALS output data. 
 *
**/
uint16_t VCNL4040_AlsOutData(void) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_ALS_DATA;	
	vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));

	return RegData.UniReg;
}

/*
 * @brief White output data (default 0x0000).
 *
 * @retval uint16_t : White output data. 
 *
**/
uint16_t VCNL4040_WhiteOutData(void) {

	static VCNL4040_UniReg_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_WHITE_DATA;	
	vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));

	return RegData.UniReg;
}

/*
 * @brief ALS, PS interrupt flags (default 0x0000).
 *
 * INT_Flag represents all of the interrupt trigger statuses for ALS and PS. If any of these flags trigger from “0” to “1”,
 * the INT pin will be pulled low. Once the host reads INT_Flag register, all the flags are cleared (reset to "0"), and the 
 * INT pin is reset to high. 
 *
 * @retval PS_IF_AWAY 	: PS drops below PS_THDL INT trigger event.
 * @retval PS_IF_CLOSE 	: PS rises above PS_THDH INT trigger event.
 * @retval ALS_IF_H 	: ALS crossing high THD INT trigger event.
 * @retval ALS_IF_L 	: ALS crossing low THD INT trigger event.
 * @retval PS_SPFLAG 	: PS entering protection mode.
 *
**/
VCNL4040_IntFlag_t VCNL4040_IntFlag(void) {

	static VCNL4040_IntFlag_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_INT_FLAG;	
	vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));

	return RegData;
}

/*
 * @brief  Device ID  (default 0x0186).
 *
 * @retval uint16_t : device ID.
 *
**/
uint16_t VCNL4040_GetId(void) {

	static VCNL4040_Id_t RegData = { 0 };

	vcnl4040_inst.MemAddr = VCNL4040_ID;	
	vcnl4040_inst.i2c_rx(vcnl4040_inst.MemAddr, (uint8_t*)&RegData, sizeof(RegData));

	return (((uint16_t)RegData.VERSION_CODE << 8) | (uint16_t)RegData.VERSION_SAMPLE);
}

 /* Hardware dependent functions. */

/*
 * @brief I2C Tx data.
 *
 **/
static void VCNL4040_Tx(uint16_t MemAddress, uint8_t *pData, uint16_t Size) {

	HAL_I2C_Mem_Write(VCNL4040_I2CInst, VCNL4040_ADDRSH, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 10);
}

/*
 * @brief I2C Rx data.
 * 
 *
 **/
static void VCNL4040_Rx(uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	
	HAL_I2C_Mem_Read(VCNL4040_I2CInst, VCNL4040_ADDRSH, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 10);
}

