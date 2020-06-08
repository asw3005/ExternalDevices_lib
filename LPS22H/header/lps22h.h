/** Created: 5.17.2020
 ** lps22h.h
 **/	
#pragma once
//#ifndef LPS22H_H_
//#define LPS22H_H_

#include "stm32l4xx_hal.h"

/**
 ** @brief Device address
 **/
#define LPS22H_ADDR_SHIFTED             0xB8	//7 bit MSB 0x76 (address) + 1 bit LSB 0x00 (read/write)
#define LPS22H_ADDR						0x5C	//It's not shifted address
///An 8 - bit sub - address(SUB) will be transmitted : the
///7 LSB represents the actual register address while the MSB enables address autoincrement.
///If the MSB of the SUB field is ‘1’, the SUB(register address) will be automatically
///increased to allow multiple data read / write.
#define LPS22H_ADDR_AUTOINC_ENABLE		0x80

/**
 ** @brief Register mapping
 ** 
 ** Provides a list of the 8-bit registers embedded in the device and the related
 ** addresses.
 **/
typedef enum 
{
	///Interrupt register
	LPS22H_INTERRUPT_CFG	= 0x0B,
	///Pressure threshold registers
	LPS22H_THS_P_L			= 0x0C,
	LPS22H_THS_P_H			= 0x0D,
	///Who I am
	LPS22H_WHO_AM_I			= 0x0F,
	///Control registers
	LPS22H_CTRL_REG1		= 0x10,
	LPS22H_CTRL_REG2		= 0x11,
	LPS22H_CTRL_REG3		= 0x12,
	///FIFO configuration register
	LPS22H_FIFO_CTRL		= 0x14,
	///Reference pressure registers
	LPS22H_REF_P_XL			= 0x15,
	LPS22H_REF_P_L			= 0x16,
	LPS22H_REF_P_H			= 0x17,
	///Pressure offset registers
	LPS22H_RPDS_L			= 0x18,
	LPS22H_RPDS_H			= 0x19,
	///Resolution register
	LPS22H_RES_CONF			= 0x1A,
	///Interrupt register
	LPS22H_INT_SOURCE		= 0x25,
	///FIFO status register
	LPS22H_FIFO_STATUS		= 0x26,
	///Status register
	LPS22H_STATUS			= 0x27,
	///Pressure output registers
	LPS22H_PRESS_OUT_XL		= 0x28,
	LPS22H_PRESS_OUT_L		= 0x29,
	LPS22H_PRESS_OUT_H		= 0x2A,
	///Temperature output registers
	LPS22H_TEMP_OUT_L		= 0x2B,
	LPS22H_TEMP_OUT_H		= 0x2C,
	///Filter reset register
	LPS22H_LPFP_RES			= 0x33,
	
}  LPS22H_AddressReg;

/**	Experimental feature, in progress.
 ** @brief Device measurement profiles.
 ** 
 ** It is recommended medes of operations.
 ** Mode "CUSTOM_PROFILE_0" is custom. You need to make settings yourself by 
 ** adding your configuration to the function LPS22H_Set_Profile.
 **
 **/
typedef enum 
{ 
	LPS22H_PROFILE_FIFO_BYPASS,
	LPS22H_PROFILE_FIFO_MODE,
	LPS22H_PROFILE_STREAM_MODE,
	LPS22H_PROFILE_STREAM_TO_FIFO_MODE,
	LPS22H_PROFILE_BYPASS_TO_STREAM_MODE,
	LPS22H_PROFILE_BYPASS_TO_FIFO_MODE,
	LPS22H_PROFILE_DYNAMIC_STREAM_MODE,
	LPS22H_PROFILE_CUSTOM_0,
	
} LPS22H_Profile;

/**
 ** @brief Interrupt configuration register const
 ** 
 **/
typedef enum 
{
	///Enable interrupt generation on pressure high event. Default value: 0
	///0: disable interrupt request, 1: enable interrupt request on pressure value higher than preset threshold
	LPS22H_PHE_DIS = 0,
	LPS22H_PHE_EN = 1,
	///Enable interrupt generation on pressure low event. Default value: 0
	///0: disable interrupt request, 1: enable interrupt request on pressure value lower than preset threshold
	LPS22H_PLE_DIS = 0,
	LPS22H_PLE_EN = 1,
	///Latch interrupt request to the INT_SOURCE (25h) register. Default value: 0
	///0: interrupt request not latched; 1: interrupt request latched
	LPS22H_LIR_DIS = 0,
	LPS22H_LIR_EN = 1,
	///Enable interrupt generation. Default value: 0
	///(0: interrupt generation disabled; 1: interrupt generation enabled
	LPS22H_DIFF_DIS = 0,
	LPS22H_DIFF_EN = 1,
	///Reset Autozero function. Default value: 0
	///0: normal mode; 1: reset Autozero function
	LPS22H_RESET_AZ_NORMAL_MODE = 0,
	LPS22H_RESET_AZ_RESET_AUTOZERO = 1,
	///Enable Autozero function. Default value: 0
	///0: normal mode; 1: Autozero enabled
	LPS22H_AUTOZERO_NORMAL_MODE = 0,
	LPS22H_AUTOZERO_EN = 1,
	///Reset AutoRifP function. Default value: 0
	///0: normal mode; 1: reset AutoRifP function
	LPS22H_RESET_ARP_NORMAL_MODE = 0,
	LPS22H_RESET_ARP_RESET_AUTORIFP = 1,
	///Enable AUTORIFP function. Default value: 0
	///0: normal mode; 1: AutoRifP enabled
	LPS22H_AUTORIFP_NORMAL_MODE = 0,
	LPS22H_AUTORIFP_EN = 1,	
	
}  LPS22H_InterruptCfg;

/**
 ** @brief Control register 1 (10h)
 ** 
 **/
typedef enum 
{
	///SPI Serial Interface Mode selection. Default value: 0
	///0: 4-wire interface; 1: 3-wire interface
	LPS22H_SIM_4_WIRE_INTERFACE = 0,
	LPS22H_SIM_3_WIRE_INTERFACE = 1,
	///Block data update. Default value: 0. To guarantee the correct behavior of BDU feature, 
	///PRESS_OUT_H (2Ah) must be the last address read.
	///When I2C is used with BDU=1, the IF_ADD_INC bit has to be set to ‘0’ in CTRL_REG2 (11h) 
	///and only a single-byte read of the output registers is allowed.
	///0: continuous update; 1: output registers not updated until MSB and LSB have been read
	LPS22H_BDU_CONTINUOUS_UPDATE = 0,
	LPS22H_BDU_DISCONTINUOUS_UPDATE = 1,
	///LPFP_CFG: Low-pass configuration register. Default value:0. 
	///EN_LPFP = 1, LPFP_CFG = x
	///EN_LPFP = 1, LPFP_CFG = 0
	///EN_LPFP = 1, LPFP_CFG = 1
	LPS22H_LPFP_CFG_LPF_DISABLE_ODR_DIV2 = 0,
	LPS22H_LPFP_CFG_LPF_ENABLE_ODR_DIV9 = 0,  
	LPS22H_LPFP_CFG_LPF_ENABLE_ODR_DIV20 = 1, 
	///Enable low-pass filter on pressure data when Continuous mode is used. Default value: 0
	///0: Low-pass filter disabled; 1: Low-pass filter enabled
	LPS22H_DIS_LPFP = 0,
	LPS22H_EN_LPFP = 1,
	///Output data rate selection. Default value: 000.
	LPS22H_ODR_POWER_DOWN_ONE_SHOT_MODE = 0,
	LPS22H_ODR_P1Hz_T1Hz = 1,
	LPS22H_ODR_P10Hz_T10Hz = 2,
	LPS22H_ODR_P25Hz_T25Hz = 3,
	LPS22H_ODR_P50Hz_T50Hz = 4,
	LPS22H_ODR_P75Hz_T75Hz = 5,	
	
}  LPS22H_CtrlReg1;

/**
 ** @brief Control register 2 (11h)
 ** 
 **/
typedef enum 
{
	///One-shot enable.Default value : 0.
	///0 : idle mode ; 1 : a new dataset is acquired.
	LPS22H_ONE_SHOT_IDLE_MODE = 0,
	LPS22H_ONE_SHOT_EN = 1,
	///Software reset.Default value : 0.
	///0 : normal mode ; 1 : software reset.
	///The bit is self - cleared when the reset is completed.
	LPS22H_SWRESET_NORMAL_MODE = 0,
	LPS22H_SWRESET_SOFTWARE_RESET = 1,
	///Disable I2C interface. Default value: 0.
	///0: I2C enabled; 1: I2C disabled.
	LPS22H_I2C_DIS = 1,
	LPS22H_I2C_EN = 0,
	///Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI). Default value: 1.
	///0: disable; 1 enable.
	///It is recommend to use a single - byte read(with IF_ADD_INC = 0) when output data registers are acquired
	///without using the FIFO. If a read of the data occurs during the refresh of the output data register, it is
	///recommended to set the BDU bit to ‘1’ in CTRL_REG1(10h) in order to avoid mixing data.
	LPS22H_IF_ADD_INC_DIS = 0,
	LPS22H_IF_ADD_INC_EN = 1,
	///Stop on FIFO watermark. Enable FIFO watermark level use. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_STOP_ON_FTH_DIS = 0,
	LPS22H_STOP_ON_FTH_EN = 1,
	///FIFO enable. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_FIFO_DIS = 0,
	LPS22H_FIFO_EN = 1,
	///Reboot memory content. Default value: 0.
	///0: normal mode; 1: reboot memory content. The bit is self-cleared when the BOOT is completed.
	LPS22H_BOOT_NORMAL_MODE = 0,
	LPS22H_BOOT_REBOOT_MEMORY_CONTENT = 1,	
	
}  LPS22H_CtrlReg2;

/**
 ** @brief Control register 3 - INT_DRDY pin control register (12h).
 ** 
 **/
typedef enum 
{
	///Data signal on INT_DRDY pin control bits. Default value: 00.
	///INT_S2_1 00 Data signal (in order of priority: DRDY or F_FTH or F_OVR or F_FSS5.
	///INT_S2_1 01 Pressure high (P_high).
	///INT_S2_1 10 Pressure low (P_low).
	///INT_S2_1 11 Pressure low OR high.
	LPS22H_INT_S_IN_ORDER_OF_PRIORITY = 0,
	LPS22H_INT_S_P_HIGH = 1,
	LPS22H_INT_S_P_LOW = 2,
	LPS22H_INT_S_P_LOW_OR_HIGH = 3,
	///Data-ready signal on INT_DRDY pin. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_DRDY_DIS = 0,
	LPS22H_DRDY_EN,
	///FIFO overrun interrupt on INT_DRDY pin. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_F_OVR_DIS = 0,
	LPS22H_F_OVR_EN = 1,
	///FIFO watermark status on INT_DRDY pin. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_F_FTH_DIS = 0,
	LPS22H_F_FTH_EN = 1,
	///FIFO full flag on INT_DRDY pin. Default value: 0.
	///0: disable; 1: enable.
	LPS22H_F_FSS5_DIS = 0,
	LPS22H_F_FSS5_EN = 1,
	///Push-pull/open drain selection on interrupt pads. Default value: 0.
	///0: push-pull; 1: open drain.
	LPS22H_PP_OD_PUSH_PULL = 0,
	LPS22H_PP_OD_OPEN_DRAIN = 1,
	///Interrupt active-high/low. Default value: 0.
	///0: active high; 1: active low.
	LPS22H_INT_H_L_ACTIVE_HIGH = 0,
	LPS22H_INT_H_L_ACTIVE_LOW = 1,
	
}  LPS22H_CtrlReg3;

/**
 ** @brief FIFO_CTRL register (14h).
 ** 
 **/
typedef enum
{
	///FIFO mode selection. Default value: 000
	LPS22H_F_MODE_RESET_FIFO_CONTENT = 0,
	///In Bypass mode (FIFO_CTRL(F_MODE2:0) = ‘000’) 
	///FIFO is not operational and the buffer remains empty.
	LPS22H_F_MODE_BYPASS_MODE = 0,
	///In FIFO mode (FIFO_CTRL(F_MODE2:0) = ‘001’) the pressure and 
	///temperature acquired are stored in the buffer.
	///When the FIFO is full or the watermark is reached, the update 
	///in the FIFO is stopped until the buffer is read or reset.
	LPS22H_F_MODE_FIFO_MODE = 1,
	///In Stream mode(FIFO_CTRL(F_MODE2 :0) = ‘010’), the pressure and 
	///temperature acquired are stored in the buffer.
	///Once the FIFO is full or the watermark level is reached, the new 
	///data replace the older data stored in the buffer.
	LPS22H_F_MODE_STREAM_MODE = 2,
	///In Stream-to-FIFO mode (FIFO_CTRL(F_MODE2:0) = ‘011’), the FIFO 
	///works in Stream mode until a trigger event is generated and then in FIFO mode.	
	///If the interrupt is triggered, the INT_SOURCE(IA) bit is equal to '1', and the 
	///FIFO switches from Stream to FIFO mode. When the interrupt is de-asserted, the 
	///INT_SOURCE(IA) bit is equal to '0', and the FIFO switches back to Stream mode.
	LPS22H_F_MODE_STREAM_TO_FIFO_MODE = 3,
	///In Bypass-to-Stream mode (FIFO_CTRL(F_MODE2:0) = ‘100’), the FIFO works in Bypass 
	///mode until a trigger event is generated and then in Stream mode.
	///If the interrupt is triggered, the INT_SOURCE(IA) bit is equal to '1', and the FIFO
	///switches from Bypass to Stream mode. When the interrupt is de-asserted, the INT_SOURCE(IA) 
	///bit is equal to '0', and the FIFO switches back to Bypass mode.
	LPS22H_F_MODE_BYPASS_TO_STREAM_MODE = 4,
	///In Dynamic Stream mode (FIFO_CTRL(F_MODE2:0) = 110) after emptying the FIFO, 
	///the first new sample that arrives becomes the first to be read in a subsequent 
	///read burst. In this way, the number of new data available in FIFO does not 
	///depend on the previous reading. In Dynamic Stream mode FIFO_STATUS(FSS5:0) is 
	///the number of new pressure and temperature samples available in the FIFO buffer.
	LPS22H_F_MODE_DYNAMIC_STREAM_MODE = 6,
	///In Bypass-to-FIFO mode (FIFO_CTRL(F_MODE2:0) = ‘111’), the FIFO works in Bypass mode
	///until a trigger event is generated and then in FIFO mode.
	///If the interrupt is triggered, the INT_SOURCE(IA) bit is equal to '1', and the FIFO 
	///switches from Bypass to FIFO mode. When the interrupt is de-asserted, INT_SOURCE(IA) bit 
	///is equal to '0', and the FIFO switches back to Bypass mode.
	LPS22H_F_MODE_BYPASS_TO_FIFO_MODE = 7,
	
} LPS22H_FifoCtrl;

/**
 ** @brief INT_SOURCE register (25h).
 ** 
 **/
typedef enum
{
	///Differential pressure High.
	///0: no interrupt has been generated; 1: high differential pressure event has occurred.
	LPS22H_PH_NO_INTERRUPT_GENERATED = 0,
	LPS22H_PH_HIGH_DIFFERENTIAL_PRESSURE = 1,
	///Differential pressure Low.
	///0: no interrupt has been generated; 1: low differential pressure event has occurred.
	LPS22H_PL_NO_INTERRUPT_GENERATED = 0,
	LPS22H_PL_LOW_DIFFERENTIAL_PRESSURE = 1,
	///Interrupt active.
	///0: no interrupt has been generated; 1: one or more interrupt events have been generated.
	LPS22H_IA_NO_INTERRUPT_GENERATED = 0,
	LPS22H_IA_ONE_OR_MORE_EVENTS = 1,	
	
} LPS22H_IntSource;

/**
 ** @brief RES_CONF register, low-power mode configuration (1Ah).
 ** 
 **/
typedef enum uint8_t
{
	///Low current mode enable. Default 0.
	///0: Normal mode (low-noise mode); 1: Low-current mode.
	LPS22H_LC_EN_NORMAL_MODE = 0,
	LPS22H_LC_EN_LOW_CURRENT_MODE = 1,
	
} LPS22H_ResConf;

/**
 ** @brief Device specific function type
 ** 
 ** @param[in] DevAddress : Device address on the I2C bus
 ** @param[in] MemAddress : Register address for device
 ** @param[in] MemAddSize : Size address of memory
 ** @param[in] *pData : Pointer on data struct instance
 ** @param[in] Size : Size of transmition/reception data
 **
 ** @return Result of API execution status
 ** @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 **/
typedef int8_t(*lps22h_communication_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
typedef void(*lps22h_delay_fptr)(uint32_t period);

/**
 ** @brief Compensated data
 **/
typedef struct {
	uint8_t dev_id;	
	int32_t TemperatureC;
	int32_t TemperatureF;
	uint32_t PressurePa;
	uint32_t PressuremmHg;
	uint32_t PressuremBar;
	
} LPS22H_TempPressStruct_typedef;

/**
 ** @brief Configuration registers
 ** 
 **/
typedef struct {
	union {	
		///Interrupt mode for pressure acquisition configuration. (0Bh)
		uint8_t InterruptCfg;
		struct {
			///Enable interrupt generation on pressure high event. Default value: 0.
			///0: disable interrupt request, 1: enable interrupt request on pressure value higher than preset threshold.
			uint8_t	phe			: 1;   
			///Enable interrupt generation on pressure low event. Default value: 0.
			///0: disable interrupt request, 1: enable interrupt request on pressure value lower than preset threshold.
			uint8_t	ple			: 1;	
			///Latch interrupt request to the INT_SOURCE (25h) register. Default value: 0.
			///0: interrupt request not latched; 1: interrupt request latched.
			uint8_t	lir			: 1;
			///Enable interrupt generation. Default value: 0.
			///(0: interrupt generation disabled; 1: interrupt generation enabled.
			uint8_t	diff_en		: 1;
			///Reset Autozero function. Default value: 0.
			///0: normal mode; 1: reset Autozero function.
			uint8_t	reset_az	: 1;
			///Enable Autozero function. Default value: 0.
			///0: normal mode; 1: Autozero enabled.
			uint8_t	autozero	: 1;
			///Reset AutoRifP function. Default value: 0.
			///0: normal mode; 1: reset AutoRifP function.
			uint8_t	reset_arp	: 1;
			///Enable AUTORIFP: function. Default value: 0
			///0: normal mode; 1: AutoRifP enabled
			uint8_t	autorifp	: 1;
			
		} bitsInterruptCfg;				
	};	

	///User-defined threshold value for pressure interrupt event. (0Ch)
	uint16_t ThresholdP;
	
	union {
		///Control register 1 (10h).
		uint8_t CtrlReg1;
		struct {
			///SPI Serial Interface Mode selection. Default value: 0.
			///0: 4-wire interface; 1: 3-wire interface.
			uint8_t sim			: 1;
			///Block data update. Default value: 0.
			///0: continuous update; 1: output registers not updated until MSB and LSB have been read.
			uint8_t bdu			: 1;
			///LPFP_CFG: Low-pass configuration register. Default value:0. Detailed see LPS22H_Ctrl_Reg1_enum.
			uint8_t lpfp_cfg	: 1;
			///Enable low-pass filter on pressure data when Continuous mode is used. Default value: 0.
			///0: Low-pass filter disabled; 1: Low-pass filter enabled.
			uint8_t en_lpfp		: 1;
			///Output data rate selection. Default value: 000. Detailed see LPS22H_Ctrl_Reg1_enum.
			uint8_t odr_2_0		: 3;
			///This bit must be set to ‘0’ for proper operation of the device.
			uint8_t reserved_7	: 1;

		} bitsCtrlReg1;
	};
	
	union {
		///Control register 2 (11h)
		uint8_t CtrlReg2;
		struct {
			///One - shot enable.Default value : 0.
			///0 : idle mode ; 1 : a new dataset is acquired.
			uint8_t one_shot	: 1;
			///This bit must be set to ‘0’ for proper operation of the device.
			uint8_t reserved_1	: 1;
			///Software reset.Default value : 0.
			///0 : normal mode ; 1 : software reset.
			///The bit is self - cleared when the reset is completed.
			uint8_t swreset		: 1;
			///Disable I2C interface. Default value: 0.
			///0: I2C enabled;1: I2C disabled.
			uint8_t i2c_dis		: 1;
			///Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI). Default value: 1.
			///0: disable; 1 enable.
			///It is recommend to use a single - byte read(with IF_ADD_INC = 0) when output data registers are acquired
			///without using the FIFO. If a read of the data occurs during the refresh of the output data register, it is
			///recommended to set the BDU bit to ‘1’ in CTRL_REG1(10h) in order to avoid mixing data.
			uint8_t if_add_inc	: 1;
			///Stop on FIFO watermark. Enable FIFO watermark level use. Default value: 0.
			///0: disable; 1: enable.
			uint8_t stop_on_fth : 1;
			///FIFO enable. Default value: 0.
			///0: disable; 1: enable.
			uint8_t fifo_en		: 1;
			///Reboot memory content. Default value: 0.
			///0: normal mode; 1: reboot memory content. The bit is self-cleared when the BOOT is completed.
			uint8_t boot		: 1;
			
		} bitsCtrlReg2;
	};
	
	union {
		///Control register 3 - INT_DRDY pin control register (12h).
		uint8_t CtrlReg3;
		struct {
			///Data signal on INT_DRDY pin control bits. Default value: 00.
			///Detailed see LPS22H_Ctrl_Reg3_enum.
			uint8_t int_s_2_1	: 2;
			///Data-ready signal on INT_DRDY pin. Default value: 0.
			///0: disable; 1: enable.
			uint8_t drdy		: 1;
			///FIFO overrun interrupt on INT_DRDY pin. Default value: 0.
			///0: disable; 1: enable.
			uint8_t f_ovr		: 1;
			///FIFO watermark status on INT_DRDY pin. Default value: 0.
			///0: disable; 1: enable.
			uint8_t f_fth		: 1;
			///FIFO full flag on INT_DRDY pin. Default value: 0.
			///0: disable; 1: enable.
			uint8_t f_fss5		: 1;
			///Push-pull/open drain selection on interrupt pads. Default value: 0.
			///0: push-pull; 1: open drain.
			uint8_t pp_od		: 1;
			///Interrupt active-high/low. Default value: 0.
			///0: active high; 1: active low.
			uint8_t int_h_l		: 1;

		} bitsCtrlReg3;
	};
	
	union {
		///FIFO control register (14h).
		uint8_t FifoCtrl;
		struct {

			///FIFO watermark level selection.
			uint8_t wtm_4_0		: 5;
			///FIFO mode selection. Default value: 000
			///Detailed see LPS22H_Fifo_Ctrl_enum.
			uint8_t f_mode_2_0	: 3;
			
		} bitsFifoCtrl;
	};
	
	///This register contains the reference pressure value, [23:0]. The value is expressed as 2’s complement (three bytes).
	///WARNING! Return only 3 low bytes.
	uint32_t RefP;
	///This register contains the pressure offset value, [15:0]. The value is expressed as 2’s complement.
	int16_t Rpds;

	union {
		///Low-power mode configuration register.
		uint8_t ResConf;
		struct 	{
			///Low current mode enable. Default 0.
			///0: Normal mode (low-noise mode); 1: Low-current mode.
			uint8_t lc_en			: 1;
			///The content of this bit must not be modified for proper operation of the device.
			uint8_t reserved_1		: 1;
			///These bits must be set to ‘0’ for proper operation of the device.
			uint8_t reserved_7_2	: 6;
			
		} bitsResConf;
	};
	
	union {
		///Interrupt source register.
		uint8_t IntSource;
		struct {
			///Differential pressure High.
			///0: no interrupt has been generated; 1: high differential pressure event has occurred.
			uint8_t ph				: 1;
			///Differential pressure Low.
			///0: no interrupt has been generated; 1: low differential pressure event has occurred.
			uint8_t pl				: 1;
			///Interrupt active.
			///0: no interrupt has been generated; 1: one or more interrupt events have been generated.
			uint8_t ia				: 1;
			uint8_t reserved_6_3	: 4;
			///If ‘1’ indicates that the Boot (Reboot) phase is running. 
			uint8_t boot_status		: 1;			
			
		} bitsIntSource;
	};
	
}  LPS22H_ConfigStruct_typedef;

/**
 ** @brief Data receive parametr
 **/
typedef struct __attribute__((aligned(1), packed)) {
		
	union {
		///Interrupt source register, 25h
		uint8_t BootStatus;
		struct 	{		
			uint8_t reserved_6_0	: 7;
			///If ‘1’ indicates that the Boot (Reboot) phase is running.
			uint8_t boot_status		: 1;
			
		} bitsBootStatus;
	};
	
	union {
		///FIFO status, 26h
		uint8_t FifoStatus;
		struct {		
			///FIFO stored data level.
			///000000: FIFO empty, 100000: FIFO is full and has 32 unread samples.
			uint8_t fss			: 6;
			///FIFO overrun status.
			///0: FIFO is not completely full; 1: FIFO is full and at least one sample in the FIFO has been overwritten.
			uint8_t ovr			: 1;
			///FIFO watermark status.
			///0: FIFO filling is lower than treshold level; 1: FIFO filling is equal or higher than treshold level.
			uint8_t fth_fifo	: 1;
			
		} bitsFifoStatus;
	};
	
	union {
		///Status register, 27h
		uint8_t Status;
		struct {		
			///Pressure data available.
			///0: new data for pressure is not yet available; 1: a new pressure data is generated.
			uint8_t p_da			: 1;
			///Temperature data available.
			///0: new data for temperature is not yet available; 1: a new temperature data is generated.
			uint8_t t_da			: 1;
			uint8_t reserved_3_2	: 2;
			///Pressure data overrun.
			///0: no overrun has occurred; 1: new data for pressure has overwritten the previous data.
			uint8_t p_or			: 1;
			///Temperature data overrun.
			///0: no overrun has occurred; 1: a new data for temperature has overwritten the previous data.
			uint8_t t_or			: 1;
			uint8_t reserved_7_6	: 2;
			
		} bitsStatus;
	};
	
	///The pressure output value is a 24-bit data that contains the measured pressure, 28h, 29h, 2Ah.
	///The value is expressed as 2’s complement.
	///The output pressure register PRESS_OUT is provided as the difference between the
	///measured pressure and the content of the register RPDS(18h, 19h).
	///This register contains the low part of the pressure output value, [7:0].
	uint8_t Press_Out_XL;
	///This register contains the mid part of the pressure output value, [15:8].
	uint8_t Press_Out_L;
	///This register contains the high part of the pressure output value, [23:16].
	uint8_t Press_Out_H;
	///The temperature output value is 16-bit data that contains the measured temperature, 2Bh, 2Ch.
	///The value is expressed as 2’s complement.
	int16_t Temp_Out;
	///Low-pass filter reset register. 
	///If the LPFP is active, in order to avoid the transitory phase, the filter can be reset
	///by reading this register before generating pressure measurements.
	uint8_t LpfpRes;
	
		
} LPS22H_DataReceiveStruct_typedef;



/**
 ** @brief HTS221 instance struct
 **/
typedef struct {
	
	uint8_t dev_address;
	LPS22H_ConfigStruct_typedef dev_configuration;
	LPS22H_DataReceiveStruct_typedef dev_uncompensated_data;
	lps22h_communication_fptr read_data_i2c;
	lps22h_communication_fptr write_data_i2c;
	lps22h_delay_fptr delay;	
	LPS22H_TempPressStruct_typedef dev_compensated_data;
	
	
} LPS22H_typedef;

/**
 ** @brief Public function prototype
 **/
void LPS22H_InitDevice(LPS22H_typedef *dev_lps22h);


//#endif /* LPS22H_H_ */


