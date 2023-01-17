/*
 * @brief Common header for ADS8671, ADS8675 from the Texas Instruments.
 * Created 08.02.21 by asw3005. 
 *
 **/

#ifndef ADS867x_H_
#define ADS867x_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32f103xb.h"

/* GPIO configuration. */
#define CS_ADC_Pin					GPIO_PIN_0
#define CS_ADC_GPIO_Port			GPIOB

/* ADC constants. */
#define ADS867x_WKEY				0x69
#define ADS867x_VALUE_OF_DIVISION	0.312f

/*
 * @brief Configuration registers mapping. LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef enum {
	
	/* This register contains the unique identification numbers associated to a device that is used in a daisy-chain
	 * configuration involving multiple devices.*/
	ADS867x_DEVICE_ID_LSW,
	ADS867x_DEVICE_ID_HSW		= 0x02,
	/* This register controls the reset and power-down features offered by the converter. Any write operation to the 
	 * RST_PWRCTL_REG register must be preceded by a write operation with the register address set to 05h and the register 
	 * data set to 69h. */
	ADS867x_RST_PWRCTL_LSW		= 0x04,
	ADS867x_RST_PWRCTL_HSW		= 0x06,
	/* This register configures the protocol used for writing data to the device. */
	ADS867x_SDI_CTL_LSW			= 0x08,
	ADS867x_SDI_CTL_HSW			= 0x0A,
	/* This register controls the data protocol used to transmit data out from the SDO-x pins of the device. */
	ADS867x_SDO_CTL_LSW			= 0x0C,
	ADS867x_SDO_STL_HSW			= 0x0E,
	/* This register controls the data output by the device. */
	ADS867x_DATAOUT_CTL_LSW		= 0x10,
	ADS867x_DATAOUT_CTL_HSW		= 0x12,
	/* This register controls the configuration of the internal reference and input voltage ranges for the converter. */
	ADS867x_RANGE_SEL_LSW		= 0x14,
	ADS867x_RANGE_SEL_HSW		= 0x16,
	/* This register contains the output alarm flags (active and tripped) for the input and AVDD alarm. */
	ADS867x_ALARM_LSW			= 0x20,
	ADS867x_ALARM_HSW			= 0x22,
	/* This register controls the hysteresis and high threshold for the input alarm. */
	ADS867x_ALARM_H_TH_LSW		= 0x24,
	ADS867x_ALARM_H_TH_HSW		= 0x26,
	/* This register controls the low threshold for the input alarm. */
	ADS867x_ALARM_L_TH_LSW		= 0x28,
	ADS867x_ALARM_L_TH_HSW		= 0x2A
	
} ADS867x_RegMap;

/*
 * @brief List of commands. All other input combinations are like a NOP command.
 *
 **/
typedef enum {
	
	/* No operation. */
	ADS867x_NOP = 0x00,
	/* Command used to clear any (or a group of) bits of a register. Any bit marked 1 in the data field results in that particular
	 * bit of the specified register being reset to 0, leaving the other bits unchanged. */
	ADS867x_RESET_HWORD		= 0x60,
	/* Command used to set any (or a group of) bits of a register. Any bit marked 1 in the data field results in that particular
	 * bit of the specified register being set to 1, leaving the other bits unchanged. */
	ADS867x_SET_HWORD		= 0x6C,
	/* Command used to perform a 16-bit read operation. Upon receiving this command, the device sends out 16 bits of the register 
	 * in the next frame. */
	ADS867x_READ_HWORD		= 0x64,
	/* Same as the READ_HWORD except that only eight bits of the register (byte read) are returned in the next frame. */
	ADS867x_READ_BYTE		= 0x24,
	/* Half-word write command (two bytes of input data are written into the specified address). */
	ADS867x_WRITE_HWORD		= 0x68,
	/* Half-word write command. With this command, only the MS byte of the 16-bit data word is written at the specified register
	 * address. The LS byte is ignored. */
	ADS867x_WRITE_MSB		= 0x69,
	/* Half-word write command. With this command, only the LS byte of the 16-bit data word is written at the specified register
	 * address. The MS byte is ignored. */
	ADS867x_WRITE_LSB		= 0x6A,
	
} ADS867x_CmdList;

/*
 * @brief SPI protocol select.
 *
 **/
typedef enum {
	
	ADS867x_CPOL0_CPHASE0,
	ADS867x_CPOL0_CPHASE1,
	ADS867x_CPOL1_CPHASE0,
	ADS867x_CPOL1_CPHASE1
	
} ADS867x_SPIProtocol;

/*
 * @brief SDO1 pin mode.
 *
 **/
typedef enum {
	
	ADS867x_SDO1_TRISTATE,
	ADS867x_SDO1_ALARM,
	ADS867x_SDO1_GPO,
	ADS867x_SDO1_2BITMODE
	
} ADS867x_SDO1Mode;

/*
 * @brief Data output format. 
 *
 **/
typedef enum {
	
	ADS867x_CONVDATA	= 3,
	ADS867x_ALLZERO		= 4,
	ADS867x_ALLONES,
	ADS867x_ALTERNATING_0_1,
	ADS867x_ALTERNATING_00_11
	
} ADS867x_DataVal;

/*
 * @brief Active input ALARM flags control.
 *
 **/
typedef enum {
	
	ADS867x_ACTIVE_IN_DO_NOT_INCL,
	ADS867x_ACTIVE_IN_H_FLAG_INCL,
	ADS867x_ACTIVE_IN_L_FLAG_INCL,
	ADS867x_BOTH_ACTIVE_IN_FLAGS_INCL
	
} ADS867x_InActiveAlarm;

/*
 * @brief Active VDD ALARM flags control.
 *
 **/
typedef enum {
	
	ADS867x_ACTIVE_VDD_DO_NOT_INCL,
	ADS867x_ACTIVE_VDD_H_FLAG_INCL,
	ADS867x_ACTIVE_VDD_L_FLAG_INCL,
	ADS867x_BOTH_ACTIVE_VDD_FLAGS_INCL
	
} ADS867x_VddActiveAlarm;

/*
 * @brief Analog input range.
 * P and N - positive and negative respectively.
 *
 **/
typedef enum {
	/* ±3 × VREF */
	ADS867x_PN3_0VREF,
	/* ±2.5 × VREF */
	ADS867x_PN2_5VREF,
	/* ±1.5 × VREF */
	ADS867x_PN1_5VREF,
	/* ±1.25 × VREF */
	ADS867x_PN1_25VREF,
	/* ±0.625 × VREF */
	ADS867x_PN0_625VREF,
	
	/* 0–3 × VREF */
	ADS867x_P3_0VREF = 0x08,
	/* 0–2.5 × VREF */
	ADS867x_P2_5VREF,
	/* 0–1.5 × VREF */
	ADS867x_P1_5VREF,
	/* 0–1.25 × VREF */
	ADS867x_P1_25VREF
	
} ADS867x_InputRange;



/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief Device id register type.
 * This register contains the unique identification numbers associated to a device that is used in a daisy-chain 
 * configuration involving multiple devices.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t DevIdReg_LSW;
		uint16_t DevIdReg_HSW;
	};
	struct {
		uint16_t RESERVED15_0	: 16;
		/* These bits can be used to identify up to 16 different devices in a system (default is 0000b). */
		uint16_t DEVICE_ADDR	: 4;
		uint16_t RESERVED23_20	: 4;
		uint16_t RESERVED31_24	: 8;
	};
	
	
} ADS867x_DevId_t;

/*
 * @brief Reset and power down register type.
 * This register controls the reset and power-down features offered by the converter. Any write operation to the 
 * RST_PWRCTL_REG register must be preceded by a write operation with the register address set to 05h and the 
 * register data set to 69h. 
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t RstPwrCtrlReg_LSW;
		uint16_t RstPwrCtrlReg_HSW;
	};
	struct {
		/* 0b = Puts the converter into active mode (default), 
		 * 1b = Puts the converter into power-down mode. */
		uint8_t PWRDN			: 1;
		/* 0b = Disables the NAP mode of the converter (default), 
		 * 1b = Enables the converter to enter NAP mode if CONVST/CS is 
		 * held high after the current conversion completes. */
		uint8_t NAP_EN			: 1;
		/* 0b = RST pin functions as a POR class reset (causes full device initialization) (default), 
		 * 1b = RST pin functions as an application reset (only user-programmed modes are cleared). */
		uint8_t RSTn_APP		: 1;
		/* Reserved. Reads return 0h. */
		uint8_t RESERVED3		: 1;
		/* 0b = Input alarm is enabled (default), 
		 * 1b = Input alarm is disabled. */
		uint8_t IN_AL_DIS		: 1;
		/* 0b = VDD alarm is enabled (default), 
		 * 1b = VDD alarm is disabled. */
		uint8_t VDD_AL_DIS		: 1;
		/* Reserved. Reads return 00b. */
		uint8_t RESERVED7_6		: 2;
		/* This value functions as a protection key to enable writes to bits 5-0. Bits are written only if WKEY is set 
		 * to 69h first. */
		uint8_t WKEY			: 8;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_16	: 16;
	};
	
} ADS867x_RstPwrCtrl_t;

/*
 * @brief Writing or reading protocol register type.
 * This register configures the protocol used for writing/reading data to the device.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t SdiCtrlReg_LSW;
		uint16_t SdiCtrlReg_HSW;
	};
	struct {
		/* These bits select the protocol for reading from or writing to the device. 
		 * 00b = Standard SPI with CPOL = 0 and CPHASE = 0 (default), 
		 * 01b = Standard SPI with CPOL = 0 and CPHASE = 1, 
		 * 10b = Standard SPI with CPOL = 1 and CPHASE = 0,
		 * 11b = Standard SPI with CPOL = 1 and CPHASE = 1. */
		uint8_t SDI_MODE		: 2;
		/* Reserved. Reads return 000000b. */
		uint8_t RESERVED7_2		: 6;
		/* Reserved. Reads return 00h. */
		uint8_t RESERVED15_8	: 8;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_16	: 16;
	};
	
} ADS867x_SdiCtrl_t;

/*
 * @brief Data protocol register type.
 * This register controls the data protocol used to transmit data out from the SDO-x pins of the device.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t SdoCtrlReg_LSW;
		uint16_t SdoCtrlReg_HSW;
	};
	struct {
		/* These bits control the data output modes of the device. 
		 * 0xb = SDO mode follows the same SPI protocol as that used for SDI, see the SDI_CTL_REG register (default), 
		 * 10b = Invalid configuration, 
		 * 11b = SDO mode follows the ADC master clock or source-synchronous protocol. */
		uint8_t SDO_MODE		: 2;
		/* Reserved. Reads return 0000b. */
		uint8_t RESERVED5_2		: 4;
		/* This bit controls the source of the clock selected for source-synchronous transmission. This bit takes effect 
		 * only in the ADC master clock or source-synchronous mode of operation.
		 * 0b = External SCLK (no division) (default),
		 * 1b = Internal clock (no division). */
		uint8_t SSYNC_CLK		: 1;
		/* Reserved. Reads return 0b. */
		uint8_t RESERVED7		: 1;
		/* Two bits are used to configure ALARM/SDO-1/GPO: 
		 * 00b = SDO-1 is always tri-stated; 1-bit SDO mode (default),
		 * 01b = SDO-1 functions as ALARM; 1-bit SDO mode,
		 * 10b = SDO-1 functions as GPO; 1-bit SDO mode,
		 * 11b = SDO-1 combined with SDO-0 offers a 2-bit SDO mode. */
		uint8_t SDO1_CONFIG		: 2;
		/* Reserved. Reads return 00b. */
		uint8_t RESERVED11_10	: 2;
		/* 1-bit value for the output on the GPO pin. */
		uint8_t GPO_VAL			: 1;
		/* Reserved. Reads return 000b. */
		uint8_t RESERVED15_13	: 3;
		/* Reserved. Reads return 0h. */
		uint16_t RESERVED31_16	: 16;
	};
	
} ADS867x_SdoCtrl_t;

/*
 * @brief Data output register type.
 * This register controls the data output by the device.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t DataOutReg_LSW;
		uint16_t DataOutReg_HSW;
	};
	struct {
		/* These bits control the data value output by the converter.
		 * 0xxb = Value output is the conversion data (default),
		 * 100b = Value output is all 0's,
		 * 101b = Value output is all 1's,
		 * 110b = Value output is alternating 0's and 1's,
		 * 111b = Value output is alternating 00's and 11's */
		uint8_t DATA_VAL				: 3;
		/* 0b = Output data does not contain parity information (default),
		 * 1b = Two parity bits (ADC output and output data frame) are appended to the LSBs of the output data.
		 * The ADC output parity bit reflects an even parity for the ADC output bits only. The output data frame parity 
		 * bit reflects an even parity signature for the entire output data frame, including the ADC output bits and any
		 * internal flags or register settings. Setting this bit increases the length of the output data by two bits. */
		uint8_t PAR_EN					: 1;
		/* Reserved. Reads return 0000b. */
		uint8_t RESERVED7_4				: 4;
		/* Control to include the 4-bit input range setting in the SDO-x output bit stream.
		 * 0b = Do not include the range configuration register value (default),
		 * 1b = Include the range configuration register value. */
		uint8_t RANGE_INCL				: 1;
		/* Reserved. Reads return 0h. */
		uint8_t RESERVED9				: 1;
		/* Control to include the active input ALARM flags in the SDO-x output bit stream. 
		 * 00b = Do not include (default),
		 * 01b = Include ACTIVE_IN_H_FLAG,
		 * 10b = Include ACTIVE_IN_L_FLAG,
		 * 11b = Include both flags. */
		uint8_t IN_ACTIVE_ALARM_INCL	: 2;
		/* Control to include the active VDD ALARM flags in the SDO-x output bit stream.
		 * 00b = Do not include (default), 
		 * 01b = Include ACTIVE_VDD_H_FLAG,
		 * 10b = Include ACTIVE_VDD_L_FLAG,
		 * 11b = Include both flags. */
		uint8_t VDD_ACTIVE_ALARM_INCL	: 2;
		/* Control to include the 4-bit DEVICE_ADDR register value in the SDO-x output bit stream.
		 * 0b = Do not include the register value (default),
		 * 1b = Include the register value. */
		uint8_t DEVICE_ADDR_INCL		: 1;
		/* Reserved. Reads return 0b. */
		uint8_t RESERVED15				: 1;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_16			: 16;
	};
	
} ADS867x_DataOut_t;

/*
 * @brief Internal reference and input voltage register type.
 * This register controls the configuration of the internal reference and input voltage ranges for the converter.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t RangeSelReg_LSW;
		uint16_t RangeSelReg_HSW;
	};
	struct {
		/* These bits comprise the 4-bit register that selects the nine input ranges of the ADC. */
		uint8_t RANGE_SEL		: 4;
		/* Reserved. Reads return 00b. */
		uint8_t RESERVED5_4		: 2;
		/* Control to disable the ADC internal reference. 
		 * 0b = Internal reference is enabled (default),
		 * 1b = Internal reference is disabled. */
		uint8_t INTREF_DIS		: 1;
		/* Reserved. Reads return 0b. */
		uint8_t RESERVED7		: 1;
		/* Reserved. Reads return 00h. */
		uint16_t RESERVED15_8	: 8;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_16	: 16;
	};
	
} ADS867x_RangeSel_t;

/*
 * @brief Output alarm flags register type.
 * This register contains the output alarm flags (active and tripped) for the input and AVDD alarm.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t AlarmReg_LSW;
		uint16_t AlarmReg_HSW;
	};
	struct {
		/* Logical OR outputs all tripped ALARM flags.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t OVW_ALARM			: 1;
		/* Reserved. Reads return 000b. */
		uint8_t RESERVED3_1			: 3;
		/* Tripped ALARM output flag for low input voltage.
		 * 0b = No ALARM condition (default), 
		 * 1b = ALARM condition exists. */
		uint8_t TRP_IN_H_FLAG		: 1;
		/* Tripped ALARM output flag for high input voltage.
		 * 0b = No ALARM condition (default), 
		 * 1b = ALARM condition exists. */
		uint8_t TRP_IN_L_FLAG		: 1;
		/* Tripped ALARM output flag for high AVDD voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t TRP_VDD_H_FLAG		: 1;
		/* Tripped ALARM output flag for low AVDD voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t TRP_VDD_L_FLAG		: 1;
		/* Reserved. Reads return 00b. */
		uint8_t RESERVED9_8			: 2;
		/* Active ALARM output flag for low input voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t ACTIVE_IN_H_FLAG	: 1;
		/* Active ALARM output flag for high input voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t ACTIVE_IN_L_FLAG	: 1;
		/* Reserved. Reads return 00b. */
		uint8_t RESERVED13_12		: 2;
		/* Active ALARM output flag for high AVDD voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t ACTIVE_VDD_H_FLAG	: 1;
		/* Active ALARM output flag for low AVDD voltage.
		 * 0b = No ALARM condition (default),
		 * 1b = ALARM condition exists. */
		uint8_t ACTIVE_VDD_L_FLAG	: 1;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_16		: 16;
	};
	
} ADS867x_Alarm_t;

/*
 * @brief Hysteresis and high threshold register type.
 * This register controls the hysteresis and high threshold for the input alarm.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t AlarmHTh_LSW;
		uint16_t AlarmHTh_HSW;
	};
	struct {
		/* Reserved, must be set to 00b. */
		uint8_t RESERVED1_0			: 2;
		/* Threshold for comparison is INP_ALRM_HIGH_TH[15:2]. INP_ALRM_HIGH_TH[1:0] must be set to 00b. Default is 0xFFFF. */
		uint16_t INP_ALRM_HIGH_TH	: 14;
		/* Reserved. Reads return 00h. */
		uint8_t RESERVED23_16		: 8;
		/* Reserved, must be se to 0000b. */
		uint8_t RESERVED27_24		: 4;
		/* INP_ALRM_HYST[7:4]: 4-bit hysteresis value for the input ALARM. INP_ALRM_HYST[3:0] must be set to 0000b. */
		uint8_t INP_ALRM_HYST		: 4;
	};
	
} ADS867x_AlarmHTh_t;

/*
 * @brief Low threshold register type.
 * This register controls the low threshold for the input alarm.
 * LSW and HSW are low significant word and high significant word respectively.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint16_t AlarmLTh_LSW;
		uint16_t AlarmLTh_HSW;
	};
	struct {
		/* Reserved, must be set to 00b. */
		uint8_t RESERVED1_0			: 2;
		/* Threshold for comparison is INP_ALRM_LOW_TH[15:2]. INP_ALRM_LOW_TH[1:0] must be set to 00b. */
		uint16_t INP_ALRM_LOW_TH	: 14;
		/* Reserved. Reads return 0000h. */
		uint16_t RESERVED31_15		: 16;
	};
	
} ADS867x_AlarmLTh_t;

/*
 * @brief Output data word with all data flags enabled or all disabled.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint32_t DataWord;	
	struct {		
		uint8_t DataWord_LSW_LSB	: 8;		
		uint8_t DataWord_LSW_MSB	: 8;
		uint8_t DataWord_HSW_LSB	: 8;
		uint8_t DataWord_HSW_MSB	: 8;
	};
	/* All data flags enabled format. */
	struct {
		uint16_t RESERVED3_0		: 4;
		uint16_t PARITY_BITS		: 2;
		uint16_t ADC_INPUT_RANGE	: 4;
		uint16_t INPUT_ALARM_FLAGS	: 2;
		uint16_t AVDD_ALARM_FLAGS	: 2;
		uint16_t DEVICE_ADDRESS		: 4;
		uint16_t CONVERSION_RESULT	: 14;
	};
	
	/* All data flags disabled format. */
	struct {
		uint32_t RESERVED17_0_ALL		: 18;
		uint32_t CONVERSION_RESULT_ALL	: 14;
	};
	
} ADS867x_OutputDataWord_t;


/*
 * @brief Read/write data struct.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		/* Cmd + address. */
		//uint16_t CmdAddr;
		uint8_t Command;
		uint8_t Address;
		/* Data. */
		uint16_t Data;		
	};
	struct {

		/* */
		uint8_t MSB_ADDR_BIT	: 1;
		/* Command code. */
		uint8_t COMMAND			: 7;
		/* Address. */
		uint16_t ADDRESS		: 8; 
		/* Read or write data word. */
		uint8_t REG_DATA_MSB	: 8;
		uint8_t REG_DATA_LSB	: 8;
	};
	
} ADS867x_InputCmd_t;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	ADS867x_InputCmd_t data;
	volatile uint16_t* tx_byte_cnt; 
	volatile uint16_t* rx_byte_cnt;
	delay_fptr delay;
	spi_txrx_fptr spi_tx;
	spi_txrx_fptr spi_rx;

} ADS867x_GInst_t;


/* Public function prototypes. */
void ADS867x_Init(void);
float ADS867x_GetVoltage(void);
ADS867x_OutputDataWord_t ADS867x_ReadADC(ADS867x_GInst_t* device);
uint16_t ADS867x_W_R_REG(ADS867x_GInst_t* device, uint8_t address);
void ADS867x_RstPwdn(ADS867x_GInst_t* device, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis);
void ADS867x_SdiCtrl(ADS867x_GInst_t* device, ADS867x_SPIProtocol protocol);
void ADS867x_SdoCtrl(ADS867x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk, ADS867x_SDO1Mode sdo1_config);
void ADS867x_SdoPinSetReset(ADS867x_GInst_t* device, uint8_t sdo_val);
void ADS867x_DataOutCtrl(ADS867x_GInst_t* device, ADS867x_DataVal data_val, uint8_t par_en, uint8_t range_incl, 
	ADS867x_InActiveAlarm in_active_alarm_incl, ADS867x_VddActiveAlarm vdd_active_alarm_incl, uint8_t device_addr_incl);
void ADS867x_RangeSel(ADS867x_GInst_t* device, ADS867x_InputRange range_sel, uint8_t intref_dis);
ADS867x_Alarm_t ADS867x_ReadAlarm(ADS867x_GInst_t* device);
void ADS867x_SetAlarmHTh(ADS867x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst);
void ADS867x_SetAlarmLTh(ADS867x_GInst_t* device, uint16_t inp_alrm_low_th);

/* Hardware dependent function prototypes. */
void ADS867x_SPI_Tx(uint8_t *pData, uint8_t size);
void ADS867x_SPI_Rx(uint8_t *pData, uint8_t size);


#endif /* ADS867x_H_ */

