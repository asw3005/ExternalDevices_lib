﻿/*
 *	@brief DS12887 driver.
 *	Created 28.10.2020
 *	DS12887_H_
 *
 **/

#ifndef DS12887_H_				  
#define DS12887_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief Hardware dependent part.
 *
 **/
#define ADDRESS_PORT GPIOA
#define CONTROL_PORT GPIOB
///When connected to VCC, Motorola bus timing is selected.When connected to GND or left disconnected, 
///Intel bus timing is selected.
#define BUS_TIMING_SEL_PIN	GPIO_PIN_1 //It has an Intel state without use MCU pin.
///Active-Low Chip-Select Input. The chip-select signal must be asserted low for a bus cycle in the 
///device to be accessed. CS must be kept in the active state during DS and AS for Motorola timing and 
///during DS and R / W for Intel timing.
#define CHIP_SELECT_PIN		GPIO_PIN_0
///A positive-going address-strobe pulse serves to demultiplex the bus. The falling edge of AS causes 
///the address to be latched within the device.The next rising edge that occurs on the AS bus clears the
///address regardless of whether CS is asserted.
#define ADDRESS_STROBE_PIN  GPIO_PIN_1
///When the MOT pin is connected to VCC for Motorola timing, R/W is at a level that indicates whether 
///the current cycle is a read or write. A read cycle is indicated with a high level on R/W while DS is high.
///A write cycle is indicated when R/W is low during DS. When the MOT pin is connected to GND for Intel 
///timing, the R/W signal is an active-low signal. In this mode, the R/W pin operates in a similar fashion
///as the write-enable signal(WE) on generic RAMs.Data are latched on the rising edge of the signal.
#define READ_WRITE_PIN		GPIO_PIN_10
///W hen the MOT pinis connected to VCC , Motorola bus timing is sel ected. In this mode, DS is a positive pulse
///during the latter portion of the bus cycle and is called data strobe. During read cycles, DS signifies the 
///time that the device is to drive the bidirectional bus. In write cycles, the trailing edge of DS causes
///the device to latch the written data. When the MOT pin is connected to GND, Intel bus timing is selected.
///DS identifies the time period when the device drives the bus with read data. In this mode, the DS pin operates
///in a similar fashion as the output enable(OE) signal on a generic RAM.
#define DATA_STROBE_PIN		GPIO_PIN_11
///Active-Low Reset Input. The RESET pin has no effect on the clock, calendar, or RAM. On power - up, the RESET
///pin can be held low for a time to allow the power supply to stabilize. The amount of time that RESET is held
///low is dependent on the application. However, if RESET is used on power - up, the time RESET is low should
///exceed 200ms to ensure that the internal timer that controls the device on powerup has timed out. When RESET
///is low and VCC is above VPF, the following occurs: 
///A. Periodic interrupt - enable(PIE) bit is cleared to 0.
///B. Alarm interrupt - enable(AIE) bit is cleared to 0.
///C. Update - ended interrupt - enable(UIE) bit is cleared to 0.
///D. Periodic - interrupt flag(PF) bit is cleared to 0.
///E. Alarm - interrupt flag(AF) bit is cleared to 0.
///F. Update - ended interrupt flag(UF) bit is cleared to 0.
///G. Interrupt - request status flag(IRQF) bit is cleared to 0.
///H. IRQ pin is in the high - impedance state.
///I. The device is not accessible until RESET is returned high.
///J. Square - wave output - enable(SQWE) bit is cleared to 0.
///In a typical application, RESET can be connected to VCC. This connection allows the device to go in and out of 
///power fail without affecting any of the control registers.
#define RESET_PIN			GPIO_PIN_1 


/*
 * @brief DS12887 register map.
 *
 **/
typedef enum
{
	///Range 00-59
	DS12887_SECONDS_REG					= 0x00,
	///00-59.
	DS12887_MINUTES_REG					= 0x02,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format. 
	DS12887_HOURS_REG					= 0x04,
	///1-7. 
	DS12887_DAY_REG						= 0x06,
	///01-31.
	DS12887_DATE_REG,
	///01-12.
	DS12887_MONTH_REG,
	///00-99.
	DS12887_YEAR_REG,
	
	///Range 00-59.
	DS12887_ALARM_SECONDS_REG			= 0x01,
	///00-59.
	DS12887_ALARM_MINUTES_REG			= 0x03,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format.
	DS12887_ALARM_HOURS_REG				= 0x05,

	///Control, status, aging and temperature registers.
	DS12887_CONTROL_REG_A				= 0x0A,
	DS12887_CONTROL_REG_B,
	DS12887_CONTROL_REG_C,
	DS12887_CONTROL_REG_D,
	
	///RAM 0Eh-32h, 114 bytes, stop address is 0x7F
	DS12887_RAM_BLOCK
	
} DS12887_RegisterMap;

/*
 * @brief SQW output frequency.
 *
 **/
typedef enum
{
	DS12887_SQW_NONE      = 0,
	DS12887_SQW_256kHz,
	DS12887_SQW_128kHz,
	DS12887_SQW_8_192kHz,
	DS12887_SQW_4_096kHz,
	DS12887_SQW_2_048kHz,
	DS12887_SQW_1_024kHz,
	DS12887_SQW_512Hz,
	DS12887_SQW_256Hz,
	DS12887_SQW_128Hz,
	DS12887_SQW_64Hz,
	DS12887_SQW_32Hz,
	DS12887_SQW_16Hz,
	DS12887_SQW_8Hz,
	DS12887_SQW_4Hz,
	DS12887_SQW_2Hz
	
} DS12887_SQWIntFreq;

/*
 * @brief Oscillator on/off.
 *
 **/
typedef enum
{
	DS12887_OSC_ON							= 2,//010
	DS12887_OSC_ON_COUNTDOWN_CHAIN_RESET	= 6,//11x	
	
} DS12887_OscillatorOnOff;

/*
 * @brief Daylight saving bit.
 *
 **/
typedef enum
{
	DS12887_DAYLIGHT_DIS,
	DS12887_DAYLIGHT_EN
	
} DS12887_DaylightSaving;

/*
 * @brief Clock format.
 *
 **/
typedef enum
{
	DS12887_CLOCK_FORMAT_12HOURS,
	DS12887_CLOCK_FORMAT_24HOURS
	
} DS12887_ClockFormat;

/*
 * @brief Data format.
 *
 **/
typedef enum
{
	DS12887_DATA_FORMAT_BCD,
	DS12887_DATA_FORMAT_BIN
	
} DS12887_DataFormat;

/*
 * @brief Square wave output en/dis.
 *
 **/
typedef enum
{
	DS12887_SQW_DIS,
	DS12887_SQW_EN
	
} DS12887_SquareWave;

/*
 * @brief Update ended interrupt enable.
 *
 **/
typedef enum
{
	DS12887_UPDATE_ENDED_INTERRUPT_DIS,
	DS12887_UPDATE_ENDED_INTERRUPT_EN	
	
} DS12887_UpdateEndedInterrupt;

/*
 * @brief Alarm interrupt enable.
 *
 **/
typedef enum
{
	DS12887_ALARM_INTERRUPT_DIS,
	DS12887_ALARM_INTERRUPT_EN	
	
} DS12887_AlarmInterrupt;

/*
 * @brief Periodic interrupt enable.
 *
 **/
typedef enum
{
	DS12887_PERIODIC_INTERRUPT_DIS,
	DS12887_PERIODIC_INTERRUPT_EN	
	
} DS12887_PeriodicInterrupt;

/*
 * @brief Update inhibited.
 *
 **/
typedef enum
{
	DS12887_UPDATE_NORMAL,
	DS12887_UPDATE_INHIBITED	
	
} DS12887_UpdateClockReg;

/*
 * @brief Clock hour of day.
 *
 **/
typedef enum
{
	DS12887_HOUR_OF_DAY_NONE = 0,
	DS12887_HOUR_OF_DAY_NONE_AM   = 0,
	DS12887_HOUR_OF_DAY_NONE_PM
	
} DS12887_HourOfDay;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *
 *  @param  MemAddress : Internal memory address.
 *  @param  pData : Pointer to data buffer.
 *  @param  Size : Amount of data to be sent.
 *
 **/
typedef void(*txrx_data_fptr)(uint8_t MemAddress, uint8_t *pData, uint8_t Size);

/*
 *	@brief Clock data.
 *
 **/
typedef struct
{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Day;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
	
} DS12887_Clock_typedef;

/*
 *	@brief General clock data struct.
 *
 **/
typedef struct __attribute__((aligned(1), packed))
{
	///Clock infrastructure.		
	///From 00 to 59.
	uint8_t Seconds;
	///From 00 to 59.
	uint8_t SecondsAlarm;
	///From 00 to 59.
	uint8_t Minutes;
	///From 00 to 59.
	uint8_t MinutesAlarm;
	///From 1 to 12 #AM/PM or from 00 to 23 other one.
	uint8_t Hours;
	///From 1 to 12 #AM/PM or from 00 to 23 other one.
	uint8_t HoursAlarm;
	///Day of week.
	uint8_t Day;
	///Day of Month. From 01 to 31.
	uint8_t Date;
	///Day of Month and century. From 01 to 12.
	uint8_t Month;
	///Just a year. from 00 to 99.
	uint8_t Year;
	
	union 
	{
		///Control Register A (0Ah).
		uint8_t ControlRegisterA;
		struct 
		{
			//Rate Selector (RS3, RS2, RS1, RS0).
			//These four rate-selection bits select one of the 13 taps on the 15 - stage divider or disable 
			//the divider output. The tap selected can be used to generate an output square wave(SQW pin) 
			//and / or a periodic interrupt. See the DS12887_SQWIntFreq enum above.
			uint8_t RS3_RS0	: 4;
			//Oscillator on/off.
			//These three bits are used to turn the oscillator on or off and to reset the countdown chain.
			//A pattern of 010 is the only combination of bits that turn the oscillator on and allow the RTC
			//to keep time. A pattern of 11x enables the oscillator but holds the countdown chain in reset. The 
			//next update occurs at 500ms after a pattern of 010 is written to DV0, DV1, and DV2.
			uint8_t DV2_DV0	: 3;
			//Update-In-Progress (UIP).
			//This bit is a status flag that can be monitored. When the UIP bit is a 1, the update transfer 
			//occurs soon. When UIP is a 0, the update transfer does not occur for at least 244μs. The time, 
			//calendar, and alarm information in RAM is fully available for access when the UIP bit is 0. 
			//The UIP bit is read-only and is not affected by RESET. Writing the SET bit in Register B to a 
			//1 inhibits any update transfer and clears the UIP status bit.
			uint8_t UIP		: 1;

		} partsCtrlRegA;
	};
	
	union 
	{
		///Status Register B (0Bh).
		uint8_t ControlRegisterB;
		struct 
		{
			//Daylight Saving Enable (DSE).
			//This bit is a read / write bit that enables two daylight saving adjustments when DSE is set to 1. 
			//On the first Sunday in April, the time increments from 1 : 59 : 59 AM to 3 : 00 : 00 AM.On the 
			//last Sunday in October when the time first reaches 1 : 59 : 59 AM, it changes to 1 : 00 : 00 AM.
			//When DSE is enabled, the internal logic test for the first / last Sunday condition at midnight.
			//If the DSE bit is not set when the test occurs, the daylight saving function does not operate correctly.
			//These adjustments do not occur when the DSE bit is 0. This bit is not affected by internal functions 
			//or RESET.
			uint8_t DSE			: 1;
			//The 24/#12 control bit establishes the format of the hours byte. A 1 indicates the 24-hour mode
			//and a 0 indicates the 12 - hour mode.This bit is read / write and is not affected by internal functions 
			//or RESET.
			uint8_t FORMAT12_24	: 1;
			//Data Mode (DM).
			//This bit indicates whether time and calendar information is in binary or BCD format. The DM bit is
			//set by the program to the appropriate format and can be read as required.This bit is not modified
			//by internal functions or RESET. A 1 in DM signifies binary data, while a 0 in DM specifies BCD data.
			uint8_t DM			: 1;
			//Square-Wave Enable (SQWE).
			//When this bit is set to 1, a square - wave signal at the frequency set by the rate - selection bits
			//RS3–RS0 is driven out on the SQW pin. When the SQWE bit is set to 0, the SQW pin is held low. SQWE
			//is a read / write bit and is cleared by RESET. SQWE is low if disabled, and is high impedance when
			//VCC is below VPF.SQWE is cleared to 0 on RESET.
			uint8_t SQWE		: 1;
			//Update-Ended Interrupt Enable (UIE).
			//This bit is a read / write bit that enables the update - end flag(UF) bit in Register C to assert 
			//IRQ. The RESET pin going low or the SET bit going high clears the UIE bit. The internal functions of 
			//the device do not affect the UIE bit, but is cleared to 0 on RESET.
			uint8_t UIE			: 1;
			//Alarm Interrupt Enable (AIE).
			//This bit is a read / write bit that, when set to 1, permits the alarm flag (AF) bit in Register C to
			//assert IRQ. An alarm interrupt occurs for each second that the three time bytes equal the three alarm
			//bytes, including a don’t - care alarm code of binary 11XXXXXX. The AF bit does not initiate the IRQ
			//signal when the AIE bit is set to 0. The internal functions of the device do not affect the AIE bit,
			//but is cleared to 0 on RESET.
			uint8_t AIE			: 1;
			//Periodic Interrupt Enable (PIE).
			//The PIE bit is a read / write bit that allows the periodic interrupt flag(PF) bit in Register C to drive
			//the IRQ pin low. When the PIE bit is set to 1, periodic interrupts are generated by driving the IRQ
			//pin low at a rate specified by the RS3–RS0 bits of Register A. A 0 in the PIE bit blocks the IRQ output
			//from being driven by a periodic interrupt, but the PF bit is still set at the periodic rate.PIE is 
			//not modified by any internal device functions, but is cleared to 0 on RESET.
			uint8_t PIE			: 1;
			//When the SET bit is 0, the update transfer functions normally by advancing the counts once per second.
			//When the SET bit is written to 1, any update transfer is inhibited, and the program can initialize 
			//the time and calendar bytes without an update occurring in the midst of initializing. Read cycles can
			//be executed in a similar manner. SET is a read / write bit and is not affected by RESET or internal 
			//functions of the device.
			uint8_t SET			: 1;
		
		} partsStatRegB;
	};
	
	union 
	{
		///Control Register C (0Ch).
		uint8_t ControlRegisterC;
		struct 
		{
			//These bits are unused in Register C. These bits always read 0 and cannot be written.
			uint8_t RESERVED_0	: 4;
			//Update-Ended Interrupt Flag (UF).
			//This bit is set after each update cycle.When the UIE bit is set to 1, the 1 in UF causes the IRQF
			//bit to be a 1, which asserts the IRQ pin.This bit can be cleared by reading Register C or with a 
			//RESET.
			uint8_t UF			: 1;
			//Alarm Interrupt Flag (AF).
			//A 1 in the AF bit indicates that the current time has matched the alarm time. If the AIE bit is 
			//also 1, the IRQ pin goes low and a 1 appears in the IRQF bit.This bit can be cleared by reading 
			//Register C or with a RESET.
			uint8_t AF			: 1;
			//Periodic Interrupt Flag (PF).
			//This bit is readonly and is set to 1 when an edge is detected on the selected tap of the divider 
			//chain. The RS3 through RS0 bits establish the periodic rate.PF is set to 1 independent of the state
			//of the PIE bit. When both PF and PIE are 1s, the IRQ signal is active and sets the IRQF bit. This
			//bit can be cleared by reading Register C or with a RESET.
			uint8_t PF			: 1;
			//Interrupt Request Flag (IRQF).
			//This bit is set to 1 when any of the following are true : PF = PIE = 1, AF = AIE = 1, UF = UIE = 1.
			//Any time the IRQF bit is 1, the IRQ pin is driven low. This bit can be cleared by reading Register C
			//or with a RESET.
			uint8_t IRQF		: 1;

		} partsCtrlRegC;
	};
	
	union 
	{
		///Control Register D (0Dh).
		uint8_t ControlRegisterD;
		struct 
		{
			//The remaining bits of Register D are not usable.They cannot be written and they always read 0.
			uint8_t RESERVED_0	: 7;
			//Valid RAM and Time (VRT).
			//This bit indicates the condition of the battery connected to the VBAT pin. This bit is not writeable
			//and should always be 1 when read. If a 0 is ever present, an exhausted internal lithium energy source
			//is indicated and both the contents of the RTC data and RAM data are questionable. This bit is unaffected
			//by RESET.
			uint8_t VRT			: 1;

		} partsCtrlRegD;
	};
	
	//RAM memory block A
	uint8_t RamBlock[114];
	
} DS12887_GClockData_typedef;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	///Local clock data.
	DS12887_Clock_typedef clock;
	///Clock registers.
	DS12887_GClockData_typedef clock_reg;
	//Pointers for the rx, tx delay functions.
	delay_fptr delay;
	txrx_data_fptr tx_data;
	txrx_data_fptr rx_data;
	
} DS12887_GDataInstance_typedef;

/*
 *	@brief Public function prototype.
 *
 **/
void DS12887_Set_Time(DS12887_GDataInstance_typedef *device, DS12887_HourOfDay am_pm, uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS12887_Set_Date(DS12887_GDataInstance_typedef *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year);
void DS12887_Set_Alarm(DS12887_GDataInstance_typedef *device, DS12887_HourOfDay am_pm, uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS12887_Set_ControlRegA(DS12887_GDataInstance_typedef *device, DS12887_SQWIntFreq rs3_0, DS12887_OscillatorOnOff dv2_0);
void DS12887_Set_ControlRegB(DS12887_GDataInstance_typedef *device, DS12887_DaylightSaving daylight, DS12887_ClockFormat clock_format, DS12887_DataFormat data_mode, 
	DS12887_SquareWave sqwe, DS12887_UpdateEndedInterrupt uie, DS12887_AlarmInterrupt aie, DS12887_PeriodicInterrupt pie, DS12887_UpdateClockReg set);
void DS12887_Get_TimeDate(DS12887_GDataInstance_typedef *device);
void DS12887_Get_ControlRegC(DS12887_GDataInstance_typedef *device);
void DS12887_Get_ControlRegD(DS12887_GDataInstance_typedef *device);
void DS12887_ReadRamBlock(DS12887_GDataInstance_typedef *device);
void DS12887_WriteRamBlock(DS12887_GDataInstance_typedef *device);

/*
 * @brief Hardware dependent function for write and read byte(s).
 *
 **/
void DS12887_WriteI(uint8_t MemAddress, uint8_t *pData, uint8_t size);
void DS12887_ReadI(uint8_t MemAddress, uint8_t *pData, uint8_t size);


#endif /* DS12887_H_ */
