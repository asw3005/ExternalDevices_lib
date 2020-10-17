/*
 *	@brief DS3231 driver.
 *	Created 28.09.2020
 *	DS3231_H_
 *
 **/

#ifndef DS3231_H_				  
#define DS3231_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief DS3231 variant of address.
 *
 **/
typedef enum
{
	DS3231_ADDR = 0x68,
	DS3231_ADDR_SHIFTED = 0xD0
	
} DS3231_DeviceAddress;

/*
 * @brief DS3231 register map.
 *
 **/
typedef enum
{
	///Range 00-59
	DS3231_SECONDS_REG = 0x00,
	///00-59.
	DS3231_MINUTES_REG,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format. 
	DS3231_HOURS_AND_DATAFORMAT_REG,
	///1-7. 
	DS3231_DAY_REG,
	///01-31.
	DS3231_DATE_REG,
	///01-12 + Century.
	DS3231_MONTH_CENTURY_REG,
	///00-99.
	DS3231_YEAR_REG,
	
	///Range 00-59.
	DS3231_ALARM1_SECONDS_REG,
	///00-59.
	DS3231_ALARM1_MINUTES_REG,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format.
	DS3231_ALARM1_HOURS_AND_DATAFORMAT,
	///1-7 for the day, 1-31 for the data.
	DS3231_ALARM1_DAY_DATE_AND_DYDT_FORMAT,	
	
	///Range 00-59.
	DS3231_ALARM2_MINUTES_REG,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format.
	DS3231_ALARM2_HOURS_AND_DATAFORMAT,
	///1-7 for the day, 1-31 for the data.
	DS3231_ALARM2_DAY_DATE_AND_DYDT_FORMAT,
	
	///Control, status, aging and temperature registers.
	DS3231_CONTROL_REG,
	DS3231_STATUS_REG,
	DS3231_AGING_OFFSET_REG,
	DS3231_MSB_OF_TEMP_REG,
	DS3231_LSB_OF_TEMP_REG
	
} DS3231_RegisterMap;

/*
 * @brief Alarm register masks.
 *
 **/
typedef enum
{
	///Bit sequence is DY/#DT -> A1M4 -> A1M3 -> A1M2 -> A1M1, then A1M1 is LSB. 
	DS3231_ALARM_ONCE_PER_SECOND					= 0x0F,
	DS3231_ALARM_SECONDS_MATCH						= 0x0E,
	DS3231_ALARM_MINUTES_SECONDS_MATCH				= 0x0C,
	DS3231_ALARM_HOURS_MINUTES_SECONDS_MATCH		= 0x08,
	DS3231_ALARM_DATE_HOURS_MINUTES_SECONDS_MATCH	= 0x00,
	DS3231_ALARM_DAY_HOURS_MINUTES_SECONDS_MATCH	= 0x10
	
} DS3231_AlarmCondition;


/*
 * @brief SQW output frequency.
 *
 **/
typedef enum
{
	DS3231_SQW_1Hz = 0,
	DS3231_SQW_1_024kHz,
	DS3231_SQW_4_096kHz,
	DS3231_SQW_8_192kHz
	
} DS3231_SQWOutFreq;

/*
 * @brief SQW output frequency.
 *
 **/
typedef enum
{
	DS3231_CLOCK_FORMAT_24HOURS = 0,
	DS3231_CLOCK_FORMAT_12HOURS
	
} DS3231_ClockFormat;

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
 *  @param  DevAddress : Target device address: The device 7 bits address value
 *          in datasheet must be shifted to the left before calling the interface
 *  @param  MemAddress : Internal memory address
 *  @param  MemAddSize : Size of internal memory address
 *  @param  pData : Pointer to data buffer
 *  @param  Size : Amount of data to be sent
 *
 **/
typedef void(*i2c_txrx_data_fptr)(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

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
	
} DS3231_Clock_typedef;

/*
 *	@brief Alarm 1 data.
 *
 **/
typedef struct
{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Day;
	uint8_t Date;
	
} DS3231_Alarm1_typedef;

/*
 *	@brief Alarm 2 data.
 *
 **/
typedef struct
{
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Day;
	uint8_t Date;
	
} DS3231_Alarm2_typedef;

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
	uint8_t Minutes;
	///From 1 to 12 #AM/PM or from 00 to 23 other one.
	uint8_t Hours;
	///Day of week.
	uint8_t Day;
	///Day of Month. From 01 to 31.
	uint8_t Date;
	///Day of Month and century. From 01 to 12.
	uint8_t MonthCentury;
	///Just a year. from 00 to 99.
	uint8_t Year;
	
	///Alarm.
	///From 00 to 59.
	uint8_t Alarm1Seconds;
	///From 00 to 59.
	uint8_t Alarm1Minutes;
	///From 1 to 12 #AM/PM or from 00 to 23 other one.
	uint8_t Alarm1Hours;
	///
	uint8_t Alarm1DayDate;
	
	///From 00 to 59.
	uint8_t Alarm2Minutes;
	///From 1 to 12 #AM/PM or from 00 to 23 other one.
	uint8_t Alarm2Hours;
	///
	uint8_t Alarm2DayDate;
	
	union 
	{
		///Control Register (0Eh).
		uint8_t ControlRegister;
		struct 
		{
			///Alarm 1 and alarm 2 Interrupt Enable. Default is 0.
			///When set to logic 1, this bit permits the alarm 1 flag(A1F) bit in the
			///status register to assert INT / SQW(when INTCN = 1).When the A1IE bit 
			///is set to logic 0 or INTCN is set to logic 0, the A1F bit does not 
			///initiate the INT / SQW signal. The A1IE bit is disabled(logic 0) when 
			///power is first applied.
			uint8_t A1IE	: 1;
			uint8_t A2IE	: 1;
			///Interrupt Control (INTCN).  Default is 1.
			///This bit controls the INT / SQW signal. When the INTCN bit is set to logic 0,
			///a square wave is output on the INT / SQW pin. When the INTCN bit is set to 
			///logic 1, then a match between the timekeeping registers and either of the 
			///alarm registers activates the INT / SQW output (if the alarm is also enabled).
			///The corresponding alarm flag is always set regardless of the state of the INTCN
			///bit. The INTCN bit is set to logic 1 when power is first applied.
			uint8_t INTCN	: 1;
			///Rate Select (RS2 and RS1).  Default is 11.
			///These bits control the frequency of the square - wave output when the square wave
			///has been enabled. See its values in the DS3231_SQWOutFreq enum above.
			uint8_t RS2_RS1 : 2;
			///Convert Temperature (CONV).  Default is 0.
			///Setting this bit to 1 forces the temperature sensor to convert the temperature into 
			///digital code and execute the TCXO algorithm to update the capacitance array to the 
			///oscillator. The user should check the status bit BSY before forcing the controller 
			///to start a new TCXO execution.
			uint8_t CONV	: 1;
			///Battery-Backed Square-Wave Enable (BBSQW).  Default is 0.
			///When set to logic 1 with INTCN = 0 and VCC < VPF, this bit enables the square wave.
			///When BBSQW is logic 0, the INT / SQW pin goes high impedance when VCC < VPF.This bit 
			///is disabled(logic 0) when power is first applied.
			uint8_t BBSQW	: 1;
			///Enable Oscillator (EOSC). Default is 0. Active level is zero.
			///When set to logic 0, the oscillator is started. When set to logic 1, the oscillator
			///is stopped when the DS3231 switches to VBAT. This bit is clear (logic 0) when power 
			///is first applied. When the DS3231 is powered by VCC, the oscillator is always on 
			///regardless of the status of the EOSC bit. When EOSC is disabled, all register data 
			///is static.
			uint8_t EOSC	: 1;

		} partsCtrlReg;
	};
	
	union 
	{
		///Status Register (0Fh).
		uint8_t StatusRegister;
		struct 
		{
			///Alarm 1 and alarm 2 Flags (A1F). Default is N/A.
			///A logic 1 in the alarm 1 flag bit indicates that the time matched the alarm 1 registers.
			///If the A1IE bit is logic 1 and the INTCN bit is set to logic 1, the INT / SQW pin is also 
			///asserted. A1F is cleared when written to logic 0. This bit can only be written to logic 0. 
			///Attempting to write to logic 1 leaves the value unchanged.
			uint8_t A1F			: 1;
			uint8_t A2F			: 1;
			///Busy (BSY). Default is N/A.
			///This bit indicates the device is busy executing TCXO functions. It goes to logic 1 when the
			///conversion signal to the temperature sensor is asserted and then is cleared when the device 
			///is in the 1 - minute idle state.
			uint8_t BSY			: 1;
			///Enable 32kHz Output (EN32kHz). Default is 1.
			///This bit controls the status of the 32kHz pin.When set to logic 1, the 32kHz pin is enabled 
			///and outputs a 32.768kHz squarewave signal. When set to logic 0, the 32kHz pin goes to a high 
			///-impedance state.The initial power - up state of this bit is logic 1, and a 32.768kHz square
			///-wave signal appears at the 32kHz pin after a power source is applied to the DS3231(if the 
			///oscillator is running).
			uint8_t EN32kHz		: 1;
			///Reading as zero.
			uint8_t RESERVED_0	: 3;
			///Oscillator Stop Flag (OSF). Default is 1.
			///A logic 1 in this bit indicates that the oscillator either is stopped or was stopped for some 
			///period and may be used to judge the validity of the timekeeping data.This bit is set to logic 
			///1 any time that the oscillator stops.
			uint8_t OSF			: 1;
		
		} partsStatReg;
	};
	
	///Aging Offset(10h)
	///The aging offset register takes a user-provided value to add to or subtract from the codes in the 
	///capacitance array registers.
	int8_t AgingOffset;
	///Temperature is represented as a 10-bit code with a resolution of 0.25°C.
	///Temperature Register (Upper Byte) (11h)
	int8_t TemperatureMSB;
	///Temperature Register(Lower Byte) (12h)
	///Only two msb bits used, another all read as zero.
	uint8_t TemperatureLSB;	
	
} DS3231_GClockData_typedef;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	///Local clock data.
	DS3231_Clock_typedef clock;
	DS3231_Alarm1_typedef alarm1;
	DS3231_Alarm2_typedef alarm2;
	///Clock registers.
	DS3231_GClockData_typedef clock_reg;
	//Pointers for the rx, tx delay functions.
	delay_fptr delay;
	i2c_txrx_data_fptr i2c_tx_data;
	i2c_txrx_data_fptr i2c_rx_data;
	
} DS3231_GDataInstance_typedef;

/*
 *	@brief Public function prototype.
 *
 **/
void DS3231_Set_Time(DS3231_GDataInstance_typedef *device, DS3231_ClockFormat clock_format, uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS3231_Set_Date(DS3231_GDataInstance_typedef *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year);
void DS3231_Set_Alarm1(DS3231_GDataInstance_typedef *device, DS3231_AlarmCondition alarm_condition, DS3231_ClockFormat clock_format, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day_date);
void DS3231_Set_Alarm2(DS3231_GDataInstance_typedef *device, DS3231_AlarmCondition alarm_condition, DS3231_ClockFormat clock_format, uint8_t minutes, uint8_t hours, uint8_t day_date);
void DS3231_Set_ControlReg(DS3231_GDataInstance_typedef *device, uint8_t a1ie, uint8_t a2ie, uint8_t intcn, 
	DS3231_SQWOutFreq rs2_rs1, 	uint8_t conv,  uint8_t bbsqw,  uint8_t eosc);
void DS3231_Set_StatusReg(DS3231_GDataInstance_typedef *device, uint8_t a1f, uint8_t a2f, uint8_t en32khz);

void DS3231_Get_TimeDate(DS3231_GDataInstance_typedef *device);
void DS3231_Get_ControlReg(DS3231_GDataInstance_typedef *device);

#endif /* DS3231_H_ */
