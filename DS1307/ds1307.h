/*
 *	@brief DS1307 driver.
 *	Created 9.11.2020
 *	DS1307_H_
 *
 **/

#ifndef DS1307_H_				  
#define DS1307_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief DS1307 variant of address.
 *
 **/
typedef enum
{
	DS1307_ADDR         = 0x68,
	DS1307_ADDR_SHIFTED = 0xD0
	
} DS1307_DeviceAddress;

/*
 * @brief DS1307 register map.
 *
 **/
typedef enum
{
	///Range 00-59
	DS1307_SECONDS_REG = 0x00,
	///00-59.
	DS1307_MINUTES_REG,
	///1-12 for the 12 hours format + #AM/PM, 00-23 for the 24 hours format. 
	DS1307_HOURS_AND_DATAFORMAT_REG,
	///1-7. 
	DS1307_DAY_REG,
	///01-31.
	DS1307_DATE_REG,
	///01-12.
	DS1307_MONTH,
	///00-99.
	DS1307_YEAR_REG,
	
	///Control, status, aging and temperature registers.
	DS1307_CONTROL_REG,
	
	//Block of RAM memory. 56 bytes, stop address is 0x3F
	DS1307_RAM_BLOCK
	
} DS1307_RegisterMap;

/*
 * @brief Oscollator en/dis.
 *
 **/
typedef enum
{
	DS1307_OSC_EN,
	DS1307_OSC_DIS
	
} DS1307_ClockHalt;

/*
 * @brief INT/SQW signal selecting on the INT/SQW pin.
 *
 **/
typedef enum
{
	DS1307_SQW_DIS,
	DS1307_SQW_EN
	
} DS1307_SqwSelector;

/*
 * @brief Control the output level of the SQW/OUT pin.
 *
 **/
typedef enum
{
	DS1307_OUT_LOW,
	DS1307_OUT_HI
	
} DS1307_OutPinLevel;

/*
 * @brief SQW output frequency.
 *
 **/
typedef enum
{
	DS1307_SQW_1Hz,
	DS1307_SQW_4_096kHz,
	DS1307_SQW_8_192kHz,
	DS1307_SQW_32_768kHz
	
} DS1307_SQWRate;

/*
 * @brief Clock format.
 *
 **/
typedef enum
{
	DS1307_CLOCK_FORMAT_24HOURS = 0,
	DS1307_CLOCK_FORMAT_12HOURS,
	
} DS1307_ClockFormat;

/*
 * @brief Clock hour of day.
 *
 **/
typedef enum
{
	DS1307_HOUR_OF_DAY_NONE = 0,
	DS1307_HOUR_OF_DAY_AM   = 0,
	DS1307_HOUR_OF_DAY_PM
	
} DS1307_HourOfDay;

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
	uint8_t AmPm;
	
} DS1307_Clock_typedef;

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
	uint8_t Month;
	///Just a year. from 00 to 99.
	uint8_t Year;
	
	union 
	{
		///Control Register (0Eh).
		uint8_t ControlRegister;
		struct 
		{
			///Rate Select (RS2 and RS1).  Default is 11.
			///These bits control the frequency of the square-wave output when the square wave
			///has been enabled. See its values in the DS1307_SQWOutFreq enum above.
			uint8_t RS1_RS0		: 2;
			///
			uint8_t RESERVED_0	: 2;
			///This bit, when set to logic 1, enables the oscillator output. The frequency of
			//the square - wave output depends upon the value of the RS0 and RS1 bits. With the square-wave 
			//output set to 1Hz, the clock registers update on the falling edge of the square wave. On initial
			//application of power to the device, this bit is typically set to a 0.
			uint8_t SQWE		: 1;
			///
			uint8_t RESERVED_1	: 2;
			//This bit controls the output level of the SQW/OUT pin when the square-wave output is disabled.
			//If SQWE = 0, the logic level on the SQW / OUT pin is 1 if OUT = 1 and is 0 if OUT = 0. On initial
			//application of power to the device, this bit is typically set to a 0.
			uint8_t OUT			: 1;

		} partsCtrlReg;
	};
	
	//RAM block.
	uint8_t RamBlock[56];
	
} DS1307_GClockData_typedef;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	///Local clock data.
	DS1307_Clock_typedef clock;
	///Clock registers.
	DS1307_GClockData_typedef clock_reg;
	//Pointers for the rx, tx delay functions.
	delay_fptr delay;
	i2c_txrx_data_fptr i2c_tx_data;
	i2c_txrx_data_fptr i2c_rx_data;
	
} DS1307_GDataInstance_typedef;

/*
 *	@brief Public function prototype.
 *
 **/



#endif /* DS1307_H_ */