/*
 *	@brief MAX31343 driver.
 *	Created 05.07.21 by asw3005.
 *	MAX31343_H_
 *
 **/

#ifndef MAX31343_H_				  
#define MAX31343_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/*
 * @brief MAX31343 address.
 *
 **/
typedef enum
{
	MAX31343_ADDR			= 0x68,
	MAX31343_ADDR_SHIFTED	= 0xD0
	
} MAX31343_DevAddr;

/*
 * @brief MAX31343 register map.
 *
 **/
typedef enum
{
	/* Status reg. */
	MAX31343_STATUS_REG,
	/* Interrupt enable register. */
	MAX31343_INT_EN_REG,
	/* RTC reset. */
	MAX31343_RTC_RST,
	/* RTC config 1. */
	MAX31343_RTC_CFG1,
	/* RTC config 2. */
	MAX31343_RTC_CFG2,
	/* Timer config. */
	MAX31343_TIMER_CFG,
	/* Seconds configuration register, range is 00-59 */
	MAX31343_SECONDS,
	/* Minutes configuration register, range is 00-59. */
	MAX31343_MINUTES,
	/* Hours configuration register, range is 00-23. */ 
	MAX31343_HOURS,
	/* Day configuration register, range is 1-7. */ 
	MAX31343_DAY,
	/* Date configuration register, range is 01-31. */
	MAX31343_DATE,
	/* Month configuration register, range is 01-12 + seventh bit is century. */
	MAX31343_MONTH_CENTURY,
	/* Year configuration register, range is 00-99. */
	MAX31343_YEAR,
	
	/* Alarm 1 seconds configuration register, range is 00-59. */
	MAX31343_ALARM1_SECONDS,
	/* Alarm 1 minutes configuration register, range is 00-59. */
	MAX31343_ALARM1_MINUTES,
	/* Alarm 1 hours configuration register, range is 0-23. */
	MAX31343_ALARM1_HOURS,
	/* Alarm 1 date/date configuration register, range of days is 1-7, date is 1-31. */
	MAX31343_ALARM1_DAY_DATE_DYDT,	
	/* Alarm 1 month configuration register, range is 1-12. */
	MAX31343_ALARM1_MONTH,	
	/* Alarm 1 year configuration register, range 0-99. */
	MAX31343_ALARM1_YEAR,	
	
	/* Alarm 2 minutes configuration register, range is 00-59. */
	MAX31343_ALARM2_MINUTES,
	/* Alarm 2 hours configuration register, range is 0-23. */
	MAX31343_ALARM2_HOURS,
	/* Alarm 2 date/date configuration register, range of days is 1-7, date is 1-31. */
	MAX31343_ALARM2_DAY_DATE_DYDT,	
	
	/* Countdown timer value register. */
	MAX31343_TIMER_CNT,
	/* Countdown timer initialisation register. */
	MAX31343_TIMER_INIT,
	/* Power management configuration register. */
	MAX31343_PWR_MGMT,
	/* Trickle charge configuration register. */
	MAX31343_TRICKLE_REG,
	/* Temperature value register, MSB. */
	MAX31343_TEMP_MSB,
	/* Temperature value register, LSB. */
	MAX31343_TEMP_LSB,
	/* Temperature sensor configuration register. */
	MAX31343_TS_CFG,
	
	/* RAM registers. */
	MAX31343_RAM_START	= 0x22,
	MAX31343_RAM_STOP	= 0x61
	
} MAX31343_RegMap;

/*
 * @brief Alarm 1 enable register masks.
 *
 **/
typedef enum
{
	/* MSB->LSB - DY_DT->A1M6->A1M5->A1M4->A1M3->A1M2->A1M1. */ 
	MAX31343_ALARM1_ONCE_PER_SEC			= 0x3F,
	MAX31343_ALARM1_SEC						= 0x3E,
	MAX31343_ALARM1_MIN_SEC					= 0x3C,
	MAX31343_ALARM1_HOUR_MIN_SEC			= 0x38,
	MAX31343_ALARM1_DATE_TIME				= 0x30,
	MAX31343_ALARM1_MONTH_DATE_TIME			= 0x20,
	MAX31343_ALARM1_YEAR_MONTH_DATE_TIME	= 0x00,
	MAX31343_ALARM1_DAY_TIME				= 0x70
	
} MAX31343_Alarm1Mask;

/*
 * @brief Alarm 2 enable register masks.
 *
 **/
typedef enum
{
	/* MSB->LSB - DY_DT->A1M4->A1M3->A1M2. */ 
	MAX31343_ALARM2_ONCE_PER_MIN	= 0x0E,
	MAX31343_ALARM2_MIN				= 0x0C,
	MAX31343_ALARM2_HOUR_MIN		= 0x08,
	MAX31343_ALARM2_DATE_HOUR_MIN	= 0x00,
	MAX31343_ALARM2_DAY_HOUR_MIN	= 0x40
	
} MAX31343_Alarm2Mask;

/*
 * @brief SQW output frequency.
 *
 **/
typedef enum
{
	MAX31343_SQW_1Hz,
	MAX31343_SQW_2Hz,
	MAX31343_SQW_4Hz,
	MAX31343_SQW_8Hz,
	MAX31343_SQW_16Hz,
	/* Range from 0x05 to 0x07. */
	MAX31343_SQW_32Hz,
	
} MAX31343_SQWRate;

/*
 * @brief CLKO output frequency.
 *
 **/
typedef enum
{
	MAX31343_CLKO_1Hz,
	MAX31343_CLKO_2Hz,
	MAX31343_CLKO_4Hz,
	MAX31343_CLKO_8Hz,
	MAX31343_CLKO_16Hz,
	MAX31343_CLKO_32Hz,
	MAX31343_CLKO_64Hz,
	MAX31343_CLKO_128Hz,
	/* Range from 0x08 to 0x0F. */
	MAX31343_CLKO_32kHz
	
} MAX31343_CLKORate;

/*
 * @brief Timer frequency.
 *
 **/
typedef enum
{
	MAX31343_TFS_1024Hz,
	MAX31343_TFS_256Hz,
	MAX31343_TFS_64Hz,
	MAX31343_TFS_16Hz,
	
} MAX31343_TFSRate;

/*
 * @brief Power fail threshold voltage.
 *
 **/
typedef enum
{
	MAX31343_PFVT_RESERVED,
	MAX31343_PFVT_1V8,
	MAX31343_PFVT_2V0,
	MAX31343_PFVT_2V4,
	
} MAX31343_PFVTVoltage;

/*
 * @brief Trickle charger enable.
 *
 **/
typedef enum
{
	/* The ranges from 0x00 to 0x04 and from 0x06 to 0x0F are disable trickle value. */
	MAX31343_TCHE_DIS,
	MAX31343_TCHE_EN = 0x05,
	
} MAX31343_TCHEEnable;

/*
 * @brief Sets the charger path for trickle charger. Must set TCHE to 0x05 to enable the setting below.
 *
 **/
typedef enum
{
	/* 3KΩ in series with a Schottky diode. */
	MAX31343_DTRICKLE0,
	/* 3KΩ in series with a Schottky diode. */
	MAX31343_DTRICKLE1,
	/* 6KΩ in series with a Schottky diode. */
	MAX31343_DTRICKLE2,
	/* 11KΩ in series with a Schottky diode. */
	MAX31343_DTRICKLE3,
	/* 3KΩ in series with a diode+Schottky diode. */
	MAX31343_DTRICKLE4,
	/* 3KΩ in series with a diode+Schottky diode */
	MAX31343_DTRICKLE5,
	/* 6KΩ in series with a diode+Schottky diode. */
	MAX31343_DTRICKLE6,
	/* 11KΩ in series with a diode+Schottky diode. */
	MAX31343_DTRICKLE7,	
	/* The ranges from 0x08 to 0x0F is no connection. */
	MAX31343_DTRICKLE_NC
	
} MAX31343_DTRICKLEPath;

/*
 * @brief Temperature measurement interval.
 *
 **/
typedef enum
{
	MAX31343_TTSINT_1Sec,
	MAX31343_TTSINT_2Sec,
	MAX31343_TTSINT_4Sec,
	MAX31343_TTSINT_8Sec,
	MAX31343_TTSINT_16Sec,
	MAX31343_TTSINT_32Sec,
	MAX31343_TTSINT_64Sec,
	MAX31343_TTSINT_128Sec
	
} MAX31343_TTSINTInterval;
	
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
	
} MAX31343_Clock_t;

/*
 *	@brief Status configuration register.
 *
 **/
typedef union
{
	uint8_t StatusReg;
	struct
	{
		/* Alarm1 Interrupt flag. 0x0: When set to zero resets the digital block. 0x01: When set one then device is not 
		 * on reset mode. Reset state is 0. */
		uint8_t A1F			: 1;
		/* Alarm2 interrupt flag. 0x00: Set to zero when RTC time doesn't match to alarm2 register. 0x01: Set to 1 when 
		 * RTC time matches the alarm2 register. When this bit is set, and A2IE = 1, an interrupt will be generated on 
		 * INT. Reset state is 0. */
		uint8_t A2F			: 1;
		/* Timer interrupt flag. 0x00: Set to zero when count down timer is not zero. 0x01: Set to 1 when countdown timer 
		 * reaches to zero. When this is set to ‘1’, and TIE = 1, an interrupt will be generated on pin INT. Reset state 
		 * is 0. */
		uint8_t TIF			: 1;
		/* Temp sense data flag: Indicates when user requested temp measurement is ready. (Only applicable in oneshot 
		 * mode). 0x00: TS data not ready 0x01: Set to 1 when TS data ready. When this is set to ‘1’, and TSIE = 1, an 
		 * interrupt will be generated on pin INT. Reset state is 0. */
		uint8_t TSF			: 1;
		/* Don't care. */
		uint8_t RESERVED4	: 1;
		/* Power-fail flag. 0x00: Set to zero when there is no power-fail condition on VBAT. 0x01: Set to 1 when there is 
		 * an external interrupt on VBAT. When this is set to ‘1’, and PFAILE = 1, an interrupt will be generated on pin 
		 * INT. After an initial power-fail condition occurs, if the condition does not persist, this bit can only be 
		 * cleared by reading the Status register. */
		uint8_t PFAIL		: 1;
		/* Oscillator stop flag. 0x00: Set to 0 when oscillator is running or when DOSF = 1. 0x01: Set to 1 when oscillator
		 * has stopped. An interrupt will not be generated on pin INT. Reset state is 1. */
		uint8_t OSF			: 1;
		/* Main supply source indication when the part is in auto mode. 0x00: Part is running on VCC 0x01: Part is running 
		 * on VBAT. Reset state is 0. */
		uint8_t PSDECT		: 1;
	};		
} MAX31343_StatusReg_t;

/*
 *	@brief Interrupt enable register.
 *
 **/
typedef union
{
	uint8_t IntEnReg;
	struct
	{
		/* Alarm1 interrupt enable. 0x00: Disable alarm1 interrupt function 0x01: Enable alarm1 interrupt function. Reset
		 * state is 0. */
		uint8_t A1IE		: 1;
		/* Alarm2 interrupt enable. 0x00: Disable alarm2 interrupt function 0x01: Enable alarm2 interrupt function. Reset 
		 * state is 0. */
		uint8_t A2IE		: 1;
		/* Timer interrupt enable. 0x00: Disable timer interrupt function 0x01: Enable timer interrupt function. Reset 
		 * state is 0. */
		uint8_t TIE			: 1;
		/* Temp sense ready flag enable. 0x00: Disable temp sense ready function 0x01: Enable temp sense ready function. Reset 
		 * state is 0. */
		uint8_t TSIE		: 1;
		/* Don't care. */
		uint8_t RESERVED4	: 1;
		/* Power fail Interrupt enable. 0x00: When set to zero, analog interrupt function is disabled. 0x01: When set to 
		 * one, analog interrupt function is enabled. Reset state is 0. */
		uint8_t PFAILE		: 1;
		/* Disable oscillator flag. 0x00: Allow the OSF to indicate the oscillator status. 0x01: Disable the oscillator 
		 * flag, irrespective of the oscillator status. Reset state is 0. */
		uint8_t DOSF		: 1;
		/* Don't care. */
		uint8_t RESERVED7	: 1;
	};		
} MAX31343_IntEnReg_t;

/*
 *	@brief RTC software reset register.
 *
 **/
typedef union
{
	uint8_t RtcResetReg;
	struct
	{
		/* Active-high software reset bit. 0x00: When set to 0, the device is in normal working mode. 0x01: Set to one to 
		 * reset digital blocks and all user registers except RAM and SWRST. Oscillator is disabled. Reset state is 0. */
		uint8_t SWRST		: 1;
		/* Don't care. */
		uint8_t RESERVED7_1	: 7;
	};		
} MAX31343_RtcResetReg_t;


/*
 *	@brief RTC configuration register 1.
 *
 **/
typedef union
{
	uint8_t RtcCfgReg1;
	struct
	{
		/* Don't care. Reset state is 0. */
		uint8_t RESERVED0	: 1;
		/* Active-high enable for oscillator. 0x00: Disable oscillator 0x01: Enable oscillator. Reset state is 1. */
		uint8_t ENOSC		: 1;
		/* Don't care. */
		uint8_t RESERVED2	: 1;
		/* I2C timeout enable. 0x00: Disables I2C timeout 0x01: Enables I2C timeout. Reset state is 1. */
		uint8_t I2C_TIMEOUT	: 1;
		/* Data retention bit. Before programming DATARETEN, user needs to wait for any ongoing temperature sensor 
		 * measurement cycles to complete. This depends on whether AUTOMODE or ONESHOTMODE is configured. In AUTOMODE, 
		 * first program AUTOMODE to 0, and then wait for 100mSec. In ONEHSOTMODE, wait for 100mSec. Reset state is 0. */
		uint8_t DATA_RET	: 1;
		/* Don't care. */
		uint8_t RESERVED7_5	: 3;
	};		
} MAX31343_RtcCfgReg1_t;

/*
 *	@brief RTC configuration register 2.
 *
 **/
typedef union
{
	uint8_t RtcCfgReg2;
	struct
	{
		/* Set output clock on SQW to specified frequency. See MAX31343_SQWRate enum to select the frequency. Reset state 
		 * is 0 (1Hz). */
		uint8_t SQW_HZ2_0	: 3;
		/* Specify uncompensated clock frequency output on pin CLKO. SeeMAX31343_CLKORate enum  to select the frequency. 
		 * Reset state is 0x08 (32kHz). */
		uint8_t CLKO_HZ6_3	: 4;
		/* CLKO enable. 0x00: Disable output clock on CLKO; CLKO set to Hi-Z 0x01: Enable output clock on CLKO. Reset 
		 * state is 0. */
		uint8_t ENCLKO		: 1;
	};		
} MAX31343_RtcCfgReg2_t;

/*
 *	@brief Countdown timer configuration register.
 *
 **/
typedef union
{
	uint8_t TimerCfgReg;
	struct
	{
		/* Timer frequency selection. See MAX31343_TFSRate enum to select the frequency. Reset state is 0 (1024Hz). */
		uint8_t TFS			: 2;
		/* Timer repeat mode. Controls the timer interrupt function. 0x00: Countdown timer will halt once it reaches zero
		 * 0x01: Countdown timer reloads the value from the timer initial register upon reaching zero and continues 
		 * counting. Reset state is 1. */
		uint8_t TRPT		: 1;
		/* Timer Pause. This field is valid only when TE = 1. When TE will be programmed to 0, this field must also be 
		 * reset to 0. 0x00: Timer continues to count down from the paused count value as per programming. 0x01: Timer is 
		 * paused, however, the count value is retained. When this bit is reset back to 0, count down continues from the 
		 * paused value. Reset state is 0. */
		uint8_t TPAUSE		: 1;
		/* Timer enable. Also refer TPAUSE field for additional information. 0x00: Timer is reset when set to zero. New 
		 * timer countdown value (Timer_Init) can be programmed in this state. Note: In this state, ensure TPAUSE is also
		 * programmed to 0, if TPAUSE was set to 1 earlier. 0x01: Timer starts counting down from the value programmed in
		 * Timer_Init. Reset state is 0. */
		uint8_t TE			: 1;
		/* Don't care. */
		uint8_t RESERVED5_7	: 3;
	};		
} MAX31343_TimerCfgReg_t;

/*
 *	@brief RTC seconds configuration register.
 *
 **/
typedef union
{
	uint8_t Seconds;
	struct
	{
		/* RTC seconds value. Reset state is 0. */
		uint8_t SECONDS		: 4;
		/* RTC seconds in multiples of 10. Reset state is 0. */
		uint8_t SECONDS10	: 3;
		/* Don't care. */
		uint8_t RESERVED0	: 1;
	};		
} MAX31343_Seconds_t;

/*
 *	@brief RTC minutes configuration register.
 *
 **/
typedef union
{
	uint8_t Minutes;
	struct
	{
		/* RTC minutes value. Reset state is 0. */
		uint8_t MINUTES		: 4;
		/* RTC minutes in multiples of 10. Reset state is 0. */
		uint8_t MINUTES10	: 3;
		/* Don't care. */
		uint8_t RESERVED0	: 1;
	};		
} MAX31343_Minutes_t;

/*
 *	@brief RTC hours configuration register.
 *
 **/
typedef union
{
	uint8_t Hour;
	struct
	{
		/* RTC hours value. Reset state is 0. */
		uint8_t HOUR		: 4;
		/* RTC hours in multiples of 10. Reset state is 0. */
		uint8_t HOUR10		: 1;
		/* Don't care. */
		uint8_t RESERVED0	: 1;
	};		
} MAX31343_Hours_t;

/*
 *	@brief RTC day configuration register.
 *
 **/
typedef union
{
	uint8_t Day;
	struct
	{
		/* RTC day value. Reset state is 1. */
		uint8_t DAY			: 3;
		/* Don't care. */
		uint8_t RESERVED7_3	: 5;
	};		
} MAX31343_Day_t;

/*
 *	@brief RTC date configuration register.
 *
 **/
typedef union
{
	uint8_t Date;
	struct
	{
		/* RTC date value. Reset state is 1. */
		uint8_t DATE		: 4;
		/* RTC date in multiples of 10. Reset state is 0. */
		uint8_t DATE10		: 2;
		/* Don't care. */
		uint8_t RESERVED7_6	: 2;
	};		
} MAX31343_Date_t;

/*
 *	@brief RTC Month configuration register.
 *
 **/
typedef union
{
	uint8_t Month;
	struct
	{
		/* RTC month value. Reset state is 1. */
		uint8_t MONTH		: 4;
		/* RTC month in multiples of 10. Reset state is 0. */
		uint8_t MONTH10		: 1;
		/* Don't care. */
		uint8_t RESERVED6_5	: 2;
		/* RTC century value. */
		uint8_t  CENTURY	: 1;
	};		
} MAX31343_Month_t;

/*
 *	@brief RTC year configuration register.
 *
 **/
typedef union
{
	uint8_t Year;
	struct
	{
		/* RTC year value. Reset state is 0. */
		uint8_t YEAR		: 4;
		/* RTC year in multiples of 10. Reset state is 0. */
		uint8_t YEAR10		: 4;
	};		
} MAX31343_Year_t;

/*
 *	@brief Alarm 1 seconds configuration register.
 *
 **/
typedef union
{
	uint8_t Seconds;
	struct
	{
		/* Alarm 1 seconds value. Reset state is 0. */
		uint8_t SECONDS		: 4;
		/* Alarm 1 seconds in multiples of 10. Reset state is 0. */
		uint8_t SECONDS10	: 3;
		/* Alarm 1 mask bit for seconds. */
		uint8_t A1M1		: 1;
	};	
} MAX31343_A1Seconds_t;

/*
 *	@brief Alarm 1 minutes configuration register.
 *
 **/
typedef union
{
	uint8_t Minutes;
	struct
	{
		/* Alarm 1 minutes value. Reset state is 0. */
		uint8_t MINUTES		: 4;
		/* Alarm 1 minutes in multiples of 10. Reset state is 0. */
		uint8_t MINUTES10	: 3;
		/* Alarm 1 mask bit for minutes. */
		uint8_t A1M2		: 1;
	};	
} MAX31343_A1Minutes_t;

/*
 *	@brief Alarm 1 hours configuration register.
 *
 **/
typedef union
{
	uint8_t Hour;
	struct
	{
		/* Alarm 1 hours value. Reset state is 0. */
		uint8_t HOUR		: 4;
		/* Alarm 1 hours in multiples of 10. Reset state is 0. */
		uint8_t HOUR10		: 1;
		/* Don't care. */
		uint8_t RESERVED6_5 : 2;
		/* Alarm 1 mask bit for hours. */
		uint8_t A1M3		: 1;
	};	
} MAX31343_A1Hours_t;

/*
 *	@brief Alarm 1 day/date configuration register.
 *
 **/
typedef union
{
	uint8_t DayDate;
	struct
	{
		/* Alarm 1 day/date value. Reset state is 0. */
		uint8_t DAYDATE	: 4;
		/* Alarm 1 date in multiples of 10. Reset state is 0. */
		uint8_t DATE10	: 2;
		/* Alarm 1 day/date select bit. */
		uint8_t DYDT	: 1;
		/* Alarm 1 mask bit for day/date. */
		uint8_t A1M4	: 1;
	};	
} MAX31343_A1DayDate_t;

/*
 *	@brief Alarm 1 month configuration register.
 *
 **/
typedef union
{
	uint8_t Month;
	struct
	{
		/* Alarm 1 months value. Reset state is 0. */
		uint8_t MONTH		: 4;
		/* Alarm 1 months in multiples of 10. Reset state is 0. */
		uint8_t MONTH10		: 1;
		/* Don't care. */
		uint8_t RESERVED5	: 1;
		/* Alarm 1 mask bit for year. */
		uint8_t A1M6		: 1;
		/* Alarm 1 mask bit for month. */
		uint8_t A1M5		: 1;
	};	
} MAX31343_A1Month_t;

/*
 *	@brief Alarm 1 year configuration register.
 *
 **/
typedef union
{
	uint8_t Year;
	struct
	{
		/* Alarm 1 year value. Reset state is 0. */
		uint8_t YEAR	: 4;
		/* Alarm 1 year in multiples of 10. Reset state is 0. */
		uint8_t YEAR10	: 4;
	};	
} MAX31343_A1Year_t;

/*
 *	@brief Alarm 2 minutes configuration register.
 *
 **/
typedef union
{
	uint8_t Minutes;
	struct
	{
		/* Alarm 2 minutes value. Reset state is 0. */
		uint8_t MINUTES		: 4;
		/* Alarm 2 minutes in multiples of 10. Reset state is 0. */
		uint8_t MINUTES10	: 3;
		/* Alarm 2 mask bit for minutes. */
		uint8_t A2M2		: 1;
	};	
} MAX31343_A2Minutes_t;

/*
 *	@brief Alarm 2 hours configuration register.
 *
 **/
typedef union
{
	uint8_t Hour;
	struct
	{
		/* Alarm 2 hours value. Reset state is 0. */
		uint8_t HOUR		: 4;
		/* Alarm 2 hours in multiples of 10. Reset state is 0. */
		uint8_t HOUR10		: 1;
		/* Don't care. */
		uint8_t RESERVED6_5 : 2;
		/* Alarm 2 mask bit for hours. */
		uint8_t A2M3		: 1;
	};	
} MAX31343_A2Hours_t;

/*
 *	@brief Alarm 2 day/date configuration register.
 *
 **/
typedef union
{
	uint8_t DayDate;
	struct
	{
		/* Alarm 2 day/date value. Reset state is 0. */
		uint8_t DAYDATE	: 4;
		/* Alarm 2 date in multiples of 10. Reset state is 0. */
		uint8_t DATE10	: 2;
		/* Alarm 2 day/date select bit. Reset state is 0. */
		uint8_t DYDT	: 1;
		/* Alarm 2 mask bit for day/date. Reset state is 0. */
		uint8_t A2M4	: 1;
	};	
} MAX31343_A2DayDate_t;

/*
 *	@brief Power management configuration register.
 *
 **/
typedef union
{
	uint8_t PwrManagement;
	struct
	{
		/* Don't care. */
		uint8_t RESERVED1_0	: 2;
		/* When this bit is low, input control block decides which supply to use. And this bit is low, power management 
		 * comparators are enabled. When this bit is high, comparators are disabled and user can manually select whether 
		 * to use VCC or VBACKUP as supply. 0x00: Circuit decides whether to use VCC or VBACKUP as supply. 0x01: User 
		 * decides whether to use VCC or VBACKUP as supply by setting D_VBACK_SEL bit. Reset state is 0. */
		uint8_t D_MAN_SEL	: 1;
		/* Backup battery select. Require D_MAN_SEL = 1 for this bit to have effect. 0x00: Use VCC as supply. 0x01: Use 
		 * Vbackup as supply. Reset state is 0. */
		uint8_t D_VBACK_SEL	: 1;
		/* Power fail threshold voltage. Sets analog comparator threshold voltage. Require D_MAN_SEL = 0 for this setting
		 * to have effect. See the MAX31343_PFVTVoltage enum to select voltage value. Reset state is 0x03 (2V4).*/
		uint8_t PFVT		: 2;
	};	
} MAX31343_PwrMgmt_t;

/*
 *	@brief Trickle charge configuration register.
 *
 **/
typedef union
{
	uint8_t TrickleReg;
	struct
	{
		/* Sets the charging path for trickle charger. Must set TCHE to 0x05 to enable the trickle path setting. Reset 
		 * state is 0 (3KΩ in series with a Schottky diode). */
		uint8_t D_TRICKLE	: 4;
		/* Trickle charger enable. Only 1 of 16 codes enables trickle charger. See the MAX31343_TCHEEnable enum to enable
		 * trickle charger. Reset state is 0 (trickle charger disabled). */
		uint8_t TCHE		: 4;
	};	
} MAX31343_TrickleReg_t;

/*
 *	@brief Temperature registers.
 *	Temperature sensor measurement is stored as a 10-bit two’s complement number in two-byte temperature register. SIGN 
 *	bit indicates if the temperature is positive or negative. When SIGN bit is 1, it represents negative temperature, and
 *	it has a weight of -128. Bit [5:0] are redundant. Bit 6 of Temperature [15:0] is LSB and has a weight of 0.25. Table
 *  below represent weight of each bit from Bit 6 to Bit 14. Assuming user reads Temp_MSB and Temp_LSB registers as 
 *  Temp_MSB = 8'b01010100, Temp_LSB = 8'b11000000. Per Table above, Bit 15 = 0 means positive temperature, 
 *  64X1 + 32X0 + 16X1 + 8X0 + 4X1 + 2X0 + 1X0 = 84ºC, 0.5X1 + 0.25X1 = 0.75ºC. Hence, temperature = 84.75ºC.
 *
 **/
typedef struct
{
	/* MSB byte of temperature. */
	union 
	{
	    uint8_t TempMSB;
		struct
		{
			uint8_t TEMP_MSB8_2	: 7;
			uint8_t SIGN		: 1;
		}; 
	};
	/* LSB byte of temperature. */
	union 
	{
		uint8_t TempLSB;
		struct
		{
			uint8_t TEMP_DONTCARE	: 6;
			uint8_t TEMP_LSB1_0		: 2;
		}; 
	};
} MAX31343_TempReg_t;

/*
 *	@brief Temperature sensor configuration register.
 *
 **/
typedef union
{
	uint8_t TsCfgReg;
	struct
	{
		/*  Reset state is 0. */
		uint8_t RESERVED2_0	: 3;
		/* Set temperature measurement interval to specified time for automatic mode of temperature measurement and 
		 * compensation. See the MAX31343_TTSINTInterval enum to select desired interval. Reset state is 0x05 (32 sec). */
		uint8_t TTSINT		: 3;
		/* One-shot user-requested temperature measurement in real-time. AUTOMODE must be 0 in one-shot measurement mode. 
		 * When this bit is set, divider ratio is updated (compensated) after temperature measurement. This bit is 
		 * self-cleared after temperature measurement complete; writes to this bit before it has been cleared have no 
		 * effect. 0x00: No temperature measurement requested 0x01: Temperature measurement requested Reset state is 0. */
		uint8_t ONESHOTMODE	: 1;
		/* Automatic mode of temperature measurement. This mode is valid only when ONESHOTMODE = 0. In this mode, temperature
		 * measurement interval is decided by TTSINT. After each temperature measurement request, the divider ratio will 
		 * be modified for temperature compensation. 0x00: Automatic measurement mode disabled 0x01: Automatic measurement 
		 * mode enabled. Reset state is 1. */
		uint8_t AUTOMODE	: 1;
	};	
} MAX31343_TsCfgReg_t;

/*
 *	@brief RAM registers.
 *
 **/
typedef union
{
	uint8_t RamReg[64];

} MAX31343_RAMReg_t;


/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	/* Clock data. */
	MAX31343_Clock_t clock;
	/* Alarm 1 data. */
	/* Alarm 2 data. */
	/* Pointers for the rx, tx delay functions. */
	delay_fptr delay;
	i2c_txrx_data_fptr i2c_tx_data;
	i2c_txrx_data_fptr i2c_rx_data;
	
} MAX31343_GIns_t;

/* Public function prototypes. */

/*Time functions. */
void MAX31343_SetTime(MAX31343_GIns_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds);
void MAX31343_SetDate(MAX31343_GIns_t *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year, uint8_t century);
void DS3231_SetAlarm1(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t year, uint8_t month, uint8_t date,
					uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS3231_SetAlarm2(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t date, uint8_t hours, uint8_t minutes);
void DS3231_GetTimeDate(MAX31343_GIns_t *device);
void MAX31343_TimerInit(MAX31343_GIns_t *device, uint8_t cnt_data);
void DS3231_WriteTimerCfg(MAX31343_GIns_t *device, uint8_t te, MAX31343_TFSRate tfs, uint8_t trpt, uint8_t tpause);


/* Configuration functions. */
void MAX31343_RTCReset(MAX31343_GIns_t *device);
uint8_t MAX31343_ReadStatusReg(MAX31343_GIns_t *device);
uint8_t MAX31343_ReadTimCntReg(MAX31343_GIns_t *device);
uint8_t MAX31343_ReadTemp(MAX31343_GIns_t *device);
void MAX31343_ReadRAM(MAX31343_GIns_t *device, uint8_t* data, uint8_t size, uint8_t offset);
void MAX31343_WriteIntEnReg(MAX31343_GIns_t *device, uint8_t a1ie, uint8_t a2ie, uint8_t tie, uint8_t tsie, uint8_t pfaile,
	uint8_t dosf);
void DS3231_WriteRTCConfig1(MAX31343_GIns_t *device, uint8_t enosc, uint8_t i2c_timeout, uint8_t dataret);
void DS3231_WriteRTCConfig2(MAX31343_GIns_t *device, uint8_t enclko, MAX31343_CLKORate clko_hz, MAX31343_SQWRate sqw_hz);
void MAX31343_PwrMgmt(MAX31343_GIns_t *device, uint8_t dman_sel, uint8_t dvback_sel, MAX31343_PFVTVoltage pfvt);
void MAX31343_TrickleReg(MAX31343_GIns_t *device, MAX31343_TCHEEnable tche, MAX31343_DTRICKLEPath dtrickle);
void MAX31343_TSConfig(MAX31343_GIns_t *device, uint8_t automode, uint8_t oneshotmode, MAX31343_TTSINTInterval ttsint);
void MAX31343_WriteRAM(MAX31343_GIns_t *device, uint8_t* data, uint8_t size, uint8_t offset);


#endif /* MAX31343_H_ */
