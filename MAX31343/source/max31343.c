/*
 *	@brief MAX31343 communication functions.
 *	Created 05.07.21 by asw3005 
 *
 **/

#include "stm32f1xx_hal.h"
#include "max31343.h"

/* Private function prototype. */


/* Public function. */
 

/*
 * @brief Set time in the RTC device.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct. 
 * @param seconds : amount of seconds in the clock.
 * @param minutes : amount of minutes in the clock.
 * @param hours : amount of hours in the clock.
 *
 **/
void MAX31343_SetTime(MAX31343_GIns_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	struct Time_t
	{
		MAX31343_Seconds_t Seconds_t;
		MAX31343_Minutes_t Minutes_t;
		MAX31343_Hours_t Hours_t;
		
	} Time_s; 	
	
	if (seconds > 59 || minutes > 59 || hours > 23) /* Time error. */ return;	
	
	Time_s.Hours_t.HOUR = hours % 10;
	Time_s.Hours_t.HOUR10 = hours / 10;
	Time_s.Minutes_t.MINUTES = minutes % 10;
	Time_s.Minutes_t.MINUTES10 = minutes / 10;
	Time_s.Seconds_t.SECONDS = seconds % 10;
	Time_s.Seconds_t.SECONDS10 = seconds / 10;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_SECONDS, 1, (uint8_t*)&Time_s, sizeof(Time_s));
	device->delay(1);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param day : day of the week.
 * @param date : just the date.
 * @param month : month of the year.
 * @param year : just the year.
 * @param century : if it one the year will be 100 + year value (up to 199).
 *
 **/
void MAX31343_SetDate(MAX31343_GIns_t *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year, uint8_t century)
{
	struct Date_t
	{
		MAX31343_Day_t Day_t;
		MAX31343_Date_t Date_t;
		MAX31343_Month_t Month_t;
		MAX31343_Year_t Year_t;
		
	} Date_s;
	
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99 || century > 1) /* Date error. */ return;
	
	Date_s.Day_t.DAY = day;
	Date_s.Date_t.DATE = date % 10;
	Date_s.Date_t.DATE10 = date / 10;
	Date_s.Month_t.MONTH = month % 10;
	Date_s.Month_t.MONTH10 = month / 10;
	Date_s.Month_t.CENTURY = century;
	Date_s.Year_t.YEAR = year % 10;
	Date_s.Year_t.YEAR10 = year / 10;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_DAY, 1, (uint8_t*)&Date_s, sizeof(Date_s));
	device->delay(1);
}

/*
 * @brief Gets time and date and convert it to decimal format.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 *
 **/
void DS3231_GetTimeDate(MAX31343_GIns_t *device)
{
	struct TimeDate_t
	{
		MAX31343_Seconds_t Seconds_t;
		MAX31343_Minutes_t Minutes_t;
		MAX31343_Hours_t Hours_t;
		MAX31343_Day_t Day_t;
		MAX31343_Date_t Date_t;
		MAX31343_Month_t Month_t;
		MAX31343_Year_t Year_t;
		
	} TimeDate_s;	
	
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_SECONDS, 1, (uint8_t*)&TimeDate_s, sizeof(TimeDate_s));
	device->delay(1);
	
	device->clock.Seconds = TimeDate_s.Seconds_t.SECONDS10 * 10 + TimeDate_s.Seconds_t.SECONDS;
	device->clock.Minutes = TimeDate_s.Minutes_t.MINUTES10 * 10 + TimeDate_s.Minutes_t.MINUTES;
	device->clock.Hours = TimeDate_s.Hours_t.HOUR10 * 10 + TimeDate_s.Hours_t.HOUR;
	device->clock.Day = TimeDate_s.Day_t.DAY;
	device->clock.Date = TimeDate_s.Date_t.DATE10 * 10 + TimeDate_s.Date_t.DATE;
	device->clock.Month = TimeDate_s.Month_t.MONTH10 * 10 + TimeDate_s.Month_t.MONTH;
	device->clock.Year = TimeDate_s.Month_t.CENTURY * 100 + TimeDate_s.Year_t.YEAR10 * 10 + TimeDate_s.Year_t.YEAR;
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param alarm_en : condition for the alarm, see MAX31343_Alarm1Mask enum.
 * @param date : day or date value depending sel_day_date parameter.
 * @param seconds : amount of seconds in the clock.
 * @param minutes : amount of minutes in the clock.
 * @param hours : amount of hours in the clock.
 * @param day_date : day or date in the calendar.
 *
 **/
void DS3231_SetAlarm1(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t year, uint8_t month, uint8_t date,
					uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	struct Alarm1_t
	{
		MAX31343_A1Seconds_t A1Seconds_t;
		MAX31343_A1Minutes_t A1Minutes_t;
		MAX31343_A1Hours_t A1Hours_t;
		MAX31343_A1DayDate_t A1DayDate_t;
		MAX31343_A1Month_t A1Month_t;
		MAX31343_A1Year_t A1Year_t;
		
	} Alarm1_s;
	
	if (seconds > 59 || minutes > 59) /* Time error. */ return;
	if ((alarm_en & 0x40) && !(date < 1 || date > 7)) return;
	else if (date < 1 || date > 31) return;		
	if (month < 1 || month > 12 || year > 99) /* Date error. */ return;
	
	Alarm1_s.A1Seconds_t.A1M1 = alarm_en & 0x01;
	Alarm1_s.A1Minutes_t.A1M2 = alarm_en & 0x02;
	Alarm1_s.A1Hours_t.A1M3 = alarm_en & 0x04;
	Alarm1_s.A1DayDate_t.A1M4 = alarm_en & 0x08;
	Alarm1_s.A1DayDate_t.DYDT = alarm_en & 0x40;
	Alarm1_s.A1Month_t.A1M5 = alarm_en & 0x10;
	Alarm1_s.A1Month_t.A1M6 = alarm_en & 0x20;
	
	
	Alarm1_s.A1Year_t.YEAR = year % 10;
	Alarm1_s.A1Year_t.YEAR10 = year / 10;
	Alarm1_s.A1Month_t.MONTH = month % 10;
	Alarm1_s.A1Month_t.MONTH10 = month / 10;
	Alarm1_s.A1Hours_t.HOUR = hours % 10;
	Alarm1_s.A1Hours_t.HOUR10 = hours / 10;
	Alarm1_s.A1Minutes_t.MINUTES = minutes % 10;
	Alarm1_s.A1Minutes_t.MINUTES10 = minutes / 10;
	Alarm1_s.A1Seconds_t.SECONDS = seconds % 10;
	Alarm1_s.A1Seconds_t.SECONDS10 = seconds / 10;	
	if (alarm_en & 0x40) {
		Alarm1_s.A1DayDate_t.DAYDATE = date;
	} else { 
		Alarm1_s.A1DayDate_t.DAYDATE = date % 10;
		Alarm1_s.A1DayDate_t.DATE10 = date / 10;
	}
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_ALARM1_SECONDS, 1, (uint8_t*)&Alarm1_s, sizeof(Alarm1_s));
	device->delay(1);
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param alarm_condition : Condition for the alarm, see MAX31343_Alarm2Mask enum.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS3231_SetAlarm2(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t date, uint8_t hours, uint8_t minutes)
{
	struct Alarm2_t
	{
		MAX31343_A2Minutes_t A2Minutes_t;
		MAX31343_A2Hours_t A2Hours_t;
		MAX31343_A2DayDate_t A2DayDate_t;
		
	} Alarm2_s;
	
	if (minutes > 59) /* Time error. */ return;
	if ((alarm_en & 0x40) && !(date < 1 || date > 7)) return;
	else if (date < 1 || date > 31) return;		
	
	Alarm2_s.A2Minutes_t.A2M2 = alarm_en & 0x02;
	Alarm2_s.A2Hours_t.A2M3 = alarm_en & 0x04;
	Alarm2_s.A2DayDate_t.A2M4 = alarm_en & 0x08;
	Alarm2_s.A2DayDate_t.DYDT = alarm_en & 0x40;

	Alarm2_s.A2Hours_t.HOUR = hours % 10;
	Alarm2_s.A2Hours_t.HOUR10 = hours / 10;
	Alarm2_s.A2Minutes_t.MINUTES = minutes % 10;
	Alarm2_s.A2Minutes_t.MINUTES10 = minutes / 10;
	if (alarm_en & 0x40) {
		Alarm2_s.A2DayDate_t.DAYDATE = date;
	} else { 
		Alarm2_s.A2DayDate_t.DAYDATE = date % 10;
		Alarm2_s.A2DayDate_t.DATE10 = date / 10;
	}
   
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_DAY, 1, (uint8_t*)&Alarm2_s, sizeof(Alarm2_s));
	device->delay(1);
}

/*
 * @brief Read status register.
 *
 **/
uint8_t MAX31343_ReadStatusReg(MAX31343_GIns_t *device)
{
	MAX31343_StatusReg_t StatusReg_t;

	return StatusReg_t.StatusReg;
}

/*
 * @brief Write bits of control and status registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param 
 * @param   
 * @param 
 * @param 
 * @param 
 * @param 
 *
 **/
void MAX31343_WriteIntEnReg(MAX31343_GIns_t *device, uint8_t a1ie, uint8_t a2ie, uint8_t tie, uint8_t tsie, uint8_t pfaile,
	uint8_t dosf)
{
	MAX31343_IntEnReg_t IntEnReg_t;	
	
}

/*
 * @brief Reset RTC.
 *
 **/
void MAX31343_RTCReset(MAX31343_GIns_t *device)
{
	
	
}

/*
 * @brief Write RTC configuation register 1.
 *		  
 * @param 
 * @param        
 * @param 
 *
 **/
void DS3231_WriteRTCConfig1(MAX31343_GIns_t *device, uint8_t enosc, uint8_t i2c_timeout, uint8_t dataret)
{
	MAX31343_RtcCfgReg1_t RtcCfgReg1_t;
	
}

/*
 * @brief Write RTC configuation register 2.
 *		  
 * @param 
 * @param        
 * @param 
 *
 **/
void DS3231_WriteRTCConfig2(MAX31343_GIns_t *device, uint8_t enclko, MAX31343_CLKORate clko_hz, MAX31343_SQWRate sqw_hz)
{
	MAX31343_RtcCfgReg2_t RtcCfgReg2_t;
	
}

/*
 * @brief Write timer configuation register.
 *		  
 * @param 
 * @param        
 * @param 
 * @param
 *
 **/
void DS3231_WriteTimerCfg(MAX31343_GIns_t *device, uint8_t te, MAX31343_TFSRate tfs, uint8_t trpt, uint8_t tpause)
{
	MAX31343_TimerCfgReg_t TimerCfg_t;
	
	
}





/*
 * @brief Private function.
 *
 **/


/*
 * @brief  Check the object reference.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct. 
 *
 **/
static void DS3231_CheckReference(MAX31343_GIns_t *device)
{
	if (device == NULL || device->delay == NULL || device->i2c_rx_data == NULL || device->i2c_tx_data == NULL)
	{
		for (;;)
		{
			///Errore. Object or function is not found. Check the object reference and function pointers.
		}
	}
}


