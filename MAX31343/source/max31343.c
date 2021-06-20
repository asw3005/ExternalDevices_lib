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
		MAX31343_Seconds_t Seconds;
		MAX31343_Minutes_t Minutes;
		MAX31343_Hours_t Hours;
		
	} Time_s; 	
	
	if (seconds > 59 || minutes > 59 || hours > 23) /* Time error. */ return;	
	
	Time_s.Hours.HOUR = hours % 10;	
	if (hours > 9) {
		Time_s.Hours.HOUR10 = hours / 10;
	}	
	Time_s.Minutes.MINUTES = minutes % 10;
	Time_s.Minutes.MINUTES10 = minutes / 10;
	Time_s.Seconds.SECONDS = seconds % 10;
	Time_s.Seconds.SECONDS10 = seconds / 10;
	
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
		MAX31343_Day_t Day;
		MAX31343_Date_t Date;
		MAX31343_Month_t Month;
		MAX31343_Year_t Year;
		
	} Date_s;
	
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99 || century > 1) /* Date error. */ return;
	
	Date_s.Day.DAY = day;
	Date_s.Date.DATE = date % 10;
	Date_s.Date.DATE10 = date / 10;
	Date_s.Month.MONTH = month % 10;
	Date_s.Month.MONTH10 = month / 10;
	Date_s.Month.CENTURY = century;
	Date_s.Year.YEAR = year % 10;
	Date_s.Year.YEAR10 = year / 10;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_DAY, 1, (uint8_t*)&Date_s, sizeof(Date_s));
	device->delay(1);
}

/*
 * @brief Gets time and date and convert it to decimal format.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 *
 **/
void MAX31343_GetTimeDate(MAX31343_GIns_t *device)
{
	struct TimeDate_t
	{
		MAX31343_Seconds_t Seconds;
		MAX31343_Minutes_t Minutes;
		MAX31343_Hours_t Hours;
		MAX31343_Day_t Day;
		MAX31343_Date_t Date;
		MAX31343_Month_t Month;
		MAX31343_Year_t Year;
		
	} TimeDate_s;	
	
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_SECONDS, 1, (uint8_t*)&TimeDate_s, sizeof(TimeDate_s));
	device->delay(1);
	
	device->clock.Seconds = TimeDate_s.Seconds.SECONDS10 * 10 + TimeDate_s.Seconds.SECONDS;
	device->clock.Minutes = TimeDate_s.Minutes.MINUTES10 * 10 + TimeDate_s.Minutes.MINUTES;
	device->clock.Hours = TimeDate_s.Hours.HOUR10 * 10 + TimeDate_s.Hours.HOUR;
	device->clock.Day = TimeDate_s.Day.DAY;
	device->clock.Date = TimeDate_s.Date.DATE10 * 10 + TimeDate_s.Date.DATE;
	device->clock.Month = TimeDate_s.Month.MONTH10 * 10 + TimeDate_s.Month.MONTH;
	device->clock.Year = TimeDate_s.Month.CENTURY * 100 + TimeDate_s.Year.YEAR10 * 10 + TimeDate_s.Year.YEAR;
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
void MAX31343_SetAlarm1(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t year, uint8_t month, uint8_t date,
					uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	struct Alarm1_t
	{
		MAX31343_A1Seconds_t A1Seconds;
		MAX31343_A1Minutes_t A1Minutes;
		MAX31343_A1Hours_t A1Hours;
		MAX31343_A1DayDate_t A1DayDate;
		MAX31343_A1Month_t A1Month;
		MAX31343_A1Year_t A1Year;
		
	} Alarm1_s;
	
	if (seconds > 59 || minutes > 59) /* Time error. */ return;
	if ((alarm_en & 0x40) && !(date < 1 || date > 7)) return;
	else if (date < 1 || date > 31) return;		
	if (month < 1 || month > 12 || year > 99) /* Date error. */ return;
	
	Alarm1_s.A1Seconds.A1M1 = alarm_en & 0x01;
	Alarm1_s.A1Minutes.A1M2 = alarm_en & 0x02;
	Alarm1_s.A1Hours.A1M3 = alarm_en & 0x04;
	Alarm1_s.A1DayDate.A1M4 = alarm_en & 0x08;
	Alarm1_s.A1DayDate.DYDT = alarm_en & 0x40;
	Alarm1_s.A1Month.A1M5 = alarm_en & 0x10;
	Alarm1_s.A1Month.A1M6 = alarm_en & 0x20;
	
	
	Alarm1_s.A1Year.YEAR = year % 10;
	Alarm1_s.A1Year.YEAR10 = year / 10;
	Alarm1_s.A1Month.MONTH = month % 10;
	Alarm1_s.A1Month.MONTH10 = month / 10;
	Alarm1_s.A1Hours.HOUR = hours % 10;
	Alarm1_s.A1Hours.HOUR10 = hours / 10;
	Alarm1_s.A1Minutes.MINUTES = minutes % 10;
	Alarm1_s.A1Minutes.MINUTES10 = minutes / 10;
	Alarm1_s.A1Seconds.SECONDS = seconds % 10;
	Alarm1_s.A1Seconds.SECONDS10 = seconds / 10;	
	if (alarm_en & 0x40) {
		Alarm1_s.A1DayDate.DAYDATE = date;
	} else { 
		Alarm1_s.A1DayDate.DAYDATE = date % 10;
		Alarm1_s.A1DayDate.DATE10 = date / 10;
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
void MAX31343_SetAlarm2(MAX31343_GIns_t *device, MAX31343_Alarm1Mask alarm_en, uint8_t date, uint8_t hours, uint8_t minutes)
{
	struct Alarm2_t
	{
		MAX31343_A2Minutes_t A2Minutes;
		MAX31343_A2Hours_t A2Hours;
		MAX31343_A2DayDate_t A2DayDate;
		
	} Alarm2_s;
	
	if (minutes > 59) /* Time error. */ return;
	if ((alarm_en & 0x40) && !(date < 1 || date > 7)) return;
	else if (date < 1 || date > 31) return;		
	
	Alarm2_s.A2Minutes.A2M2 = alarm_en & 0x02;
	Alarm2_s.A2Hours.A2M3 = alarm_en & 0x04;
	Alarm2_s.A2DayDate.A2M4 = alarm_en & 0x08;
	Alarm2_s.A2DayDate.DYDT = alarm_en & 0x40;

	Alarm2_s.A2Hours.HOUR = hours % 10;
	Alarm2_s.A2Hours.HOUR10 = hours / 10;
	Alarm2_s.A2Minutes.MINUTES = minutes % 10;
	Alarm2_s.A2Minutes.MINUTES10 = minutes / 10;
	if (alarm_en & 0x40) {
		Alarm2_s.A2DayDate.DAYDATE = date;
	} else { 
		Alarm2_s.A2DayDate.DAYDATE = date % 10;
		Alarm2_s.A2DayDate.DATE10 = date / 10;
	}
   
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_DAY, 1, (uint8_t*)&Alarm2_s, sizeof(Alarm2_s));
	device->delay(1);
}

/*
 * @brief Read status register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @return : status register value of type of MAX31343_StatusReg_t.
 *
 **/
uint8_t* MAX31343_ReadStatusReg(MAX31343_GIns_t *device)
{
	static MAX31343_StatusReg_t StatusReg;
	
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_STATUS_REG, 1, (uint8_t*)&StatusReg, sizeof(StatusReg));
	device->delay(1);

	return (uint8_t*)&StatusReg;
}

/*
 * @brief Write bits of control and status registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param a1ie : if zero the alarm1 interrupt function is disabled, if one it is enabled.
 * @param a2ie : if zero the alarm2 interrupt function is disabled, if one it is enabled.
 * @param tie : if zero the timer interrupt function is disabled, if one it is enabled.
 * @param tsie : if zero the temp sense ready function is disabled, if one it is enabled.
 * @param pfaile : when set to zero, analog interrupt function is disabled, when set to one it is enabled.
 * @param dosf : zero allow the OSF to indicate the oscillator status, one isn't.
 *
 **/
void MAX31343_WriteIntEnReg(MAX31343_GIns_t *device, uint8_t a1ie, uint8_t a2ie, uint8_t tie, uint8_t tsie, uint8_t pfaile,
	uint8_t dosf)
{
	MAX31343_IntEnReg_t IntEnReg;	
	
	IntEnReg.A1IE = a1ie;
	IntEnReg.A2IE = a2ie;
	IntEnReg.TIE = tie;
	IntEnReg.TSIE = tsie;
	IntEnReg.PFAILE = pfaile;
	IntEnReg.DOSF = dosf;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_INT_EN_REG, 1, (uint8_t*)&IntEnReg, sizeof(IntEnReg));
	device->delay(1);	
}

/*
 * @brief Reset RTC.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 *
 **/
void MAX31343_RTCReset(MAX31343_GIns_t *device)
{
	MAX31343_RtcResetReg_t reset; 
	
	reset.SWRST = 1;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_RTC_RST, 1, (uint8_t*)&reset, 1);
	device->delay(1);
}

/*
 * @brief Write RTC configuation register 1.
 *		  
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param enosc : when 0 the oscillator is disable, when 1 the oscillator is enabled (default 1). 
 * @param i2c_timeout : 1 enables the i2c timeout, 0 isn't (default 1).        
 * @param dataret : if 1 the data retention mode is enabled, if 0 the notmal operation mode is enabled (default 0).
 *
 **/
void MAX31343_WriteRTCConfig1(MAX31343_GIns_t *device, uint8_t enosc, uint8_t i2c_timeout, uint8_t dataret)
{
	MAX31343_RtcCfgReg1_t RtcCfgReg1;
	
	RtcCfgReg1.ENOSC = enosc;
	RtcCfgReg1.I2C_TIMEOUT = i2c_timeout;
	RtcCfgReg1.DATA_RET = dataret;
	RtcCfgReg1.RESERVED0 = 0;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_RTC_CFG1, 1, (uint8_t*)&RtcCfgReg1, sizeof(RtcCfgReg1));
	device->delay(1);
	
}

/*
 * @brief Write RTC configuation register 2.
 *		  
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param enclko : 0 disables the clock on CLKO, 1 enables clock on CLKO (default 0).
 * @param clko_hz : uncompensated clock frequency output, see the MAX31343_CLKORate enum (default 0x08).
 * @param sqw_hz : output clock on SQW, see the MAX31343_SQWRate (default 0).
 *
 **/
void MAX31343_WriteRTCConfig2(MAX31343_GIns_t *device, uint8_t enclko, MAX31343_CLKORate clko_hz, MAX31343_SQWRate sqw_hz)
{
	MAX31343_RtcCfgReg2_t RtcCfgReg2;
	
	RtcCfgReg2.ENCLKO = enclko;
	RtcCfgReg2.CLKO_HZ6_3 = clko_hz;
	RtcCfgReg2.SQW_HZ2_0 = sqw_hz;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_RTC_CFG2, 1, (uint8_t*)&RtcCfgReg2, sizeof(RtcCfgReg2));
	device->delay(1);
}

/*
 * @brief Write timer configuation register.
 *		  
 * @param te : timer is reset when set to zero, otherwise (set to 1) timer starts counting down from the value programmed in Timer_Init.
 * @param tfs : timer frequency selection, see the MAX31343_TFSRate enum.
 * @param trpt : when zero countdown timer will halt once it reaches zero, when one countdown timer reloads the value from the timer initial 
 * register upon reaching zero and continues counting.
 * @param tpause : if 0 the timer continues to count down from the paused count value as per programming, if 1 the timer is paused, however, 
 * the count value is retained. When this bit is reset back to 0, count down continues from the paused value.
 *
 **/
void MAX31343_WriteTimerCfg(MAX31343_GIns_t *device, uint8_t te, MAX31343_TFSRate tfs, uint8_t trpt, uint8_t tpause)
{
	MAX31343_TimerCfgReg_t TimerCfg;
	
	TimerCfg.TE = te;
	TimerCfg.TFS = tfs;
	TimerCfg.TRPT = trpt;
	TimerCfg.TPAUSE = tpause;

	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_TIMER_CFG, 1, (uint8_t*)&TimerCfg, sizeof(TimerCfg));
	device->delay(1);
}

/*
 * @brief Read countdown timer value register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @return : count down timer current count value.
 *
 **/
uint8_t MAX31343_ReadTimCntReg(MAX31343_GIns_t *device)
{
	uint8_t CntValue;
	
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_TIMER_CNT, 1, (uint8_t*)&CntValue, sizeof(CntValue));
	device->delay(1);

	return CntValue;
}

/*
 * @brief Writes value to the countdow timer initialization register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param cnt_data : count down timer initial value. 
 *
 **/
void MAX31343_TimerInit(MAX31343_GIns_t *device, uint8_t cnt_data)
{
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_RTC_RST, 1, (uint8_t*)&cnt_data, 1);
	device->delay(1);
}

/*
 * @brief Writes value to the power management configuration register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param dman_sel : if zero internal circuit decides whether to use VCC or VBACKUP as supply, if one User decides whether to use VCC or 
 * VBACKUP as supply by setting D_VBACK_SEL bit.
 * @param dvback_sel : if zero the circuit uses VCC as supply, if one it uses Vbackup as supply.
 * @param pfvt : power fail threshold value. See the MAX31343_PFVTVoltage enum.
 *
 **/
void MAX31343_PwrMgmt(MAX31343_GIns_t *device, uint8_t dman_sel, uint8_t dvback_sel, MAX31343_PFVTVoltage pfvt)
{
	MAX31343_PwrMgmt_t PwrMgmt;
	
	PwrMgmt.D_MAN_SEL = dman_sel;
	PwrMgmt.D_VBACK_SEL = dvback_sel;
	PwrMgmt.PFVT = pfvt;	
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_PWR_MGMT, 1, (uint8_t*)&PwrMgmt, sizeof(PwrMgmt));
	device->delay(1);
}

/*
 * @brief Writes value to the trickle charge configuration register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param tche : trickle charger enable, see the MAX31343_TCHEEnable enum.
 * @param dtrickle : sets the charging path for trickle charger, see the MAX31343_DTRICKLEPath enum.
 *
 **/
void MAX31343_TrickleReg(MAX31343_GIns_t *device, MAX31343_TCHEEnable tche, MAX31343_DTRICKLEPath dtrickle)
{
	MAX31343_TrickleReg_t TrickleReg;
	
	TrickleReg.TCHE = tche;
	TrickleReg.D_TRICKLE = dtrickle;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_PWR_MGMT, 1, (uint8_t*)&TrickleReg, sizeof(TrickleReg));
	device->delay(1);
}

/*
 * @brief Reads temperature registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @return value of temperature registers.
 *
 **/
uint8_t* MAX31343_ReadTemp(MAX31343_GIns_t *device)
{
	static MAX31343_TempReg_t TempReg;
	
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_TEMP_MSB, 1, (uint8_t*)&TempReg, sizeof(TempReg));
	device->delay(1);
	
	return (uint8_t*)&TempReg;
}

/*
 * @brief Writes value to the trickle charge configuration register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param automode : if 0 automatic measurement mode is disabled, if 1 it is disabled.
 * @param oneshotmode : 0 no temperature measurement requested, 1 temperature measurement requested.
 * @param ttsint : set temperature measurement interval to specified time for automatic mode of temperature measurement and compensation.
 * See the MAX31343_TTSINTInterval enum.
 *
 **/
void MAX31343_TSConfig(MAX31343_GIns_t *device, uint8_t automode, uint8_t oneshotmode, MAX31343_TTSINTInterval ttsint)
{
	MAX31343_TsCfgReg_t TSConfig;
	
	TSConfig.AUTOMODE = automode;
	TSConfig.ONESHOTMODE = oneshotmode;
	TSConfig.TTSINT = ttsint;
	
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_TS_CFG, 1, (uint8_t*)&TSConfig, sizeof(TSConfig));
	device->delay(1);
}

/*
 * @brief Reads RAM registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param data* : pointer to the data.
 * @param size : size of data bytes will be transmitted.
 * @param offset : offset the RAM address.
 *
 **/
void MAX31343_WriteRAM(MAX31343_GIns_t *device, uint8_t* data, uint8_t size, uint8_t offset)
{	
	if (size > 64) return;
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_RAM_START + offset, 1, data, size);
	device->delay(1);
}

/*
 * @brief Reads RAM registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param data* : pointer to the data.
 * @param size : size of data bytes will be received.
 * @param offset : offset the RAM address.
 *
 **/
void MAX31343_ReadRAM(MAX31343_GIns_t *device, uint8_t* data, uint8_t size, uint8_t offset)
{	
	if (size > 64) return;
	device->i2c_rx_data(MAX31343_ADDR_SHIFTED, MAX31343_RAM_START + offset, 1, data, size);
	device->delay(1);
}




/* Private functions. */


/*
 * @brief  Check the object reference.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct. 
 *
 **/
static void MAX31343_CheckReference(MAX31343_GIns_t *device)
{
	if (device == NULL || device->delay == NULL || device->i2c_rx_data == NULL || device->i2c_tx_data == NULL)
	{
		for (;;)
		{
			///Error. Object or function is not found. Check the object reference and function pointers.
		}
	}
}


