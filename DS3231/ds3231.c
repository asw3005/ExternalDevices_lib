/*
 *	@brief DS3231 communication functions.
 *	Created 28.09.2020
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds3231.h"

/*
 * @brief Private function prototype.
 *
 **/
static void DS3231_ReadTimeDateReg(DS3231_GDataInstance_typedef *device);
static void DS3231_ReadAlarm1Reg(DS3231_GDataInstance_typedef *device);
static void DS3231_ReadAlarm2Reg(DS3231_GDataInstance_typedef *device);
static void DS3231_ReadControlReg(DS3231_GDataInstance_typedef *device);
static void DS3231_ReadStatusReg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteStatusReg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteTimeReg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteDateReg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteAlarm1Reg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteAlarm2Reg(DS3231_GDataInstance_typedef *device);
static void DS3231_WriteControlReg(DS3231_GDataInstance_typedef *device);
static void DS3231_CheckReference(DS3231_GDataInstance_typedef *device);
/*
 * @brief 
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. * 
 * @param clock_format : See enum DS3231_ClockFormat.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 *
 **/
void DS3231_Set_Time(DS3231_GDataInstance_typedef *device, DS3231_ClockFormat clock_format, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	if (seconds > 59 || minutes > 59 || clock_format > 1) /*Time errore*/ return;
	else if (clock_format == 1 && hours > 12) /*Time errore*/ return;
	else if(clock_format == 0 && hours > 23) /*Time errore*/ return;
	
	device->clock_reg.Seconds = ((seconds / 10) << 4) | (seconds % 10);
	device->clock_reg.Minutes = ((minutes / 10) << 4) | (minutes % 10);
	if (clock_format == 0 && hours >= 20)
	{
		device->clock_reg.Hours = ((hours / 10) << 5) | (hours % 10);
	} else
	{
		device->clock_reg.Hours = ((hours / 10) << 4) | (hours % 10) | (clock_format << 6);
	}
	
	DS3231_WriteTimeReg(device);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 * @param day : Day of week.
 * @param month : Month of the Year.
 * @param year : Just the year.
 *
 **/
void DS3231_Set_Date(DS3231_GDataInstance_typedef *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99) /*Date errore*/ return;
	
	device->clock_reg.Day = day;
	device->clock_reg.Date = ((date / 10) << 4) | (date % 10);
	device->clock_reg.MonthCentury = ((month / 10) << 4) | (month % 10);
	device->clock_reg.Year = ((year / 10) << 4) | (year % 10);
	
	DS3231_WriteDateReg(device);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 * @param alarm_condition : Condition for the alarm, see DS3231_AlarmCondition enum.
 * @param clock_format : See enum DS3231_ClockFormat.
 * @param dy_dt : Selects day or date for alarm condition, one is day, zero is date.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS3231_Set_Alarm1(DS3231_GDataInstance_typedef *device, DS3231_AlarmCondition alarm_condition, DS3231_ClockFormat clock_format, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day_date)
{
	if (seconds > 59 || minutes > 59 || clock_format > 1) /*Time errore*/ return;
	else if(clock_format == 1 && hours > 12) /*Time errore*/ return;
	else if(clock_format == 0 && hours > 23) /*Time errore*/ return;
	if((alarm_condition & 0x10) == 0x10 && (day_date < 1 || day_date > 7)) /*Time errore*/ return;
	else if(day_date < 1 || day_date > 31) /*Time errore*/ return;
	
	device->clock_reg.Alarm1Seconds = ((seconds / 10) << 4) | (seconds % 10) | ((alarm_condition & 0x01) << 7);
	device->clock_reg.Alarm1Minutes = ((minutes / 10) << 4) | (minutes % 10) | ((alarm_condition & 0x02) << 7);
	if (clock_format == 0 && hours >= 20)
	{
		device->clock_reg.Alarm1Hours = ((hours / 10) << 5) | (hours % 10) | ((alarm_condition & 0x04) << 7);
	} else
	{
		device->clock_reg.Alarm1Hours = ((hours / 10) << 4) | (hours % 10) | (clock_format << 6) | ((alarm_condition & 0x04) << 7);
	}
	
	device->clock_reg.Alarm1DayDate = ((day_date / 10) << 4) | (day_date % 10) | ((alarm_condition & 0x10) << 6) | ((alarm_condition & 0x08) << 7);
	
	DS3231_WriteAlarm1Reg(device);
}

/*
 * @brief 
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 * @param alarm_condition : Condition for the alarm, see DS3231_AlarmCondition enum.
 * @param clock_format : See enum DS3231_ClockFormat.
 * @param dy_dt : Selects day or date for alarm condition, one is day, zero is date.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS3231_Set_Alarm2(DS3231_GDataInstance_typedef *device, DS3231_AlarmCondition alarm_condition, DS3231_ClockFormat clock_format, uint8_t minutes, uint8_t hours, uint8_t day_date)
{
	if (minutes > 59 || clock_format > 1) /*Time errore*/ return;
	else if(clock_format == 1 && hours > 12) /*Time errore*/ return;
	else if(clock_format == 0 && hours > 23) /*Time errore*/ return;
	if ((alarm_condition & 0x10) == 0x10 && (day_date < 1 || day_date > 7)) /*Time errore*/ return;
	else if(day_date < 1 || day_date > 31) /*Time errore*/ return;
	
	device->clock_reg.Alarm2Minutes = ((minutes / 10) << 4) | (minutes % 10) | ((alarm_condition & 0x02) << 7);
	if (clock_format == 0 && hours >= 20)
	{
		device->clock_reg.Alarm2Hours = ((hours / 10) << 5) | (hours % 10) | ((alarm_condition & 0x04) << 7);
	}
	else
	{
		device->clock_reg.Alarm2Hours = ((hours / 10) << 4) | (hours % 10) | (clock_format << 6) | ((alarm_condition & 0x04) << 7);
	}
	
	device->clock_reg.Alarm2DayDate = ((day_date / 10) << 4) | (day_date % 10) | ((alarm_condition & 0x10) << 6) | ((alarm_condition & 0x08) << 7);
	
	DS3231_WriteAlarm2Reg(device);
}

/*
 * @brief Set bits of control and status registers.
 *		  For the all param see DS3231_GClockData_typedef.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 * @param a1ie, a2ie : Alarm 1 and alarm 2 Interrupt Enable. Default is 0, it is disable.
 * @param intcn : Interrupt Control (INTCN).  Default is 1. If it is logic 0, a square wave 
 *		  is output on the INT / SQW pin, otherwise controls the INT / SQW signal.
 * @param rs2_rs1 : See DS3231_SQWOutFreq enum.
 * @param conv : Convert Temperature (CONV).  Default is 0.
 * @param bbsqw : Battery-Backed Square-Wave Enable (BBSQW). Default is 0.
 * @param eosc : Enable Oscillator (EOSC). Default is 0. Active level is zero.
 *
 **/
void DS3231_Set_ControlReg(DS3231_GDataInstance_typedef *device, uint8_t a1ie, uint8_t a2ie, uint8_t intcn, 
	DS3231_SQWOutFreq rs2_rs1, 	uint8_t conv,  uint8_t bbsqw,  uint8_t eosc)
{
	device->clock_reg.partsCtrlReg.A1IE = a1ie;
	device->clock_reg.partsCtrlReg.A2IE = a2ie;
	device->clock_reg.partsCtrlReg.INTCN = intcn;
	device->clock_reg.partsCtrlReg.RS2_RS1 = rs2_rs1;
	device->clock_reg.partsCtrlReg.CONV = conv;
	device->clock_reg.partsCtrlReg.BBSQW = bbsqw;
	device->clock_reg.partsCtrlReg.EOSC = eosc;
	DS3231_WriteControlReg(device);
}

/*
 * @brief Set bits of control and status registers.
 *		  For the all param see DS3231_GClockData_typedef.
 *		  
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 * @param a1f, a2f : A1F is cleared when written to logic 0. This bit can 
 *        only be written to logic 0.
 * @param en32khz : Enable 32kHz Output (EN32kHz). Default is 1.
 *
 **/
void DS3231_Set_StatusReg(DS3231_GDataInstance_typedef *device, uint8_t a1f, uint8_t a2f, uint8_t en32khz)
{
	DS3231_ReadStatusReg(device);
	device->clock_reg.partsStatReg.A1F = a1f;
	device->clock_reg.partsStatReg.A2F = a2f;
	device->clock_reg.partsStatReg.EN32kHz = en32khz;	
	DS3231_WriteControlReg(device);
}

/*
 * @brief Gets time and date and convert it to decemal format. Data contained in the DS3231_Clock_typedef struct.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 *
 **/
void DS3231_Get_TimeDate(DS3231_GDataInstance_typedef *device)
{
	DS3231_ReadTimeDateReg(device);
	device->clock.Seconds = (device->clock_reg.Seconds & 0x0F) + ((device->clock_reg.Seconds & 0x70) >> 4) * 10;
	device->clock.Minutes = (device->clock_reg.Minutes & 0x0F) + ((device->clock_reg.Minutes & 0x70) >> 4) * 10;
	if (device->clock_reg.Hours & 0x40)
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x10) >> 4) * 10;								
	}
	else
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x10)  >> 4) * 10 + 
								((device->clock_reg.Hours & 0x20)  >> 5) * 20;
	}
	device->clock.Day = device->clock_reg.Day & 0x07;
	device->clock.Date = (device->clock_reg.Date & 0x0F) + ((device->clock_reg.Date & 0x30) >> 4) * 10;
	device->clock.Month = (device->clock_reg.MonthCentury & 0x0F) + ((device->clock_reg.MonthCentury & 0x10) >> 4) * 10;
	device->clock.Year = (device->clock_reg.Year & 0x0F) + ((device->clock_reg.Year & 0xF0) >> 4) * 10;
}


/*
 * @brief Get control and status registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 *
 **/
void DS3231_Get_ControlReg(DS3231_GDataInstance_typedef *device)
{
	DS3231_ReadControlReg(device);	
}

/*
 * @brief Private function.
 *
 **/

/*
 * @brief Read timekeeping registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct.
 *
 **/
static void DS3231_ReadTimeDateReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS3231_ADDR_SHIFTED, DS3231_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Seconds, 19);//7
	device->delay(5);	
}

/*
 * @brief Read alarm 1 registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_ReadAlarm1Reg(DS3231_GDataInstance_typedef *device)
{
	
	device->i2c_rx_data(DS3231_ADDR_SHIFTED, DS3231_ALARM1_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Alarm1Seconds, 4);
	device->delay(5);	
}

/*
 * @brief Read alarm 2 registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_ReadAlarm2Reg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS3231_ADDR_SHIFTED, DS3231_ALARM2_MINUTES_REG, 1, (uint8_t *) &device->clock_reg.Alarm2Minutes, 3);
	device->delay(5);	
}

/*
 * @brief Read control register.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_ReadControlReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS3231_ADDR_SHIFTED, DS3231_CONTROL_REG, 1, (uint8_t *) &device->clock_reg.ControlRegister, 1);
	device->delay(5);	
}

/*
 * @brief Read status register.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_ReadStatusReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS3231_ADDR_SHIFTED, DS3231_STATUS_REG, 1, (uint8_t *) &device->clock_reg.StatusRegister, 1);
	device->delay(5);	
}


/*
 * @brief Write timekeeping registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteTimeReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Seconds, 3);
	device->delay(5);	
}

/*
 * @brief Write date registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteDateReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_DAY_REG, 1, (uint8_t *) &device->clock_reg.Day, 4);
	device->delay(5);	
}

/*
 * @brief Write alarm 1 registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteAlarm1Reg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_ALARM1_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Alarm1Seconds, 4);
	device->delay(5);	
}

/*
 * @brief Write alarm 2 registers.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteAlarm2Reg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_ALARM2_MINUTES_REG, 1, (uint8_t *) &device->clock_reg.Alarm2Minutes, 3);
	device->delay(5);	
}

/*
 * @brief  Write control register.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteControlReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_CONTROL_REG, 1, (uint8_t *) &device->clock_reg.ControlRegister, 1);
	device->delay(5);	
}

/*
 * @brief  Write status register.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_WriteStatusReg(DS3231_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS3231_ADDR_SHIFTED, DS3231_STATUS_REG, 1, (uint8_t *) &device->clock_reg.StatusRegister, 1);
	device->delay(5);	
}

/*
 * @brief  Check the object reference.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS3231_CheckReference(DS3231_GDataInstance_typedef *device)
{
	if (device == NULL || device->delay == NULL || device->i2c_rx_data == NULL || device->i2c_tx_data == NULL)
	{
		for (;;)
		{
			///Errore. Object or function is not found. Check the object reference and function pointers.
		}
	}
}


