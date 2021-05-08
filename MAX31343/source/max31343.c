/*
 *	@brief MAX31343 communication functions.
 *	Created 05.07.21 by asw3005 
 *
 **/

#include "stm32f1xx_hal.h"
#include "max31343.h"

/* Private function prototype. */


/*
 * @brief Public function.
 *
 **/

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct. 
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 *
 **/
void MAX31343_Set_Time(MAX31343_GIns_t *device, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	if (seconds > 59 || minutes > 59 || hours > 23) /* Time errore */ return;
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param day : Day of week.
 * @param month : Month of the Year.
 * @param year : Just the year.
 *
 **/
void MAX31343_Set_Date(MAX31343_GIns_t *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99) /* Date error. */ return;
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param alarm_condition : Condition for the alarm, see DS3231_AlarmCondition enum.
 * @param dy_dt : Selects day or date for alarm condition, one is day, zero is date.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS3231_Set_Alarm1(MAX31343_GIns_t *device, uint8_t day_date, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	if (seconds > 59 || minutes > 59) /*Time errore*/ return;
}

/*
 * @brief 
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param alarm_condition : Condition for the alarm, see DS3231_AlarmCondition enum.
 * @param dy_dt : Selects day or date for alarm condition, one is day, zero is date.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS3231_Set_Alarm2(MAX31343_GIns_t *device, uint8_t day_date, uint8_t hours, uint8_t minutes)
{
	if (minutes > 59) /*Time errore*/ return;

}

/*
 * @brief Set bits of control and status registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 * @param 
 * @param 
 *		  
 * @param 
 * @param 
 * @param 
 * @param 
 *
 **/
void MAX31343_Set_ControlReg(MAX31343_GIns_t *device)
{
	
}

/*
 * @brief Set bits of control and status registers.
 *		  
 * @param 
 * @param 
 *        
 * @param 
 *
 **/
void DS3231_Set_StatusReg(MAX31343_GIns_t *device)
{

}

/*
 * @brief Gets time and date and convert it to decemal format. Data contained in the DS3231_Clock_typedef struct.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 *
 **/
void DS3231_Get_TimeDate(MAX31343_GIns_t *device)
{
	
}


/*
 * @brief Get control and status registers.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct.
 *
 **/
void DS3231_Get_ControlReg(MAX31343_GIns_t *device)
{
	
}

/*
 * @brief Private function.
 *
 **/


/*
 * @brief  Write status register.
 * 
 * @param *device : Instance of MAX31343_GIns_t data struct. 
 *
 **/
static void DS3231_WriteStatusReg(MAX31343_GIns_t *device)
{
	device->i2c_tx_data(MAX31343_ADDR_SHIFTED, MAX31343_STATUS_REG, 1, (uint8_t *)NULL, 1);
	device->delay(5);	
}

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


