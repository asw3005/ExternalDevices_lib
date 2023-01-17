/*
 *	@brief DS1307 communication functions.
 *	Created 9.11.2020
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds1307.h"

/*
 * @brief Private function prototype.
 *
 **/
/*
 *	@brief DS1307 communication functions.
 *	Created 9.11.2020
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds3231.h"

/*
 * @brief Private function prototype.
 *
 **/
static void DS1307_ReadTimeDateReg(DS1307_GDataInstance_typedef *device);
static void DS1307_WriteTimeReg(DS1307_GDataInstance_typedef *device);
static void DS1307_WriteDateReg(DS1307_GDataInstance_typedef *device);
static void DS1307_WriteControlReg(DS1307_GDataInstance_typedef *device);
static void DS1307_CheckReference(DS1307_GDataInstance_typedef *device);

/*
 * @brief Public function.
 *
 **/

/*
 * @brief 
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 * @param clk_halt: Clock halt bit. See enum DS1307_ClockHalt.
 * @param clock_format : See enum DS1307_ClockFormat.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 *
 **/
void DS1307_Set_Time(DS1307_GDataInstance_typedef *device, DS1307_ClockHalt clk_halt, DS1307_ClockFormat clock_format, DS1307_HourOfDay am_pm, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	if (seconds > 59 || minutes > 59 || clock_format > 1) /*Time errore*/ return;
	else if(clock_format == DS1307_CLOCK_FORMAT_12HOURS && hours > 12) /*Time errore*/ return;
	else if(clock_format == DS1307_CLOCK_FORMAT_24HOURS && hours > 23) /*Time errore*/ return;
	
	//Hours set.
	if(hours >= 20)
	{
		device->clock_reg.Hours = (1 << 5) | (0 << 4) | (hours % 10) | (clock_format << 6);
	} else
	{
		device->clock_reg.Hours = ((hours / 10) << 4) | (hours % 10) | (am_pm << 5) | (clock_format << 6);
	}
	//Minutes set.
	device->clock_reg.Minutes = ((minutes / 10) << 4) | (minutes % 10);
	//Seconds set.
	device->clock_reg.Seconds = ((seconds / 10) << 4) | (seconds % 10) | (clk_halt << 7);
	
	DS1307_WriteTimeReg(device);	
}

/*
 * @brief 
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct.
 * @param day : Day of week.
 * @param month : Month of the Year.
 * @param year : Just the year.
 *
 **/
void DS1307_Set_Date(DS1307_GDataInstance_typedef *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99) /*Date errore*/ return;
	
	device->clock_reg.Day = day;
	device->clock_reg.Date = ((date / 10) << 4) | (date % 10);
	device->clock_reg.Month = ((month / 10) << 4) | (month % 10);
	device->clock_reg.Year = ((year / 10) << 4) | (year % 10);
	
	DS1307_WriteDateReg(device);	
}

/*
 * @brief Set bits of control and status registers.
 *		  For the all param see DS1307_GClockData_typedef.
 *		  
 * @param rs1_rs0 : Rate Select. See the DS1307_SQWOutFreq enum.
 * @param sqwe : Square-Wave Enable. See the DS1307_SqwSelector.
 * @param out : Output control. See the DS1307_OutPinLevel.
 *
 **/
void DS1307_Set_ControlReg(DS1307_GDataInstance_typedef *device,
	DS1307_SQWRate rs1_rs0,
	DS1307_SqwSelector sqwe,
	DS1307_OutPinLevel out)
{
	device->clock_reg.partsCtrlReg.RS1_RS0 = rs1_rs0;
	device->clock_reg.partsCtrlReg.SQWE = sqwe;
	device->clock_reg.partsCtrlReg.OUT = out;

	DS1307_WriteControlReg(device);
}

/*
 * @brief Gets time and date and convert it to decemal format. Data contained in the DS1307_Clock_typedef struct.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct.
 *
 **/
void DS1307_Get_TimeDate(DS1307_GDataInstance_typedef *device)
{
	DS1307_ReadTimeDateReg(device);
	device->clock.Seconds = (device->clock_reg.Seconds & 0x0F) + ((device->clock_reg.Seconds & 0x70) >> 4) * 10;
	device->clock.Minutes = (device->clock_reg.Minutes & 0x0F) + ((device->clock_reg.Minutes & 0x70) >> 4) * 10;
	if (device->clock_reg.Hours & 0x40)
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x10) >> 4) * 10;	
		device->clock.AmPm = (device->clock_reg.Hours & 0x20) >> 5;
	}
	else
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x10)  >> 4) * 10 + 
								((device->clock_reg.Hours & 0x20)  >> 5) * 20;
	}
	device->clock.Day = device->clock_reg.Day & 0x07;
	device->clock.Date = (device->clock_reg.Date & 0x0F) + ((device->clock_reg.Date & 0x30) >> 4) * 10;
	device->clock.Month = (device->clock_reg.Month & 0x0F) + ((device->clock_reg.Month & 0x10) >> 4) * 10;
	device->clock.Year = (device->clock_reg.Year & 0x0F) + ((device->clock_reg.Year & 0xF0) >> 4) * 10;
}

/*
 * @brief  Read RAM block.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 *
 **/
void DS1307_ReadRamBlock(DS1307_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS1307_ADDR_SHIFTED, DS1307_RAM_BLOCK, 1, (uint8_t *) &device->clock_reg.RamBlock, 56);
	device->delay(5);	
}

/*
 * @brief  Write RAM block.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 *
 **/
void DS1307_WriteRamBlock(DS1307_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS1307_ADDR_SHIFTED, DS1307_RAM_BLOCK, 1, (uint8_t *) &device->clock_reg.RamBlock, 56);
	device->delay(5);	
}

/*
 * @brief Private function.
 *
 **/

/*
 * @brief Read timekeeping registers.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct.
 *
 **/
static void DS1307_ReadTimeDateReg(DS1307_GDataInstance_typedef *device)
{
	device->i2c_rx_data(DS1307_ADDR_SHIFTED, DS1307_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Seconds, 7);    //7
	device->delay(5);	
}

/*
 * @brief Write time registers.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 *
 **/
static void DS1307_WriteTimeReg(DS1307_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS1307_ADDR_SHIFTED, DS1307_SECONDS_REG, 1, (uint8_t *) &device->clock_reg.Seconds, 3);
	device->delay(5);	
}

/*
 * @brief Write date registers.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 *
 **/
static void DS1307_WriteDateReg(DS1307_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS1307_ADDR_SHIFTED, DS1307_DAY_REG, 1, (uint8_t *) &device->clock_reg.Day, 4);
	device->delay(5);	
}


/*
 * @brief  Write control register.
 * 
 * @param *device : Instance of DS1307_GDataInstance_typedef data struct. 
 *
 **/
static void DS1307_WriteControlReg(DS1307_GDataInstance_typedef *device)
{
	device->i2c_tx_data(DS1307_ADDR_SHIFTED, DS1307_CONTROL_REG, 1, (uint8_t *) &device->clock_reg.ControlRegister, 1);
	device->delay(5);	
}

/*
 * @brief  Check the object reference.
 * 
 * @param *device : Instance of DS3231_GDataInstance_typedef data struct. 
 *
 **/
static void DS1307_CheckReference(DS1307_GDataInstance_typedef *device)
{
	if (device == NULL || device->delay == NULL || device->i2c_rx_data == NULL || device->i2c_tx_data == NULL)
	{
		for (;;)
		{
			///Errore. Object or function is not found. Check the object reference and function pointers.
		}
	}
}