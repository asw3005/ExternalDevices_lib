/*
 *	@brief DS12887 communication functions.
 *	Created 28.10.2020
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds12887.h"

/*
 * @brief Private function prototype.
 *
 **/
static void DS12887_ReadTimeDateReg(DS12887_GDataInstance_typedef *device);
static void DS12887_ReadControlReg(DS12887_GDataInstance_typedef *device);
static void DS12887_WriteTimeDateReg(DS12887_GDataInstance_typedef *device);
static void DS12887_WriteControlReg(DS12887_GDataInstance_typedef *device);
static void DS12887_CheckReference(DS12887_GDataInstance_typedef *device);
static uint8_t DS12887_BitSwap(uint8_t byte);

/*
 * @brief Hardware depending function. 
 *
 **/
static void DS12887_DataPinsAsInput(void);
static void DS12887_DataPinsAsOut(void);
static uint8_t DS12887_WriteReadData(uint8_t rw_bit, uint8_t rw_byte);

/*
 * @brief 
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct. * 
 * @param clock_format : See enum DS12887_ClockFormat.
 * @param am_pm : time of day in the twelve hour clock format.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 *
 **/
void DS12887_Set_Time(DS12887_GDataInstance_typedef *device, DS12887_HourOfDay am_pm, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	device->rx_data(DS12887_CONTROL_REG_B, &device->clock_reg.ControlRegisterB, 1);
	
	if (seconds > 59 || minutes > 59) /*Time errore*/ return;
	else if((device->clock_reg.ControlRegisterB & 0x02) == DS12887_CLOCK_FORMAT_12HOURS && hours > 12) /*Time errore*/ return;
	else if((device->clock_reg.ControlRegisterB & 0x02) == DS12887_CLOCK_FORMAT_24HOURS && hours > 23) /*Time errore*/ return;
	
	//Hours, minutes and seconds set depending on the time format in the RTC.
	if((device->clock_reg.ControlRegisterB & 0x04) == DS12887_DATA_FORMAT_BCD)
	{
		device->clock_reg.Hours = ((hours / 10) << 4) | (hours % 10) | (am_pm << 7);
		device->clock_reg.Minutes = ((minutes / 10) << 4) | (minutes % 10);
		device->clock_reg.Seconds = ((seconds / 10) << 4) | (seconds % 10);		
	}
	else
	{
		device->clock_reg.Hours = hours | (am_pm << 7);
		device->clock_reg.Minutes = minutes;
		device->clock_reg.Seconds = seconds;		
	}	
	
	device->tx_data(DS12887_SECONDS_REG, &device->clock_reg.Seconds, 1);
	device->delay(1);
	device->tx_data(DS12887_MINUTES_REG, &device->clock_reg.Minutes, 1);
	device->delay(1);
	device->tx_data(DS12887_HOURS_REG, &device->clock_reg.Hours, 1);
	device->delay(1);
	
}

/*
 * @brief 
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 * @param day : Day of week.
 * @param month : Month of the Year.
 * @param year : Just the year.
 *
 **/
void DS12887_Set_Date(DS12887_GDataInstance_typedef *device, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
	if (day < 1 || day > 7 || date < 1 || date > 31 || month < 1 || month > 12 || year > 99) /*Date errore*/ return;
	
	device->clock_reg.Day = day;
	device->clock_reg.Date = ((date / 10) << 4) | (date % 10);
	device->clock_reg.Month = ((month / 10) << 4) | (month % 10);
	device->clock_reg.Year = ((year / 10) << 4) | (year % 10);
	
	device->tx_data(DS12887_DAY_REG, &device->clock_reg.Day, 4);
	device->delay(1);

}

/*
 * @brief 
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 * @param alarm_condition : Condition for the alarm, see DS12887_AlarmCondition enum.
 * @param clock_format : See enum DS12887_ClockFormat.
 * @param dy_dt : Selects day or date for alarm condition, one is day, zero is date.
 * @param seconds : Amount of seconds in the clock.
 * @param minutes : Amount of minutes in the clock.
 * @param hours : Amount of hours in the clock.
 * @param day_date : Day or date in the calendar.
 *
 **/
void DS12887_Set_Alarm(DS12887_GDataInstance_typedef *device, DS12887_HourOfDay am_pm, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	device->rx_data(DS12887_CONTROL_REG_B, &device->clock_reg.ControlRegisterB, 1);
	
	if (seconds > 59 || minutes > 59) /*Time errore*/ return;
	else if((device->clock_reg.ControlRegisterB & 0x02) == DS12887_CLOCK_FORMAT_12HOURS && hours > 12) /*Time errore*/ return;
	else if((device->clock_reg.ControlRegisterB & 0x02) == DS12887_CLOCK_FORMAT_24HOURS && hours > 23) /*Time errore*/ return;
	
	//Alarm's hours, minutes and seconds set depending on the time format in the RTC.
	if((device->clock_reg.ControlRegisterB & 0x04) == DS12887_DATA_FORMAT_BCD)
	{
		device->clock_reg.HoursAlarm = ((hours / 10) << 4) | (hours % 10) | (am_pm << 7);
		device->clock_reg.MinutesAlarm = ((minutes / 10) << 4) | (minutes % 10);
		device->clock_reg.SecondsAlarm = ((seconds / 10) << 4) | (seconds % 10);		
	}
	else
	{
		device->clock_reg.HoursAlarm = hours | (am_pm << 7);
		device->clock_reg.MinutesAlarm = minutes;
		device->clock_reg.SecondsAlarm = seconds;
	}	
	
	device->tx_data(DS12887_ALARM_SECONDS_REG, &device->clock_reg.SecondsAlarm, 1);
	device->delay(1);
	device->tx_data(DS12887_ALARM_MINUTES_REG, &device->clock_reg.MinutesAlarm, 1);
	device->delay(1);
	device->tx_data(DS12887_ALARM_HOURS_REG, &device->clock_reg.HoursAlarm, 1);
	device->delay(1);
}


/*
 * @brief Set bits of control register A.
 *		  For the all param see DS12887_GClockData_typedef.
 *		  
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 * @param rs3_0 : Rate Selector, see the ControlRegisterA in the DS12887_GClockData_typedef struct.
 * @param dv2_0 : Oscillator on/off,  see the ControlRegisterA in the DS12887_GClockData_typedef struct.
 *
 **/
void DS12887_Set_ControlRegA(DS12887_GDataInstance_typedef *device, DS12887_SQWIntFreq rs3_0, DS12887_OscillatorOnOff dv2_0)
{
	device->clock_reg.partsCtrlRegA.RS3_RS0 = rs3_0;
	device->clock_reg.partsCtrlRegA.DV2_DV0 = dv2_0;
	
	device->tx_data(DS12887_CONTROL_REG_A, &device->clock_reg.ControlRegisterA, 1);
	device->delay(1);
}

/*
 * @brief Set bits of control register B.
 *		  For the all param see DS12887_GClockData_typedef.
 *		  
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 * @param daylight : Daylight Saving Enable (DSE). Adjustments do not occur when the DSE bit is 0.
 * @param clock_format : The 24/#12 control bit. A 1 indicates the 24-hour mode, zero is 12.
 * @param data_mode : Data Mode (DM). A 1 in DM signifies binary data, while a 0 in DM specifies BCD data.
 * @param sqwe : Square-Wave Enable (SQWE). When the SQWE bit is set to 0, the SQW pin is held low, otherwise
 *		  a square-wave signal at the frequency set by the rate (see enum DS12887_SQWIntFreq and ControlRegisterA).
 * @param uie : Update-Ended Interrupt Enable (UIE). See the ControlRegisterB in the DS12887_GClockData_typedef struct.
 * @param aie : Alarm Interrupt Enable (AIE). See the ControlRegisterB in the DS12887_GClockData_typedef struct.
 * @param pie : Periodic Interrupt Enable (PIE). See the ControlRegisterB in the DS12887_GClockData_typedef struct.
 * @param set : See the ControlRegisterB in the DS12887_GClockData_typedef struct.
 *
 **/
void DS12887_Set_ControlRegB(DS12887_GDataInstance_typedef *device, DS12887_DaylightSaving daylight, DS12887_ClockFormat clock_format, DS12887_DataFormat data_mode, 
	DS12887_SquareWave sqwe, DS12887_UpdateEndedInterrupt uie, DS12887_AlarmInterrupt aie, DS12887_PeriodicInterrupt pie, DS12887_UpdateClockReg set)
{
	device->clock_reg.partsStatRegB.DSE = daylight;
	device->clock_reg.partsStatRegB.FORMAT12_24 = clock_format;
	device->clock_reg.partsStatRegB.DM = data_mode;
	device->clock_reg.partsStatRegB.SQWE = sqwe;
	device->clock_reg.partsStatRegB.UIE = uie;
	device->clock_reg.partsStatRegB.AIE = aie;
	device->clock_reg.partsStatRegB.PIE = pie;
	device->clock_reg.partsStatRegB.SET = set;
	
	device->tx_data(DS12887_CONTROL_REG_B, &device->clock_reg.ControlRegisterB, 1);
	device->delay(1);
}

/*
 * @brief Gets time and date and convert it to decemal format. Data contained in the DS12887_Clock_typedef struct.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 *
 **/
void DS12887_Get_TimeDate(DS12887_GDataInstance_typedef *device)
{
	device->rx_data(DS12887_SECONDS_REG, &device->clock_reg.Seconds, 10);
	device->delay(1);
	device->rx_data(DS12887_CONTROL_REG_B, &device->clock_reg.ControlRegisterB, 1);
	
	device->clock.Seconds = (device->clock_reg.Seconds & 0x0F) + ((device->clock_reg.Seconds & 0x70) >> 4) * 10;
	device->clock.Minutes = (device->clock_reg.Minutes & 0x0F) + ((device->clock_reg.Minutes & 0x70) >> 4) * 10;
	if ((device->clock_reg.ControlRegisterB & 0x02) == DS12887_CLOCK_FORMAT_12HOURS)
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x10) >> 4) * 10;								
	}
	else
	{
		device->clock.Hours = (device->clock_reg.Hours & 0x0F) + ((device->clock_reg.Hours & 0x30)  >> 4) * 10;
	}
	device->clock.Day = device->clock_reg.Day & 0x07;
	device->clock.Date = (device->clock_reg.Date & 0x0F) + ((device->clock_reg.Date & 0x30) >> 4) * 10;
	device->clock.Month = (device->clock_reg.Month & 0x0F) + ((device->clock_reg.Month & 0x10) >> 4) * 10;
	device->clock.Year = (device->clock_reg.Year & 0x0F) + ((device->clock_reg.Year & 0xF0) >> 4) * 10;
}

/*
 * @brief Get status bits from the ControlRegC.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 *
 **/
void DS12887_Get_ControlRegC(DS12887_GDataInstance_typedef *device)
{	
	device->rx_data(DS12887_CONTROL_REG_C, &device->clock_reg.ControlRegisterC, 1);
	device->delay(1);
}

/*
 * @brief Get status bits from the ControlRegD.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 *
 **/
void DS12887_Get_ControlRegD(DS12887_GDataInstance_typedef *device)
{	
	device->rx_data(DS12887_CONTROL_REG_D, &device->clock_reg.ControlRegisterD, 1);
	device->delay(1);
}

/*
 * @brief Read RAM block.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 *
 **/
void DS12887_ReadRamBlock(DS12887_GDataInstance_typedef *device)
{	
	device->rx_data(DS12887_RAM_BLOCK, (uint8_t *) &device->clock_reg.RamBlock, 114);
	device->delay(1);
}

/*
 * @brief  Write RAM block.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct.
 *
 **/
void DS12887_WriteRamBlock(DS12887_GDataInstance_typedef *device)
{	
	device->tx_data(DS12887_RAM_BLOCK, (uint8_t *) &device->clock_reg.RamBlock, 114);
	device->delay(1);
}

#ifndef USE_DATA_RW_FSMC

/*
 * @brief Config the control pins.
 *
 **/
void DS12887_ConfigControlPins(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	/*Configure GPIO pins. */
	GPIO_InitStruct.Pin = CHIP_SELECT_PIN | DATA_STROBE_PIN | READ_WRITE_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = ADDRESS_STROBE_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ADDR_STROBE_PIN_PORT, &GPIO_InitStruct);
}

#endif /*USE_DATA_RW_FSMC*/

/*Static functions.*/

/*
 * @brief  Check the object reference.
 * 
 * @param *device : Instance of DS12887_GDataInstance_typedef data struct. 
 *
 **/
static void DS12887_CheckReference(DS12887_GDataInstance_typedef *device)
{
	if (device == NULL || device->delay == NULL || device->rx_data == NULL || device->tx_data == NULL)
	{
		for (;;)
		{
			///Errore. Object or function is not found. Check the object reference and function pointers.
		}
	}
}

/*
 * @brief Private function.
 *
 **/

static uint8_t DS12887_BitSwap(uint8_t byte)
{
	uint8_t tempByte = 0;
	
	__NOP();
	for (uint8_t i = 0; i < 8; i++)
	{
		tempByte |= ((byte & (0x01 << i)) >> i) << (7 - i);		
	}	
	return tempByte;
}


/*Hardware dependent function for write and read byte(s).*/

#ifdef USE_DATA_RW_FSMC

/*
 * @brief Read data with FSMC. Intel 8080 protocol.
 *
 **/
void DS12887_ReadData(uint8_t MemAddress, uint8_t *pData, uint8_t size)
{
	uint8_t* byte_data = (uint8_t* )0x60000000;
	
	for (uint8_t i = 0; i < size; i++) {
		*pData = byte_data[MemAddress + i];
		pData++;
	}
}

/*
 * @brief Write data with FSMC. Intel 8080 protocol.
 *
 **/
void DS12887_WriteData(uint8_t MemAddress, uint8_t *pData, uint8_t size)
{
	uint8_t* byte_data = (uint8_t*)0x60000000;
	
	for (uint8_t i = 0; i < size; i++) {
		byte_data[MemAddress + i] = *pData;
		pData++;
	}	
}

#else

/*
 * @brief Write data with I80 emulation. Intel 8080 protocol.
 *
 **/
void DS12887_WriteData(uint8_t MemAddress, uint8_t *pData, uint8_t size)
{
	DS12887_DataPinsAsOut();
	
	for (uint8_t i = 0; i < size; i++)
	{	
		//Asserting chip select condition and sets data, r/w and address pins to HIGH state.
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DS_PIN_PORT, DATA_STROBE_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RW_PIN_PORT, READ_WRITE_PIN, GPIO_PIN_SET);
		//Note. This signal inverted for my way. Check your signal polarity.   
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_SET);	
		__NOP(); __NOP(); __NOP(); __NOP();	
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_RESET);
		
		//Sets address and latch it.
		/* If you use direct order to connect the data/address bus, use youre own implementation to write byte to the port. */
		DS12887_WriteReadData(0, MemAddress + i);
		__NOP(); __NOP(); __NOP(); __NOP();	
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);
		//Note. This signal inverted for my way. Check your signal polarity. 
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_SET);
		__NOP(); __NOP(); __NOP(); __NOP(); 					//90nS wait
	
		//Sets data and writes it.
		/* If you use direct order to connect the data/address bus, use youre own implementation to write byte to the port. */
		DS12887_WriteReadData(0, *pData);
		HAL_GPIO_WritePin(RW_PIN_PORT, READ_WRITE_PIN, GPIO_PIN_RESET);
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();  							//150nS wait
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();  							//150nS wait
		HAL_GPIO_WritePin(RW_PIN_PORT, READ_WRITE_PIN, GPIO_PIN_SET);
		pData++;
	
		//Release CS and AS.
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		//Note. This signal inverted for my way. Check your signal polarity. 
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_RESET);
	}
}

/*
 * @brief Read data with I80 emulation. Intel 8080 protocol.
 *
 **/
void DS12887_ReadData(uint8_t MemAddress, uint8_t *pData, uint8_t size)
{

	for (uint8_t i = 0; i < size; i++)
	{	
		DS12887_DataPinsAsOut();
	
		//Asserting chip select condition and sets data, r/w and address pins to HIGH state.
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DS_PIN_PORT, DATA_STROBE_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RW_PIN_PORT, READ_WRITE_PIN, GPIO_PIN_SET);
		//Note. This signal inverted for my way. Check your signal polarity. 
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_SET);	
		__NOP(); __NOP(); __NOP(); __NOP();	
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_RESET);
		//Sets address and latch it.	
		/* If you use direct order to connect the data/address bus, use youre own implementation to write byte to the port. */
		DS12887_WriteReadData(0, MemAddress + i);
		__NOP(); __NOP(); __NOP(); __NOP();	
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);	
		//Note. This signal inverted for my way. Check your signal polarity. 
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_SET);
		__NOP(); __NOP(); __NOP(); __NOP();  					//90nS wait
	
		//Reads data.	
		DS12887_DataPinsAsInput();	
	
		HAL_GPIO_WritePin(DS_PIN_PORT, DATA_STROBE_PIN, GPIO_PIN_RESET);
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();   							//150nS wait
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();   							//150nS wait
		 /* If you use direct order to connect the data/address bus, use youre own implementation to read byte from the port. */
		*pData = DS12887_WriteReadData(1, 0);
		HAL_GPIO_WritePin(DS_PIN_PORT, DATA_STROBE_PIN, GPIO_PIN_SET);
		pData++;	
	
		//Release CS and AS.
		HAL_GPIO_WritePin(CS_PIN_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);
		//Note. This signal inverted for my way. Check your signal polarity. 
		HAL_GPIO_WritePin(ADDR_STROBE_PIN_PORT, ADDRESS_STROBE_PIN, GPIO_PIN_RESET);
	}
}



/*
 * @brief Make the set of data pins.
 * @param rw_bit : 0 to write byte to the data port or 1 to read data from the data port.
 *
 **/
static uint8_t DS12887_WriteReadData(uint8_t rw_bit, uint8_t rw_byte)
{
	uint8_t read_byte = 0;
	uint16_t tmp;
	
	if (rw_bit)
	{
		tmp = GPIOD->IDR;
		read_byte |= (uint8_t)((tmp & 0x4000) >> 14);
		read_byte |= (uint8_t)((tmp & 0x8000) >> 14);
		read_byte |= (uint8_t)((tmp & 0x0001) << 2);
		read_byte |= (uint8_t)((tmp & 0x0002) << 2);
		tmp = GPIOE->IDR;
		read_byte |= (uint8_t)((tmp & 0x0080) >> 3);
		read_byte |= (uint8_t)((tmp & 0x0100) >> 3);
		read_byte |= (uint8_t)((tmp & 0x0200) >> 3);
		read_byte |= (uint8_t)((tmp & 0x0400) >> 3);
	}
	else
	{
		GPIOD->BSRR = 0xC0030000;
		GPIOE->BSRR = 0x07800000;
		GPIOD->BSRR = ((uint16_t)(rw_byte) & 0x0001) << 14;
		GPIOD->BSRR = ((uint16_t)(rw_byte) & 0x0002) << 14;
		GPIOD->BSRR = ((uint16_t)(rw_byte) & 0x0004) >> 2;
		GPIOD->BSRR = ((uint16_t)(rw_byte) & 0x0008) >> 2;
		GPIOE->BSRR = ((uint16_t)(rw_byte) & 0x0010) << 3;
		GPIOE->BSRR = ((uint16_t)(rw_byte) & 0x0020) << 3;
		GPIOE->BSRR = ((uint16_t)(rw_byte) & 0x0040) << 3;
		GPIOE->BSRR = ((uint16_t)(rw_byte) & 0x0080) << 3;
	}
	
	return read_byte;
}

/*
 * @brief Make the pins as input.
 *
 **/
static void DS12887_DataPinsAsInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	/*Configure GPIO pins. */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	
	
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);		
}

/*
 * @brief Make the pins as output.
 *
 **/
static void DS12887_DataPinsAsOut(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	/*Configure GPIO pins. */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
}

#endif /* USE_DATA_RW_FSMC */






