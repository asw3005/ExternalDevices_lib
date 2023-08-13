/*
 * @brief HD44780 driver.
 * Created 1.12.2020
 * 
 **/

#include "stm32f1xx_hal.h"
#include "hd44780.h"
#include "almanac.h"

/*
 * @brief Private function prototype.
 *
 **/
static void HD44780_Set_Line(HD44780_Inst_t* device, const uint8_t line, uint8_t offset);
static void HD44780_WriteRAM(HD44780_Inst_t* device);

/*Hardware depending functions*/
static void HD44780_WriteByte(const uint8_t byte);


/*
 * @brief Table your own symbols into the display's RAM.
 *
 **/
const uint8_t HD44780_RamChar[64] = 
{
	/* Lightning. */
	0x00, 0x0E, 0x0E, 0x00, 0x00, 0x0E, 0x0E, 0x00,       
	/* Celcius degree. */
	0x07, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
	/* Parametr selector. */
	0x00, 0x02, 0x06, 0x0E, 0x0E, 0x06, 0x02, 0x00,
	/* +- */
	//0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00,  
	/* Battery. */
	0x0E, 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F,    
	/* \ */
	0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00,    
	/* | */
	0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00,    
	/* Home. */
	//0x00, 0x04, 0x0E, 0x1F, 0x0E, 0x0A, 0x0E, 0x00,
	/* { */
	//0x04, 0x08, 0x08, 0x10, 0x08, 0x08, 0x04, 0x00,   
	/* } */
	//0x04, 0x02, 0x02, 0x01, 0x02, 0x02, 0x04, 0x00,   
	/* Plane. */
	//0x04, 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x0E,   
	/* Water tap. */
	//0x1F, 0x04, 0x0E, 0x1F, 0x1F, 0x0E, 0x04, 0x1F,   
	/* Lightning. */
	// 0x01, 0x02, 0x04, 0x0F, 0x01, 0x02, 0x04, 0x08,  
	/* Cold storage. */
	0x1F, 0x1F, 0x1F, 0x1F, 0x11, 0x11, 0x11, 0x1F,   
	/* Freezer. */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F
};

/*
 * @brief ASCII decodin table.
 *
 **/
const uint8_t HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[256] = 
{
//                                                   BS
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x99, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  16   
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  32

//  0xFF   !     "     #     $     %     $     '     (     )     *     +    0x2C   -     .     /    
    0x20, 0x21,	0x22, 0x23,	0x24, 0x25,	0x26, 0x27,	0x28, 0x29,	0x2A, 0x2B,	0x2C, 0x2D,	0x2E, 0x2F,  //  48
//   0     1     2     3     4     5     6     7     8     9     :     ;     <     =     >     ?
    0x30, 0x31,	0x32, 0x33,	0x34, 0x35,	0x36, 0x37,	0x38, 0x39,	0x3A, 0x3B,	0x3C, 0x3D,	0x3E, 0x3F,  //  64
//   @     A     B     C     D     E     F     G     H     I     J     K     L     M     N     O
  	0x40, 0x41,	0x42, 0x43,	0x44, 0x45,	0x46, 0x47,	0x48, 0x49,	0x4A, 0x4B,	0x4C, 0x4D,	0x4E, 0x4F,  //  80
//   P     Q     R     S     T     U     V     W     X     Y     Z     [     \   ]     ^     _ 
  	0x50, 0x51,	0x52, 0x53,	0x54, 0x55,	0x56, 0x57,	0x58, 0x59,	0x5A, 0x5B,	0x04, 0x5D,	0x5E, 0x5F,  //  96
//   `     a     b     c     d     e     f     g     h     i     j     k     l     m     n     o
  	0x60, 0x61,	0x62, 0x63,	0x64, 0x65,	0x66, 0x67,	0x68, 0x69,	0x6A, 0x6B,	0x6C, 0x6D,	0x6E, 0x6F,  //  112
//   p     q     r     s     t     u     v     w     x     y     z     {     |     }     ~    0xFF
  	0x70, 0x71,	0x72, 0x73,	0x74, 0x75,	0x76, 0x77,	0x78, 0x79,	0x7A, 0x06,	0x05, 0x07,	0xE9, 0xFF,  //  128

  	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  144
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  160  
//                                             §     Ё     ©                             ®       
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xA2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  176
//   °     ±                                         ё
    0xDF, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //  192
//   А     Б     В     Г     Д     Е     Ж     З     И     Й     К     Л     М     Н     О     П
    0x41, 0xA0, 0x42, 0xA1, 0xE0, 0x45, 0xA3, 0xA4, 0xA5, 0xA6, 0x4B, 0xA7, 0x4D, 0x48, 0x4F, 0xA8,  //  208
//   Р     С     Т     У     Ф     Х     Ц     Ч     Ш     Щ     Ъ     Ы     Ь     Э     Ю     Я
    0x50, 0x43, 0x54, 0xA9, 0xAA, 0x58, 0xE1, 0xAB, 0xAC, 0xE2, 0xAD, 0xAE, 0x62, 0xAF, 0xB0, 0xB1,  //  224
//   а     б     в     г     д     е     ж     з     и     й     к     л     м     н     о     п
    0x61, 0xB2, 0xB3, 0xB4, 0xE3, 0x65, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0x6F, 0xBE,  //  240
//   р     с     т     у     ф     х     ц     ч     ш     щ     ъ     ы     ь     э     ю     я
    0x70, 0x63, 0xBF, 0x79, 0xE4, 0x78, 0xE5, 0xC0, 0xC1, 0xE6, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,  //  256

};

/*
 *
 *
 **/
static uint8_t bus_lenght = HD44780_BUS_TYPE_8BITS;


/*
 * @brief Public function.
 *
 **/

/*
 * @brief Initialization screen.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param bus_type : It can be set to HD44780_BUS_TYPE_4BITS or HD44780_BUS_TYPE_8ITS.
 *
 **/		
void HD44780_InitScreen(HD44780_Inst_t* device, const uint8_t bus_type)
{			 
	/*Wait 50mS after power up.*/
	device->user_delay(50);
	
	/**/
	bus_lenght = bus_type;
	
	/**/
	if (bus_type == HD44780_BUS_TYPE_4BITS)
	{		
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
	
		device->user_write_command(HD44780_CMD_DATA_LENGTH_4BITS);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DATA_LENGTH_4BITS | HD44780_CMD_DISPLAY_2LINE | HD44780_CMD_DISPLAY_FONT5_8);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DISPLAY_ON | HD44780_CMD_CURSOR_OFF | HD44780_CMD_CURSOR_BLINKING_ON);      	
		device->user_delay(1);
		
//		device->user_write_command(HD44780_CMD_INC_MODE | HD44780_CMD_DISPLAY_SHIFT_DIS);
//		device->user_delay(1);
		
		device->user_write_command(HD44780_CMD_CLEAR_SCREEN);    						      
		device->user_delay(2);
		device->user_write_command(HD44780_CMD_INC_MODE);    						  	
		device->user_delay(1);
		
	} else if (bus_type == HD44780_BUS_TYPE_8BITS)
	{
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS);	
		device->user_delay(5);
	
		device->user_write_command(HD44780_CMD_DATA_LENGTH_8BITS | HD44780_CMD_DISPLAY_2LINE | HD44780_CMD_DISPLAY_FONT5_8);	
		device->user_delay(5);
		device->user_write_command(HD44780_CMD_DISPLAY_ON | HD44780_CMD_CURSOR_OFF | HD44780_CMD_CURSOR_BLINKING_OFF);   
		
//		device->user_write_command(HD44780_CMD_INC_MODE | HD44780_CMD_DISPLAY_SHIFT_DIS);
//		device->user_delay(1);
		
		device->user_delay(1);
		device->user_write_command(HD44780_CMD_CLEAR_SCREEN);    						      
		device->user_delay(2);
		device->user_write_command(HD44780_CMD_INC_MODE);    						  	
		device->user_delay(1);
	}	
	
	/* Writes your own massive of symbols. See the HD44780_RamChar abouve. */
	HD44780_WriteRAM(device);
	
	/**/

}

/*
 * @brief Clear screen.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 *
 **/
void HD44780_ClearScreen(HD44780_Inst_t* device)
{
	device->user_write_command(HD44780_CMD_CLEAR_SCREEN);	
	device->user_delay(1);
}

/*
 * @brief Clear specific position of the screen.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param size : size of data that will be clear.
 *
 **/
void HD44780_ClearScreenPosition(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t size)
{
	HD44780_Set_Line(device, line, offset);
	
	for (uint8_t i = 0; i < size; i++) {
		device->user_write_data(' ');
	}
}

/*
 * @brief 
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 *
 **/
void HD44780_WriteChar(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t code)
{
	//device->user_delay(100);
	HD44780_Set_Line(device, line, offset);
	device->user_write_data(code);	
}

/*
 * @brief 
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 *
 **/
void HD44780_CursorMove(HD44780_Inst_t* device, HD44780_CursorMoveDirection direction, uint8_t offset)
{
	if (direction == HD44780_CURSOR_MOVE_LEFT)
	{
		for (uint8_t i = offset; i != 0; i--) {
			device->user_write_command(HD44780_CMD_CURSOR_MOVE | HD44780_CMD_SHIFT_TO_LEFT);
		}
	}
	else if (direction == HD44780_CURSOR_MOVE_RIGHT)
	{
		for (uint8_t i = offset; i != 0; i--) {
			device->user_write_command(HD44780_CMD_CURSOR_MOVE | HD44780_CMD_SHIFT_TO_RIGHT);
		}
	}
}
		
/*
 * @brief Send text message to the screen.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : It's number of line on the screen.
 * @param message : It's string of chars which function sends to screen.
 * @param size : It's number of chars.
 *
 **/
void HD44780_SendMessage(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t* message, uint8_t size)
{
	HD44780_Set_Line(device, line, offset);				
			
	for(uint8_t i = 0 ; i < size ; i++)
	{						
		device->user_write_data(*message);  			
		message++;
	}			
}

/*
 * @brief Sends message (string) and float number to the screen. Float number diplays in the format 2.1f.
 *
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : It's number of line on the screen.
 * @param offset : It's position offset on the screen line.
 * @param in_float : It's just float number.
 *
 **/
void HD44780_SendNumberF2_1(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float in_float)
{
	uint8_t float_length = 4; 	
	uint16_t float_to_char;
	float three_dig_float = ((int16_t)(in_float * 10)) / 10.0f;
	
	float_to_char = (uint16_t)(three_dig_float * 10);	
	if (three_dig_float < 0) float_to_char = (uint16_t)(-(three_dig_float * 10));	
	
	/*Convetion digit to string. Cursor always set to sign place.*/	
	for(uint8_t i = float_length ; i > 0 ; i--)
	{
		if (i != 3) {
			HD44780_Set_Line(device, line, offset + i);
			device->user_write_data((float_to_char % 10) + 48);
			if (i == 1) {
				if (three_dig_float >= 10)
				{
					HD44780_CursorMove(device, HD44780_CURSOR_MOVE_LEFT, 2);
				}
				else if (three_dig_float < 10 && three_dig_float >= 0)
				{
					HD44780_Set_Line(device, line, offset);
					device->user_write_data(' ');
					device->user_write_data(' ');
					HD44780_CursorMove(device, HD44780_CURSOR_MOVE_LEFT, 1);
				}
				else if (three_dig_float < 0 && three_dig_float > -10)
				{	
					HD44780_Set_Line(device, line, offset);
					device->user_write_data(' ');
					device->user_write_data('-');
					HD44780_CursorMove(device, HD44780_CURSOR_MOVE_LEFT, 1);					
				}
				else if (three_dig_float <= -10)
				{
					HD44780_Set_Line(device, line, offset);
					device->user_write_data('-');
					HD44780_CursorMove(device, HD44780_CURSOR_MOVE_LEFT, 1);
				}
			}			
			float_to_char /= 10;
		}
		else {
			HD44780_Set_Line(device, line, offset + i);
			device->user_write_data('.');
		}		
	}
}

/*
 * @brief  Sends time value to the screen (from RTC etc.).
 *
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param hour : value of hours.
 * @param min : value of minutes.
 * @param sec : value of seconds.
 * @param delometer : if it is zero delimeter does not appears, if 1 this one appears.
 *
 **/
void HD44780_SendTime(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t hour, uint8_t min, uint8_t sec, uint8_t delimeter)
{	
		
	HD44780_Set_Line(device, line, offset);	
	
	if (hour < 10) {
		/* Clear leading zero of hours. */
		device->user_write_data(' ');		
	} else {
		device->user_write_data((hour / 10) + 48);  
	}	
	device->user_write_data((hour % 10) + 48);  	
	if (!delimeter) {
		HD44780_Set_Line(device, line, offset + 3);	
	} else {
		device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[':']);  
	}	
	device->user_write_data((min / 10) + 48);  
	device->user_write_data((min % 10) + 48);  	
	if (!delimeter) {
		HD44780_Set_Line(device, line, offset + 6);	
	} else {
		device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[':']);  
	}	 	
	device->user_write_data((sec / 10) + 48);  
	device->user_write_data((sec % 10) + 48);  

}	

/*
 * @brief Sends date value to the screen (from RTC etc.).
 *
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param date : value of date.
 * @param month : number of month.  
 * @param year : value of year.
 * @param year_prefix : if it is zero the year dislayed like a xx (21th year and etc), else when you insert 20? the year displayed like a 20xx 
 * (2021th year and etc.).
 *
 **/
void HD44780_SendDate(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t date, uint8_t month, uint8_t year, uint8_t year_prefix, uint8_t delimeter)
{	
	HD44780_Set_Line(device, line, offset);	
	
	device->user_write_data((date / 10) + 48);  
	device->user_write_data((date % 10) + 48);  
	if (!delimeter) {
		HD44780_Set_Line(device, line, offset + 3);	
	}
	else {
		device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[':']);  
	}	
	device->user_write_data((month / 10) + 48);  
	device->user_write_data((month % 10) + 48);  
	if (!delimeter) {
		HD44780_Set_Line(device, line, offset + 6);	
	}
	else {
		device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[':']);  
	} 
	if (year_prefix == 0) {
		device->user_write_data((year / 10) + 48);  
		device->user_write_data((year % 10) + 48);  
	} else {		
		device->user_write_data((year_prefix / 10) + 48);  
		device->user_write_data((year_prefix % 10) + 48); 
		device->user_write_data((year / 10) + 48);  
		device->user_write_data((year % 10) + 48); 
	}	

}	

/*
 * @brief Sends temperature value to the screen (three dogits, up to +-99.9).
 *
 * @param device : instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param temp : it is temperature from some sensor.
 *
 **/
void HD44780_SendTemperature(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float temp)
{	
	int16_t int_temperature = (int16_t)(temp * 10);	
	uint16_t uint_temperature = 0;
		
	if (int_temperature >= 1000) {
		int_temperature = 999;
	}
	
	if (int_temperature >= 0) {
		uint_temperature = (uint16_t)(int_temperature);
	} else if (int_temperature < 0) {
		uint_temperature = (uint16_t)(-int_temperature);
	}
	
	HD44780_Set_Line(device, line, offset);
	
	/*Convetion digit to string.*/
	device->user_write_data((uint_temperature / 100) + 48);
	device->user_write_data(((uint_temperature / 10) % 10) + 48);
	device->user_write_data('.');
	device->user_write_data((uint_temperature % 10) + 48);

	/*writes symbols of '°' and 'C'*/
	device->user_write_data(1); // 0xDF
	device->user_write_data('C');
	
	/**/
	if (int_temperature >= 100) {
		HD44780_Set_Line(device, line, offset - 1);
		device->user_write_data(' ');
	} else if (int_temperature >= 0 && int_temperature < 100) {
		HD44780_Set_Line(device, line, offset);
		device->user_write_data(' ');		
	} else if (int_temperature < 0 && int_temperature > -100) {
		HD44780_Set_Line(device, line, offset);
		device->user_write_data('-');		
	} else if (int_temperature <= -100)	{
		HD44780_Set_Line(device, line, offset - 1);
		device->user_write_data('-');
	}
	__NOP();
}	

/*
 * @brief Sends humidity value to the screen (three dogits, up to 99.9).
 * 
 * @param device : instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param hum : humidity from some sensor.
 *
 **/
void HD44780_SendHumidity(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float hum)
{	
	uint16_t humidity = (uint16_t)(hum * 10);	

	if (hum == 100) {
		humidity = 999;
	}
	
	HD44780_Set_Line(device, line, offset);
	
//	device->user_write_data('R');
//	device->user_write_data('h');
//	device->user_write_data('=');
	
	/*Convetion digit to string.*/
	device->user_write_data((humidity / 100) + 48);
	device->user_write_data(((humidity / 10) % 10) + 48);
	device->user_write_data('.');
	device->user_write_data((humidity % 10) + 48);

	/* Writes symbol '%' */
	device->user_write_data('%');
}	

/*
 * @brief Sends pressure value to the screen (three dogits, up to 999.9).
 * 
 * @param device : instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param press : pressure from some sensor.
 *
 **/
void HD44780_SendPressure(HD44780_Inst_t* device, uint8_t line, uint8_t offset, float press)
{	
	uint16_t pressure = (uint16_t)(press * 10);	
	
	HD44780_Set_Line(device, line, offset);
	
	/*Convetion digit to string.*/
	device->user_write_data((pressure / 1000) + 48);
	device->user_write_data(((pressure / 100) % 10) + 48);
	device->user_write_data(((pressure / 10) % 10) + 48);
	device->user_write_data('.');
	device->user_write_data((pressure % 10) + 48);

	/* Writes symbol '%' */
	device->user_write_data('m');
	device->user_write_data('m');
	device->user_write_data('H');
	device->user_write_data('g');
}

/*
 * @brief Decods and dislays day number to the weekday name.
 * 
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param day : number of day (1 to 7).
 * @param language : 0 is EN, 1 is RU.
 *
 **/
void HD44780_SendDay(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t day, HD44780_CalendarLanguage language)
{
	static uint8_t symbol = 0;
	
	HD44780_Set_Line(device, line, offset);
	
	if (!language)
	{		
		switch (day) {
		
		case MO: 
			device->user_write_data('M');
			device->user_write_data('o');
			break;
		
		case TU: 
			device->user_write_data('T');
			device->user_write_data('u');
			break;
		
		case WE: 
			device->user_write_data('W');
			device->user_write_data('e');
			break;
		
		case TH: 
			device->user_write_data('T');
			device->user_write_data('h');
			break;
		
		case FR: 
			device->user_write_data('F');
			device->user_write_data('r');
			break;
		
		case SA: 
			device->user_write_data('S');
			device->user_write_data('a');
			break;
		
		case SU: 
			device->user_write_data('S');
			device->user_write_data('u');
			break;
		
		default: break;
		
		}
	}
	else {	
	
		switch (day) {
		
		case MO: 
			device->user_write_data(0xA8);
			device->user_write_data(0xBD);
			break;
		
		case TU: 
			device->user_write_data(0x42);
			device->user_write_data(0xBF);
			break;
		
		case WE: 
			device->user_write_data(0x43);
			device->user_write_data(0x70);
			break;
		
		case TH: 
			device->user_write_data(0xAB);
			device->user_write_data(0xBF);
			break;
		
		case FR: 
			device->user_write_data(0xA8);
			device->user_write_data(0xBF);
			break;
		
		case SA: 
			device->user_write_data(0x43);
			device->user_write_data(0xB2);
			break;
		
		case SU: 
			device->user_write_data(0x42);
			device->user_write_data(0x63);
			break;
		
		default: break;
		
		}
	}	
}

/*
 * @brief Decods and dislays name of the month.
 * 
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 * @param day : number of month (1 to 12).
 * @param language : 0 is EN, 1 is RU.
 *
 **/
void HD44780_SendMonth(HD44780_Inst_t* device, uint8_t line, uint8_t offset, uint8_t month, HD44780_CalendarLanguage language)
{
	HD44780_Set_Line(device, line, offset);
	
	if (!language) {
	
		switch (month) {
		
		case JAN: 
			device->user_write_data('J');
			device->user_write_data('a');
			device->user_write_data('n');
			break;
		
		case FEB: 
			device->user_write_data('F');
			device->user_write_data('e');
			device->user_write_data('b');
			break;
		
		case MAR: 
			device->user_write_data('M');
			device->user_write_data('a');
			device->user_write_data('r');
			break;
		
		case APR: 
			device->user_write_data('A');
			device->user_write_data('p');
			device->user_write_data('r');
			break;
		
		case MAY: 
			device->user_write_data('M');
			device->user_write_data('a');
			device->user_write_data('y');
			break;
		
		case JUN: 
			device->user_write_data('J');
			device->user_write_data('u');
			device->user_write_data('n');
			break;
		
		case JUL: 
			device->user_write_data('J');
			device->user_write_data('u');
			device->user_write_data('l');
			break;
		
		case AUG: 
			device->user_write_data('A');
			device->user_write_data('u');
			device->user_write_data('g');
			break;
		
		case SEP: 
			device->user_write_data('S');
			device->user_write_data('e');
			device->user_write_data('p');
			break;
		
		case OCT: 
			device->user_write_data('O');
			device->user_write_data('c');
			device->user_write_data('t');
			break;
		
		case NOV: 
			device->user_write_data('N');
			device->user_write_data('o');
			device->user_write_data('v');
			break;
		
		case DEC: 
			device->user_write_data('D');
			device->user_write_data('e');
			device->user_write_data('c');
			break;
		
		default: break;		
		}
		
	} else {
		
		switch (month) {
		
		case JAN: 
			device->user_write_data(0xB1);
			device->user_write_data(0xBD);
			device->user_write_data(0xB3);
			break;
		
		case FEB: 
			device->user_write_data(0xAA);
			device->user_write_data(0x65);
			device->user_write_data(0xB3);
			break;
		
		case MAR: 
			device->user_write_data(0x4D);
			device->user_write_data(0x61);
			device->user_write_data(0x70);
			break;
		
		case APR: 
			device->user_write_data(0x41);
			device->user_write_data(0xBE);
			device->user_write_data(0x70);
			break;
		
		case MAY: 
			device->user_write_data(0x4D);
			device->user_write_data(0x61);
			device->user_write_data(0xB9);
			break;
		
		case JUN: 
			device->user_write_data(0xA5);
			device->user_write_data(0xC6);
			device->user_write_data(0xBD);
			break;
		
		case JUL: 
			device->user_write_data(0xA5);
			device->user_write_data(0xC6);
			device->user_write_data(0xBB);
			break;
		
		case AUG: 
			device->user_write_data(0x41);
			device->user_write_data(0xB3);
			device->user_write_data(0xB4);
			break;
		
		case SEP: 
			device->user_write_data(0x43);
			device->user_write_data(0x65);
			device->user_write_data(0xBD);
			break;
		
		case OCT: 
			device->user_write_data(0x4F);
			device->user_write_data(0xBA);
			device->user_write_data(0xBF);
			break;
		
		case NOV: 
			device->user_write_data(0x48);
			device->user_write_data(0x6F);
			device->user_write_data(0xC7);
			break;
		
		case DEC: 
			device->user_write_data(0xE0);
			device->user_write_data(0x65);
			device->user_write_data(0xBA);
			break;
		
		default: break;		
		}
	}
}

/*
 * @brief screen test function.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param number_of_lines : number of lines on the screen.
 * @param number_of_symbols : number of symbols on the line of the screen.
 *
 **/
void HD44780_TestScreen(HD44780_Inst_t* device, uint8_t number_of_lines, uint8_t number_of_symbols)
{			
	uint16_t iterator = 0;
	
	while (1)
	{
		HD44780_Set_Line(device, 1, 0);							 																			
		for (uint8_t i = 1; i <= number_of_symbols; i++)							
		{														
			device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[iterator++]);   							
			device->user_delay(50);  		
			if (iterator == 256) {							
				iterator = 0;  
				break;
			}	
		}
			
		if (number_of_lines != 1)
		{
			HD44780_Set_Line(device, 2, 0);						
			for (uint8_t i = 1; i <= number_of_symbols; i++)
							
			{							
				device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[iterator++]);  							
				device->user_delay(50);   
				if (iterator == 256) {							
					iterator = 0;  
					break;
				}	
			}
		}		
		
		if (number_of_lines == 4)
		{
			HD44780_Set_Line(device, 3, 0);							 																			
			for (uint8_t i = 1; i <= number_of_symbols; i++)							
			{														
				device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[iterator++]);   							
				device->user_delay(50);  
				if (iterator == 256) {							
					iterator = 0;  
					break;
				}	
			}
						
			HD44780_Set_Line(device, 4, 0);						
			for (uint8_t i = 1; i <= number_of_symbols; i++)							
			{							
				device->user_write_data(HD44780_SCREEN_EN_RUS_ASCI_WIN1251_v1[iterator++]);  							
				device->user_delay(50);
				if (iterator == 256) {							
					iterator = 0;  
					break;
				}	
			}
		}
	}						
}



/* Private functions. */

/*
 * @brief
 * 
 * @param device : instance of the HD44780_Inst_t struct which has needed transport functions.
 * @param line : it's number of line on the screen.
 * @param offset : it's position offset on the screen line.
 *
 **/
static void HD44780_Set_Line(HD44780_Inst_t* device, const uint8_t line, uint8_t offset)
{				
	if (line == 1) {						
		device->user_write_command(HD44780_CMD_SET_LINE1 + offset);       						
	} else if (line == 2) {						
		device->user_write_command(HD44780_CMD_SET_LINE2 + offset);      						
	} else if (line == 3) {						
		device->user_write_command(HD44780_CMD_SET_LINE3 + offset);      						
	} else if (line == 4) {						
		device->user_write_command(HD44780_CMD_SET_LINE4 + offset);      						
	}				
}

/*
 * @brief Writes your own eight symbols from massive HD44780_RamChar.
 * 
 * @param device : Instance of the HD44780_Inst_t struct which has needed transport functions.
 *
 **/	
static void HD44780_WriteRAM(HD44780_Inst_t* device)
{			
	device->user_write_command(HD44780_CMD_SET_CGRAM_ADDR);   						  									  
														
	for (uint8_t i = 0; i < sizeof(HD44780_RamChar); i++) {								
		device->user_write_data(HD44780_RamChar[i]);							 												
	}								
	device->user_write_command(HD44780_CMD_SET_LINE1);  			  							  	
}

/* Hardware depending function. */

/*
 * @brief Command transfer to screen.
 * 
 **/
void HD44780_WriteCommand(const uint8_t command)
{
	/*Set command lines. */
	HD44780_RS_PORT->BSRR = HD44780_RS_INSTR_REG;        
	HD44780_RW_PORT->BSRR = HD44780_RW_WRITE;  
	
	HD44780_WriteByte(command);
	HD44780_CheckBusy();
}	
			
/*
 * @brief Data transfer to screen.
 *
 **/
void HD44780_WriteData(uint8_t data)
{
	/*Set command lines. */
	HD44780_RS_PORT->BSRR = HD44780_RS_DATA_REG;        
	HD44780_RW_PORT->BSRR = HD44780_RW_WRITE;      	   			 		
				
	HD44780_WriteByte(data);
	HD44780_CheckBusy();
}
	
/*
 * @brief Read/write functions for mixed data bus.
 *
 **/

/*
 * @brief Make the set of data pins.
 * @param byte : 
 *
 **/
static void HD44780_WriteByte(const uint8_t byte)
{
	if (bus_lenght == HD44780_BUS_TYPE_8BITS) {		
		
		/* Reset data bits on the bus. */
		HD44780_LEAST4B_DATA_PORT->BSRR = 0xC0030000;
		HD44780_MOST4B_DATA_PORT->BSRR = 0x07800000;
		
		/* Write data to the bus. */
		HD44780_LEAST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0001) << 14;
		HD44780_LEAST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0002) << 14;
		HD44780_LEAST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0004) >> 2;
		HD44780_LEAST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0008) >> 2;
		HD44780_MOST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0010) << 3;
		HD44780_MOST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0020) << 3;
		HD44780_MOST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0040) << 3;
		HD44780_MOST4B_DATA_PORT->BSRR = ((uint16_t)(byte) & 0x0080) << 3;
		
		/* Strob data. */
		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();


	}
	else if (bus_lenght == HD44780_BUS_TYPE_4BITS) {
	
		/* Reset data bits on the bus. */
		HD44780_MOST4B_DATA_PORT->BSRR = 0x07800000;
		/* Write data to the bus. */
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0010) << 3;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0020) << 3;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0040) << 3;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0080) << 3;
		
		/* Strob data. */
		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

		/* Reset data bits on the bus. */
		HD44780_MOST4B_DATA_PORT->BSRR = 0x07800000;
		/* Write data to the bus. */
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0001) << 7;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0002) << 7;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0004) << 7;
		GPIOE->BSRR = ((uint16_t)(byte) & 0x0008) << 7;
		
		/* Strob data. */
		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();

	}
}

void HD44780_CheckBusy(void) 
{	
	uint32_t cntTimeOut = 1000;

	/*Set command lines. */
	HD44780_RS_PORT->BSRR = HD44780_RS_INSTR_REG;
	HD44780_RW_PORT->BSRR = HD44780_RW_READ;
	
	/*Sets the input mode for 7th bit.*/
	HD44780_BUSY_PIN_PORT->CRH &= ~(GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0);
	HD44780_BUSY_PIN_PORT->CRH &= ~(GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0);
	HD44780_BUSY_PIN_PORT->CRH |= GPIO_CRH_CNF10_1;
	/*Pull down this pin.*/
	HD44780_BUSY_PIN_PORT->BSRR = HD44780_BUSY_PIN << 16;

	/*Wait bus will be open.*/
	while (HD44780_BUSY_PIN_PORT->IDR & HD44780_BUSY_PIN)
	{
		cntTimeOut--;
		if (!cntTimeOut)
		{
			break;
		}
	}	
	
	/*Sets to output mode for 7th bit.*/
	HD44780_BUSY_PIN_PORT->CRH |= GPIO_CRH_MODE10_1;
	HD44780_BUSY_PIN_PORT->CRH &= ~(GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0);	
}



/*
 * @brief Check busy flag.
 *
 **/
//void HD44780_CheckBusy(void) 
//{
//	uint32_t cntTimeOut = 1000;
//	/*Set command lines. */
//	HD44780_RS_PORT->BSRR = HD44780_RS_INSTR_REG;
//	HD44780_RW_PORT->BSRR = HD44780_RW_READ;
//	
//	/*Sets to input mode for 7th bit.*/
//	HD44780_DATA_PORT->CRL &= ~(GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0);
//	HD44780_DATA_PORT->CRL &= ~(GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0);
//	HD44780_DATA_PORT->CRL |= GPIO_CRL_CNF7_1;
//	/*Pull down this pin.*/
//	HD44780_DATA_PORT->BSRR = GPIO_PIN_7 << 16;
//	
//	/*Wait bus will be open.*/
//	while (HD44780_DATA_PORT->IDR & 0x80)
//	{
//		cntTimeOut--;
//		if (!cntTimeOut)
//		{
//			break;
//		}
//	}	
//	
//	/*Sets to output mode for 7th bit.*/
//	HD44780_DATA_PORT->CRL |= GPIO_CRL_MODE7_1;
//	HD44780_DATA_PORT->CRL &= ~(GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0);	
//}

/*
 * @brief Write byte to the bus. Bus is eight or four bits. Note, for 4-bit bus it use from 4 to 7 bits on the bus.
 * For bits 0 to 3 masks must be 0x000F0000, (data & 0xF0) >> 4 and 0x000F0000, data & 0x0F. For 8-bit bus it use 
 * from 0 to 7 bits on the bus.
 *
 **/
//static void HD44780_WriteByte(const uint8_t byte)
//{
//	if (bus_lenght == HD44780_BUS_TYPE_8BITS) {
//		
//		/*Reset least 8 bits.*/
//		HD44780_DATA_PORT->BSRR = 0x00FF0000; 
//		/*Write least 8 bits data.*/
//		HD44780_DATA_PORT->BSRR = byte;
//		
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		
//	}
//	else if (bus_lenght == HD44780_BUS_TYPE_4BITS) {
//		
//		/*Reset most 4 bits.*/
//		HD44780_DATA_PORT->BSRR = 0x00F00000; 
//		/*Write most 4 bit datas.*/
//		HD44780_DATA_PORT->BSRR = byte & 0xF0;
//		
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		
//		/*Reset least 4 bits.*/
//		HD44780_DATA_PORT->BSRR = 0x00F00000; 
//		/*Write least 4 bits data.*/
//		HD44780_DATA_PORT->BSRR = (byte & 0x0F) << 4;
//		
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_UP;   											         	
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		HD44780_E_PORT->BSRR  = HD44780_E_CLOCK_DOWN;   		
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); 
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
//
//	}	
//}



