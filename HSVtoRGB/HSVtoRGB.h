/**
 **\file HSVtoRGB.h
 **\brief Convertion HSV color model to RGB, heder file.
 **
***/


#pragma once

#include "stm32f1xx_hal.h"

///HSV struct
typedef struct 
{
	uint16_t hue;
	uint8_t saturation;
	uint8_t value;
	
} HSV_DATA_Typedef;

///RGB struct
typedef struct 
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	
} RGB_DATA_Typedef;

///Public function prototype
RGB_DATA_Typedef HSVtoRGB(uint16_t hue, float saturation, float value);///Function convertion HSV color model to RGB