/**
 **\file HSVtoRGB.c
 **\brief Convertion HSV color model to RGB.
 **
***/

///Includes
#include "HSVtoRGB.h"

///Variable instance
RGB_DATA_Typedef rgb_data;

///This function converted HSV color model to RGB, not optimized
RGB_DATA_Typedef HSVtoRGB(uint16_t hue, float saturation, float value)
{
	if (hue > 360) 
	{		
		rgb_data.red = 0;
		rgb_data.green = 0;
		rgb_data.blue = 0;
		return rgb_data;
	}

	float R = 0, G = 0, B = 0;	
	float chroma = value * saturation;
	uint16_t hh = (hue / 60.0f) * 100;
	float hh_mod_2 =  ((hh % 200) - 100.0f) / 100;
	if (hh_mod_2 < 0) hh_mod_2 = -1 * hh_mod_2;
	float x = chroma * (1 - hh_mod_2);
	
	hh /= 100; ///< hh = H / 60
	switch (hh)
	{
		case 0:
			R = chroma;	G = x;	B = 0;	///<range 0 <= H < 60 degree
			break;
		case 1:
			R = x;	G = chroma;	B = 0;	///<range 60 <= H < 120 degree
			break;
		case 2:
			R = 0;	G = chroma;	B = x;	///<range 120 <= H < 180 degree
			break;		
		case 3:
			R = 0;	G = x;	B = chroma;	///<range 180 <= H < 240 degree
			break;		
		case 4:
			R = x;	G = 0;	B = chroma;	///<range 240 <= H < 300 degree
			break;		
		case 5:	
		default:
			R = chroma;	G = 0;	B = x;	///<range 300 <= H <= 360 degree
			break;
	}
	
	float m = value - chroma;
	rgb_data.red = (R + m) * 255;
	rgb_data.green = (G + m) * 255;
	rgb_data.blue = (B + m) * 255;	
	
	return rgb_data;
}