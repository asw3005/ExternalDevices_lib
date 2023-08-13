/*
 * @brief 
 * Created 03.14.22 by asw3005. 
 *
 **/



#include "stm32f4xx.h"
#include "tlc592x.h"

/* Numbers for the 7-seg led indicator. */
static const uint8_t TLC592x_Dig0[11] = {
	/* 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, . */
	0xED, 0x28, 0xCE, 0xAE, 0x2B,
	0xA7, 0xE7, 0x2C, 0xEF, 0xAF,
	0x10
};

static const uint8_t TLC592x_Dig1[11] = {
	/* 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, . */
	0xFD , 0x41, 0xDC, 0xD5, 0x71, 
	0xB5, 0xBD, 0xC1, 0xFD, 0xF5,
	0x02
};



/*
 * @brief Disable display.
 *
 **/
void TLC592x_Clr_Display(TLC592x_GInst_t *device) {
	
	/* Check out is dot here for first number. */
	if ((device->TxData.Data[0] & TLC592x_Dig0[10])) {
		device->TxData.Data[0] = 0x00 | TLC592x_Dig0[10];
	}
	else {
		device->TxData.Data[0] = 0x00;
	}
	/* Check out is dot here for second number. */
	if ((device->TxData.Data[1] & TLC592x_Dig1[10])) {
		device->TxData.Data[1] = 0x00 | TLC592x_Dig1[10];
	}
	else {
		device->TxData.Data[1] = 0x00;
	}
	
	ESPI_LatchDataEn(GPIOG);
	ESPI_TxData(GPIOG, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOG);
}

/*
 * @brief Transmit device address.
 *
 **/
void TLC592x_Set_Address(TLC592x_GInst_t *device, uint8_t address) {
	
	/* Check out is dot here for first number. */
	if ((device->TxData.Data[0] & TLC592x_Dig0[10])) {
		device->TxData.Data[0] = TLC592x_Dig0[address % 10] | TLC592x_Dig0[10];
	} else {
		device->TxData.Data[0] = TLC592x_Dig0[address % 10];
	}
	
	/* Check out is dot here for second number. */
#ifndef ZERO_LEADING_DELETE
	if ((device->TxData.Data[1] & TLC592x_Dig1[10])) {
		device->TxData.Data[1] = TLC592x_Dig1[address / 10] | TLC592x_Dig1[10];
	}
	else {
		device->TxData.Data[1] = TLC592x_Dig0[address / 10];
	}
#else	
	if((device->TxData.Data[1] & TLC592x_Dig1[10])) {		
		if (!(address / 10)) {
		device->TxData.Data[1]  = 0x00 | TLC592x_Dig1[10];
		} else {
		device->TxData.Data[1] = TLC592x_Dig1[address / 10] | TLC592x_Dig1[10];
		}			
	} else {		
		if (!(address / 10)) {
		device->TxData.Data[1] = 0;
		} else {
		device->TxData.Data[1] = TLC592x_Dig1[address / 10];
		}
	}
#endif
	/* Trensmit data. */
	ESPI_LatchDataEn(GPIOG);
	ESPI_TxData(GPIOG, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOG);
}

/*
 * @brief Transmit device address.
 *
 **/
void TLC592x_Set_Dot0(TLC592x_GInst_t *device, uint8_t state) {
	
	/* Dot 0 management. */
	if (state) {
		device->TxData.Data[0] |= TLC592x_Dig0[10];
	} else {
		device->TxData.Data[0] &= ~TLC592x_Dig0[10];
	}
	
	/* Send data to LED. */
	ESPI_LatchDataEn(GPIOG);
	ESPI_TxData(GPIOG, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOG);
}

/*
 * @brief Transmit device address.
 *
 **/
void TLC592x_Set_Dot1(TLC592x_GInst_t *device, uint8_t state) {
	
	/* Dot 1 management. */
	if (state) {
		device->TxData.Data[1] |= TLC592x_Dig1[10];
	} else {
		device->TxData.Data[1] &= ~TLC592x_Dig1[10];
	}		
	
	/* Send data to LED. */
	ESPI_LatchDataEn(GPIOG);
	ESPI_TxData(GPIOG, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOG);
}

/* Hardware dependent functions. */

/*
 * @brief Enable latching data from register to output.
 *
 **/
void ESPI_LatchDataEn(GPIO_TypeDef* gpio) {
	
	/* Pull up the latch. */
	gpio->BSRR = GPIO_BSRR_BS11;
}

/*
 * @brief Enable latching data from register to output.
 *
 **/
void ESPI_LatchDataDis(GPIO_TypeDef* gpio) {
	
	/* Pull down the latch. */
	gpio->BSRR = GPIO_BSRR_BR11;	
}

/*
 * @brief Clocking data.
 *
 **/
void ESPI_ClockData(GPIO_TypeDef* gpio) {	
	
	/* Pull up the clock. */
	gpio->BSRR = GPIO_BSRR_BS13;
	/* Pull down the clock. */
	gpio->BSRR = GPIO_BSRR_BR13;	
}

void ESPI_TxData(GPIO_TypeDef* gpio, uint8_t *pData, uint8_t Size) {
	
	uint16_t TxData;
	
	/* Transmitting the data to the shift register. */
	pData++;
	for (uint8_t i = Size; i > 0; i--) {	
		TxData = *pData;		
		for (uint8_t j = 8; j > 0; j--) {	
			/* MSB first. */
			gpio->BSRR = (TxData & 0x80) ? GPIO_BSRR_BS14 : GPIO_BSRR_BR14;
			/* Clocking data. */
			ESPI_ClockData(gpio);		
			/* Getting next bit.*/
			TxData <<= 1;
		}
		pData--;		
	}
}







