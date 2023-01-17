/*
 * @brief MBI5167 driver.
 * Created 09.03.21 by asw3005. 
 *
 **/

#include "stm32f103xb.h"
#include "mbi5167.h"

/* Numbers for the 7-seg led indicator. */
static const uint8_t MBI5167_Number[11] = {
		/* 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, . */
		0xEE, 0x28, 0xCD, 0x6D, 0x2B, 
		0x67, 0xE7, 0x2C, 0xEF, 0x6F,
		0x10
	};

/*
 * @brief Disable display.
 *
 **/
void MBI5167_Clr_Display(MBI5167_GInst_t *device) {
	
	/* Check out is dot here for first number. */
	if ((device->TxData.Data[0] & MBI5167_Number[10])) {
		device->TxData.Data[0] = 0x00 | MBI5167_Number[10];
	}
	else {
		device->TxData.Data[0] = 0x00;
	}
	/* Check out is dot here for second number. */
	if ((device->TxData.Data[1] & MBI5167_Number[10])) {
		device->TxData.Data[1] = 0x00 | MBI5167_Number[10];
	}
	else {
		device->TxData.Data[1] = 0x00;
	}
	
	ESPI_LatchDataEn(GPIOA);
	ESPI_TxData(GPIOA, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOA);	
}

/*
 * @brief Transmit device address.
 *
 **/
void MBI5167_Set_Address(MBI5167_GInst_t *device, uint8_t address) {
	
	/* Check out is dot here for first number. */
	if ((device->TxData.Data[0] & MBI5167_Number[10])) {
		device->TxData.Data[0] = MBI5167_Number[address % 10] | MBI5167_Number[10];
	} else {
		device->TxData.Data[0] = MBI5167_Number[address % 10];
	}
	
	/* Check out is dot here for second number. */
#ifndef ZERO_LEADING_DELETE
	if ((device->TxData.Data[1] & MBI5167_Number[10])) {
		device->TxData.Data[1] = MBI5167_Number[address / 10] | MBI5167_Number[10];
	}
	else {
		device->TxData.Data[1] = MBI5167_Number[address / 10];
	}
#else	
	if((device->TxData.Data[1] & MBI5167_Number[10])) {		
		if (!(address / 10)) {
		device->TxData.Data[1]  = 0x00 | MBI5167_Number[10];
		} else {
		device->TxData.Data[1] = MBI5167_Number[address / 10] | MBI5167_Number[10];
		}			
	} else {		
		if (!(address / 10)) {
		device->TxData.Data[1] = 0;
		} else {
		device->TxData.Data[1] = MBI5167_Number[address / 10];
		}
	}
#endif
	/* Trensmit data. */
	ESPI_LatchDataEn(GPIOA);
	ESPI_TxData(GPIOA, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOA);	
}

/*
 * @brief Transmit device address.
 *
 **/
void MBI5167_Set_Dot0(MBI5167_GInst_t *device, uint8_t state) {
	
	/* Dot 0 management. */
	if (state) {
		device->TxData.Data[0] |= MBI5167_Number[10];
	} else {
		device->TxData.Data[0] &= ~MBI5167_Number[10];
	}
	
	/* Send data to LED. */
	ESPI_LatchDataEn(GPIOA);
	ESPI_TxData(GPIOA, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOA);	
}

/*
 * @brief Transmit device address.
 *
 **/
void MBI5167_SET_Dot1(MBI5167_GInst_t *device, uint8_t state) {
	
	/* Dot 1 management. */
	if (state) {
		device->TxData.Data[1] |= MBI5167_Number[10];
	} else {
		device->TxData.Data[1] &= ~MBI5167_Number[10];
	}		
	
	/* Send data to LED. */
	ESPI_LatchDataEn(GPIOA);
	ESPI_TxData(GPIOA, device->TxData.Data, 2);
	ESPI_LatchDataDis(GPIOA);	
}

/* Hardware dependent functions. */

/*
 * @brief Enable latching data from register to output.
 *
 **/
void ESPI_LatchDataEn(GPIO_TypeDef* gpio) {
	
	/* Pull up the latch. */
	gpio->BSRR = GPIO_BSRR_BS0;
}

/*
 * @brief Enable latching data from register to output.
 *
 **/
void ESPI_LatchDataDis(GPIO_TypeDef* gpio) {
	
	/* Pull down the latch. */
	gpio->BSRR = GPIO_BSRR_BR0;	
}

/*
 * @brief Clocking data.
 *
 **/
void ESPI_ClockData(GPIO_TypeDef* gpio) {	
	
	/* Pull up the clock. */
	gpio->BSRR = GPIO_BSRR_BS1;
	/* Pull down the clock. */
	gpio->BSRR = GPIO_BSRR_BR1;	
}

void ESPI_TxData(GPIO_TypeDef* gpio, uint8_t *pData, uint8_t Size) {
	
	uint16_t TxData;
	
	/* Transmitting the data to the shift register. */
	for (uint8_t i = Size; i > 0; i--) {	
		TxData = *pData;		
		for (uint8_t j = 8; j > 0; j--) {	
			/* MSB first. */
			gpio->BSRR = (TxData & 0x80) ? GPIO_BSRR_BS2 : GPIO_BSRR_BR2;
			/* Clocking data. */
			ESPI_ClockData(gpio);		
			/* Getting next bit.*/
			TxData <<= 1;
		}
		pData++;		
	}
}







