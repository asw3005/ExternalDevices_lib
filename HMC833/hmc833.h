/*
 * hmc833.h header file.
 *
 * Created on: Sep 7, 2023
 * Author: asw3005
 */

#ifndef HMC833_H_
#define HMC833_H_

#include "stm32f1xx.h"

/* Pin definitions serial port. */
#define HMC833_CS_PIN 		GPIO_PIN_0
#define HMC833_CS_PORT 		GPIOA



/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t* pData, uint8_t Size);

/*
 * @brief General struct.
 */
typedef struct {

	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr spi_rx_fp;
	rxtx_fptr spi_tx_fp;

} HMC833_GStr_t;

/* Public function prototypes. */


#endif /* HMC833_H_ */
