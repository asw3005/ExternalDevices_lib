/*
 *	@brief VFD48_1202F driver.
 *	Created 04.24.21
 *	VFD48_1202F_H_
 *	
 **/

#ifndef VFD48_1202F_H_				  
#define VFD48_1202F_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL








/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit or receive data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef uint8_t(*vfd48_txrx_data_fptr)(uint8_t *buffer, uint16_t size);






/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointers for the rx, tx and delay functions.
	delay_fptr delay_fptr;
	vfd48_txrx_data_fptr spi_tx_data_fptr;
	vfd48_txrx_data_fptr spi_rx_data_fptr;
	
} VFD48_GInst_t;


/* Public function prototypes. */


/* Hardware dependent functions. */


#endif /* VFD48_1202F_H_ */