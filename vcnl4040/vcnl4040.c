/*
 * @brief Common source for VCNL4040.
 * Created 03.31.26 by asw3005. 
 *
 **/
#include "vcnl4040.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

 /* External variables. */

 

/* Private variables. */


/* Private function prototypes. */



/* General struct. */


/*
 * @brief 
 *
**/















 /* Hardware dependent functions. */

/*
 * @brief UART Tx data.
 *
 **/
static void VCNL4040_Tx(uint8_t *pData, uint8_t size) {

	//HAL_UART_Transmit(LdUart, pData, size, 10);
}

/*
 * @brief UART Rx data.
 * 
 *
 **/
static void VCNL4040_Rx(uint8_t *pData, uint8_t size) {
	
	//HAL_UART_Receive(LdUart, pData, size, 10);
}

