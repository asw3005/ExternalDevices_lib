/*
 * @brief Common header for TMC262C.
 * Created 02.12.26 by asw3005. 
 *
 **/

#ifndef TMC262C_H_
#define TMC262C_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* GPIO configuration. */
#define TMC262C_EN_Pin 			GPIO_PIN_0
#define TMC262C_EN_GPIO_Port	GPIOC
#define TMC262C_DIR_Pin 		GPIO_PIN_1
#define TMC262C_DIR_GPIO_Port	GPIOC
#define TMC262C_STEP_Pin 		GPIO_PIN_2
#define TMC262C_STEP_GPIO_Port	GPIOC

#define TMC262C_CS_Pin 			GPIO_PIN_15
#define TMC262C_CS_GPIO_Port	GPIOA
#define TMC262C_SCK_Pin 		GPIO_PIN_10
#define TMC262C_SCK_GPIO_Port	GPIOC
#define TMC262C_MISO_Pin 		GPIO_PIN_11
#define TMC262C_MISO_GPIO_Port	GPIOC
#define TMC262C_MOSI_Pin 		GPIO_PIN_12
#define TMC262C_MOSI_GPIO_Port	GPIOC

/*
 * @brief 
 *
 **/

/*
 * @brief Register map.
 *
 **/
typedef enum {
	
	TMC262C_DRVCTRL,
	TMC262C_CHOPCONF = 0x04,
	TMC262C_SMARTEN,
	TMC262C_SFCSCONF,
	TMC262C_DRVCONF
	
} TMC262C_REG_MAP_t;

/*
 * @brief Pins.
 *
 **/
typedef enum {
	
	TMC262C_PIN
	
} TMC262C_Pins;





/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	delay_fptr delay;
	spi_txrx_fptr spi_tx;
	spi_txrx_fptr spi_rx;

} TMC262C_GInst_t;


/* Public function prototypes. */


#endif /* TMC262C_H_ */








