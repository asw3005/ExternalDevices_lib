/*
 * @brief Common header for brt38.
 * Created 04.29.26 by asw3005. 
 *
 **/

#ifndef BRT38_H_
#define BRT38_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* GPIO configuration. */
#define START_PIN			    GPIO_PIN_5
#define START_GPIO_Port			GPIOA
#define ZERO_PIN			    GPIO_PIN_0
#define ZERO_GPIO_Port			GPIOB
#define MID_PIN					GPIO_PIN_0
#define MID_GPIO_Port			GPIOB



/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*brt38_delay_fptr)(uint32_t delay);

/*
 * @brief BRT38 typedef.
 *
 **/
 typedef union __attribute__((aligned(1), packed)){

    uint16_t Data;

    struct {
        uint8_t Data_MSB;
        uint8_t Data_LSB;
    };

    /* 12-bit, 16-turns */
    struct {
        uint8_t SINGLE11_8  : 4;
        uint8_t TURNS       : 4;
        uint8_t SINGLE7_0   : 8;
    };

 } BRT38_Data_t;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {


} BRT38_GInst_t;


/* Public function prototypes. */
uint32_t BRT38_GetRawPosition(void);
BRT38_Data_t BRT38_GetRawData(void);

#endif /* BRT38_H_ */








