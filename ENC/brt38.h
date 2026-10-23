/*
 * @brief Common header for brt38.
 * Created 04.29.26 by asw3005. 
 *
 **/

#ifndef BRT38_H_
#define BRT38_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* GPIO configuration. */
#define START_PIN			    GPIO_PIN_5
#define START_GPIO_Port			GPIOA
#define ZERO_PIN			    GPIO_PIN_7
#define ZERO_GPIO_Port			GPIOA
#define MID_PIN					GPIO_PIN_4
#define MID_GPIO_Port			GPIOA



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
  * @brief Encoder data struct typedef.
  */
 typedef struct __attribute__((aligned(1), packed)) {

	/* Static data. */
	const uint8_t FactoryTurns;
	const uint16_t FactoryResolution;

	/* Dynamic data. */
	uint16_t Single;
	uint16_t Turns;
	uint32_t AbsPosition;
	uint32_t LimMin;
	uint32_t LimMax;
	uint32_t FullDistance;
	uint32_t KeepAliveDelta;
	uint32_t DeltaPos;
	uint32_t PreviousPos;
	uint32_t ForwardPos;
	uint32_t BackwardPos;

	/* 0 - zero home, 1 - middle home. */
	uint8_t HomePos;

	/* Read back. */
	BRT38_Data_t EncReadBack;

 } ENCData_t;

/*
 * @brief Encode instances.
 */
 typedef struct {

		ENCData_t EncInst0;

 } ENCInst_t;


/* Public function prototypes. */
void BRT38_SetSero(void);
void BRT38_SetMiddle(void);
ENCInst_t* EncoderGetInst(void);
void BRT38_ReadPosition(ENCData_t* enc_inst);

#ifdef __cplusplus
}
#endif

#endif /* BRT38_H_ */








