/*
 * @brief Common header for VCNL4040.
 * Created 03.31.26 by asw3005. 
 *
 **/

#ifndef VCNL4040_H_
#define VCNL4040_H_

#include <stdint.h>
#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"


/* GPIO configuration. */


/*
 * @brief Register's map.
 *
 **/
typedef enum {

  VCNL4040_ALS_CONF,
  VCNL4040_ALS_THDH,
  VCNL4040_ALS_THDL,
  VCNL4040_PS_CONF2_1,
  VCNL4040_PS_CONF3_MS,
  VCNL4040_PS_CANC,
  VCNL4040_PS_THDL,
  VCNL4040_PS_THDH,
  VCNL4040_PS_DATA,
  VCNL4040_ALS_DATA,
  VCNL4040_WHITE_DATA,
  VCNL4040_INT_FLAG,
  VCNL4040_ID  

} VCNL4040_REG_MAP_t;

/*
 * @brief Universal register type.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    uint16_t UniReg;
     
    struct {
        uint8_t UniReg_LSB;
        uint8_t uniReg_MSB;
    };

} VCNL4040_UniReg_t;

/*
 * @brief ALS_CONF.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    uint16_t AlsConf;

    struct { 
        uint8_t AlsConf_LSB;
        uint8_t AlsConf_MSB;
    };

    struct {
        uint8_t ALS_SD          : 1;
        uint8_t ALS_INT_EN      : 1;
        uint8_t ALS_PERS        : 2;
        uint8_t RESERVED5_4     : 2;
        uint8_t ALS_IT          : 2;
        uint8_t RESERVED15_8    : 8;
    };

} VCNL4040_AlsConf_t;

/*
 * @brief PS_CONF2_1.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    uint16_t PsConf21;

    struct { 
        uint8_t PsConf21_LSB;
        uint8_t PsConf21_MSB;
    };

    struct {
        uint8_t PS_SD           : 1;
        uint8_t PS_IT           : 3;
        uint8_t PS_PERS         : 2;
        uint8_t PS_DUTY         : 2;

        uint8_t PS_INT          : 2;
        uint8_t RESERVED2       : 1;
        uint8_t PS_HD           : 1;
        uint8_t RESERVED16_13   : 4;
    };

} VCNL4040_PsConf2_1_t;

/*
 * @brief PS_CONF3_MS.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    uint16_t PsConf3Ms;

    struct { 
        uint8_t PsConf3Ms_LSB;
        uint8_t PsConf3Ms_MSB;
    };

    struct {
        uint8_t PS_SC_EN        : 1;
        uint8_t RESERVED1       : 1;
        uint8_t PS_TRIG         : 1;
        uint8_t PS_AF           : 1;
        uint8_t PS_SMART_PERS   : 1;
        uint8_t PS_MPS          : 2;
        uint8_t RESERVED7       : 1;

        uint8_t LED_I           : 3;
        uint8_t RESERVED5_3     : 3;
        uint8_t PS_MS           : 1;
        uint8_t WHITE_EN        : 1;
    };

} VCNL4040_PsConf3Ms_t;

/*
 * @brief INT_FLAG readback type.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

    uint16_t IntFlag;

    struct { 
        uint8_t Reserved_LSB;
        uint8_t IntFlag_MSB;
    };

    struct {
        uint8_t RESERVED7_0     : 8;
        
        uint8_t PS_IF_AWAY      : 1;
        uint8_t PS_IF_CLOSE     : 1;
        uint8_t RESERVED12_11   : 2;
        uint8_t ALS_IF_H        : 1;
        uint8_t ALS_IF_L        : 1;
        uint8_t PS_SPFLAG       : 1;
        uint8_t RESERVED16      : 1;
    };

} VCNL4040_IntFlag_t;

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*i2c_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

    i2c_txrx_fptr i2c_tx;
    i2c_txrx_fptr i2c_rx;

} VCNL4040_GInst_t;


/* Public function prototypes. */








#endif /* VCNL4040_H_ */
