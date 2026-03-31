/*
 * @brief Common header for U81X.
 * Created 03.24.26 by asw3005. 
 *
 **/

#ifndef U81X_H_
#define U81X_H_

#include <stdint.h>
#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

#define U81X_WRITE          0x00
#define U81X_READ           0x80
#define U81X_HEAD           0xAA
#define U81X_RCMD_DEF       0x05
#define U81X_WCMD_DEF       0x09
#define U81X_RCMD_MEAS      0x0D
#define U81X_RCMD_CHECKSUM  0x03
#define U81X_WCMD_CHECKSUM  0x07
#define U81X_STOP_CONT_MEAS 0x58 

/* GPIO configuration. */




/*
 * @brief Status codes.
 *
 **/
typedef enum {

    U81X_NO_ERROR,
    U81X_LOW_INPUT_POWER,
    /* Don't care. */
    U81X_INTERNAL_ERROR,
    U81X_LOW_MODULE_TEMP,
    U81X_HIGH_MODULE_TEMP,
    U81X_TARGET_OUT_OF_RANGE,
    U81X_INVALID_MEAS_RESULT,
    U81X_BACKGROUND_LIGHT_TOO_STRONG,
    U81X_LASER_SIGNAL_TOO_WEAK,
    U81X_LASER_SIGNAL_TOO_STRONG,
    U81X_HARDFAULT_1,
    U81X_HARDFAULT_2,
    U81X_HARDFAULT_3,
    U81X_HARDFAULT_4,
    U81X_HARDFAULT_5,
	U81X_LASER_SIGNAL_NOT_STABLE,
    U81X_HARDFAULT_6,
    U81X_HARDFAULT_7,
    U81X_INVALID_FRAME = 0x81

} U81X_STATUS_CODE_t;

/*
 * @brief Register's map.
 *
 **/
typedef enum {

    U81X_ERR_CODE   = 0x0000,
    U81X_BAT_VLTG   = 0x0006,
    U81X_ADDRESS    = 0x0010,
    U81X_OFFSET     = 0x0012,
    U81X_MEA_START  = 0x0020,
    U81X_MEA_RESULT = 0x0022,
    U81X_CTRL_LD    = 0x01BE,
    U81X_HWVERSION  = 0x000A,
    U81X_SWVERSION  = 0x000C,
    U81X_SNUMBER    = 0x000E

} U81X_REG_MAP_t;

/*
 * @brief Command frame format.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {

    /* Write command frame. */
    struct {
        uint8_t W_Head;
        uint8_t W_RW7_Addr6_0;
        uint8_t W_RegAddr_MSB;
        uint8_t W_RegAddr_LSB;
        uint8_t W_PayloadCnt_MSB;
        uint8_t W_PayloadCnt_LSB;
        uint8_t W_Payload_MSB;
        uint8_t W_Payload_LSB;
        uint8_t W_Checksum;
    };

    /* Read command frame. */
    struct {
        uint8_t R_Head;
        uint8_t R_RW7_Addr6_0;
        uint8_t R_RegAddr_MSB;
        uint8_t R_RegAddr_LSB;
        uint8_t R_Checksum;
        uint8_t R_Reserved_6;
        uint8_t R_Reserved_7;
        uint8_t R_Reserved_8;
        uint8_t R_Reserved_9;
    };

} U81X_RWCmdFrame_t;

/*
 * @brief Read back register format.
 *
 **/
 typedef struct __attribute__((aligned(1), packed)) {
     
    uint8_t Head;
    uint8_t RW7_Addr6_0;
    uint8_t RegAddr_MSB;
    uint8_t RegAddr_LSB;
    uint8_t PayloadCnt_MSB;
    uint8_t PayloadCnt_LSB;
    uint8_t Payload_MSB;
    uint8_t Payload_LSB;
    uint8_t Checksum;

} U81X_ReadBackReg_t;

 /*
 * @brief Read back measument format.
 *
 **/
 typedef struct __attribute__((aligned(1), packed)) {
     
    uint8_t Head;
    uint8_t RW7_Addr6_0;
    uint8_t RegAddr_MSB;
    uint8_t RegAddr_LSB;
    uint8_t PayloadCnt_MSB;
    uint8_t PayloadCnt_LSB;
    uint8_t PayloadDist_MSB_H;
    uint8_t PayloadDist_MSB_L;
    uint8_t PayloadDist_LSB_H;
    uint8_t PayloadDist_LSB_L;
    uint8_t PayloadSQ_MSB;
    uint8_t PayloadSQ_LSB;
    uint8_t Checksum;

} U81X_MeasReg_t;

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*uart_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

    U81X_RWCmdFrame_t RWCommandFrame;
    U81X_ReadBackReg_t ReadBackReg;
    U81X_MeasReg_t ReadBackMeas;
    uart_txrx_fptr uart_tx;
    uart_txrx_fptr uart_rx;


} U81X_GInst_t;


/* Public function prototypes. */
void U81x_Init(void);

U81X_MeasReg_t U81X_GetRawMeasData(void);
uint16_t U81X_GetSNumber(uint8_t module_addr);
uint16_t U81X_GetHWVersion(uint8_t module_addr);
uint16_t U81X_GetSWVersion(uint8_t module_addr);
uint16_t U81X_GetInVoltage(uint8_t module_addr);
uint32_t U81X_ReadDistance(uint8_t module_addr);
uint16_t U81X_LaserOnOff(uint8_t module_addr, uint16_t on_off);
uint8_t U81X_WriteAddress(uint8_t module_addr, uint8_t new_addr);
int16_t U81X_WriteMeasOffset(uint8_t module_addr, uint16_t offset);

void U81X_StopContMeas(void);
void U81X_StartMultiSlaveMeasure(void);
uint32_t U81X_StartOneShotAutoMeas(uint8_t module_addr);
uint32_t U81X_StartOneShotSlowMeas(uint8_t module_addr);
uint32_t U81X_StartOneShotFastMeas(uint8_t module_addr);
uint32_t U81X_StartContAutoMeas(uint8_t module_addr);
uint32_t U81X_StartContSlowMeas(uint8_t module_addr);
uint32_t U81X_StartContFastMeas(uint8_t module_addr);

uint8_t U81X_GetCheckSum(uint8_t* data, uint8_t size);
U81X_STATUS_CODE_t U81X_GetStatus(uint8_t module_addr);
uint16_t U81X_ReadX16(uint8_t module_addr, uint16_t reg_addr);
uint16_t U81X_WriteX16(uint8_t module_addr, uint16_t reg_addr, uint16_t data);
uint32_t U81X_StartMeasX(uint8_t module_addr, uint16_t reg_addr, uint16_t data);







#endif /* U81X_H_ */
