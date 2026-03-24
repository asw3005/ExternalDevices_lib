/*
 * @brief Common source for U81X.
 * Created 03.24.26 by asw3005. 
 *
 **/
#include "u81x.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>

 /* External variables. */
extern UART_HandleTypeDef huart1;
UART_HandleTypeDef* LdUart = &huart1;

/* Private variables. */


/* Private function prototypes. */
static void U81X_Tx(uint8_t *pData, uint8_t size);
static void U81X_Rx(uint8_t *pData, uint8_t size);

/* General struct. */
static U81X_GInst_t u81x_inst = {

    .RWCommandFrame.W_Head = U81X_HEAD,
    .uart_tx = U81X_Tx,
    .uart_rx = U81X_Rx
};

/*
 * @brief 
 *
**/

/*
 * @brief Init control pins.
 *
**/
void U81x_Init(void) {


}

/*
 * @brief Read module status.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval U81X_STATUS_CODE_t : U81X_NO_ERROR,
 *                              U81X_LOW_INPUT_POWER,
 *                              U81X_INTERNAL_ERROR,
 *                              U81X_LOW_MODULE_TEMP,
 *                              U81X_HIGH_MODULE_TEMP,
 *                              U81X_TARGET_OUT_OF_RANGE,
 *                              U81X_INVALID_MEAS_RESULT,
 *                              U81X_BACKGROUND_LIGHT_TOO_STRONG,
 *                              U81X_LASER_SIGNAL_TOO_WEAK,
 *                              U81X_LASER_SIGNAL_TOO_STRONG,
 *                              U81X_HARDFAULT_1,
 *                              U81X_HARDFAULT_2,
 *                              U81X_HARDFAULT_3,
 *                              U81X_HARDFAULT_4,
 *                              U81X_HARDFAULT_5,
 *	                            U81X_LASER_SIGNAL_NOT_STABLE,
 *                              U81X_HARDFAULT_6,
 *                              U81X_HARDFAULT_7,
 *                              U81X_INVALID_FRAME
 *
**/
U81X_STATUS_CODE_t U81X_GetStatus(uint8_t module_addr) {

    return U81X_ReadX16(module_addr, U81X_ERR_CODE);
}

/*
 * @brief Read hardware version.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint16_t : 0xXXYY.
 *
**/
uint16_t U81X_GetHWVersion(uint8_t module_addr) {

    return U81X_ReadX16(module_addr, U81X_HWVERSION);
}

/*
 * @brief Read software version.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint16_t : 0xXXYY.
 *
**/
uint16_t U81X_GetSWVersion(uint8_t module_addr) {

    return U81X_ReadX16(module_addr, U81X_SWVERSION);
}

/*
 * @brief Read serial number.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint16_t : 0xXXYY.
 *
**/
uint16_t U81X_GetSNumber(uint8_t module_addr) {

    return U81X_ReadX16(module_addr, U81X_SNUMBER);
}

/*
 * @brief Read input voltage.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint16_t : Voltage, mV.
 *
**/
uint16_t U81X_GetInVoltage(uint8_t module_addr) {

    static uint16_t InVoltage;

    InVoltage = U81X_ReadX16(module_addr, U81X_BAT_VLTG);

    return ( ((InVoltage >> 8) * 100) + (InVoltage & 0x00FF) );
}

/*
 * @brief Read measure result.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, mm.
 *
**/
uint32_t U81X_ReadDistance(uint8_t module_addr) {

    u81x_inst.RWCommandFrame.R_RW7_Addr6_0 = U81X_READ | (0x7F & module_addr);
    u81x_inst.RWCommandFrame.R_RegAddr_MSB = 0;
    u81x_inst.RWCommandFrame.R_RegAddr_LSB = U81X_MEA_RESULT;
    u81x_inst.RWCommandFrame.R_Checksum = U81X_GetCheckSum(&u81x_inst.RWCommandFrame.R_RW7_Addr6_0, U81X_RCMD_CHECKSUM);


    u81x_inst.uart_rx(&u81x_inst.ReadBackMeas.Head, sizeof(u81x_inst.ReadBackMeas));
    u81x_inst.uart_tx(&u81x_inst.RWCommandFrame.R_Head, U81X_RCMD_DEF);

    if(U81X_GetCheckSum(&u81x_inst.ReadBackMeas.RW7_Addr6_0, sizeof(u81x_inst.ReadBackMeas) - 2) != u81x_inst.ReadBackMeas.Checksum) {
        return 1;
    }

    return (u81x_inst.ReadBackMeas.PayloadDist_MSB_H | u81x_inst.ReadBackMeas.PayloadDist_MSB_L |u81x_inst.ReadBackMeas.PayloadDist_LSB_H | u81x_inst.ReadBackMeas.PayloadDist_LSB_L);
}

/*
 * @brief Write module address. Do not set slave address to broadcast address 0x7F, this address is reserved for one master to
 *        multi-slave network which needs all slave to measure distance at the same time, and no slave reply measure result until 
 *        master ask one of them to.
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @param new_addr      : New address for module.
 *
 * @retval uint8_t      : Return value of new address.
 *
**/
uint8_t U81X_WriteAddress(uint8_t module_addr, uint8_t new_addr) {

    static uint8_t CurrentAddress;

    CurrentAddress = (U81X_WriteX16(module_addr, U81X_ADDRESS, (0x7F & new_addr))  & 0x007F);
    return CurrentAddress;
}

/*
 * @brief Set module measure offset.
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 * @param offset        : Offset value. For example, if the offset 0xZZYY = 0x7B(+123) , it means the final output of measure result 
 *                        will PLUS 123 millimeters , if the offset 0xZZYY = 0xFF85(-123), it means the final output of measure result will MINUS 
 *                        123 millimeters.
 *
 * @retval int16_t      : Current offeset value, 0xZZYY.
 *
**/
int16_t U81X_WriteMeasOffset(uint8_t module_addr, uint16_t offset) {

    static int16_t CurrentOffset;

    CurrentOffset = U81X_WriteX16(module_addr, U81X_OFFSET, offset);
    return CurrentOffset;
}

/*
 * @brief Turn om or turn off laser.
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 * @param on_off        : Turn on or turn off laser beam.
 *                          0 - laser off,
 *                          1 - laser on.
 *
 * @retval uint16_t      : Led state.
 *
**/
uint16_t U81X_LaserOnOff(uint8_t module_addr, uint16_t on_off) {

    static uint16_t LedState;

    LedState = U81X_WriteX16(module_addr, U81X_CTRL_LD, on_off);
    return LedState;
}

/*
 * @brief Start one-shot auto distance measure. Initiate slave to do 1-shot measure in auto mode.
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartOneShotAutoMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0000);
    return Distance;
}

/*
 * @brief Start one-shot slow distance measure. Initiate slave to do 1-shot measure in slow mode. 
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartOneShotSlowMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0001);
    return Distance;
}

/*
 * @brief Start one-shot fast distance measure. Initiate slave to do 1-shot measure in fast mode. 
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartOneShotFastMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0002);
    return Distance;
}

/*
 * @brief Start continious auto distance measure. Initiate slave to do continuous measure in auto mode. 
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartContAutoMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0004);
    return Distance;
}

/*
 * @brief Start continious slow distance measure. Initiate slave to do continuous measure in slow mode. 
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartContSlowMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0005);
    return Distance;
}

/*
 * @brief Start continious slow distance measure. Initiate slave to do continuous measure in fast mode. 
 *
 * @param module_addr   : Module address (current). Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval uint32_t : 0xAABBCCDD, distance in mm.
 *
**/
uint32_t U81X_StartContFastMeas(uint8_t module_addr) {

    static uint32_t Distance;

    Distance = U81X_StartMeasX(module_addr, U81X_MEA_START, 0x0006);
    return Distance;
}

/*
 * @brief Exit from continious measure.
 *
**/
void U81X_StopContMeas(void) {

    static uint8_t StopContMeas = U81X_STOP_CONT_MEAS;
    u81x_inst.uart_tx(&StopContMeas, sizeof(StopContMeas));
}

/*
 * @brief Master send out 1-shot measure commands to slave address 0x7F, that will make all online slaves to
 *          measure distance at the same time，but none of them will return its measure result to master until
 *          master ask each one to return the measure result. Before master send out the reading measure result
 *          command, master should read the slave’s status code to make sure there was no error occurred during
 *          this slave measuring. After sending this command out, master polling each slave address for their 
 *          status, if slave replies its status code with 0x0000, means no error, then send Read-measure-Result 
 *          command to read back the distance. Measure result for each slave will NOT overwrite until next successful 
 *          measure command with a new distance result.
 *
**/
void U81X_StartMultiSlaveMeasure(void) {

    u81x_inst.RWCommandFrame.W_RW7_Addr6_0 = U81X_WRITE | 0x7F;
    u81x_inst.RWCommandFrame.W_RegAddr_MSB = 0;
    u81x_inst.RWCommandFrame.W_RegAddr_LSB = U81X_MEA_START;
    u81x_inst.RWCommandFrame.W_PayloadCnt_MSB = 0;
    u81x_inst.RWCommandFrame.W_PayloadCnt_LSB = 1;
    u81x_inst.RWCommandFrame.W_Payload_MSB = 0;
    u81x_inst.RWCommandFrame.W_Payload_LSB = 0;
    u81x_inst.RWCommandFrame.W_Checksum = U81X_GetCheckSum(&u81x_inst.RWCommandFrame.W_RW7_Addr6_0, U81X_WCMD_CHECKSUM);

    u81x_inst.uart_tx(&u81x_inst.RWCommandFrame.W_Head, U81X_WCMD_DEF);
}

/*
 * @brief Get RAW measure data from private data struct.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 *
 * @retval U81X_MeasReg_t : Raw data struct.
 *
**/
U81X_MeasReg_t U81X_GetRawMeasData(void) {

    return u81x_inst.ReadBackMeas;
}

/*
 * @brief Read 16-bit data.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 * @param reg_addr    : Register's address. See valid values in the U81X_REG_MAP_t. 
 *
 * @retval uint16_t : 0xXXYY.
 *
**/
uint16_t U81X_ReadX16(uint8_t module_addr, uint16_t reg_addr) {

    u81x_inst.RWCommandFrame.R_RW7_Addr6_0 = U81X_READ | (0x7F & module_addr);
    u81x_inst.RWCommandFrame.R_RegAddr_MSB = reg_addr >> 8;
    u81x_inst.RWCommandFrame.R_RegAddr_LSB = reg_addr;
    u81x_inst.RWCommandFrame.R_Checksum = U81X_GetCheckSum(&u81x_inst.RWCommandFrame.R_RW7_Addr6_0, U81X_RCMD_CHECKSUM);


    u81x_inst.uart_rx(&u81x_inst.ReadBackReg.Head, sizeof(u81x_inst.ReadBackReg));
    u81x_inst.uart_tx(&u81x_inst.RWCommandFrame.R_Head, U81X_RCMD_DEF);

    return (u81x_inst.ReadBackReg.Payload_MSB | u81x_inst.ReadBackReg.Payload_LSB);
}

/*
 * @brief Write 16-bit data.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 * @param reg_addr    : Register's address. See valid values in the U81X_REG_MAP_t. 
 * @param data        : Data to write.
 *
 * @retval uint16_t : 0xXXYY.
 *
**/
uint16_t U81X_WriteX16(uint8_t module_addr, uint16_t reg_addr, uint16_t data) {

    u81x_inst.RWCommandFrame.W_RW7_Addr6_0 = U81X_WRITE | (0x7F & module_addr);
    u81x_inst.RWCommandFrame.W_RegAddr_MSB = reg_addr >> 8;
    u81x_inst.RWCommandFrame.W_RegAddr_LSB = reg_addr;
    u81x_inst.RWCommandFrame.W_PayloadCnt_MSB = 0;
    u81x_inst.RWCommandFrame.W_PayloadCnt_LSB = 1;
    u81x_inst.RWCommandFrame.W_Payload_MSB = data >> 8;
    u81x_inst.RWCommandFrame.W_Payload_LSB = data;
    u81x_inst.RWCommandFrame.W_Checksum = U81X_GetCheckSum(&u81x_inst.RWCommandFrame.W_RW7_Addr6_0, U81X_WCMD_CHECKSUM);

    u81x_inst.uart_rx(&u81x_inst.ReadBackReg.Head, sizeof(u81x_inst.ReadBackReg));
    u81x_inst.uart_tx(&u81x_inst.RWCommandFrame.W_Head, U81X_WCMD_DEF);

    return (u81x_inst.ReadBackReg.Payload_MSB | u81x_inst.ReadBackReg.Payload_LSB);
}

/*
 * @brief Start measurement.
 *
 * @param module_addr : Module address. Address only take bit[6:0].
 *                          default     - 0x00, 
 *                          broadcast   - 0x7F.
 * @param reg_addr    : Register's address. See valid values in the U81X_REG_MAP_t. 
 * @param data        : Data to write.
 *
 * @retval uint32_t : 0xAABBCCDD, mm.
 *
**/
uint32_t U81X_StartMeasX(uint8_t module_addr, uint16_t reg_addr, uint16_t data) {

    u81x_inst.RWCommandFrame.W_RW7_Addr6_0 = U81X_WRITE | (0x7F & module_addr);
    u81x_inst.RWCommandFrame.W_RegAddr_MSB = reg_addr >> 8;
    u81x_inst.RWCommandFrame.W_RegAddr_LSB = reg_addr;
    u81x_inst.RWCommandFrame.W_PayloadCnt_MSB = 0;
    u81x_inst.RWCommandFrame.W_PayloadCnt_LSB = 1;
    u81x_inst.RWCommandFrame.W_Payload_MSB = data >> 8;
    u81x_inst.RWCommandFrame.W_Payload_LSB = data;
    u81x_inst.RWCommandFrame.W_Checksum = U81X_GetCheckSum(&u81x_inst.RWCommandFrame.W_RW7_Addr6_0, U81X_WCMD_CHECKSUM);

    u81x_inst.uart_rx(&u81x_inst.ReadBackMeas.Head, sizeof(u81x_inst.ReadBackMeas));
    u81x_inst.uart_tx(&u81x_inst.RWCommandFrame.W_Head, U81X_WCMD_DEF);

    return (u81x_inst.ReadBackMeas.PayloadDist_MSB_H | u81x_inst.ReadBackMeas.PayloadDist_MSB_L |u81x_inst.ReadBackMeas.PayloadDist_LSB_H | u81x_inst.ReadBackMeas.PayloadDist_LSB_L);
}

/*
 * @brief Checksum calculation.
 *
**/
uint8_t U81X_GetCheckSum(uint8_t* data, uint8_t size) {

    uint8_t data_check = 0;

    for(uint8_t i = 0; i < size; i++) {
        data_check += *data;
        data++;
    }

    return data_check;
}

 /* Hardware dependent functions. */

/*
 * @brief UART Tx data.
 *
 **/
static void U81X_Tx(uint8_t *pData, uint8_t size) {

	HAL_UART_Transmit(LdUart, pData, size, 10);
}

/*
 * @brief UART Rx data.
 * 
 *
 **/
static void U81X_Rx(uint8_t *pData, uint8_t size) {
	
	HAL_UART_Receive(LdUart, pData, size, 10);
}



/* Interrupt callbacks. */

