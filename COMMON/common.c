/*
 * @brief Set of common data and functions.
 * 
 * Created 08.19.21
 *
 **/

#include "stm32f103xb.h"
#include "FreeRTOS.h"
#include "spi.h"
#include "common.h"
#include "semphr.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
SemaphoreHandle_t SWBtnPressed_SH;
//extern CAN_HandleTypeDef hcan;

/*
 * @brief Set-up 16-bit list mode identifier to specific bank.
 * 
 * @param hcan : Pointer to a CAN_HandleTypeDef structure that contains
 *        the configuration information for the specified CAN.
 * @param bank : Specifies the filter bank which will be initialized. This parameter must be a number between Min_Data = 0
 *        and Max_Data = 13.
 * @param address : Device address that you wish from 0x00 to 0xFF.
 * @param cmd_x : These four command may be enything what you wish, but you can use this one from the HVU_NarrowCommand or
 *        HVU_BroadcastCommand enums.
 *
 **/
void CAN_ConfigAddr(CAN_HandleTypeDef *hcan, const uint8_t bank, const uint8_t address, const uint8_t cmd_1,
					const uint8_t cmd_2,  const uint8_t cmd_3,  const uint8_t cmd_4)
{
	CAN_FilterBankRegOrg16_t CanRegOrg16;	
	CMD_DevId_t DevId;
	
	DevId.ID_ADDRESS = address;
	DevId.RESERVED15_11 = 0;
	
	/* Activate initialization mode. */
	hcan->Instance->FMR |= CAN_FMR_FINIT;
	
	/* Set-up identifiers. */
	CanRegOrg16.IDE = 0;
	CanRegOrg16.RTR = 0;
	CanRegOrg16.EXID18_16 = 0;
	DevId.ID_TODO = cmd_1;
	CanRegOrg16.STID31_21 = DevId.Identifier;
	hcan->Instance->sFilterRegister[bank].FR1 = (uint32_t)CanRegOrg16.FilterBankReg;
	DevId.ID_TODO = cmd_2;
	CanRegOrg16.STID31_21 = DevId.Identifier;
	hcan->Instance->sFilterRegister[bank].FR1 |= (uint32_t)CanRegOrg16.FilterBankReg << 16U;
	DevId.ID_TODO = cmd_3;
	CanRegOrg16.STID31_21 = DevId.Identifier;
	hcan->Instance->sFilterRegister[bank].FR2 = (uint32_t)CanRegOrg16.FilterBankReg;
	DevId.ID_TODO = cmd_4;
	CanRegOrg16.STID31_21 = DevId.Identifier;
	hcan->Instance->sFilterRegister[bank].FR2 |= (uint32_t)CanRegOrg16.FilterBankReg << 16U;
	
	/* Deactivate initialization mode. */
	hcan->Instance->FMR &= ~CAN_FMR_FINIT;
}

/*
 * @brief
 * 
 * @param hcan : pointer to a CAN_HandleTypeDef structure that contains
 *        the configuration information for the specified CAN.
 *
 **/
void CAN_ConfigBank(CAN_HandleTypeDef *hcan)
{
	/* Activate initialization mode. */
	hcan->Instance->FMR |= CAN_FMR_FINIT;
	
	/* These bits are not matter because the MCU has only one CAN interface. Default value is 14. */
	hcan->Instance->FMR |= 14 << CAN_FMR_CAN2SB_Pos;
	/* Enable list mode for the bank 0 to 13. By defalt is identifier mask mode. */
	hcan->Instance->FM1R = CAN_FM1R_FBM;
	/* Filter scale is 16 bit value. By default the banks are 16 bit scale configuration. */
	hcan->Instance->FS1R = 0;
	/* FIFO assignemt. Bank zero, one in FIFO0, all remaining banks in the FIFO1. */
	hcan->Instance->FFA1R = (CAN_FFA1R_FFA & ~( CAN_FFA1R_FFA0 | CAN_FFA1R_FFA1));
	/* Activate specific filterbanks. */	
	hcan->Instance->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3;
	/* Can interrupt configuration. */
	hcan->Instance->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_ERRIE;
	/* Set-up identifiers. */
	hcan->Instance->sFilterRegister[0].FR1 = 0x00010000;
	hcan->Instance->sFilterRegister[0].FR2 = 0x00030002;
	hcan->Instance->sFilterRegister[1].FR1 = 0x00050004;
	hcan->Instance->sFilterRegister[1].FR2 = 0x00070006;
	
	/* Deactivate initialization mode. */
	hcan->Instance->FMR &= ~CAN_FMR_FINIT;
}

/*
 *@brief Software button trigger.
 *
 **/
void BTN_SWPress(void) {
	
	/* Giving the semaphore. */
	xSemaphoreGive(SWBtnPressed_SH);
	/* Software triggered PA3 external interrupt input. It's like press a button. */
	EXTI->SWIER = EXTI_SWIER_SWI3;
}

/* Transmitting functions. */

/*
 * @brief
 *
 **/
void SPI_Tx(uint8_t *pData, uint8_t size) {
	
	HAL_SPI_Transmit(&hspi1, pData, size, 10);
}

/*
 * @brief
 *
 **/
void SPI_Rx(uint8_t *pData, uint8_t size) {
	
	HAL_SPI_Receive(&hspi1, pData, size, 10);
}

/*
 * @brief
 *
 **/
void I2C_Tx(uint16_t address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *pData, uint16_t size) {
	
	HAL_I2C_Master_Transmit(&hi2c1, address, pData, size, 10);
}

/*
 * @brief
 *
 **/
void I2C_Rx(uint16_t address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *pData, uint16_t size) {
	
	HAL_I2C_Master_Receive(&hi2c1, address, pData, size, 10);
}


/*
 * @brief
 *
 **/