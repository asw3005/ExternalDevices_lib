/*
 * @brief Header file for common.c.
 * 
 * Created 08.19.21
 *
 **/

#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f103xb.h"

#define MAX_ADDRESES_NUMBER		16
#define LED_TIMEOUT				10000

/*
 * @brief Broadcast command set.
 *
 **/
typedef enum {

	CMD_DISABLE_DEVICE,
	CMD_WRITE_VOLTAGE,
	CMD_VIEW_ADDRESS,
	CMD_RESERVED3,
	CMD_RESERVED4,
	CMD_RESERVED5,
	CMD_RESERVED6,
	CMD_RESERVED7
	
} CMD_BroadcastCommand;

/*
 * @brief General command set.
 *
 **/
typedef enum {

	CMD_GCS_IS_READY,
	CMD_GCS_DISABLE_DEVICE,
	CMD_GCS_WRITE_VOLTAGE,
	CMD_GCS_READ_VOLTAGE,
	CMD_GCS_SET_ADDRESS,
	CMD_GCS_RESERVED5,
	CMD_GCS_RESERVED6,
	CMD_GCS_RESERVED7
	
} CMD_GeneralCommand;

/*
 * @brief  Data type external non volatile memory.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {
	
	uint8_t DeviceAddress;
	
} CMD_EPROMData;

/*
 * @brief Queue data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	struct {
		uint8_t CmdNumber;	
		int16_t DacVoltage;
	};
	
	struct {
		uint8_t Command;	
		uint8_t Data[8];
	};
	
} CMD_QueueData_t;

/*
 * @brief Device identificator and command data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	uint16_t Identifier;
	struct {
		uint16_t ID_TODO		: 3;
		uint16_t ID_ADDRESS		: 8;
		uint16_t RESERVED15_11	: 5; 
	};		
} CMD_DevId_t;

/*
 * @brief 32 bits filter bank register organization data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	struct {
		uint16_t FilterBankReg_LSB;
		uint16_t FilterBankReg_MSB;
	};
	struct {
		/* Must be zero. */
		uint32_t RESERVED0	: 1;
		uint32_t RTR		: 1;
		uint32_t IDE		: 1;
		/* Extended identifier. */
		uint32_t EXID20_3	: 18;
		/* Standart identifier. */
		uint32_t STID31_21	: 11;		
	};
	
} CAN_FilterBankRegOrg32_t;

/*
 * @brief 16 bits filter bank register organization data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint16_t FilterBankReg;

	struct {
		/* Extended identifier. */
		uint16_t EXID18_16	: 3;
		/* A dominant single identifier extension (IDE) bit means that a standard CAN identifier with no extension is being
		 * transmitted. */
		uint16_t IDE		: 1;
		/* The single remote transmission request bit is dominant when information is required from another node. */
		uint16_t RTR		: 1;
		/* Standart identifier. */
		uint16_t STID31_21	: 11;		
	};
	
} CAN_FilterBankRegOrg16_t;


/* Public function prototypes. */

void CAN_ConfigAddr(CAN_HandleTypeDef *hcan, const uint8_t bank, const uint8_t address, const uint8_t cmd_1,
					const uint8_t cmd_2,  const uint8_t cmd_3,  const uint8_t cmd_4);
void CAN_ConfigBank(CAN_HandleTypeDef *hcan);

void BTN_SWPress(void);

void SPI_Tx(uint8_t *pData, uint8_t size);
void SPI_Rx(uint8_t *pData, uint8_t size);

void I2C_Tx(uint16_t address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *pData, uint16_t size);
void I2C_Rx(uint16_t address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *pData, uint16_t size);


#endif /* COMMON_H_ */