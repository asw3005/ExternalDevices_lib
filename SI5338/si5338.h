/*
 * si5338.h header file.
 *
 * Created on: Sep 1, 2023
 * Author: asw3005
 */

#ifndef SI5338_H_
#define SI5338_H_

#include "stm32f4xx.h"

#define SI5338_NUM_REGS_MAX 		350

/* I2C bus addresses. */
#define SI5338_ADDRESS0				0x70
#define SI5338_ADDRESS1				0x71
#define SI5338_ADDRESS_SHIFTED 		(SI5338_ADDRESS0 << 1)

/*
 * @brief Register maps.
 */
typedef enum {

	SI5338_DEV_REV_ID,
	SI5338_DEV_START_CFG = 2

} SI5338_REG_MAP_t;

/*
 * @brief Register maps.
 */
typedef enum {

	SI5338_BASE_PART_NUMBER = 38

} SI5338_DEV_CFG_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*si5338rxtx_fptr)(uint8_t MemAddr, uint8_t* pData, uint8_t Size);

/*
 * @brief Register addressing bytes.
 */
typedef struct {
   uint8_t RegAddr;
   uint8_t RegVal;
   uint8_t RegMask;

} SI5338_RegData_t;

/*
 * @brief Ready to use configuration (text name).
 */
typedef struct {
	uint8_t BasePartNumber;
	uint8_t DevGrade;
	uint32_t NVMCodeNumber;

} SI5338_ReadyDevCfg_t;

/*
 * @brief
 */
typedef struct __attribute__((aligned(1), packed)) {

	union {
		uint8_t DevCfg2;
		struct {
			uint8_t DEV_CFG2_5_0	: 6;
			uint8_t RESERVED0 		: 2;
		};
	};

	union {
		uint8_t DevCfg3;
		struct {
			uint8_t DEV_CFG3_0 		: 1;
			uint8_t RESERVED1 		: 2;
			uint8_t DEV_CFG3_7_3	: 5;
		};
	};

		uint8_t DevCfg4;
		uint8_t DevCfg5;

} SI5338_DevCfg_t;

/*
 * @brief General struct.
 */
typedef struct {

	/*I2C addr. */
	uint8_t addr;
	/* Function pointers. */
	delay_fptr delay_fp;
	si5338rxtx_fptr i2c_rx_fp;
	si5338rxtx_fptr i2c_tx_fp;

} SI5338_GStr_t;


/* Public function prototypes. */
void SI5338_Init(void);
uint8_t SI5338_GetDevRevID(void);
SI5338_ReadyDevCfg_t* SI5338_GetDevCfg(void);

uint8_t SI5338_ReadReg(uint8_t address);
void SI5338_WriteReg(uint8_t address, uint8_t value);

#endif /* SI5338_H_ */
