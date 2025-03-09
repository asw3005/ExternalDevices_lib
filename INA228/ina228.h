/*
 * ina228.h header file.
 *
 * Created on: Jan 23, 2025
 * Author: asw3005
 */

#ifndef INA228_H_
#define INA228_H_

#include "stm32f1xx.h"

/* Chip ID. */
#define	INA228_ID_MANF				0x5449
#define	INA228_ID_DIE				0x0228
#define INA228_ID_REV				0x0001

/* I2C bus addresses. */
#define INA228_ADDR0				0x40
#define INA228_ADDR_SHIFTED 		(INA228_ADDR0 << 1)
#define INA228_CURRENT_ADDR 		INA228_ADDR_SHIFTED

/*
 * @brief Register maps.
 */
typedef enum {

	INA228_CFG,
	INA228_ADCCFG,
	INA228_SHUNT_CAL,
	INA228_SHUNT_TEMPCO,
	INA228_VSHUNT,
	INA228_VBUS,
	INA228_DIETEMP,
	INA228_CURRENT,
	INA228_POWER,
	INA228_ENERGY,
	INA228_CHARGE,
	INA228_DIAG_ALRT,
	INA228_SOVL,
	INA228_SUVL,
	INA228_BOVL,
	INA228_BUVL,
	INA228_TEMP_LIMIT,
	INA228_PWR_LIMIT,
	INA228_MANUFACTURER_ID 			= 0x3E,
	INA228_DIE_ID 					= 0x3F

} INA228_REG_MAP_t;

/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t MemAddr, uint8_t* pData, uint8_t Size);



/*
 * @brief General struct.
 */
typedef struct {

	uint8_t DevAddress;
	uint8_t Data_MSB;
	uint8_t Data_LSB;
	/* Function pointers. */
	delay_fptr delay_fp;
	rxtx_fptr i2c_rx_fp;
	rxtx_fptr i2c_tx_fp;

} INA228_GStr_t;


/* Public function prototypes. */

#endif /* INA228_H_ */
