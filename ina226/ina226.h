/*
 * ina226.h header file.
 *
 * Created on: Jan 9, 2025
 * Author: asw3005
 */

#ifndef INA226_H_
#define INA226_H_

#include "stm32f1xx.h"

/* Chip ID. */
#define	INA226_ID_MANUFACTURER		0x5449
#define	INA226_ID_DIE				0x2260

/* I2C bus addresses. */
#define INA226_ADDR0				0x40
#define INA226_ADDR_SHIFTED 		(INA226_ADDR0 << 1)
#define INA226_CURRENT_ADDR 		INA226_ADDR_SHIFTED

/* Constants. */
#define INA226_MAX_EXPECTED_CURRENT 10.0f /* A */
#define INA226_RSHUNT				0.010f /* Ohm */
#define INA226_SHUNT_VOLTAGE_LSB 	0.0000025f /* V */
#define INA226_BUS_VOLTAGE_LSB 		0.00125f /* V */
#define INA226_CURRENT_LSB			(INA226_MAX_EXPECTED_CURRENT / 32768.0f)
#define INA226_POWER_LSB			(25.0f * INA226_CURRENT_LSB)
#define INA226_CALIBRATION_VAL		((0.00512f) / (INA226_CURRENT_LSB * RSHUNT))

/*
 * @brief Register maps.
 */
typedef enum {

	INA226_CFG,
	INA226_VSHUNT,
	INA226_VBUS,
	INA226_POWER,
	INA226_CURRENT,
	INA226_CALIBRATION,
	INA226_MASK_EN,
	INA226_ALERT_LIMIT,
	INA226_MANUFACTURER_ID 			= 0xFE,
	INA226_DIE_ID 					= 0xFF

} INA226_REG_MAP_t;


/* Function pointer prototypes. */
typedef void(*delay_fptr)(uint32_t);
typedef void(*rxtx_fptr)(uint8_t MemAddr, uint8_t* pData, uint8_t Size);

/*
 * @brief Configuration register.
 */
typedef union __attribute__((aligned(1), packed)) {

	uint8_t CfgReg_LSB;
	uint8_t CfgReg_MSB;
	struct {
		uint8_t MODE2_0		: 3;
		uint8_t VSHCT2_0 	: 3;
		uint8_t VBUSCT2_0 	: 3;
		uint8_t AVG2_0 		: 3;
		/* Must be set to 0x04. */
		uint8_t RSVD14_12 	: 3;
		uint8_t RST 		: 1;
	};

} INA226_CfgReg_t;

/*
 * @brief Mask/Enable register.
 */
typedef union __attribute__((aligned(1), packed)) {

	uint8_t MaskEnReg_LSB;
	uint8_t MaskEnReg_MSB;
	struct {
		uint8_t LEN 		: 1;
		uint8_t APOL 		: 1;
		uint8_t OVF 		: 1;
		uint8_t CVRF 		: 1;
		uint8_t AFF 		: 1;
		/* Must be zero. */
		uint8_t RESERVED9_5 : 5;
		uint8_t CNVR 		: 1;
		uint8_t POL 		: 1;
		uint8_t BUL 		: 1;
		uint8_t BOL 		: 1;
		uint8_t SUL 		: 1;
		uint8_t SOL 		: 1;
	};

} INA226_MaskEn_t;

/*
 * @brief Chip ID.
 */
typedef struct {

	uint16_t ManufactID;
	uint16_t DieID;

} INA226_ChipID_t;

/*
 * @brief Measurements.
 */
typedef struct {

	float ShuntVoltage;
	float BusVoltage;
	float Current;
	float Power;

} INA226_Meas_t;

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

} INA226_GStr_t;


/* Public function prototypes. */
void INA226_Init(void);
uint16_t INA226_ReadReg(uint8_t reg_address);
void INA226_WriteReg(uint8_t reg_address, uint16_t value);
void INA226_SetCfgReg(uint8_t mode, uint8_t vshct, uint8_t vbusct, uint8_t avg, uint8_t reset);
INA226_ChipID_t INA226_ReadID(void);
INA226_MaskEn_t INA226_RWMaskEnReg(uint8_t rw, uint8_t len, uint8_t apol, uint8_t ovf, uint8_t cvrf, uint8_t aff,
				uint8_t cnvr, uint8_t pol, uint8_t bul, uint8_t bol, uint8_t sul, uint8_t sol);

#endif /* INA226_H_ */
