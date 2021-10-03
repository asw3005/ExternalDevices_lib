/*
 * @brief Common header for DAC60501, DAC70501, DAC80501 from the Texas Instruments.
 * Created 07.19.21 by asw3005. 
 *
 **/

#ifndef DACx0501_H_
#define DACx0501_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32f103xb.h"

/* Define your hardvare configuration. */
#define SPI_COMMUNICATION
//#define I2C_COMMUNICATION

/* GPIO configuration. */
#define CS_DAC_Pin					GPIO_PIN_1
#define CS_DAC_GPIO_Port			GPIOB

/* DAC constants. */
#define DAC60501_VALUE_OF_DIVISION	0.8192f
#define DAC70501_VALUE_OF_DIVISION	3.2768f
#define DAC80501_VALUE_OF_DIVISION	13.1072f

/*
 * @brief DAC resolution values.
 *
 **/
typedef enum {
	
	DAC60501_12BIT = 4095,
	DAC70501_14BIT = 16383,
	DAC80501_16BIT = 65535
	
	
} DACx0501_RESOLUTION;

/*
 * @brief Slave addrreses if you are using I2C bus (addresses are shifted 1 bit to the left).
 *
 **/
typedef enum {
	
	DACx0501_ADDR0_SH = 0x90,
	DACx0501_ADDR1_SH = 0x92,
	DACx0501_ADDR2_SH = 0x94,
	DACx0501_ADDR3_SH = 0x96
		
} DACx0501_I2C_Addr;

/*
 * @brief DAC type list.
 *
 **/
typedef enum {
	
	DACx0501_DAC60501,
	DACx0501_DAC70501,
	DACx0501_DAC80501
	
} DACx0501_Type;

/*
 * @brief DAC command bytes.
 *
 **/
typedef enum {

	DACx0501_NOOP,
	DACx0501_DEVID,
	DACx0501_SYNC,
	DACx0501_CONFIG,
	DACx0501_GAIN,
	DACx0501_TRIGGER,
	DACx0501_STATUS   = 0x07,
	DACx0501_DAC_DATA,
	
	DACx0501_SOFT_RESET = 0x0A

} DACx0501_Command;

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_txrx_fptr)(uint8_t *pData, uint8_t size);
typedef void(*i2c_txrx_fptr)(uint16_t address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *pData, uint16_t size);


/*
 * @brief Noop register structure.
 *
 **/
typedef struct {

	uint16_t Noop;
} DACx0501_Noop_t;

/*
 * @brief  DEVID register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	/* Device ID register. */
	uint16_t DevId;
	struct {
		uint8_t DevIdLSB;
		uint8_t DevIdMSB;
	};
	struct {
		/* Default 0x15. */
		uint8_t RESERVED6_0     : 7;
		/* DAC Power on Reset: 0h (DAC80501Z reset to zero scale) 
		 *                     1h (DAC80501M reset to midscale) */
		uint8_t RSTSEL          : 1;
		/* Default 0x02. */
		uint8_t RESERVED11_8    : 4;
		/* DAC Resolution: 0000h (DAC80501 16-bit)
		 *                 0001h (DAC70501 14-bit)
		 *                 0020h (DAC60501 12-bit) */
		uint8_t RESOLUTION      : 3;
		/* Default 0x00. */
		uint8_t RESERVED15      : 1;
	};
} DACx0501_Devid_t;

/*
 * @brief  SYNC register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	/* Sync register. */
	uint16_t Sync;
	struct {
		uint8_t SyncLSB;
		uint8_t SyncMSB;
	};
	struct {
		/* When set to 1, the DAC output is set to update in response to
		 * an LDAC trigger (synchronous mode). When cleared to 0 ,the DAC 
		 * output is set to update immediately (asynchronous mode), default. */
		uint8_t DAC_SYNC_EN     : 1;
		/* Default 0x00. */
		uint8_t RESERVED7_1		: 7;
		uint8_t RESERVED15_8    : 8;
	};
} DACx0501_Sync_t;

/*
 * @brief CONFIG register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint16_t Config;
	struct {
		uint8_t ConfigLSB;
		uint8_t ConfigMSB;
	};
	struct {
		/* When set to 1, the DAC in power-down mode and the DAC output is connected to GND through a 1-kΩ internal resistor. 
		 * default is 0. */
		uint8_t DAC_PWDWN		: 1;
		uint8_t RESERVED7_1		: 7;
		/* When set to 1, this bit disables the device internal reference. */
		uint8_t REF_PWDWN		: 1;
		uint8_t RESERVED15_9	: 7;
	};	
	
} DACx0501_Config_t;

/*
 * @brief GAIN register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	
	uint16_t Gain;
	struct {
		uint8_t GainLSB;
		uint8_t GainMSB;
	};
	struct {
		/* When set to 1, the buffer amplifier for corresponding DAC has a gain of 2. When cleared to 0, the buffer amplifier 
		 * for corresponding DAC has a gain of 1. Default is 1. */
		uint8_t BUFF_GAIN		: 1;
		uint8_t RESERVED7_1		: 7;
		/* When REF-DIV set to 1, the reference voltage is internally divided by a factor of 2. When REF-DIV is cleared to 0, 
		 * the reference voltage is unaffected. Default is 0. The reference voltage to the device (either from the internal or
		 * external reference) can be divided by a factor of two by setting the REF-DIV bit to 1. Make sure to configure REF-DIV
		 * so that there is sufficient headroom from VDD to the DAC operating reference voltage. Improper configuration of the 
		 * reference divider triggers a reference alarm condition. In the case of an alarm condition, the reference buffer is 
		 * shut down, and all the DAC outputs go to 0 V. The DAC data registers are unaffected by the alarm condition, and thus
		 * enable the DAC output to return to normal operation after the reference divider is configured correctly.*/
		uint8_t REF_DIV			: 1;
		uint8_t RESERVED15_9	: 7;
	};
	
} DACx0501_Gain_t;


/*
 * @brief TRIGGER register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed))  {
	
	uint16_t Trigger;
	struct {
		uint8_t TriggerLSB;
		uint8_t TriggerMSB;
	};
	struct {
		/* When set to the reserved code of 1010, this bit resets the device to the default state. These bits are self 
		 * resetting. */
		uint8_t SOFT_RESET		: 4;
		/* Set this bit to 1 to synchronously load the DAC in synchronous mode, this bit is self resetting. */
		uint8_t LDAC			: 1;
		uint8_t RESERVED7_5		: 3;
		uint8_t RESERVED_15_8	: 8;
	};
	
} DACx0501_Trigger_t;

/*
 * @brief STATUS register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed))  {
	
	uint16_t Status;
	struct {
		uint8_t StatusLSB;
		uint8_t StatusMSB;
	};
	struct {
		/* REF-ALARM bit. Reads 1 when the difference between the reference and supply pins is below a minimum analog 
		 * threshold. Reads 0 otherwise. When 1, the reference buffer is shut down, and the DAC outputs are all zero 
		 * volts. The DAC codes are unaffected, and the DAC output returns to normal when the difference is above the 
		 * analog threshold. */
		uint8_t REF_ALARM		: 1;
		uint8_t RESERVED7_1		: 7;
		uint8_t RESERVED15_8	: 8;
	};
	
} DACx0501_Status_t;

/*
 * @brief DAC register structure.
 *
 **/
typedef union __attribute__((aligned(1), packed))  {
	
	/* DAC data register. Default 0000h for DACx0501Z, 8000h for DACx0501M. Data are MSB aligned in straight binary 
	 * format, and use the following format: */
	uint16_t Data;	
	struct {
		uint8_t DataLSB;
		uint8_t DataMSB;
	};
	
   /* For 16 bit DAC. */
	struct {
		uint16_t DAC80501_DATA : 16;
	};
	/* For 14 bit DAC. */
	struct {
		uint16_t RESERVED1_0	: 2;
		uint16_t DAC70501_DATA	: 14;
	};
	/* For 12 bit DAC. */
	struct {
		uint16_t RESERVED3_0	: 4;
		uint16_t DAC60501_DATA	: 12;
	};
	
} DACx0501_DACReg_t;

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	uint8_t Command;
	uint8_t DataMSB;
	uint8_t DataLSB;
	delay_fptr delay;
	spi_txrx_fptr spi_tx;
	i2c_txrx_fptr i2c_rx;
	i2c_txrx_fptr i2c_tx;

} DACx0501_GInst_t;

/* Public function prototypes. */

void DAC60501_SetVoltage(float voltage);


/* SPI functions. */
void DACx0501_SPI_Sync(DACx0501_GInst_t *device, uint8_t dac_sync);
void DACx0501_SPI_Config(DACx0501_GInst_t *device, uint8_t dac_pwdwn, uint8_t ref_pwdwn);
void DACx0501_SPI_Gain(DACx0501_GInst_t *device, uint8_t buff_gain, uint8_t ref_div);
void DACx0501_SPI_Trigger(DACx0501_GInst_t *device, uint8_t soft_reset, uint8_t ldac);
void DACx0501_SPI_WriteData(DACx0501_GInst_t *device, uint8_t dac_type, uint16_t dac_data);

/* I2C functions. */
void DACx0501_I2C_Sync(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_sync);
void DACx0501_I2C_Config(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_pwdwn, uint8_t ref_pwdwn);
void DACx0501_I2C_Gain(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t buff_gain, uint8_t ref_div);
void DACx0501_I2C_Trigger(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t soft_reset, uint8_t ldac);
void DACx0501_I2C_WriteData(DACx0501_GInst_t *device, DACx0501_I2C_Addr address, uint8_t dac_type, uint16_t dac_data);

DACx0501_Devid_t DACx0501_I2C_ReadDevId(DACx0501_GInst_t *device, DACx0501_I2C_Addr address);
DACx0501_Status_t DACx0501_I2C_ReadStatus(DACx0501_GInst_t *device, DACx0501_I2C_Addr address);

/* Hardware dependent function prototypes. */
void DACx0501_SPI_Tx(uint8_t *pData, uint8_t size);


#endif /* DACx0501_H_ */