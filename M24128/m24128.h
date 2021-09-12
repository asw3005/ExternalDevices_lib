/*
 * @brief External eprom memory header.
 *
 **/


#ifndef M24128_H_
#define M24128_H_

#include "stm32f1xx_hal.h"

/* Page's addreses. */
#define EEP_DATA_ADDR			0x50
#define	EEP_DATA_ADDR_SHIFTED	0xA0
#define EEP_ID_PAGE				0x58
#define EEP_ID_PAGE_SHIFTED		0xB0

/*
 * @brief Page's codes.
 *
 **/
typedef enum
{
	ST_MANUFACTURER_CODE	= 0x00,
	I2C_FAMILY_CODE			= 0x01,
	MEMORY_DENSITY_CODE		= 0x02,
	M24128_ERT				= 0x04
	
} M24128_IDPageAddr;

/*
 * @brief Internal memory organization struct.
 *
 **/
typedef	union __attribute__((aligned(1), packed)) {
	uint16_t MemAddr;
	struct {
		uint8_t BYTE_ADDR;
		uint8_t PAGE_ADDR;			
	};	
} M24128_MemAddr_t;


#endif /* M24128_H_ */