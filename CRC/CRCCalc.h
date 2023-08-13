/*
 * @brief CRC header.
 *
 *  Created 03.24.22.
 *  Author: asw3005
 *
 */

#ifndef CRCCALC_H_
#define CRCCALC_H_

#include "stm32f1xx.h"

/* Public function prototypes. Polynom value is 0xD5 (x^8 + x^7 + x^6 + x^4 + x^2 + 1) */
uint8_t Soft_CRC8(const uint8_t* _buffer, const uint16_t _length);

/* Hardware crc unit used */
uint32_t Hard_CRC32ETH(uint8_t* _buffer, uint8_t _length);

#endif /* CRCCALC_H_ */
