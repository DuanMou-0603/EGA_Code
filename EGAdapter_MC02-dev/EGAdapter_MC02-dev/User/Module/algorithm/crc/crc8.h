#ifndef __CRC8_H
#define __CRC8_H

#include "main.h"

/**
 * 由于crc足够简单，直接写成c函数，不封装成class
 */

#define CRC_START_8 0x00

uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif
