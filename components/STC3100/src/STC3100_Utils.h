//
// Created by Maxim Dobryakov on 19/10/2020.
//

#ifndef STC3100_STC3100_UTILS_H
#define STC3100_STC3100_UTILS_H

#include <stdint.h>

uint8_t crc8(const uint8_t *data, uint8_t dataLength);

uint16_t convertToUInt16(uint8_t high, uint8_t low, uint16_t mask);
int16_t convertToInt16(uint8_t high, uint8_t low, uint16_t mask);

#endif //STC3100_STC3100_UTILS_H
