//
// Created by Maxim Dobryakov on 19/10/2020.
//

#include "STC3100_Utils.h"

uint8_t crc8(const uint8_t *data, uint8_t dataLength) {
    uint32_t crc = 0;
    for (uint8_t i = 0; i < dataLength; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x100)
                crc ^= 7;
        }
    }
    return crc & 0xFF;
}

uint16_t convertToUInt16(uint8_t high, uint8_t low, uint16_t mask) {
    return ((high << 8) | low) & mask;
}

int16_t convertToInt16(uint8_t high, uint8_t low, uint16_t mask) {
    uint16_t unsignedValue = convertToUInt16(high, low, mask);
    return unsignedValue > (mask >> 1) ? unsignedValue - (mask + 1) : unsignedValue;
}
