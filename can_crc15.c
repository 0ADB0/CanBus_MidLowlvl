#include "can_crc15.h"

#define CAN_CRC15_POLY 0x4599u
#define CAN_CRC15_MASK 0x7FFFu /* keep only 15 bits */

uint16_t can_crc15_update(uint16_t crc, uint8_t bit){
    /* Compare the incoming bit with the register's top bit. If they differ,
    the polynomial gets XORed in. This is a hardware-friendly way to do polynomial 
    long division one bit at a time - a shift register plus a few XOR gates in silicon.*/
    uint16_t top = (uint16_t)((crc >> 14) & 1u);
    uint16_t next = (uint16_t)((bit & 1u) ^ top);

    crc = (uint16_t)((crc << 1) & CAN_CRC15_MASK);
    if (next != 0u){
        crc ^= CAN_CRC15_POLY;
    }
    return crc;
}

uint16_t can_crc15(const uint8_t *bits, size_t count){
    uint16_t crc = 0;
    for(size_t i = 0; i < count; i++){
        crc = can_crc15_update(crc, bits[i]);
    }
    return crc;
}