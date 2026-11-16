/* 
    The CRC used by CAN: 15 bits, polynomial x^15+x^14+x^10+x^8+x^7+x^4+x^3+1(hex 0x4599).
    Earlier we used CRC-8 at the application level; this is the CRC every CAN controller
    computed automatically for EVERY frame.
    
    It works BIT BY BIT, which is why we feed it single bits.
*/
#ifndef CAN_CRC15_H
#define CAN_CRC15_H

#include "can_frame.h"

/*Feed ONE bit into a running CRC register and return the new register. */
uint16_t can_crc15_update(uint16_t crc, uint8_t bit);

/* CRC over an array of bits (each element 0 or 1)*/
uint16_t can_crc15(const uint8_t *bits,size_t count);

#endif