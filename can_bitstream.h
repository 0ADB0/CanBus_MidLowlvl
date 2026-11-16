/* 
    A sequence of individual bits. We store ONE BIT PER BYTE. That wastes memory, 
    but it makes the code easy to read and debug. A real controller shift single bits 
    through a hardware shift register instead.
    
    Bus levels in CAN:
       0 = DOMINANT
       1 = RECESSIVE
*/
#ifndef CAN_BITSTREAM_H
#define CAN_BITSTREAM_H

#include "can_frame.h"

/* Longest possible standart frame is ~135 bits including stuff bits.
   165 gives comfortable headroom. */
#define CAN_MAX_BITS 160

typedef struct {
    uint8_t bits[CAN_MAX_BITS]; /*each element is 0 or 1.*/
    size_t len; /*How many bits are valid.*/
} BitStream;

void    bs_init(BitStream *bs);

/* Append one bit (0 or 1)*/
CanStatus bs_push(BitStream *bs, uint8_t bit);

/* Append the lowest count bits of value, MOST significant bit first.
   Example: bs_push_bits(bs, 0x5,4) appends 0,1,0,1 */
CanStatus bs_push_bits(BitStream *bs, uint32_t value, uint8_t count);

#endif
