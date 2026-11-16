#include "can_bitstream.h"

void bs_init(BitStream *bs){
    if(bs != NULL){
        bs->len = 0;
    }
}

CanStatus bs_push(BitStream *bs, uint8_t bit){
    if(bs == NULL){
        return CAN_ERR_NULL;
    }
    if(bs->len >= CAN_MAX_BITS){
        return CAN_ERR_OVERFLOW;
    }
    bs->bits[bs->len++] = (uint8_t)(bit & 1u);
    return CAN_OK;
}

CanStatus bs_push_bits(BitStream *bs, uint32_t value, uint8_t count){
    if(bs == NULL){
        return CAN_ERR_NULL;
    }
    if(count > 32u){
        return CAN_ERR_BAD_ARG;
    }

    /* Walk from the highest requested bit down to bit 0.
    (value >> shift) & 1 isolates one bit - the same shift-and-mask
    idea you saw in the speed example, just one bit at a time. */
    for (uint8_t i = 0; i < count; i++){
        uint8_t shift = (uint8_t)(count - 1u - i);
        CanStatus st = bs_push(bs,(uint8_t)((value >> shift) & 1u));
        if(st != CAN_OK){
            return st;
        }
    }
    return CAN_OK;
}