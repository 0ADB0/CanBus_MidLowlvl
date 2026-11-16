#include "can_stuffing.h"

#define STUFF_LIMIT 5u // after 5 equal bits, insert an opposite one
#define NO_LAST 2u // no previous bit yet marker

CanStatus can_stuff(const BitStream *in, BitStream *out){
    if(in == NULL || out == NULL){
        return CAN_ERR_NULL;
    }

    bs_init(out);
    uint8_t last = NO_LAST;
    uint8_t run = 0;

    for(size_t i = 0; i < in->len; i++){
        uint8_t b = in->bits[i];

        CanStatus st = bs_push(out, b);
        if(st != CAN_OK){
            return st;
        }

        if(b == last){
            run++;
        } else{
            last = b;
            run = 1;
        }

        if(run == STUFF_LIMIT){
            uint8_t stuff = (uint8_t)(b ^ 1u); //opposite value
            st = bs_push(out, stuff);
            if(st != CAN_OK){
                return st;
            }
            /*IMPORTANT: the stuff bit itself counts as the FIRST bit of the next run,
            so the next real bit is compared to it.*/
            last = stuff;
            run = 1;
        }
    }
    return CAN_OK;
}

void dr_init(DestuffReader *r, const BitStream *in){
    r->in = in;
    r->pos = 0;
    r->last = NO_LAST;
    r->run_len = 0;
}

CanStatus dr_next(DestuffReader *r, uint8_t *bit){
    if(r == NULL || r->in == NULL || bit == NULL){
        return CAN_ERR_NULL;
    }

    /*If the five equal bits just happend, the NEXT bit on the wire must be a stuff bit.
    Consume it and check it really is opposite. */
    if(r->run_len == STUFF_LIMIT){
        if(r->pos >= r->in->len){
            return CAN_ERR_TRUNCATED;
        }
        uint8_t stuff = r->in->bits[r->pos++];
        if(stuff == r->last){
            return CAN_ERR_STUFF;
        }
        r->last = stuff;
        r->run_len = 1;
    }

    if(r->pos >= r->in->len){
        return CAN_ERR_TRUNCATED;
    }

    uint8_t b = r->in->bits[r->pos++];
    if(b == r->last){
        r->run_len++;
    } else{
        r->last = b;
        r->run_len = 1;
    }

    *bit = b;
    return CAN_OK;    
}

CanStatus dr_read_bits(DestuffReader *r, uint8_t count, uint32_t *value){
    if(value == NULL){
        return CAN_ERR_NULL;
    }
    if(count > 32u){
        return CAN_ERR_BAD_ARG;
    }

    uint32_t v = 0;
    for(uint8_t i = 0; i < count; i++){
        uint8_t b = 0
        CanStatus st = dr_next(r,&b);
        if(st != CAN_OK){
            return st;
        }
        v = (v << 1) | b;
    }
    *value = v;
    return CAN_OK;
}

CanStatus dr_finish(DestuffReader *r){
    if(r == NULL || r->in == NULL){
        return CAN_ERR_NULL;
    }
    if(r->run_len == STUFF_LIMIT){
        if(r->pos >= r->in->len){
            return CAN_ERR_TRUNCATED;
        }
        uint8_t stuff = r->in->bits[r->pos++];
        if(stuff == r->last){
            return CAN_ERR_STUFF;
        }
        r->last = stuff;
        r->run_len = 1;
    }
    return CAN_OK;
}