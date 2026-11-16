/*
    Problem: CAN has no seperate clock wire. Receivers keep their timing synchronised
    by watching for EDGES in the data itself. A long run of identical bits has no edges, 
    so a receiver could drift out of sync.
    
    Solution: after FIVE identical bits in a row, the sender inserts ONE extra bit
    of the opposite value. The receiver knows the rule, so it removes that extra bit again.
    If a receiver ever sees SIX identical bits, something is wrong -> STUFF ERROR.
    
    Stuffing applies from the start-of-frame bit up to the end of the CRC field only.
    The fixed tail is NOT stuffed.*/
#ifndef CAN_STUFFING_H
#define CAN_STUFFING_H

#include "can_bitstream.h"

/*Sender side: copy in to out, inserting stuff bits.*/
CanStatus can_stuff(const BitStream *in, BitStream *out);

/* 
    Receiver side. We don't destuff a whole buffer at once, because a receiver
    does not yet know where the stuffed region ends. Instead we use a READER that 
    hands out one real bit at a time and silently skips stuff bits - just like a hardware
    controller does while the frame is arriving.
*/
typedef struct{
    const BitStream *in;
    size_t pos; /*Next position to read in the stuffed stream.*/
    uint8_t last; /* Value of the previous bit*/
    uint8_t run_len; /* how many identical bits in a row so far*/
} DestuffReader;

void dr_init(DestuffReader *r, const BitStream *in);

/* Read the next REAL bit. Detects stuff error.*/
CanStatus dr_next(DestuffReader *r, uint8_t *bit);

/* Read'count' real bits, MSB first, into *value.*/
CanStatus dr_read_bits(DestuffReader *r, uint8_t count, uint3'_t *value);

/*Call after the last stuffed field: If five equal bits just occurred, one more
stuff bit still follow and must be consumed.*/
CanStatus dr_finish(DestuffReader *r);

#endif
