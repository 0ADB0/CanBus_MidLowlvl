/*
    Turns a CanFrame into the exact bit sequence sent on the bus, and back again.
    Layout of a classic CAN 2.0A DATA frame:

        SOF | ID (11) | RTR | IDE | r0 | DLC (4) | DATA (0-64) | CRC (15)
        |<----------------- stuffed region ------------------------>|
        CRC delim | ACK slot | ACK delim | EOF (7 x recessive)
        |<----------------- fixed tail, 10 bits, NOT stuffed ------>|

        SOF  start of frame, always 0 (dominant)
        RTR  0 for a data frame (1 would mean "remote request")
        IDE  0 for standard 11-bit IDs (1 would mean extended 29-bit IDs)
        r0   reserved, sent as 0
        ACK  the sender transmits 1; ANY receiver that got the frame
                correctly overwrites it with 0 (dominant). If it stays 1,
            nobody heard us -> the sender flags an ACK error.
*/
#ifndef CAN_WIRE_H
#define CAN_WIRE_H

#include "can_bitstream.h"

//Number of bits in the fixed, unstuffed tail.
#define CAN_TAIL_BITS 10u

// Build the UNSTUFFED bits SOF..CRC 
CanStatus can_build_raw(const CanFrame *frame, BitStream *raw);

// Full encode: raw -> stuff -> append fixed tail.
CanStatus can_encode_wire(const CanFrame *frame, BitStream *wire);

/*
    Full decode. `ack_seen` (may be NULL) is set to 1 if the ACK slot was dominant
    (someone acknowledged), 0 otherwise.
*/
CanStatus can_decode_wire(const BitStream *wire, CanFrame *out,
                        uint8_t *ack_seen);

// Index of the ACK slot inside a wire stream, or SIZE_MAX if too short.
size_t can_wire_ack_index(const BitStream *wire);

#endif //CAN_WIRE_H