/* 
    In the previous level we worked at the APPLICATION layer: we had a frame with
    an ID and bytes, and never asked "how does this actually travelover two wires?"
    This level goes DOWN one layer and builds the real bit sequence that a CAN controller puts
    on the bus.
    
    On real hardware all of this is done by a dedicated hardware block - your C code
    only hands over an ID and some bytes. We write it in software ON PURPOSE, to understand
    exactly what that hardware block is doing for you.
*/
#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>
#include <stddef.h>

#define CAN_MAX_DLC 8
#define CAN_ID_MAX 0x7FFu

typedef struct{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DLC];
} CanFrame;

typedef enum {
    CAN_OK = 0,
    CAN_ERR_NULL, // a required pointer was NULL
    CAN_ERR_BAD_ARG, // value outside what the procotol allows
    CAN_ERR_OVERFLOW, // bit buffer too small
    CAN_ERR_TRUNCATED, //ran out of bits while parsing
    CAN_ERR_STUFF, // stuff error: 6 identical bits in a row
    CAN_ERR_CRC, // CRC-15 mismatch
    CAN_ERR_FORM //a fixed-format bit had the wrong value
} CanStatus;

const char *can_status_str(CanStatus s);

#endif