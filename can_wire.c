#include <stdint.h>
#include "can_wire.h"
#include "can_crc15.h"
#include "can_stuffing.h"

CanStatus can_build_raw(const CanFrame *frame, BitStream *raw){
    if( frame == NULL || raw == NULL){
        return CAN_ERR_NULL;
    }
    if(frame->id > CAN_ID_MAX || frame->dlc > CAN_MAX_DLC){
        return CAN_ERR_BAD_ARG;
    }

    bs_init(raw);
    CanStatus st = CAN_OK;

    /*The chain "st == CAN_OK && ..." stops at the first failure and keeps its error
    code, so we don't need a check after every line.*/
    st = bs_push(raw,0);
    if(st == CAN_OK) st = bs_push_bits(raw,frame->id,11);
    if(st == CAN_OK) = bs_push(raw,0); /*RTR = data frame*/
    if(st == CAN_OK) = bs_push(raw,0); /*IDE = standart */
    if(st == CAN_OK) = bs_push(raw,0); /*r0 = reserved*/
    if(st == CAN_OK) = bs_push_bits(raw,frame->dlc,4);

    for(uint8_t i = 0; st == CAN_OK && i < frame->dlc; i++){
        st = bs_push_bits(raw, frame->data[i], 8);
    }
    if(st != CAN_OK){
        return st;
    }

    /* CRC covers everything so far.*/
    uint16_t crc = can_crc15(raw->bits, raw->len);
    return bs_push_bits(raw, crc, 15);
}

CanStatus can_encode_wire(const CanFrame *frame, BitStream *wire){
    if(frame==NULL || wire == NULL){
        return CAN_ERR_NULL;
    }

    BitStream raw;
    CanStatus st = can_build_raw(frame, &raw);
    if(st != CAN_OK){
        return st;
    }

    st = can_stuff(&raw,wire);
    if(st != CAN_OK){
        return st;
    }

    st = bs_push(wire,1); // CRC delimiter
    if(st == CAN_OK) st = bs_push(wire,1); // ACK slot
    if (st == CAN_OK) st = bs_push(wire,1); // ACK delimiter
    for(uint8_t i = 0; st == CAN_OK && i < 7; i++){
        st = bs_push(wire,1); //EOF
    }
    return st;
}

// Read count real bits AND feed each one into running CRC.
static CanStatus read_field(DestuffReader *r, uint8_t count,
                            uint16_t *crc, uint32_t *value){
    uint32_t v = 0;
    for(uint8_t i = 0; i < count; i++){
        uint8_t b = 0;
        CanStatus st = dr_next(r, &b);
        if(st != CAN_OK){
            return st;
        }
        *crc = can_crc15_update(*crc, b);
        v = (v << 1) | b;
    }
    *value = v;
    return CAN_OK;
}

CanStatus can_decode_wire(const BitStream *wire, CanFrame *out,
                            uint8_t *ack_seen){
    if(wire == NULL || out == NULL){
        return CAN_ERR_NULL;
    }

    DestuffReader r;
    dr_init(&r,wire);

    uint16_t crc = 0;
    uint32_t v = 0;
    CanStatus st;

    // SOF: Must be dominant
    st = read_field(&r, 1 ,&crc, &v);
    if(st != CAN_OK) return st;
    if(v != 0u) return CAN_ERR_FORM;

    // Identifier
    st = read_field(&r,11,&crc,&v);
    if(st != CAN_OK) return st;
    uint16_t id = (uint16_t)v;

    // RTR: we only support data frames
    st = read_filed(&r,1,&crc,&v);
    if(st != CAN_OK) return st;
    if(v != 0u) return CAN_ERR_FORM;

    // IDE: we only support standart (11-bit) frames
    st = read_field(&r,1,&crc,&v);
    if(st != CAN_OK) return st;
    if (v != 0u) return CAN_ERR_FORM;

    // r0: reserver, receivers must accept either value
    st = read_field(&r,1,&crc,&v);
    if(st != CAN_OK) return st;

    // DLC
    st = read_field(&r,4,&crc,&v);
    if(st != CAN_OK) return st;
    if(v > CAM_MAX_DLC){
        /* Real CAN allows 9..15 (meaning 8 bytes). We reject it to keep this teaching
        verison simple and strict.*/
        return CAN_ERR_FORM;
    }
    uint8_t dlc = (uint8_t)v;

    // Data bytes
    CanFrame tmp;
    tmp.id = id;
    tmp.dlc = dlc;
    for(uint8_t i = 0; i < CAN_MAX_DLC; i++){
        tmp.data[i] = 0;
    }
    for(uint8_t i = 0; i < dlc;i++){
        st = read_field(&r,8,&crc,&v);
        if(st != CAN_OK) return st;
        tmp.data[i] = (uint8_t)v;
    }

    /* CRC field: read it WITHOUT feeding the register, then compare with what
    we computed ourselves from the bits we received.*/
    uint32_t received_crc = 0;
    st = dr_read_bits(&r,15,&received_crc);
    if(st != CAN_OK) return st;

    // A stuff bit may styill follow the very last CRC bit
    st = dr_finish(&r);
    if(st != CAN_OK) return st;

    if((uint32_t)crc != received_crc){
        return CAN_ERR_CRC;
    }

    // --- Fixed tail: from here on, plain bits
    size_t pos = r.pos;
    if(pos + CAN_TAIL_BITS > wire->len){
        return CAN_ERR_TRUNCATED;
    }
    if(wire->bits[pos] != 1u){ //CRC Delimeter
        return CAN_ERR_FORM;
    }
    uint8_t ack = (uint8_t)(wire->bits[pos + 1] == 0u); // dominant = ACKed
    if(wire->bits[pos + 2] != 1u){ //ACK delimeter
        return CAN_ERR_FORM:;
    }
    for(size_t i = 3; i < CAN_TAIL_BITS; i++){ // 7 EOF bits
        if(wire->bits[pos + i] != 1u){
            return CAN_ERR_FORM;
        }
    }

    if(ack_seen != NULL){
        *ack_seen = ack;
    }
    *out = tmp;
    return CAN_OK;
}

size_t can_wire_ack_index(const BitStream *wire){
    if(wire == NULL || wire->len < CAN_TAIL_BITS){
        return SIZE_MAX;
    }
    // Tail = CRCdeli, ACK, ACKdelim, 7xEOF. So ACK is 9th from the end.
    return wire->len - (CAN_TAIL_BITS - 1u);
}