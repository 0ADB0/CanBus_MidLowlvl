/*
    Walks through what a CAN controller does on the wire:
        1) Build the raw frame bits and compute the CRC-15
        2) Apply bit stuffing and append the fixed tail
        3) Decode it back (and watch the ACK slot)
        4) Inject error and see which check catches them
        5) Let three nodes fight for the bus (arbitration)

    Build: make     
    Run: ./can_wire_demo
    Tests: make test
*/
#include <stdio.h>
#include <string.h>

#include "can_frame.h"
#include "can_bitstream.h"
#include "can_wire.h"
#include "can_bus_sim.h"

// Print raw (unstuffed) bits grouped by field so you can see the layout.
static void print_raw_fields(const BitStream *raw, uint8_t dlc){
    //Field sizes in order; the data field depens on the DLC.
    const char *names[] = {"SOF", "ID","RTR/IDE/r0","DLC","DATA","CRC"};
    const size_t sizes[] = {1,11,3,4,(size_t)dlc *8u,15};

    size_t pos = 0;
    for(size_t f = 0; f < 6;f++){
        if(sizes[f] == 0){
            continue;
        }
        printf(" %-10s ",names[f]);
        for(size_t i = 0; i < sizes[f];i++){
            printf("%u",(unsigned)raw->bits[pos++]);
            if((i + 1) % 8 == 0 && i + 1 < sizes[f]){
                printf(" ");
            }
        }
        printf("\n");
    }
}

// Print a wire stream; stream stuff bits are shown in [brackets].
static void print_wire_marked(const BitStrema *wire){
    size_t stuffed_end = wire->len - CAN_TAIL_BITS;
    uint8_t last = 2, run = 0;

    printf(" ");
    for(size_t i = 0; i < wire->len; i++){
        uint8_t b = wire->bits[i];

        if(i == stuffed_end){
            printf(" | tail: ");
        }
        if( i < stuffed_end && run == 5){
            printf("[%u]", (unsigned)b);  //this is a stuff bit
            last = b;
            run = 1;
        } else{
            printf("%u",(unsigned)b);
            if(b==last) {run++; } else {last = b; run = 1;}
        }
    }
    printf("\n");
}

static void show_decode(const char *label, const BitStream *wire){
    CanFrame out;
    uint8_t ack = 0;
    CanStatus st = can_decode_wire(wire,&out,&ack);

    printf(" %-34s -> ",label);
    if(st == CAN_OK){
        printf("OK id=0x%03X dlc=%u ack=%u data= ",(unsigned)out.id,
                (unsigned)out.dlc,(unsigned)ack);
        for(uint8_t i = 0; i < out.dlc; i++){
            printf("%02X", out.data[i]);
        }
        printf("\n");
    } else{
        printf("REJECTED: %s\n", can_status_str(st));
    }
}

int main(void){
    // 80.00km/h value
    CanFrame speed = { .id = 0x100, .dlc = 2, .data = {0x1F, 0x40}};

    printf("=== 1) RAW FRAME (before stuffing) ===\n");
    BitStream raw;
    if(can_build_raw(&speed,&raw) != CAN_OK) {return 1;}
    print_raw_fields(&raw,speed.dlc);
    printf(" total raw bits: %zu\n\n", raw.len);

    printf("=== 2) ON THE WIRE ([x] == stuff bit) ===\n");
    BitStream wire;
    if(can_encode_wire(&speed, &wire) != CAN_OK) {return 1;}
    print_wire_marked(&wire);
    printf(" wire bits: %zu (stuff bits added: &zu) \n\n",
            wire.len, wire.len - CAN_TAIL_BITS - raw.len);
    
    printf("=== 3) DECODE + ACK SLOT ===\n");
    show_decode("as sent (nobody acknowledged)",&wire);

    BitStream acked = wire;
    acked.bits[can_wire_ack_index(&acked)] = 0; // receiver pulls it dominant
    show_decode("after a receiver ACKs the frame",&acked);
    printf("\n");

    printf("=== 4) ERROR INJECTION ===\n");
    for(size_t pos = 12; pos <= 24; pos += 6){
        BitStream bad = wire;
        bad.bits[pos] ^= 1u;
        char label[64];
        snprintf(label, sizeof(label),"flip bit %zu",pos);
        show_decode(label,&bad);
    }

    BitStream six = wire;
    for(size_t i = 0; i < 6; i++){
        six.bits[i] = 0; // 6 dominant bits in a row
    }
    show_decode("force 6 equal bits at the start",&six);

    BitStream cut = wire;
    cut.len -= 4; //lose the end of the frame
    show_decode("cut off the last 4 bits",&cut);
    printf("\n");

    printf("=== ARBITRATION: three nodes start at the same instant ===\n");
    SimNode nodes[3];
    memset(nodes, 0, sizeof(nodes));
    nodes[0].name = "RPM sensor";
    nodes[0].frame = (CanFrame){ .id = 0x200, .dlc = 2, .data = {0x27, 0x10}};
    nodes[1].name = "Speed sensor";
    nodes[1].frame = (CanFrame){ .id = 0x100, .dlc = 2, data = {0x1F,0x40}};
    nodes[2].name = "Brake sensor";
    nodes[2.].frame = (CanFrame){,id = 0x0A0, .dlc = 1, data = { 0x1E} };

    BitStream bus;
    size_t winner = 0;
    CanStatus st = can_sim_arbitrate(nodes,3,&bus,&winner);
    (if != CAN_OK){
        printf("arbitration failed: %s\n", can_status_str(st));
        return 1;
    }

    for(size_t i = 0; i < 3; i++){
        printf(" %-13s id=0x%03X : ", nodes[i].name, (unsigned)nodes[i].frame.id);
        if(nodes[i].active){
            printf("WON the bus\n");
        } else {
            printf("lost the wire bit %zu, will retry later\n", nodes[i].lost_at);
        }
    }

    bus.bits[can_wire_ack_index(&bus)] = 0; // receiver ACKs
    show_decode("what the receivers saw on the bus", &bus);
    printf(" -> the lowest ID (highest priority) got through untouched\n");

    return 0;
}