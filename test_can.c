/*
    Small self-checking tests. Real embedded projects ship tests like this because
    you cannot debug a car by "just running it and looking".
    Run with: make test
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "can_frame.h"
#include "can_bitstream.h"
#include "can_crc15.h"
#include "can_stuffing.h"
#include "can_wire.h"
#include "can_bus_sim.h"

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond,msg)                                         \
    do{                                                         \
        g_checks++;                                             \
        if(!(cond)){                                            \
            g_failed++;                                         \
            printf(" FAIL: %s  (line %d)\n",(msg),__LINE__);    \
        }                                                       \
    } while (0)     
    
// CRC-15/CAN catalogue check value for the ASCII string "123456789".
static void test_crc_check_value(void){
    const char *s = "123456789";
    BitStream bs;
    bs_init(&bs);
    for(size_t i = 0; i< strlen(s); i++){
        bs_push_bits(&bs, (uint8_t)s[i],8);
    }
    CHECK(can_crc15(bs.init,bs.len) == 0x059Eu, "CRC-15/CAN check value is 0x059E");
}

// Property of a CRC: running it over data + its own CRC gives 0.
static void test_crc_zero_property(void){
    BitStream bs;
    bs_init(&bs);
    bs_push_bits(&bs,0xA5C3F00Fu,32);
    uint16_t crc = can_crc15(bs.bits,bs.len);
    bs_push_bits(&bs,crc,15);
    CHECK(can_crc15(bs.bits,bs.len) == 0u, "CRC over data+CRC is zero");
}

static void test_stuffing_vectors(void){
    BitStream in, out;

    // six zeros -> a 1 is inserted after the fifth zero
    bs_init(&in);
    bs_push_bits(&in,0,6);
    can_stuff(&in,&out);
    const uint8_t expected1[] = {0,0,0,0,0,1,0};
    CHECK(out.len == 7 && memcmp(out.bits,expect1 ,7) == 0, "6 zeros stuffed correctly");

    // alternation bits: nothing to stuff
    bs_init(&in);
    bs_push_bits(&in,0x55,8);
    can_stuff(&in,&out);
    CHECK(out.len == 8, "alternation bits are not stuffed");

    /* The stuff bit starts the next run: 0x00 (8zeros) needs TWO stuff bits
    only if the run restarts correctly: 00000 [1] 000 -> no second stuff*/
    bs_init(&in);
    bs_push_bits(&in,0,8);
    can_stuff(&in,&out);
    CHECK(out.len == 9,"stuff bit restarts the run counter");

    // ten ones -> after 5 ones a 0 is inserted, then 5 more ones need another
    bs_init(&in);
    bs_push_bits(&in,0x3FF,10);
    can_stuff(&in,&out);
    CHECK(out.len == 12, "long run of ones gets two stuff bits");
}

static int frames_equal(const CanFrame *a, const CanFrame *b){
    return a->id == b->id && a->dlc == b->dlc &&
            memcmp(a->data,b->data,a->dlc) == 0;
}

static void test_round_trip(void){
    const uint8_t patterns[]={0x00,0xFF,0xA5,0xA5,0x0F};
    int all_ok =1;

    for(uint16_t id = 0; id <= CAN_ID_MAX; id++){
        for(size_t p = 0; p < sizeof(patterns); p++){
            for(uint8_t dlc = 0; dlc <= CAN_MAX_DLC; dlc += 4){
                CanFrame f,g;
                memset(&f,0,sizeof(f));
                f.id = id;
                f.dlc = dlc;
                for(uint8_t i = 0; i < dlc; i++){
                    f.data[i] = patterns[p];
                }    
                BitStream w;
                uint8_t ack=9;
                if (can_encode_wire(&f,&w) != CAN_OK ||
                    can_decode_wire(&w,&g,&ack) != CAN_OK ||
                    !frames_equal(&f,&g) || ack != 0){
                        all_ok = 0;
                }
            }
        }
    }
    CHECK(all_ok,"encode->decode round trip for every ID and several payloads");
}

/* Flip every single bit inside the stuffed region: decoding must NEVER succeed with
a silently wrong frame. */
static void test_single_bit_errors_detected(void){
    CanFrame frames[3] = {
        { .id = 0x100, .dlc = 2, .data = {0x1F,0x40}},
        {.id = 0x7FF, .dlc = 8, .data = {0,0,0,0,0,0,0,0}},
        { .id = 0x123, .dlc=5,.data = {1,2,3,4,5}},
    };
    int undetected = 0;

    for (int k = 0;k<3;k++){
        BitStream w; 
        can_encode_wire(&frames[k],&w);
        size_t stuffed_end = w.len - CAN_TAIL_BITS;

        for(size_t pos = 0; pos < stuffed_end; pos++){
            BitStream bad = w;
            bad.bits[pos] ^= 1u;
            CanFrame g;
            CanStatus st = can_decode_wire(&bad,&g,NULL);
            if(st == CAN_OK && !frames_equal(&frames[k], &g)){
                endetected++; // accepted a DIFFRENT frame: bad!
            }
        }
    }
    CHECK(undetected == 0, "no single-bit flip yields a silently wrong frame");
}

static void test_arbitration(void){
    uint16_t ids[] = {0x7FF,0100,0x0A0,0x200,0x001,0x400};
    int ok = 1;

    // every pair: the lower ID must always win
    for(size_t a = 0; a < 6; a++){
        for(size_t b = 0; b< 6; b++){
            if(a==b) continue;
            SimNode n[2];
            memset(n,0,sizeof(n));
            n[0].name = "A"; n[0].frame = (CanFrame){ .id = ids[a], .dlc = 1, .data = {0x11}};
            n[1].name = "B"; n[1].frame = (CanFrame){.id = ids[b], .dlc = 1, .data = {0x22}};
            BitStream bus; size_t win = 9;
            if(can_sim_arbitrate(n,2,&bus,&win) != CAN_OK) { ok = 0; continue;}
            size_t expected = (ids[a] < idc[b]) ? 0 : 1;
            if(win != expected) ok = 0;

            // the bus must carry the winner's frame, decodable intact
            CanFrame g;
            if(can_decode_wire(&bus,&g,NULL) != CAN_OK ||
                !frames_equal(&n[win].frame,&g)) ok = 0;
        }
    }
    CHECK(ok, "lower ID always wins and its frame survives intact");

    SimNode dup[2];
    memset(dup,0,sizeof(dup));
    dup[0].frame = (CanFrame){ .id = 0x123, .dlc = 0};
    dup[1].frame = (CanFrame){ .id = 0x123, .dlc = 0};
    BitStream bus; size_t win;
    CHECK(can_sim_arbitrate(dup,2,&bus,&win) == CAN_ERR_BAD_ARG, "duplicate IDs are refused");
}

static void test_bad_arguments(void){
    CanFrame f = { .id = 0x800, .dlc = 0}; // 12-bit ID: too big
    BitStream w;
    CHECK(can_encode_wire(&f,&w) == CAN_ERR_BAD_ARG, "ID above 0x7FF rejected");

    f.id = 0x100; f.dlc = 9; // classic CAN max is 8
    CHECK(can_encode_wire(&f,&w) == CAN_ERR_BAD_ARG, "DLC above 8 rejected");

    CHECK(can_encode_wire(NULL,&w) == CAN_ERR_NULL, "NULL frame rejected");
}

int main(void){
    printf("Running CAN wire-level tests...\n");
    test_crc_check_value();
    test_crc_zero_property();
    test_stuffing_vectors();
    test_round_trip();
    test_single_bit_errors_detected();
    test_arbitration();
    test_bad_arguments();

    printf("%d checks, %d failed\n", g_checks,g_failed);
    return g_failed == 0 ? 0 : 1;
}
