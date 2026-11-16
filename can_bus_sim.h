/*  Simulates several nodes starting to transmit AT THE SAME TIME
on one shared wire pair, and shows how CAN decides who may continue.

    The physical rule("wired-AND"):
       If ANY node drives a 0 (dominant), hte bus reads 0.
       The bus only reaqds 1 if EVERY node drives 1(recessive).

    
    Arbitration: while sending, each node also READs the bus. If a node sends a 1 
    but reads a 0, someone else is sending a dominant bit at the same moment -> it 
    has LOST and must stop at once end retry later. Because the ID is sent first, MSB first,
    the node with the LOWEST ID keeps winning. This is why a low ID means a HIGH priority, and
    nothing is ever destroyed: the winner's frame passes through intact.*/

#ifndef CAN_BUS_SIM_H
#define CAN_BUS_SIM_H

#include "can_wire.h"  

typedef struct {
    const char *name; 
    CanFrame frame; //What this node wants to send
    BitStream  wire; //Its full bit sequence
    uint8_t active; //still transmitting?
    size_t lost_at; // bit index where it lost 
} SimNode;

/*
    Runs arbitration. on return:
        - nodes[i].active tells who survived
        - nodes[i].lost_at tells where each loser dropped out
        - bus_out holds what actually appeared on the bus
        - *winner receives the index of the winning node
*/
CanStatus can_sim_arbitrate(SimNode *nodes, size_t count,
                            BitStream *but_out, size_t *winner);

#endif