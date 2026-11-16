#include "can_bus_sim.h"

CanStatus can_sim_arbitrate(SimNode *nodes, size_t count,
                            BitStream *bus_out, size_t *winner){
    if(nodes == NULL || bus_out == NULL || winner == NULL){
        return CAN_ERR_NULL;
    }
    if(count == 0){
        return CAN_ERR_BAD_ARG;
    }

    /* Reject duplicate IDs then encode every node's frame. */
    for (size_t i = 0; i < count; i++){
        for(size_t j = i + 1;j < count; j++){
            if(nodes[i].frame.id == nodes[j].frame.id){
                return CAN_ERR_BAD_ARG;
            }
        } 
        CanStatus st = can_encode_wire(&nodes[i].frame, &nodes[i].wire);
        if(st != CAN_OK){
            return st;
        }
        nodes[i].active = 1;
        nodes[i].lost_at = 0;
    }

    bs_init(bus_out);

    size_t longest = 0;
    for(size_t i = 0; i < count; i++){
        if(nodes[i].wire.len > longest){
            longest = nodes[i].wire.len;
        }
    }

    size_t active_count = count;
    for(size_t bit = 0; bit < longest; bit++){
        /* 1) Wired-AND: start recessive (1), any dominant (0) pulls it down. */
        uint8_t bus = 1;
        for(size_t n = 0; n < count; n++){
            if(nodes[n].active && bit < nodes[n].wire.len){
                bus &= nodes[n].wire.bits[bit];
            }
        }

        /* 2) While MORE than on enode is active, each one compares what it sent
        with what it reads. Sent 1 but read 0 -> lost. */
        if(active_count > 1){
            for(size_t n = 0; n < count; n++){
                if(nodes[n].active && bit < nodes[n].wire.len){
                    if(nodes[n].wire.bits[bit] == 1u && bus == 0u){
                        nodes[n].active = 0;
                        nodes[n].lost_at = bit;
                        active_count--;
                    }
                }
            }
        }

        CanStatus st = bs_push(bus_out, bus);
        if(st != CAN_OK){
            return st;
        }
    }

    for (size_t n = 0; n < count; n++){
        if(nodes[n].active){
            *winner = n;
            /* The bus carried the WINNER's frame. Anything aftert its last bit is just 
            idle bus, so trim to the winner's lenght. Otherwise a caller computing positions
            from bus_out->len would be off. */
            bus_out->len = nodes[n].wire.len;
            return CAN_OK;
        }
    }
    return CAN_ERR_BAD_ARG;
}


