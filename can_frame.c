#include "can_frame.h"

const char *can_status_str(CanStatus s){
    switch(s){
        case CAN_OK: return "OK";
        case CAN_ERR_NULL: return "null pointer";
        case CAN_ERR_BAD_ARG: return "bad argument";
        case CAN_ERR_OVERFLOW: return "bit buffer overflow";
        case CAN_ERR_TRUNCATED: return "frame truncated";
        case CAN_ERR_STUFF: return "STUFF error ( 6 equal bits )";
        case CAN_ERR_CRC: return "CRC error";
        case CAN_ERR_FORM: return "FORM error (fixed bit wrong)";
    }
    return "unknown status";
}