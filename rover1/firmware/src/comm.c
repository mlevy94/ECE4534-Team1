// DO NOT INCLUDE IN comm.h file
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "uart_tx_app_public.h"
// END OF DO NOT INCLUDE

#include "comm.h"



InternalMessage makeMessage(char msgType, char* msg, char size) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    int i;
    for (i = 0; i < size; i++) {
        newMsg.msg[i] = msg[i];
    }
    newMsg.msg[size] = '\0';
    newMsg.size = size;
    return newMsg;
}

void sendDebugMessage(char* msg) {
    InternalMessage newMsg;
    newMsg.type = DEBUG_MSG;
    int i;
    for (i = 0; msg[i] != '\0' && i < INTERNAL_MSG_SIZE; i++) {
        newMsg.msg[i] = msg[i];
    }
    newMsg.size = i;
    addToUartTXQ(newMsg);
}

InternalMessage makeMessageChar(char msgType, char val) {
    InternalMessage newMsg;
    newMsg.size = 1;
    newMsg.type = msgType;
    newMsg.msg[0] = val;
    return newMsg;
}

InternalMessage makeMessageInt(char msgType, int val) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    newMsg.size = 4;
    newMsg.msg[0] = (val >> 24) & 0xff;
    newMsg.msg[1] = (val >> 16) & 0xff;
    newMsg.msg[2] = (val >>  8) & 0xff;
    newMsg.msg[3] = val & 0xff;
    return newMsg;
}

InternalMessage makeRoverMove(char direction, char distance) {
    InternalMessage newMsg;
    newMsg.type = ROVER_MOVE;
    newMsg.size = 2;
    newMsg.msg[0] = direction;
    newMsg.msg[1] = distance;
    return newMsg;
}

InternalMessage roverStopped() {
    return makeRoverMove(ROVER_STOP, 0);
}
