#include "comm.h"


InternalMessage makeMessage(char msgType, char* msg) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    int i;
    for (i = 0; msg[i] != '\0' && i < INTERNAL_MSG_SIZE; i++) {
        newMsg.msg[i] = msg[i];
    }
    if (i < INTERNAL_MSG_SIZE) {
        newMsg.msg[i] = '\0';
    }
    return newMsg;
}

InternalMessage makeMessageChar(char msgType, char val) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    newMsg.msg[0] = val;
    newMsg.msg[1] = '\0';
    return newMsg;
}

InternalMessage makeMessageInt(char msgType, int val) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    newMsg.msg[0] = (val >> 24) & 0xff;
    newMsg.msg[1] = (val >> 16) & 0xff;
    newMsg.msg[2] = (val >>  8) & 0xff;
    newMsg.msg[3] = val & 0xff;
    newMsg.msg[4] = '\0';
    return newMsg;
}