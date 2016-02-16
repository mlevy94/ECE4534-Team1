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
