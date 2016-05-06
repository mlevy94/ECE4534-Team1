#include "comm.h"


InternalMessage makeMessage(char msgType, char* msg, char size) {
    InternalMessage newMsg;
    newMsg.type = msgType;
    newMsg.size = size;
    int i;
    for (i = 0; i < size; i++) {
        newMsg.msg[i] = msg[i];
    }
    return newMsg;
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

InternalMessage debugMessage(OBJECT_STRUCTURE obj)
{
    InternalMessage message;
    message.type = DEBUG_MSG;
    message.size = 11;
    message.msg[0] = obj.type;
    // MSB then LSB
    message.msg[1] = obj.xPos >> 8;
    message.msg[2] = obj.xPos & 0x00FF;
    
    message.msg[3] = obj.yPos >> 8;
    message.msg[4] = obj.yPos & 0x00FF;
    
    message.msg[5] = obj.angle >> 8;
    message.msg[6] = obj.angle & 0x00FF;
    
    message.msg[7] = obj.length >> 8;
    message.msg[8] = obj.length & 0x00FF;
    
    message.msg[9] = obj.width >> 8;
    message.msg[10] = obj.width & 0x00FF;
    
    return message;
}

InternalMessage makeLocationMessage(OBJECT_STRUCTURE obj)
{
    InternalMessage message;
    message.type = OBJECT_POS;
    message.size = 11;
    message.msg[0] = obj.type;
    // MSB then LSB
    message.msg[1] = obj.xPos >> 8;
    message.msg[2] = obj.xPos & 0x00FF;
    
    message.msg[3] = obj.yPos >> 8;
    message.msg[4] = obj.yPos & 0x00FF;
    
    message.msg[5] = obj.angle >> 8;
    message.msg[6] = obj.angle & 0x00FF;
    
    message.msg[7] = obj.length >> 8;
    message.msg[8] = obj.length & 0x00FF;
    
    message.msg[9] = obj.width >> 8;
    message.msg[10] = obj.width & 0x00FF;
    
    return message;
}

void convertMessage(InternalMessage message, OBJECT_STRUCTURE* obj)
{   
    obj->type = message.msg[0];
    obj->xPos = message.msg[1] << 8 | message.msg[2];
    obj->yPos = message.msg[3] << 8 | message.msg[4];
    obj->angle = message.msg[5] << 8 | message.msg[6];
    obj->length = message.msg[7] << 8 | message.msg[8];
    obj->width = message.msg[9] << 8 | message.msg[10];
}