/* 
 * File:   comm.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 4:48 AM
 */

#ifndef COMM_H
#define	COMM_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
    
///////////////////////////////////////////////////////////////////////////////
// Roles / Msg Sources
///////////////////////////////////////////////////////////////////////////////
#define CLIENT              0x01
#define LEAD_ROVER          0x02
#define FOLLOWER            0x04
#define SENSORS             0x08
#define COORDINATOR         0x10
#define MONITOR             0x11
#define ROUTER              0x12

///////////////////////////////////////////////////////////////////////////////    
// Define This Device's Role
///////////////////////////////////////////////////////////////////////////////
#define MY_ROLE CLIENT
    
///////////////////////////////////////////////////////////////////////////////
// Message and buffer sizes
///////////////////////////////////////////////////////////////////////////////
#define TX_BUF_SIZE       64 // bytes
#define RX_BUF_SIZE       64 // bytes
#define SENT_MSG_Q_SIZE   20 // messages
#define OUT_BUF_SIZE      16 // messages
#define IN_BUF_SIZE       16 // messages
#define HEADER_SIZE        5 // bytes
#define TAIL_SIZE          1 // bytes
#define INTERNAL_MSG_SIZE 12 // bytes
#define NET_MSG_SIZE      HEADER_SIZE + INTERNAL_MSG_SIZE + TAIL_SIZE
#define MAX_MSG_COUNT     255 // messages

///////////////////////////////////////////////////////////////////////////////
// Message Structure for network communication
//  0 Start Byte 
//  1 Sender
//  2 Message number
//  3 Message Type
//  4 Message Size
//  x Message  
// -1 End Byte
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    char sender;
    char number;
    char type;
    char msgsize;
    char msg[INTERNAL_MSG_SIZE];
} NetMessage;

///////////////////////////////////////////////////////////////////////////////
// Special Characters
///////////////////////////////////////////////////////////////////////////////
#define START_BYTE 0x00
#define END_BYTE   0xff

///////////////////////////////////////////////////////////////////////////////
// Message Types
///////////////////////////////////////////////////////////////////////////////
#define DEBUG_MSG           0x01
#define NET_STAT            0x02
#define CLIENT_ROLE         0x04
#define INITIALIZE          0x08
#define READY_TO_START      0x10
#define ROVER_MOVE          0x11
#define OBJECT_POS          0x14
#define TOKEN_FOUND         0x18

//////////////////////////////////////////////////////////////////////////////
// OBJECT_STRUCTURE defines
//////////////////////////////////////////////////////////////////////////////
#define ROVER       0xA0
#define OBSTACLE    0xA1


///////////////////////////////////////////////////////////////////////////////
// ROVER_MOVE defines
///////////////////////////////////////////////////////////////////////////////

#define ROVER_FORWARD      0x01
#define ROVER_BACKWARD     0x02
#define ROVER_LEFT         0x04
#define ROVER_RIGHT        0x08
#define ROVER_STOP         0x10

///////////////////////////////////////////////////////////////////////////////
// Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    char type;
    char size;
    char msg[INTERNAL_MSG_SIZE];
    
}InternalMessage;

InternalMessage makeMessage(char msgType, char* msg, char size);
InternalMessage makeMessageChar(char msgType, char val);
InternalMessage makeMessageInt(char msgType, int val);

///////////////////////////////////////////////////////////////////////////////
// ROVER_MOVE helpers
///////////////////////////////////////////////////////////////////////////////

InternalMessage makeRoverMove(char direction, char distance);

// Struct for the message structure of the messages I will receive
typedef struct
{
    char type;
    uint16_t xPos;
    uint16_t yPos;
    uint16_t angle;
    uint16_t length;
    uint16_t width;
} OBJECT_STRUCTURE;

InternalMessage makeLocationMessage(OBJECT_STRUCTURE obj);

#ifdef	__cplusplus
}
#endif

#endif	/* COMM_H */

