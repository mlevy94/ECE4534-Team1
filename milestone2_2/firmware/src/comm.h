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
#define TAIL_SIZE          3 // bytes
#define INTERNAL_MSG_SIZE 12 // bytes
#define NET_MSG_SIZE      HEADER_SIZE + INTERNAL_MSG_SIZE + TAIL_SIZE

///////////////////////////////////////////////////////////////////////////////
// Message Structure for network communication
//  0 Start Byte 
//  1 Sender
//  2 Message number
//  3 Message Type
//  4 Message Size
//  x Message 
// -3 Checksum -- 2 7 bit bytes.  
// -1 End Byte
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Special Characters
///////////////////////////////////////////////////////////////////////////////
#define START_BYTE 0x00
#define END_BYTE   0xff

///////////////////////////////////////////////////////////////////////////////
// Message Types
///////////////////////////////////////////////////////////////////////////////
#define DEBUG_MSG           0x01
#define BAD_MSG             0x02 // makeMessage(BAD_MSG, 0);
#define MSG_REQUEST         0x04 // makeMessageChar(MSG_REQUEST, <msg#>);
#define CLIENT_ROLE         0x08
#define INITIALIZE          0x10
#define READY_TO_START      0x11
#define LEAD_PO             0x12
#define FOLLOW_PO           0x14
#define OBS_INFO            0x18
#define MOTOR_MOVE          0x20
#define TOKEN_FOUND         0x21

///////////////////////////////////////////////////////////////////////////////
// Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    char type;
    char msg[INTERNAL_MSG_SIZE];
    
}InternalMessage;

InternalMessage makeMessage(char msgType, char* msg);
InternalMessage makeMessageChar(char msgType, char val);
InternalMessage makeMessageInt(char msgType, int val);

#ifdef	__cplusplus
}
#endif

#endif	/* COMM_H */

