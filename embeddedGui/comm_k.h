#ifndef COMM_K_H
#define COMM_K_H

/*
 * File:   comm_k.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 4:48 AM
 */

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#include <QMetaType>

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
#define GUI                 0x14

#define OBS1                0x15
#define OBS2                0x16
#define OBS3                0x17
#define OBS4                0x18
#define TOK1                0x19
#define TOK2                0x20
#define TOK3                0x21
#define TOK4                0x22


#define T1  0
#define T2  1
#define T3  2
#define T4  3
#define O1  4
#define O2  5
#define O3  6
#define O4  7

///////////////////////////////////////////////////////////////////////////////
// TO DO - Define This Device's Role
///////////////////////////////////////////////////////////////////////////////
#define MY_ROLE MONITOR

///////////////////////////////////////////////////////////////////////////////
// Queue sizes
///////////////////////////////////////////////////////////////////////////////

// TX APP Queues
#define WIFLY_TX_Q_SIZE         16 //
#define WIFLY_RX_Q_SIZE         64 // bytes
#define PI_TX_Q_SIZE            16 // messages
#define PI_RX_Q_SIZE            16 //

// TX Interupt Queues
#define OUTGOING_BYTE_Q         16 // bytes
#define IN_BUF_SIZE             16 // messages

// RX APP Queues

// RX Interupt Queues

// SysMesg Queues
#define SYS_MSG_Q_SIZE          16 // SysMessages

///////////////////////////////////////////////////////////////////////////////
// Message sizes
///////////////////////////////////////////////////////////////////////////////

#define HEADER_SIZE             5 // bytes
#define SYS_MSG_PAYLOAD_SIZE    12 // bytes
#define SYS_MSG_SIZE            13 // bytes (one TYPE byte plus SYS PAYLOAD)
#define RPI_MSG_SIZE            14 // bytes (SYS_MSG_SIZE plus one)
#define MAX_MSG_COUNT           255 // messages

// Flagged for removal:
#define NET_MSG_SIZE      HEADER_SIZE + INTERNAL_MSG_SIZE + TAIL_SIZE
#define TAIL_SIZE               1 // bytes

///////////////////////////////////////////////////////////////////////////////
// NetMessage - Message Structure for network communication
//  0 Start Byte
//  1 Sender
//  2 Message number
//  3 Message Type
//  4 Message Size
//  5 - 16 Message
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    char sender;
    char number;
    char type;
    char msgsize;
    char msg[SYS_MSG_PAYLOAD_SIZE];
    // add end byte
} NetMessage;

///////////////////////////////////////////////////////////////////////////////
// Special Characters
///////////////////////////////////////////////////////////////////////////////
#define START_BYTE 0x00
#define END_BYTE   0xff

#define OBSTACLE    0xA1
#define TOKEN       0xA2
#define LEAD        0xA3
#define FOLLOW      0xA4

///////////////////////////////////////////////////////////////////////////////
// NetMessage Types
///////////////////////////////////////////////////////////////////////////////
#define DEBUG_MSG           0x01
#define CLIENT_ROLE         0x04
#define INITIALIZE          0x08
#define READY_TO_START      0x10

#define ROVER_MOVE          0x11
#define OBJECT_POS          0x14
#define LEAD_POS            0x15
#define FOLLOW_POS          0x16
#define TOKEN_POS           0x17
#define TOKEN_FOUND         0x18

#define INITIAL_ROUTE_S     0x19
#define INITIAL_ROUTE_D     0x20
#define INITIAL_ROUTE_E     0x21
#define INITIAL_OBJ_POS     0x22

#define SENSOR_MODE         0x61 // 0x01 - Data mode 0x02 - Data collection
#define CALIBRATE_ROVER     0x62

#define ROT_ERR_MAX         0x63
#define ROT_ERR             0x64
#define TRV_ERR_MAX         0x65
#define TRV_ERR             0x66

#define ACC_STATS           0x67
#define CLEAR_AC_STATS      0x68

#define TOKEN_PICKED        0x69
#define HEARTBEAT           0x70
#define END_GAME            0x71
#define START_GAME          0x72
#define SEND_NET_STATS      0x74
#define CLEAR_NET_STATS     0x75
#define NET_STAT            0x76
#define TOGGLE_DEBUG_LED    0x77
#define TEST_RPI_MSG        0x78
#define OUTPUT_TO_MONITOR   0x79

///////////////////////////////////////////////////////////////////////////////
// SysMsg - Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    uint8_t type;
    uint8_t msg[SYS_MSG_PAYLOAD_SIZE];

}SysMsg;

///////////////////////////////////////////////////////////////////////////////
// SysMsg Types
///////////////////////////////////////////////////////////////////////////////

// Uses same mesaage types as NetMessage

///////////////////////////////////////////////////////////////////////////////
// RpiMsg - Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    char source;
    char type;
    char size;
    char msg[SYS_MSG_PAYLOAD_SIZE];

}RpiMsg;

///////////////////////////////////////////////////////////////////////////////
// RpiMsg Types
///////////////////////////////////////////////////////////////////////////////

// Uses same message types as NetMessage

///////////////////////////////////////////////////////////////////////////////
// Link Status
///////////////////////////////////////////////////////////////////////////////
#define GREEN   0xBB
#define YELLOW  0xCC
#define RED     0xDD

typedef  QList<QPair<QString,QPointF> > initialList;

Q_DECLARE_METATYPE(RpiMsg)
Q_DECLARE_METATYPE(initialList)

///////////////////////////////////////////////////////////////////////////////
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
///////////////////////////////////////////////////////////////////////////////

SysMsg makeMessage(char msgType, char* msg);
SysMsg makeMessageChar(char msgType, char val);
SysMsg makeMessageInt(char msgType, int val);

///////////////////////////////////////////////////////////////////////////////
// ROVER_MOVE helpers
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

SysMsg makeRoverMove(char direction, char distance);

///////////////////////////////////////////////////////////////////////////////
// ROVER_MOVE defines
///////////////////////////////////////////////////////////////////////////////

#define ROVER_FORWARD      0x01
#define ROVER_BACKWARD     0x02
#define ROVER_LEFT         0x04
#define ROVER_RIGHT        0x08
#define ROVER_STOP         0x10


#endif // COMM_K_H
