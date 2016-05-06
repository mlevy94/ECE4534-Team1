/* 
 * File:   comm_k.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 4:48 AM
 */

#ifndef COMM_K_H
#define	COMM_K_H

#ifdef	__cplusplus
extern "C" {
#endif
    
bool readDebugLED();

///////////////////////////////////////////////////////////////////////////////
// Queue sizes
///////////////////////////////////////////////////////////////////////////////

// TX APP Queues
#define WIFLY_TX_Q_SIZE         16 // messages
#define PI_TX_Q_SIZE            16 // messages

// RX APP Queues
#define PI_RX_Q_SIZE            255  // bytes
#define WIFLY_RX_Q_SIZE         256  // bytes

// TX/RX Interupt Queues
#define OUTGOING_BYTE_Q         16 // bytes
#define IN_BUF_SIZE             16 // messages
   
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
#define NET_MSG_SIZE            HEADER_SIZE + INTERNAL_MSG_SIZE + TAIL_SIZE
#define TAIL_SIZE               1 // bytes
    
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
    
///////////////////////////////////////////////////////////////////////////////    
// TO DO - Define This Device's Role
///////////////////////////////////////////////////////////////////////////////
#define MY_ROLE MONITOR
    
///////////////////////////////////////////////////////////////////////////////
// NetMessage - Message Structure for network communication
//  0 Start Byte        // Added during implimentation
//  1 Sender
//  2 Message number
//  3 Message Type
//  4 Message Size
//  5 - 16 Message
//  17 End Byte         // Added during implimentation
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    unsigned char sender;
    unsigned char number;
    unsigned char type;
    unsigned char msgsize;
    unsigned char msg[SYS_MSG_PAYLOAD_SIZE];
} NetMessage;

///////////////////////////////////////////////////////////////////////////////
// Special Characters
///////////////////////////////////////////////////////////////////////////////
#define START_BYTE              0x00
#define END_BYTE                0xff

///////////////////////////////////////////////////////////////////////////////
// Sensor Object Identifiers
///////////////////////////////////////////////////////////////////////////////

#define OBSTACLE                0xA1
#define TOKEN                   0xA2
#define LEAD                    0xA3
#define FOLLOW                  0xA4

#define OBS1                    0x15
#define OBS2                    0x16
#define OBS3                    0x17
#define OBS4                    0x18
#define TOK1                    0x19
#define TOK2                    0x20
#define TOK3                    0x21
#define TOK4                    0x22

///////////////////////////////////////////////////////////////////////////////
// NetMessage Types
///////////////////////////////////////////////////////////////////////////////
#define DEBUG_MSG               0x01
#define CLIENT_ROLE             0x04
#define INITIALIZE              0x08
#define READY_TO_START          0x10

#define ROVER_MOVE              0x11
#define OBJECT_POS              0x14
#define LEAD_POS                0x15
#define FOLLOW_POS              0x16
#define TOKEN_POS               0x17
#define TOKEN_FOUND             0x18

#define INITIAL_ROUTE_START     0x19
#define INITIAL_ROUTE_DATA      0x20
#define INITIAL_ROUTE_END       0x21
#define INITIAL_OBJ_POS         0x22
#define TOKEN_PICKET            0x23

#define SENSOR_MODE         0x61 // 0x01 - Data mode 0x02 - Data collection
#define CALIBRATE_ROVER     0x62

#define ROT_ERR_MAX             0x63
#define ROT_ERR                 0x64
#define TRV_ERR_MAX             0x65
#define TRV_ERR                 0x66

#define ACC_STATS               0x67
#define CLEAR_AC_STATS          0x68

#define PONG                    0x69
#define PING                    0x70
#define END_GAME                0x71
#define START_GAME              0x72


#define ROVER_FINISH            0x73
#define SEND_NET_STATS          0x74
#define CLEAR_NET_STATS         0x75
#define NET_STAT                0x76
#define TOGGLE_DEBUG_LED        0x77
#define TEST_RPI_MSG            0x78
#define OUTPUT_TO_MONITOR       0x79

///////////////////////////////////////////////////////////////////////////////
// SysMsg - Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    unsigned char type;
    unsigned char size;
    unsigned char msg[SYS_MSG_PAYLOAD_SIZE];
    
}SysMsg;

///////////////////////////////////////////////////////////////////////////////
// SysMsg Types
///////////////////////////////////////////////////////////////////////////////

// Uses same mesaage types as NetMessage

///////////////////////////////////////////////////////////////////////////////
// RpiMsg - Internal message structure for passing between threads
///////////////////////////////////////////////////////////////////////////////
typedef struct{

    unsigned char source;
    unsigned char type;
    unsigned char size;
    unsigned char msg[SYS_MSG_PAYLOAD_SIZE];
    
}RpiMsg;

///////////////////////////////////////////////////////////////////////////////
// RpiMsg Types
///////////////////////////////////////////////////////////////////////////////

// Uses same message types as NetMessage

///////////////////////////////////////////////////////////////////////////////
// Link Status
///////////////////////////////////////////////////////////////////////////////
#define GREEN                       0xBB
#define YELLOW                      0xCC
#define RED                         0xDD
#define UNKOWN_LINK_STATE           0xEE

///////////////////////////////////////////////////////////////////////////////
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
// vvvvvvvvvvvvv FLAGGED FOR REMOVAL vvvvvvvvvvvvv
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// ROVER_MOVE defines
///////////////////////////////////////////////////////////////////////////////

#define ROVER_FORWARD      0x01
#define ROVER_BACKWARD     0x02
#define ROVER_LEFT         0x04
#define ROVER_RIGHT        0x08
#define ROVER_STOP         0x10

#define LR_YELLOW_DLY       50
#define FR_YELLOW_DLY       50
#define SE_YELLOW_DLY       5
#define CO_YELLOW_DLY       50

#define LR_RED_DLY       7
#define FR_RED_DLY       7
#define SE_RED_DLY       7
#define CO_RED_DLY       7

#ifdef	__cplusplus
}
#endif

#endif	/* COMM_H */

