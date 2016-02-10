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

#define MAX_MSG_SIZE 112
#define OUT_BUF_SIZE 16
#define IN_BUF_SIZE 16

typedef unsigned MSG_FIELD;
    
typedef struct{
    
    // MESSAGE STRUCTURE
    //
    // | Message Source | Message Number | ->
    // Message Type | Payload Size | ->
    // Payload (variable?) | Checksum | ACK |
    
    MSG_FIELD MSG_SRC:8;
    MSG_FIELD MSG_NUM:8;
    MSG_FIELD MSG_TYPE:8;
    MSG_FIELD PAYLOAD_SIZE:8;
    MSG_FIELD PAYLOAD:32;
    MSG_FIELD CHKSUM:8;
    MSG_FIELD ACK_FIELD:8; 
    
} TEAM1_MSG;

// Sources

#define LEAD_ROVER          0x01;
#define FOLLOWER            0x02;
#define SENSORS             0x04;
#define COORDINATOR         0x08;
#define MONITOR             0x10;

// Message Type

#define INITIALIZE          0x01;
#define READY_TO_START      0x02;
#define LEAD_PO             0x04;
#define FOLLOW_PO           0x08;
#define OBS_INFO            0x10;
#define MOTOR_FB            0x11;
#define ROVER_CMD           0x12;
#define TOKEN_FOUND         0x14;

// Message counter
typedef struct{
    
    unsigned INITIALIZE_COUNT;
    unsigned RTSTART_COUNT;
    unsigned LEAD_PO_COUNT;
    unsigned FOLLOW_PO_COUNT;
    unsigned OBS_INFO_COUNT;
    unsigned MOTOR_FB_COUNT;
    unsigned ROVER_CMD_COUNT;
    unsigned TOKEN_FOUND_COUNT;
    
} MSG_COUNTER;



#ifdef	__cplusplus
}
#endif

#endif	/* COMM_H */

