/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    wifly_rx.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "wifly_rx.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

WIFLY_RX_DATA wifly_rxData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void WIFLY_RX_Initialize ( void )

  Remarks:
    See prototype in wifly_rx.h.
 */

void WIFLY_RX_Initialize ( void )
{
    setDebugVal(WIFLYRX_INIT_TOP);
    /* Place the App state machine in its initial state. */
    wifly_rxData.state = WIFLY_RX_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    wifly_rxData.wiflyRxByteQ = xQueueCreate(WIFLY_RX_Q_SIZE, 8);
    wifly_rxData.wiflyRxInternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    
    clearInitialRoute();
    
}


/******************************************************************************
  Function:
    void WIFLY_RX_Tasks ( void )

  Remarks:
    See prototype in wifly_rx.h.
 */

void WIFLY_RX_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( wifly_rxData.state )
    {
        /* Application's initial state. */
        case WIFLY_RX_STATE_INIT:
        {
            setDebugVal(WIFLYRX_INIT_STATE);
            wifly_rxData.state = WIFLY_RX_STATE_ONE;
            
            break;
        }
        
        /* TODO: implement your application state machine.*/
        
        case WIFLY_RX_STATE_ONE:
        {
            setDebugVal(WIFLYRX_S_ONE);
                      
            buildIncomingMsg();
            
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            setDebugVal(WIFLYRX_DEFAULT_STATE);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    setDebugVal(WIFLYRX_TASKS_COMPLETE);
}

BaseType_t writeTo_WiFly_rxByteQueue_fromISR(char incomingByte){
    return xQueueSendFromISR(wifly_rxData.wiflyRxByteQ, &incomingByte, 0);
}

BaseType_t sendMsgToWiFlyRxApp(SysMsg inSysMsg){
    return xQueueSend(wifly_rxData.wiflyRxInternalQ, &inSysMsg, portMAX_DELAY);
}

BaseType_t sendMsgToWiFlyRxApp_fromISR(SysMsg inSysMsg){
   return xQueueSendFromISR(wifly_rxData.wiflyRxInternalQ, &inSysMsg, 0);     
}

void buildIncomingMsg(){
    
    int i = 0;
    
        setDebugVal(WIFLYRX_BLD_INCOMING_MSG);
        if (xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY)) {
            // get byte
            if ((wifly_rxData.incomingChar & 0xff) == START_BYTE) {       
                // get sender
                while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                wifly_rxData.incomingMsg.sender = wifly_rxData.incomingChar;
                // get message number
                while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                wifly_rxData.incomingMsg.number = wifly_rxData.incomingChar;
                // get message type
                while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                wifly_rxData.incomingMsg.type = wifly_rxData.incomingChar;
                // get message length
                while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                wifly_rxData.incomingMsg.msgsize = wifly_rxData.incomingChar;
                // get message

                for (i = 0; i < wifly_rxData.incomingMsg.msgsize ; i++ ) {
                    while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                    wifly_rxData.incomingMsg.msg[i] = wifly_rxData.incomingChar;
                }
                
                //for ( ; i < SYS_MSG_PAYLOAD_SIZE ; i++ ) {
                    //while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                    //wifly_rxData.incomingMsg.msg[i] = 0xAA;
                //}
                
                // Ditch end byte
                while (!xQueueReceive(wifly_rxData.wiflyRxByteQ, &wifly_rxData.incomingChar, portMAX_DELAY));
                
                processIncomingWiflyMsg();
                
                
            }
            
        }  
}

void processIncomingWiflyMsg(){
    
    
    setDebugVal(WIFLYRX_PROC_MSG);
    RpiMsg rpiMsg;
    int itt;
    
    switch(wifly_rxData.incomingMsg.type){

        //////////////////////////////////////////////////////////////////////
        // NETSTAT and DEBUG MSGS - APP0
        //////////////////////////////////////////////////////////////////////
        case READY_TO_START:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            buildSysMsg();
            sendMsgToApp(wifly_rxData.sysMsg); // Send to App0
            break;
        case OUTPUT_TO_MONITOR:
        case DEBUG_MSG:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            buildSysMsg_modDebug();
            sendMsgToApp(wifly_rxData.sysMsg); // Send to App0
            break;
        
        //////////////////////////////////////////////////////////////////////
        // POSITION AND ROUTE MSGS - APP1 
        //////////////////////////////////////////////////////////////////////    
        case TOKEN_FOUND:
        case ROVER_MOVE:
        case ROVER_FINISH:
        case OBJECT_POS:
        case INITIAL_ROUTE_START:
        case INITIAL_ROUTE_DATA:
        case INITIAL_ROUTE_END:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            buildSysMsg();
            sendMsgToApp1(wifly_rxData.sysMsg); // Send to App1
            break;
            
        //////////////////////////////////////////////////////////////////////    
        // OTHER MESSAGES
        //////////////////////////////////////////////////////////////////////    
        case TOGGLE_DEBUG_LED:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            toggleDebugLED();
            break;
            
        //////////////////////////////////////////////////////////////////////    
        // START/FINISH - GO Straight to RPI
        //////////////////////////////////////////////////////////////////////     
        
        case START_GAME:
        case END_GAME:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            rpiMsg.source = MY_ROLE;
            rpiMsg.type = wifly_rxData.incomingMsg.type;
            rpiMsg.size = 0x0C;
            for(itt = 0 ; itt < 12 ; itt ++){
                rpiMsg.msg[itt] = 0xAA;
            }

            sendOutgoingMsgToPiTx(rpiMsg);
            
            break;
            
        //////////////////////////////////////////////////////////////////////    
        // DON'T CARES
        //////////////////////////////////////////////////////////////////////    
        case CLIENT_ROLE:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            break;
        case NET_STAT:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            break;
        case INITIALIZE:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            wifly_rxData.sysMsg.type = INITIALIZE;
    
            for(wifly_rxData.itt =0 ; wifly_rxData.itt < SYS_MSG_PAYLOAD_SIZE  ; wifly_rxData.itt++ ){
                wifly_rxData.sysMsg.msg[wifly_rxData.itt] = wifly_rxData.incomingMsg.msg[wifly_rxData.itt];
            }
            
            sendToInitTXQ(wifly_rxData.sysMsg);
        case PONG:
            setDebugVal(WIFLYRX_MSG_IDENTIFIED);
            setDebugVal(wifly_rxData.incomingMsg.type);
            setDebugVal(wifly_rxData.incomingMsg.sender);
        default:
            break;
    }
    
    ////////////////////////////////////////////////////////////////////// 
    // Build Netstats
    ////////////////////////////////////////////////////////////////////// 
    
    wifly_rxData.netStat.type = NET_STAT;
    wifly_rxData.netStat.msg[0] = wifly_rxData.incomingMsg.sender;
    wifly_rxData.netStat.msg[1] = wifly_rxData.incomingMsg.number;
    wifly_rxData.netStat.msg[2] = wifly_rxData.incomingMsg.msg[0];
    wifly_rxData.netStat.size = 0x03;
    
    sendMsgToApp(wifly_rxData.netStat); // Send to App0
                    
}

void buildSysMsg(){
    
    setDebugVal(WIFLYRX_BLD_SYS_MSG);
    
    wifly_rxData.sysMsg.type = wifly_rxData.incomingMsg.type;
    wifly_rxData.sysMsg.size = wifly_rxData.incomingMsg.msgsize;
    
    for(wifly_rxData.itt =0 ; wifly_rxData.itt < wifly_rxData.sysMsg.size  ; wifly_rxData.itt++ ){
        wifly_rxData.sysMsg.msg[wifly_rxData.itt] = wifly_rxData.incomingMsg.msg[wifly_rxData.itt];
    }
    
}

void buildSysMsg_modDebug(){
    
    // This puts the source in the type message
    
    wifly_rxData.sysMsg.type = wifly_rxData.incomingMsg.sender;
    wifly_rxData.sysMsg.size = wifly_rxData.incomingMsg.msgsize;
    
    for(wifly_rxData.itt =0 ; wifly_rxData.itt < wifly_rxData.sysMsg.size  ; wifly_rxData.itt++ ){
        wifly_rxData.sysMsg.msg[wifly_rxData.itt] = wifly_rxData.incomingMsg.msg[wifly_rxData.itt];
    }
    
}


 
/*******************************************************************************
 End of File
 */
