/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pi_rx.c

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

#include "pi_rx.h"

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

PI_RX_DATA pi_rxData;

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
    void PI_RX_Initialize ( void )

  Remarks:
    See prototype in pi_rx.h.
 */

void PI_RX_Initialize ( void )
{
    setDebugVal(PIRX_INIT_TOP);
    /* Place the App state machine in its initial state. */
    pi_rxData.state = PI_RX_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    pi_rxData.pi_RxInternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    pi_rxData.pi_RxByteQ = xQueueCreate(PI_RX_Q_SIZE, 8);
    pi_rxData.pi_RxMsgQ = xQueueCreate(PI_RX_Q_SIZE, SYS_MSG_SIZE);
    
}


/******************************************************************************
  Function:
    void PI_RX_Tasks ( void )

  Remarks:
    See prototype in pi_rx.h.
 */

void PI_RX_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( pi_rxData.state )
    {
        /* Application's initial state. */
        case PI_RX_STATE_INIT:
        {
            setDebugVal(PIRX_INIT_STATE);
            pi_rxData.state=PI_RX_STATE_ONE;
            break;
        }
        
        case PI_RX_STATE_ONE:
        {
            setDebugVal(PIRX_S_ONE);
            buildIncomingGuiMsg();
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            setDebugVal(PIRX_DEFAULT_STATE);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    setDebugVal(PIRX_TASKS_COMPLETE);
}

BaseType_t writeTo_Pi_rxByteQueue_fromISR(char incomingByte){
    return xQueueSendFromISR(pi_rxData.pi_RxByteQ, &incomingByte, 0);
}

BaseType_t sendMsgToPiRxApp(SysMsg inSysMsg){
    return xQueueSend(pi_rxData.pi_RxInternalQ, &inSysMsg, portMAX_DELAY);  
}

BaseType_t sendMsgToPiRxApp_fromISR(SysMsg inSysMsg){
   return xQueueSendFromISR(pi_rxData.pi_RxInternalQ, &inSysMsg, 0);     
}

void buildIncomingGuiMsg(){
    
    int i = 0;

        setDebugVal(PIRX_BLD_INCOMING_MSG);
        if (xQueueReceive(pi_rxData.pi_RxByteQ, &pi_rxData.incomingChar, portMAX_DELAY)) {
            // get byte
            if ((pi_rxData.incomingChar & 0xff) == START_BYTE) { 
                // get message source (should be GUI - 0x14)
                while (!xQueueReceive(pi_rxData.pi_RxByteQ, &pi_rxData.incomingChar, portMAX_DELAY));
                pi_rxData.incomingPiMsg.source = pi_rxData.incomingChar;
                // get message type
                while (!xQueueReceive(pi_rxData.pi_RxByteQ, &pi_rxData.incomingChar, portMAX_DELAY));
                pi_rxData.incomingPiMsg.type = pi_rxData.incomingChar;
                // get message size
                //while (!xQueueReceive(pi_rxData.pi_RxByteQ, &pi_rxData.incomingChar, portMAX_DELAY));
                //pi_rxData.incomingPiMsg.size = pi_rxData.incomingChar;
                // get message
                for (i = 0; i < SYS_MSG_PAYLOAD_SIZE; i++ ) {
                    while (!xQueueReceive(pi_rxData.pi_RxByteQ, &pi_rxData.incomingChar, portMAX_DELAY));
                    pi_rxData.incomingPiMsg.msg[i] = pi_rxData.incomingChar;
                }
                processIncomingGuiMsg();
            }
        }
    
}

void processIncomingGuiMsg(){
    setDebugVal(PIRX_PROC_MSG);
    SysMsg temp;
    
    

    switch(pi_rxData.incomingPiMsg.type){
        
        case TEST_RPI_MSG:
            setDebugVal(PIRX_MSG_IDENTIFIED);
            setDebugVal(pi_rxData.incomingPiMsg.type);
            break;
            
        case CLEAR_AC_STATS:
            setDebugVal(PIRX_MSG_IDENTIFIED);   
            setDebugVal(pi_rxData.incomingPiMsg.type);
            pi_rxData.sysMsg.type = pi_rxData.incomingPiMsg.type;
            pi_rxData.sysMsg.size = pi_rxData.incomingPiMsg.size;
            for(pi_rxData.itt =0 ; pi_rxData.itt < pi_rxData.sysMsg.size  ; pi_rxData.itt++ ){
                pi_rxData.sysMsg.msg[pi_rxData.itt] = pi_rxData.incomingPiMsg.msg[pi_rxData.itt];
            }
            sendMsgToApp1(pi_rxData.sysMsg);
            break;
        case CLEAR_NET_STATS:
            setDebugVal(PIRX_MSG_IDENTIFIED);   
            setDebugVal(pi_rxData.incomingPiMsg.type);
            pi_rxData.sysMsg.type = pi_rxData.incomingPiMsg.type;
            pi_rxData.sysMsg.size = pi_rxData.incomingPiMsg.size;
            for(pi_rxData.itt =0 ; pi_rxData.itt < pi_rxData.sysMsg.size  ; pi_rxData.itt++ ){
                pi_rxData.sysMsg.msg[pi_rxData.itt] = pi_rxData.incomingPiMsg.msg[pi_rxData.itt];
            }
            sendMsgToApp(pi_rxData.sysMsg);
            break;
        case INITIAL_OBJ_POS:
            setDebugVal(PIRX_MSG_IDENTIFIED);   
            setDebugVal(pi_rxData.incomingPiMsg.type);
            pi_rxData.sysMsg.type = pi_rxData.incomingPiMsg.type;
            pi_rxData.sysMsg.size = pi_rxData.incomingPiMsg.size;
            for(pi_rxData.itt =0 ; pi_rxData.itt < pi_rxData.sysMsg.size  ; pi_rxData.itt++ ){
                pi_rxData.sysMsg.msg[pi_rxData.itt] = pi_rxData.incomingPiMsg.msg[pi_rxData.itt];
            }
            sendMsgToApp1(pi_rxData.sysMsg);
        break;
        
        case START_GAME:

            temp.type = START_GAME;
            temp.msg[0] = pi_rxData.incomingPiMsg.msg[0];
            temp.size = 0x01;
            sendOutgoingMsgToWiflyTx(temp);
            
        break;
        
        case SENSOR_MODE:
        case CALIBRATE_ROVER:
            
            temp.type = pi_rxData.incomingPiMsg.type;
            temp.size = 0x01;
            temp.msg[0] = pi_rxData.incomingPiMsg.msg[0];
            
            sendOutgoingMsgToWiflyTx(temp);
            
            break;
        default:
            break;
            
        
    }
    
}
 

/*******************************************************************************
 End of File
 */
