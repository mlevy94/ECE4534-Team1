/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    wifly_tx.c

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

#include "wifly_tx.h"

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

WIFLY_TX_DATA wifly_txData;

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
    void WIFLY_TX_Initialize ( void )

  Remarks:
    See prototype in wifly_tx.h.
 */

void WIFLY_TX_Initialize ( void )
{
    setDebugVal(WIFLYTX_INIT_TOP);
    /* Place the App state machine in its initial state. */
    wifly_txData.state = WIFLY_TX_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    wifly_txData.wifly_txOutgoingMessageQ = xQueueCreate(WIFLY_TX_Q_SIZE, SYS_MSG_SIZE);
    wifly_txData.wifly_txInternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    wifly_txData.initQ = xQueueCreate(1, sizeof(char));
    wifly_txData.debugItt = 0;
    wifly_txData.itt = 0;
}


/******************************************************************************
  Function:
    void WIFLY_TX_Tasks ( void )

  Remarks:
    See prototype in wifly_tx.h.
 */

void WIFLY_TX_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( wifly_txData.state )
    {
        /* Application's initial state. */
        case WIFLY_TX_STATE_INIT:
        {
            setDebugVal(WIFLYTX_INIT_STATE);
            char start;
                       
            // Disable TX interrupt until data is ready to be sent
            // Will be re-enabled by function that writes to 
            // the wifly_tx app's outgoing message queue
            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
            
            while(!xQueueReceive(wifly_txData.initQ, &start, portMAX_DELAY));
            
            sendInitResponse();
            
            wifly_txData.state = WIFLY_TX_STATE_ONE;
            
            break;
        }
        
        case WIFLY_TX_STATE_ONE:
        {
            setDebugVal(WIFLYTX_S_ONE);

            if(xQueueReceive(wifly_txData.wifly_txOutgoingMessageQ, &wifly_txData.outgoingSystemMsg, portMAX_DELAY )){
                sendSerialToWifly(wifly_txData.outgoingSystemMsg);
            }
            
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            setDebugVal(WIFLYTX_DEFAULT_STATE);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    setDebugVal(WIFLYTX_TASKS_COMPLETE);
}

///////////////////////////////////////////////////////////////////////////////
// Kyle's test string function
///////////////////////////////////////////////////////////////////////////////
void uartTestString_MaxLegnth(){
    
            static uint8_t i;
            static uint8_t j;
            static uint8_t k;
                
            // 1
                for (i = 0x41 ; i < 0x5B ; i ++){
                    writeTo_WiFly_OutgoingByte_Q(i);
                }
                
                for (j = 0x61 ; j < 0x7B ; j ++){
                    writeTo_WiFly_OutgoingByte_Q(j);
                }
                
                writeTo_WiFly_OutgoingByte_Q(0x20);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x31);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x20);
                
            // 2
                for (i = 0x41 ; i < 0x5B ; i ++){
                    writeTo_WiFly_OutgoingByte_Q(i);
                }
                
                for (j = 0x61 ; j < 0x7B ; j ++){
                    writeTo_WiFly_OutgoingByte_Q(j);
                }
                
                writeTo_WiFly_OutgoingByte_Q(0x20);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x32);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x20);
                
            // 3
                for (i = 0x41 ; i < 0x5B ; i ++){
                    writeTo_WiFly_OutgoingByte_Q(i);
                }
                
                for (j = 0x61 ; j < 0x7B ; j ++){
                    writeTo_WiFly_OutgoingByte_Q(j);
                }
                
                writeTo_WiFly_OutgoingByte_Q(0x20);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x33);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x20);
                
            // 4
                for (i = 0x41 ; i < 0x5B ; i ++){
                    writeTo_WiFly_OutgoingByte_Q(i);
                }
                
                for (j = 0x61 ; j < 0x7B ; j ++){
                    writeTo_WiFly_OutgoingByte_Q(j);
                }
                
                writeTo_WiFly_OutgoingByte_Q(0x20);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x34);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x20);
                
                
            // 5
                for (i = 0x41 ; i < 0x5B ; i ++){
                    writeTo_WiFly_OutgoingByte_Q(i);
                }
                
                for (j = 0x61 ; j < 0x7B ; j ++){
                    writeTo_WiFly_OutgoingByte_Q(j);
                }
                
                writeTo_WiFly_OutgoingByte_Q(0x20);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x35);
                writeTo_WiFly_OutgoingByte_Q(0x21);
                writeTo_WiFly_OutgoingByte_Q(0x20);
                
            // Send end byte
                writeTo_WiFly_OutgoingByte_Q(END_BYTE);
                
                
    
}

void sendInitResponse(){
    
        writeTo_WiFly_OutgoingByte_Q(START_BYTE);               // Write Start Byte
        writeTo_WiFly_OutgoingByte_Q(MY_ROLE);                  // Write My Role
        writeTo_WiFly_OutgoingByte_Q(CLIENT_ROLE);              // Write msg number
        writeTo_WiFly_OutgoingByte_Q(CLIENT_ROLE);              // Write msg type    
        writeTo_WiFly_OutgoingByte_Q(0x01);                     // Write msg size

        // 12 bytes in msg
        writeTo_WiFly_OutgoingByte_Q(MY_ROLE);
        // Send end byte
        writeTo_WiFly_OutgoingByte_Q(END_BYTE);
    
}

void sendDebugToggle(){
    
    //  net message debug toggle
    // count to 10000 so blink will be observable
    
    if(wifly_txData.debugItt == 10000){
        wifly_txData.debugItt = 0;
        
        writeTo_WiFly_OutgoingByte_Q(START_BYTE);               // Write Start Byte
        writeTo_WiFly_OutgoingByte_Q(MY_ROLE);                  // Write My Role
        writeTo_WiFly_OutgoingByte_Q(0x45);                     // Write msg number
        writeTo_WiFly_OutgoingByte_Q(TOGGLE_DEBUG_LED);         // Write msg type    
        writeTo_WiFly_OutgoingByte_Q(SYS_MSG_PAYLOAD_SIZE);     // Write msg size

        // 12 bytes in msg
        writeTo_WiFly_OutgoingByte_Q(0x41);    
        writeTo_WiFly_OutgoingByte_Q(0x42);

        writeTo_WiFly_OutgoingByte_Q(0x43);    
        writeTo_WiFly_OutgoingByte_Q(0x44);

        writeTo_WiFly_OutgoingByte_Q(0x45);    
        writeTo_WiFly_OutgoingByte_Q(0x46);

        writeTo_WiFly_OutgoingByte_Q(0x47);    
        writeTo_WiFly_OutgoingByte_Q(0x48);

        writeTo_WiFly_OutgoingByte_Q(0x49);    
        writeTo_WiFly_OutgoingByte_Q(0x4A);

        writeTo_WiFly_OutgoingByte_Q(0x4B);    
        writeTo_WiFly_OutgoingByte_Q(0x4C);
        
        writeTo_WiFly_OutgoingByte_Q(0x0A);
        writeTo_WiFly_OutgoingByte_Q(0x0D);
        
        // Send end byte
        writeTo_WiFly_OutgoingByte_Q(END_BYTE);
       
        //toggleDebugLED();
    }
    
    else{
        wifly_txData.debugItt++;
    }
    
}

BaseType_t sendMsgToWiFlyTx(SysMsg inSysMsg){
    return xQueueSend(wifly_txData.wifly_txInternalQ, &inSysMsg, portMAX_DELAY);
}

BaseType_t sendMsgToWiFlyTx_fromISR(SysMsg inSysMsg){
    return xQueueSendFromISR(wifly_txData.wifly_txInternalQ, &inSysMsg, 0);
}

BaseType_t sendOutgoingMsgToWiflyTx(SysMsg inSysMsg){
    return xQueueSend(wifly_txData.wifly_txOutgoingMessageQ, &inSysMsg, portMAX_DELAY);    
}

BaseType_t sendOutgoingMsgToWiflyTx_fromISR(SysMsg inSysMsg){
    return xQueueSendFromISR(wifly_txData.wifly_txOutgoingMessageQ, &inSysMsg, 0);
}

BaseType_t sendToInitTXQ(SysMsg inSysMsg) {
    return xQueueSend(wifly_txData.initQ, &inSysMsg, portMAX_DELAY);
}

BaseType_t sendSerialToWifly(SysMsg outSysMsg){
    
        writeTo_WiFly_OutgoingByte_Q(START_BYTE);               // Write Start Byte
        writeTo_WiFly_OutgoingByte_Q(MY_ROLE);                  // Write My Role
        writeTo_WiFly_OutgoingByte_Q(CLIENT_ROLE);              // Write msg number
        writeTo_WiFly_OutgoingByte_Q(outSysMsg.type);           // Write msg type    
        writeTo_WiFly_OutgoingByte_Q(outSysMsg.size);                     // Write msg size

        // 12 bytes in msg
        writeTo_WiFly_OutgoingByte_Q(outSysMsg.msg[0]);
        // Send end byte
        writeTo_WiFly_OutgoingByte_Q(END_BYTE);
    
}

/*******************************************************************************
 End of File
 */
