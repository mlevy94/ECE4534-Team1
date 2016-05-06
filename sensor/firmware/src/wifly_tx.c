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

static WIFLY_TX_DATA wifly_txData;

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

BaseType_t addToInitTXQ(char msg) {
    return xQueueSend(wifly_txData.initQ, &msg, portMAX_DELAY);
}

BaseType_t addToUartTXQ(InternalMessage msg) {
    return xQueueSend(wifly_txData.txMessageQ, &msg, portMAX_DELAY);
}

BaseType_t addToUartTXQFromISR(InternalMessage msg) {
    return xQueueSendFromISR(wifly_txData.txMessageQ, &msg, 0);
}

BaseType_t priorityAddToUartTXQ(InternalMessage msg) {
    return xQueueSendToFront(wifly_txData.txMessageQ, &msg, portMAX_DELAY);
}

BaseType_t priorityAddToUartTXQFromISR(InternalMessage msg) {
    return xQueueSendToFrontFromISR(wifly_txData.txMessageQ, &msg, 0);
}

BaseType_t addToTXBufferQ(char msg) {
    // Turn TX Interrupt on
    PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
    return xQueueSend(wifly_txData.txInterruptQ, &msg, portMAX_DELAY);
}
    
BaseType_t addToTXBufferQFromISR(char msg) {
    PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
    return xQueueSendFromISR(wifly_txData.txInterruptQ, &msg, 0);
}

BaseType_t takeFromTXBufferQ(char* msg) {
    xQueueReceive(wifly_txData.txInterruptQ, msg, portMAX_DELAY);
}

BaseType_t takeFromTXBufferQFromISR(char* msg) {
    xQueueReceiveFromISR(wifly_txData.txInterruptQ, msg, 0);
}
void packAndSend(InternalMessage msg) {
    int i = 0;
    setDebugVal(0x70);
    addToTXBufferQ(START_BYTE);
    setDebugVal(0x71);
    addToTXBufferQ(MY_ROLE);
    setDebugVal(0x72);
    addToTXBufferQ(wifly_txData.msgCount);
    setDebugVal(0x73);
    addToTXBufferQ(msg.type);
    setDebugVal(0x74);
    addToTXBufferQ(msg.size);
    setDebugVal(0x75);
    // sent message
    for (i = 0; i < msg.size; i++) {
        addToTXBufferQ(msg.msg[i]);
    }
    setDebugVal(0x76);
    addToTXBufferQ(END_BYTE);
    setDebugVal(0x77);
    // update message count
    if (wifly_txData.msgCount < MAX_MSG_COUNT) {
        wifly_txData.msgCount++;
    }
    else {
        wifly_txData.msgCount = 0;
    }
    setDebugVal(0x78);
}

void WIFLY_TX_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    wifly_txData.initQ = xQueueCreate(1, 8);
    wifly_txData.txMessageQ = xQueueCreate(WIFLY_TX_Q_SIZE, sizeof(InternalMessage));
    wifly_txData.txInterruptQ = xQueueCreate(TX_BUF_SIZE, 8);
    setDebugVal(WIFLY_RX_INIT);
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void WIFLY_TX_Tasks ( void )

  Remarks:
    See prototype in wifly_tx.h.
 */

void WIFLY_TX_Tasks ( void )
{
    //sendDebugToggle();
    //sendObstacleData();
    char start;
    while(!xQueueReceive(wifly_txData.initQ, &start, portMAX_DELAY));
//    setDebugBool(pdTRUE);
//    setDebugVal(0x55);
//    setDebugBool(pdFALSE);
    packAndSend(makeMessageChar(CLIENT_ROLE, MY_ROLE));
//    setDebugBool(pdTRUE);
//    setDebugVal(0x56);
//    setDebugBool(pdFALSE);
    // process other messages
    while(1) {
        setDebugVal(0x79);
        while(!xQueueReceive(wifly_txData.txMessageQ, &wifly_txData.msg, portMAX_DELAY));
        packAndSend(wifly_txData.msg);
    }
}

/*******************************************************************************
 End of File
 */
