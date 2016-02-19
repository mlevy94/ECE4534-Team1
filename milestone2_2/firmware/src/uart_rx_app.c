/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_rx_app.c

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

#include "uart_rx_app.h"
#include "uart_rx_app_public.h"

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

UART_RX_APP_DATA uart_rx_appData;

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

BaseType_t addToUartRXQ(char msg) {
    return xQueueSend(uart_rx_appData.rxMessageQ, &msg, portMAX_DELAY);
}

BaseType_t addToUartRXQFromISR(char msg) {
    return xQueueSendFromISR(uart_rx_appData.rxMessageQ, &msg, 0);
}

BaseType_t addToUartSentQ(char num, InternalMessage msg) {
    SentMessage newMsg = { num, msg };
    return xQueueSend(uart_rx_appData.sentMessageQ, &newMsg, portMAX_DELAY);
}

BaseType_t addToUartSentQFromISR(char num, InternalMessage msg) {
    SentMessage newMsg = { num, msg };
    return xQueueSendFromISR(uart_rx_appData.sentMessageQ, &newMsg, 0);
}

int getChecksum(char msg[]) {
    int checksum = 0;
    int i;
    for (i = 1; i < HEADER_SIZE + msg[4]; i++) {
        checksum += msg[i];
    }
    return checksum;
}

InternalMessage processMessage(char msg[], int len) {
    // check message for errors
    int checksum = msg[HEADER_SIZE + msg[4]] << 7 + msg[HEADER_SIZE + msg[4] + 1];
    if (msg[0] == START_BYTE &&
        len == HEADER_SIZE + TAIL_SIZE + msg[4] &&
        checksum == getChecksum(msg)) {
        // check for any missed messages and request them
        for (; uart_rx_appData.msgCount < msg[2]; uart_rx_appData.msgCount++) {
            priorityAddToUartTXQ(makeMessageChar(MSG_REQUEST, uart_rx_appData.msgCount));
        }
        // process message
        char msgcontent[msg[4]];
        int i;
        for (i = 0; i < msg[4]; i++) {
            msgcontent[i] = msg[HEADER_SIZE + i];
        }
        return makeMessage(msg[3], msgcontent);
    }
    else {
        return makeMessage(BAD_MSG, 0);
    }
}

void sortMessage(InternalMessage msg) {
    // place message in correct Q based on type field
    int i;
    switch(msg.type) {
        case MSG_REQUEST:
            
            break;
        case DEBUG_MSG:
        default:
            for (i = 0; msg.msg[i] != '\0'; i++) {
                setDebugVal(msg.msg[i]);
            }
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_RX_APP_Initialize ( void )

  Remarks:
    See prototype in uart_rx_app.h.
 */

void UART_RX_APP_Initialize ( void )
{
    uart_rx_appData.rxMessageQ = xQueueCreate(RX_BUF_SIZE, 8);
    uart_rx_appData.sentMessageQ = xQueueCreate(SENT_MSG_Q_SIZE, sizeof(SentMessage));
    uart_rx_appData.msgCount = 0;
}


/******************************************************************************
  Function:
    void UART_RX_APP_Tasks ( void )

  Remarks:
    See prototype in uart_rx_app.h.
 */

void UART_RX_APP_Tasks ( void )
{
    char inChar;
    char inmsg[INTERNAL_MSG_SIZE];
    int idx = 0;
    InternalMessage processedMsg;
    while(1) {
#ifdef DEBUG_ON
        setDebugVal(TASK_UART_RX_APP);
#endif
        if (xQueueReceive(uart_rx_appData.rxMessageQ, &inChar, portMAX_DELAY)) {
            inmsg[idx] = inChar;
            if (inChar == END_BYTE) {
                processedMsg = processMessage(inmsg, idx + 1);
                // check for error message
                setDebugVal(2);
                if (processedMsg.type != BAD_MSG) {
                    // place in correct Q based on message type
                    sortMessage(processedMsg);
                }
                idx = 0; // reset message buffer
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
