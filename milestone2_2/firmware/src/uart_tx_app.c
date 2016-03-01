/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    uart_tx_app.c

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

#include "uart_tx_app.h"
#include "uart_tx_app_public.h"

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

UART_TX_APP_DATA uart_tx_appData;

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

BaseType_t addToUartTXQ(InternalMessage msg) {
    return xQueueSend(uart_tx_appData.txMessageQ, &msg, portMAX_DELAY);
}

BaseType_t addToUartTXQFromISR(InternalMessage msg) {
    return xQueueSendFromISR(uart_tx_appData.txMessageQ, &msg, 0);
}

BaseType_t priorityAddToUartTXQ(InternalMessage msg) {
    return xQueueSendToFront(uart_tx_appData.txMessageQ, &msg, portMAX_DELAY);
}

BaseType_t priorityAddToUartTXQFromISR(InternalMessage msg) {
    return xQueueSendToFrontFromISR(uart_tx_appData.txMessageQ, &msg, 0);
}

void packAndSend(InternalMessage msg) {
    int i = 0;
    int msgSize = 0;
    // get size of message
    for (msgSize = 0; msg.msg[msgSize] != '\0' && msgSize <= INTERNAL_MSG_SIZE; msgSize++);
    addToTXBufferQ(START_BYTE);
    addToTXBufferQ(MY_ROLE);
    addToTXBufferQ(uart_tx_appData.msgCount);
    addToTXBufferQ(msg.type);
    addToTXBufferQ(msgSize);
    // sent message
    for (i = 0; i < msgSize; i++) {
        addToTXBufferQ(msg.msg[i]);
    }
    addToTXBufferQ(END_BYTE);
    // update message count
    if (uart_tx_appData.msgCount < MAX_MSG_COUNT) {
        uart_tx_appData.msgCount++;
    }
    else {
        uart_tx_appData.msgCount = 0;
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void UART_TX_APP_Initialize ( void )

  Remarks:
    See prototype in uart_tx_app.h.
 */

void UART_TX_APP_Initialize ( void )
{
    uart_tx_appData.txMessageQ = xQueueCreate(16, sizeof(InternalMessage));
    uart_tx_appData.msgCount = 0;
}


/******************************************************************************
  Function:
    void UART_TX_APP_Tasks ( void )

  Remarks:
    See prototype in uart_tx_app.h.
 */

void UART_TX_APP_Tasks ( void )
{    
#ifdef DEBUG_ON
    setDebugVal(TASK_UART_TX_APP);
#endif
    InternalMessage msg;
    // declare role
    msg.type = CLIENT_ROLE;
    msg.msg[0] = MY_ROLE;
    msg.msg[1] = '\0';
    packAndSend(msg);
    // process other messages
    while(1) {
#ifdef DEBUG_ON
        setDebugVal(TASK_UART_TX_APP);
#endif
        if (xQueueReceive(uart_tx_appData.txMessageQ, &msg, portMAX_DELAY)) {
            packAndSend(msg);
        }
    }
}
 

/*******************************************************************************
 End of File
 */
