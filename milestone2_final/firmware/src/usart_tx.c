/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    usart_tx.c

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

#include "usart_tx.h"
#include "usart_tx_public.h"

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

USART_TX_DATA usart_txData;

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
    void USART_TX_Initialize ( void )

  Remarks:
    See prototype in usart_tx.h.
 */

void USART_TX_Initialize ( void )
{
    usart_txData.usart_txQ = xQueueCreate(OUT_BUF_SIZE, MAX_MSG_SIZE);
    usart_txData.usart_prioirtyTxQ = xQueueCreate(TX_PRIORITY_Q_SIZE,
                                                  MAX_MSG_SIZE);
    usart_txData.message_counter = 0;
    
}


/******************************************************************************
  Function:
    void USART_TX_Tasks ( void )

  Remarks:
    See prototype in usart_tx.h.
 */

void USART_TX_Tasks ( void )
{
    INTERNAL_MSG internal_msg;
    char outCharArray[MAX_MSG_SIZE];
    char outChar;
    int i = 0;
    while(1) {
        
        // Check PRIORITY queue 
        // Check outQ for message
        //if(xQueueReceive(usart_txData.usart_prioirtyTxQ, &out_msg, portMAX_DELAY)) {
            
           
               
            // Header stuff goes here ( encapsulation if necessary )
            //for (i = 0; out_msg[i] != '\0'; i++) {
                   
                // Send to Front of Interrupt's buffQ
                
            // }
            // tail stuff goes here
            //outChar = '\0';
            
       //}
        
        // Check outQ for message
        if(xQueueReceive(usart_txData.usart_txQ, &internal_msg, portMAX_DELAY)){
 // char source, char message_counter, char type, char payloadSize, char payload
            buildMessageCharArray( outCharArray, MY_ROLE, 
                    usart_txData.message_counter, internal_msg.INTERNAL_TYPE, 
                    PAYLOAD_4BYTE, internal_msg.INTERNAL_MESSAGE );      
            putInTXBufferQ(outCharArray);
        }
    }
}


void addToOutQ(char* val){
    xQueueSend(usart_txData.usart_txQ, val, portMAX_DELAY);
}

void addToPrioirtyTxQ(char* val){
    
    xQueueSend(usart_txData.usart_prioirtyTxQ, val, portMAX_DELAY);
    
}

BaseType_t addToPrioirtyTxQFromISR(char* val){
    
    return xQueueSend(usart_txData.usart_prioirtyTxQ, val, portMAX_DELAY);
    
}

BaseType_t addToOutQFromISR(char* val) {
    return xQueueSendFromISR(usart_txData.usart_txQ, val, portMAX_DELAY);
}

void buildMessageCharArray(char * outCharArray, char source, char msg_counter, char type, char payloadSize, char * payload){
    
    TEAM1_MSG out_msg;
    
    out_msg.MSG_START = 0x01;
    out_msg.MSG_SRC = source;
    out_msg.MSG_NUM = msg_counter;
    out_msg.MSG_TYPE = type;
    out_msg.PAYLOAD_SIZE = payloadSize;
    out_msg.PAYLOAD = (unsigned)payload;
    
    out_msg.CHKSUM = (int)source + (int)msg_counter + (int)type + (int)payloadSize +
            ( (int)payload & 0xFF) +
            ( (int)payload >> 8 & 0xFF) +
            ( (int)payload >> 16 & 0xFF) +
            ( (int)payload >> 24 & 0xFF);
    
    out_msg.ACK_FIELD = 0x06;
    
    msg_counter = (int)msg_counter + 1;
    
    outCharArray[0] = out_msg.MSG_START;
    outCharArray[1] = out_msg.MSG_SRC;
    outCharArray[2] = out_msg.MSG_NUM;
    outCharArray[3] = out_msg.PAYLOAD_SIZE;
    outCharArray[4] = out_msg.PAYLOAD;
    outCharArray[5] = out_msg.CHKSUM;
    outCharArray[6] = out_msg.ACK_FIELD;
    
}
 
/*******************************************************************************
 End of File
 */
