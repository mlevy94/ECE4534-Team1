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
    
    initMessageCounter();
    
}


/******************************************************************************
  Function:
    void USART_TX_Tasks ( void )

  Remarks:
    See prototype in usart_tx.h.
 */

void USART_TX_Tasks ( void )
{
    char out_msg[MAX_MSG_SIZE];
    
    char outChar;
    int i = 0;
    while(1){
        
        // Check outQ for message
        if(xQueueReceive(usart_txData.usart_txQ, &out_msg, portMAX_DELAY)) {
               
            // Header stuff goes here ( encapsulation if necessary )
            for (i = 0; out_msg[i] != '\0'; i++) {
                   
                
            }
            // tail stuff goes here
            outChar = '\0';
            
         
        }
    }
}

void addToOutQ(char* val){
    xQueueSend(usart_txData.usart_txQ, val, portMAX_DELAY);
}

BaseType_t addToOutQFromISR(char* val) {
    return xQueueSendFromISR(usart_txData.usart_txQ, val, portMAX_DELAY);
}

TEAM1_MSG buildMessage(char source, char message_number, char type,
        char payloadSize, char payload){
    
    TEAM1_MSG msg;
    
    msg.MSG_SRC = source;
    msg.MSG_NUM = message_number;
    msg.MSG_TYPE = type;
    msg.PAYLOAD_SIZE = payloadSize;
    msg.PAYLOAD = payload;
    
    msg.CHKSUM = (int)source +  (int)message_number + (int)type +
            (int)payloadSize + (int)payload;
    
    msg.ACK_FIELD = 0x06;

    return msg;
    
}

void initMessageCounter(){
    
    usart_txData.message_counter.FOLLOW_PO_COUNT = 0;
    usart_txData.message_counter.INITIALIZE_COUNT = 0;
    usart_txData.message_counter.LEAD_PO_COUNT = 0;
    usart_txData.message_counter.MOTOR_FB_COUNT = 0;
    usart_txData.message_counter.OBS_INFO_COUNT = 0;
    usart_txData.message_counter.ROVER_CMD_COUNT = 0;
    usart_txData.message_counter.RTSTART_COUNT = 0;
    usart_txData.message_counter.TOKEN_FOUND_COUNT = 0;

}
 

/*******************************************************************************
 End of File
 */
