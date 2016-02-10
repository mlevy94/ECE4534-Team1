/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"

#include "txbuffer_public.h"
#include "app_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

// ADC
void IntHandlerDrvAdc(void)
{
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}

// UART

QueueHandle_t txbufferQ;
void initializeTXBufferQ() {
    txbufferQ = xQueueCreate(16, MAX_MSG_SIZE);
}

BaseType_t putInTXBufferQ(char* msg) {
    return xQueueSend(txbufferQ, msg, portMAX_DELAY);
}
    
BaseType_t putInTXBufferQFromISR(char* msg) {
    return xQueueSendFromISR(txbufferQ, msg, 0);
}

char* txBuffer[MAX_MSG_SIZE] = { '\0' };
int txBufferIdx = 0;
char* rxBuffer[MAX_MSG_SIZE] = { '\0' };
int rxBufferIdx = 0;

void txIntteruptHandler() {
    
    while (!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)){
        // get new message
        if (txBuffer[txBufferIdx == '\0']) {
            // check for message in Q
            if (!xQueueReceiveFromISR(txbufferQ, txBuffer, 0)) {
                // no message in Q
                break;
            }
            // message found. reset index to 0
            else {
                txBufferIdx = 0;
            }
        }
        // send character
        PLIB_USART_TransmitterByteSend(USART_ID_1, txBuffer[txBufferIdx]);
        txBufferIdx++;
    }
    
}

void rxIntteruptHandler() {
    // while there are characters to read
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        // read a character
        rxBuffer[rxBufferIdx] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        // if its the end of the message
        if (rxBuffer[rxBufferIdx] == '\0') {
            // copy message to a temp val
            char tempBuffer[rxBufferIdx + 1];
            int i;
            for (i = 0; rxBuffer[rxBufferIdx]; i++) {
                tempBuffer[i] = rxBuffer[i];
            }
            // add message to Q
            addToInMsgQFromISR(tempBuffer);
            rxBufferIdx = 0;
        }
        else {
            rxBufferIdx++;
        }
        
    }
    
    
}



void IntHandlerDrvUsartInstance0(void)
{
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
        txIntteruptHandler();
    }

    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)){
        rxIntteruptHandler();
    }
    
    
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)){
        //not sure what we are doing yet
    }

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);

}
 
 
 

 
 

 

 
 
  
/*******************************************************************************
 End of File
*/

