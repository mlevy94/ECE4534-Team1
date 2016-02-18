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
#include "usart_rx.h"
#include "system_definitions.h"
#include "app_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
char rxBuffer[MAX_MSG_SIZE] = { '\0' };
int rxBufferIdx = 0;
/*
 * It makes it here to the rxInterruptHandler,
 * it appears that it is not getting the correct value from the Rx Usart line
 * I cannot get it to print out the key press that i am sending through
 * the XBee.  However it does appear to step through the interrupt and then 
 * into the addToUsartRxQFromISR function which is located in usart_rx.c
 * it's prototype is in app_public.h
 * Update (2/16 @ 11:40)
 * I cannot figure out why it isnt reading in the correct value. Ive tried 
 * different function  calls such as DRV_USART0_ReadByte(void) and couldnt
 * get that to output the correct value either. I have marked inside of 
 * rxInterruptHandler() where i implemented this. 
 * I also tried to send the value rxBuffer[rxBufferIdx] to the Debug lines and
 * i couldnt get the correct key that i had pressed through the XBee using
 * this method either.
 * I Do not think the '\0' check is working because i was able to 
 * get into the if (rxBuffer[rxBufferIdx] == '\0') statement without sending
 * '\0'.  I know this because i could hit a setDebugVal I had within the
 * if statement.
 * Talking to Michael I think that he is wanting to break on char 255 like
 * Jones said instead of using a null terminator for messages.  This needs
 * to be verified.
 * 
 * Final Notes(2/16):
 * I think that we are not reading in the messages correctly and i am not quite
 * how to do this yet.  I'm thinking it might require looking at the 
 * drv_usart_static.c file or usart_p32mx795f512l.h and possibly trying new
 * functions.  That being said my cpu was acting weird the entire time while
 * testing and it could just be that my cpu isn't working correctly right now.
 * When I have more time to play around with the debugs i will check some
 * of the suggestions i mentioned.
 * *****IMPORTANT******
 * Make sure that you configure the project appropriately and that you set
 * the heap size to at least 80560 (I doubled the original of 40280) if this
 * is not done then you will not see anything happening in the interrupt from
 * my experience.  Also the Debug pins need to be set in the configuration
 * they are the following:
 * PORT G = PIN 7
 * PORT E = PIN 0 - PIN 7
 * Other Configs:
 * Driver->USART->Static
 */

void rxInterruptHandler() {
 #ifdef DEBUG_ON
    setDebugVal(USART0_IH_RX_START);
#endif
    //Setting the trigger low
    setDebugBool(pdFALSE);
    // while there are characters to read
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        // read a character
        //setDebugVal(DRV_USART0_ReadByte()); // Checking Value Here
        rxBuffer[rxBufferIdx] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        //setDebugVal(rxBuffer[rxBufferIdx]); // Checking Value Here
        // if its the end of the message
        if (rxBuffer[rxBufferIdx] == '\0') {
            // copy message to a temp val
            char tempBuffer[MAX_MSG_SIZE];
            int i;
            for (i = 0; i < MAX_MSG_SIZE ; i++) {
                 #ifdef DEBUG_ON
                    setDebugVal(IH_FOR_LOOP_TEMP_BUF);
                #endif
                tempBuffer[i] = rxBuffer[i];
            }
            //setDebugVal(tempBuffer); // Checking Value Here
            // add message to Q
            addToUsartRxQFromISR(tempBuffer);
            rxBufferIdx = 0;
        }
        else {
            rxBufferIdx++;
        }
        
    }
    // Setting the trigger high
    setDebugBool(pdTRUE);
    
#ifdef DEBUG_ON
    setDebugVal(USART0_IH_RX_END);
#endif
    
}

/*
 * It makes it through this interrupt to the rxInterruptHandler() function
 */
void IntHandlerDrvUsartInstance0(void)
{
#ifdef DEBUG_ON
    setDebugVal(USART0_IH_MASTER_START);
#endif
    
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
        //txInterruptHandler();
    }

    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)){
        rxInterruptHandler();
    }
    
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)){
        //not sure what we are doing yet
    }

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    
#ifdef DEBUG_ON
    setDebugVal(USART0_IH_MASTER_END);
#endif

}
 
 
 

 
 

 

 
 
  
/*******************************************************************************
 End of File
*/

