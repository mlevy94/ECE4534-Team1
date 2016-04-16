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
#include "motorapp.h"
#include "nfc_app.h"
#include "uart_tx_app.h"
#include "uart_rx_app.h"
#include "system_definitions.h"

#include "comm.h"
#include "debug.h"
#include "uart_rx_app_public.h"
#include "uart_tx_app_public.h"
#include "motorapp_public.h"
#include "nfc_app_public.h"


// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerExternalInterruptInstance0(void)
{           
    incLeftEn();
    incMoveCount();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);

}

void IntHandlerExternalInterruptInstance1(void)
{     
    incRightEn();
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
    
}


void IntHandlerDrvTmrInstance0(void)

{
    PLIB_TMR_Counter16BitClear(TMR_ID_2);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);

}
 
void IntHandlerDrvUsartInstance0(void)
{
#ifdef DEBUG_ON
    setDebugVal(INT_UART0_START);
#endif
    char sendbyte;
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
#ifdef DEBUG_ON
        setDebugVal(INT_UART0_TX);
#endif
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
            if(readFromTXBufferQFromISR(&sendbyte)) {
               PLIB_USART_TransmitterByteSend(USART_ID_1, sendbyte);
            }
            else {
               // nothing to write. Disable interrupt
               PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
               break;
            }
        }
    }

    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)){
#ifdef DEBUG_ON
        setDebugVal(INT_UART0_RX);
#endif
        // while there are characters to read
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
            // read a character
            sendbyte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            addToUartRXQFromISR(&sendbyte);
        }
    }
    
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)){
        //not sure what we are doing yet
    }
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
#ifdef DEBUG_ON
    setDebugVal(INT_UART0_END);
#endif
}
 
 
 


void IntHandlerDrvUsartInstance1(void)
{
    char sendbyte;
    setDebugVal(0xCC);
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT)){
        setDebugVal(0xCD);
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_2)) {
            if(readFromNFCtxQFromISR(&sendbyte)) {
               PLIB_USART_TransmitterByteSend(USART_ID_2, sendbyte);
            }
            else {
               // nothing to write. Disable interrupt
               PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
               break;
            }
        }
    }
    setDebugVal(0xCE);
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_RECEIVE)){
        setDebugVal(0xCF);
        // while there are characters to read
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_2)){
            setDebugVal(0xC1);
            // read a character
            setDebugVal(0xC2);
            addToNFCrxQFromISR(PLIB_USART_ReceiverByteReceive(USART_ID_2));
            setDebugVal(0xC3);
        }
    }
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_ERROR)){
        //not sure what we are doing yet
    }
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);

}
 
 
 
 

 

 
 
 
 
/*******************************************************************************
 End of File
*/

