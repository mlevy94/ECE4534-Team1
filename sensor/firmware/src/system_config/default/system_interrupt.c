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
#include "wifly_rx.h"
#include "wifly_tx.h"
#include "pixy_rx.h"
#include "system_definitions.h"
//#include "debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
/*if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_RECEIVE))
    {
        while (!DRV_USART0_ReceiverBufferIsEmpty())
        {
            readByte = DRV_USART0_ReadByte();
            setDebugVal('H');
        }
    }*/

uint8_t readByte = 0x00;
void IntHandlerDrvUsartInstance0(void)
{
    setDebugVal(UART_INTERRUPT);
    /* TODO: Add code to process interrupt here */ 
    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_RECEIVE))
    {
        setDebugVal(RECEIVING);
        while (!DRV_USART0_ReceiverBufferIsEmpty())
        {
            readByte = DRV_USART0_ReadByte();
            //SixteenBitsetDebugVal(readByte);
            //SixteenBitsetDebugVal(readByte | 0x0000);
            //add readByte to queue
            addToPixyQFromISR(readByte);
        }
    }
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);
    

}
 
 
void IntHandlerDrvUsartInstance1(void)
{
#ifdef DEBUG_ON
    setDebugVal(INT_UART0_START);
#endif
    char sendbyte;
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
#ifdef DEBUG_ON
        setDebugVal(INT_UART0_TX);
#endif
        setDebugVal(0x64);
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
            setDebugVal(0x65);
            if(takeFromTXBufferQFromISR(&sendbyte)) {
               PLIB_USART_TransmitterByteSend(USART_ID_1, sendbyte);
            }
            else {
               // nothing to write. Disable interrupt
               PLIB_INT_SourceDisable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
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
            addToUartRXQFromISR(sendbyte);
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

  
/*******************************************************************************
 End of File
*/

