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
#include "app.h"
#include "app1.h"
#include "wifly_tx.h"
#include "wifly_rx.h"
#include "pi_tx.h"
#include "pi_rx.h"
#include "system_definitions.h"

#include <queue.h>
#include "comm_k.h"
#include "debug_k.h"
#include "publicFunctions.h"

static QueueHandle_t wiflyOutgoingByteQ;
static QueueHandle_t piOutgoingByteQ;
static BaseType_t tempT;
static BaseType_t tempS;
static char sendbyteT;
static char sendbyteS;
static char incomingByteT;
static char incomingByteS;

static char debugVal;

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerDrvTmrInstance0(void)

{
    
    
    SysMsg tempSysMsg;
    
    tempSysMsg.type = SEND_NET_STATS;
    
    sendMsgToApp_fromISR(tempSysMsg);
    
    //tempSysMsg.type = SEND_HEARTBEAT;
    //sendMsgToWiFlyTx_fromISR(tempSysMsg);
    
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);

}
 
void IntHandlerDrvUsartInstance0(void)
{
    // RTOS USART 1 "WIFLY" - UART port 1 on Max32
    // Pin0(RX0) and Pin1(TX0)

    /* TODO: Add code to process interrupt here */
    
    // SOURCE = UART1 TX
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT)){
        setDebugVal(WIFLY_TX_INTH);
        
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
            if(xQueueReceiveFromISR(wiflyOutgoingByteQ, &sendbyteS, 0)) {
               setDebugVal(WIFLY_OUTGOING_BYTE);
               setDebugVal(sendbyteS);
               DRV_USART0_WriteByte(sendbyteS);
            }
            else {
               // txbufferQ is empty. Disable interrupt
               PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
               break;
            }
        }
        setDebugVal(WIFLY_TX_INTH_O);
    }
    
    // SOURCE = UART1 RX
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE)){
        setDebugVal(WIFLY_RX_INTH);
        
        // while there are characters to read
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
            incomingByteS = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            setDebugVal(incomingByteS);
            writeTo_WiFly_rxByteQueue_fromISR(incomingByteS);
            
            

        }
        
        setDebugVal(WIFLY_RX_INTH_O);
    }

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);

}
 
void IntHandlerDrvUsartInstance1(void)
{

    // RTOS USART 2 "PI" - UART port 3 on Max32
    // Pin17(RX2) and Pin16(TX2)

    /* TODO: Add code to process interrupt here */
    
    // SOURCE = UART2 TX
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT)){
        setDebugVal(PI_TX_INTH);  
        
        while(!PLIB_USART_TransmitterBufferIsFull(USART_ID_2)) {
            if(xQueueReceiveFromISR(piOutgoingByteQ, &sendbyteT, 0)) {
               setDebugVal(PI_OUTGOING_BYTE);
               setDebugVal(sendbyteT);
               DRV_USART1_WriteByte(sendbyteT);
            }
            else {
               // txBufferQ is empty. Disable interrupt
               PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
               break;
            }
        }
        
        setDebugVal(PI_TX_INTH_O);

    }
    
    // SOURCE = UART2 RX
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_2_RECEIVE)){
        setDebugVal(PI_RX_INTH);
        
        // while there are characters to read
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_2)){
            // read a character
            incomingByteT = PLIB_USART_ReceiverByteReceive(USART_ID_2);
            writeTo_Pi_rxByteQueue_fromISR(incomingByteT);
        }
        
        setDebugVal(PI_RX_INTH_O);
    }

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);

}


        

     
        
        
    




 
 // *****************************************************************************
// *****************************************************************************
// KYLE: Queues for TX interrupts
// *****************************************************************************
// *****************************************************************************
   
   void initTxByteQueues(){
       
       piOutgoingByteQ = 0;
       wiflyOutgoingByteQ = 0;
       
       // These by queues store bytes waiting to be sent to their
       // corresponding FIFO. The inturrupt routine should be written
       // to read from these queues and send their contents to the 
       // outgoing USART FIFO
       
      piOutgoingByteQ = xQueueCreate(OUTGOING_BYTE_Q, 8);
      
      while(piOutgoingByteQ == 0){
          setDebugVal(WAITING_ON_PI_Q_CREATE);
      }
    
      wiflyOutgoingByteQ = xQueueCreate(OUTGOING_BYTE_Q, 8);
      
      while(wiflyOutgoingByteQ == 0){
          setDebugVal(WAITING_ON_WIFLY_Q_CREATE);
      }
      
      setDebugVal(DONE_BUILDING_USART_QUEUES);
      
   }

   BaseType_t writeTo_WiFly_OutgoingByte_Q(char outgoingByte){
       setDebugVal(WIFLY_WRITE_BYTE);
       setDebugVal(outgoingByte);
       tempS = xQueueSend(wiflyOutgoingByteQ, &outgoingByte, portMAX_DELAY);     
       PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
       return  tempS; 
       
   }
   
   BaseType_t writeTo_Pi_OutgoingByte_Q(char outgoingByte){
       setDebugVal(PI_WRITE_BYTE);
       setDebugVal(outgoingByte);
       tempT = xQueueSend(piOutgoingByteQ, &outgoingByte, portMAX_DELAY);     
       PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
       return  tempT;         
   }
   
   void updateDebugVal(char inVal){
       debugVal = inVal;
   }
 

 
 
  
/*******************************************************************************
 End of File
*/

