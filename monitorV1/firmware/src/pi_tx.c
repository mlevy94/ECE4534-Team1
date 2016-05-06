/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pi_tx.c

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

#include "pi_tx.h"

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

PI_TX_DATA pi_txData;

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

    ////////////////////////////////////////////////////////////////////////    
    // MSG QUEUE ACCESS FUNCTIONS
    //////////////////////////////////////////////////////////////////////// 
    BaseType_t sendMsgToPiTxApp(SysMsg inSysMsg){
        return xQueueSend(pi_txData.pi_txInternalQ, &inSysMsg, portMAX_DELAY);
    }
    BaseType_t sendMsgToPiTxApp_fromISR(SysMsg inSysMsg){
        return xQueueSendFromISR(pi_txData.pi_txInternalQ, &inSysMsg, 0);
    }
    BaseType_t sendOutgoingMsgToPiTx(RpiMsg inRpiMsg){
        return xQueueSend(pi_txData.pi_txOutgoingMessageQ, &inRpiMsg, portMAX_DELAY);    
    }
    BaseType_t sendOutgoingMsgToPiTx_fromISR(RpiMsg inRpiMsg){
        return xQueueSendFromISR(pi_txData.pi_txOutgoingMessageQ, &inRpiMsg, 0);
    }

    ////////////////////////////////////////////////////////////////////////    
    // SERIAL TX FUNCTION
    //////////////////////////////////////////////////////////////////////// 
    void sendSerialToPi(RpiMsg outgoingRpiMsg){

        writeTo_Pi_OutgoingByte_Q(0xAA); // Rpi Startbyte
        writeTo_Pi_OutgoingByte_Q(outgoingRpiMsg.type);
        writeTo_Pi_OutgoingByte_Q(outgoingRpiMsg.source);
        writeTo_Pi_OutgoingByte_Q(outgoingRpiMsg.size);

        for(pi_txData.itt = 0 ; pi_txData.itt < outgoingRpiMsg.size ; pi_txData.itt ++){
            writeTo_Pi_OutgoingByte_Q(outgoingRpiMsg.msg[pi_txData.itt]);
        }  

    }

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PI_TX_Initialize ( void )

  Remarks:
    See prototype in pi_tx.h.
 */

void PI_TX_Initialize ( void )
{
    setDebugVal(PITX_INIT_TOP);
    pi_txData.state = PI_TX_STATE_INIT;
    
    pi_txData.pi_txOutgoingMessageQ = xQueueCreate(PI_TX_Q_SIZE, RPI_MSG_SIZE);
    pi_txData.pi_txInternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    
}


/******************************************************************************
  Function:
    void PI_TX_Tasks ( void )

  Remarks:
    See prototype in pi_tx.h.
 */

void PI_TX_Tasks ( void )
{

    switch ( pi_txData.state )
    {

        case PI_TX_STATE_INIT:
        {
            setDebugVal(PITX_INIT_STATE);

            pi_txData.rpiMsg.source = MONITOR;
            pi_txData.rpiMsg.type = OUTPUT_TO_MONITOR;
            pi_txData.rpiMsg.size = 0x03;
            for( pi_txData.itt = 0 ; pi_txData.itt <  pi_txData.rpiMsg.size; pi_txData.itt++){
                pi_txData.rpiMsg.msg[pi_txData.itt] = pi_txData.itt + 0x41;
            }           
            sendSerialToPi(pi_txData.rpiMsg);
            pi_txData.state= PI_TX_STATE_ONE;
            break;
        }

        case PI_TX_STATE_ONE:
        {
            setDebugVal(PITX_S_ONE);
                        
            if(xQueueReceive(pi_txData.pi_txOutgoingMessageQ, &pi_txData.rpiMsg, portMAX_DELAY )){
                sendSerialToPi(pi_txData.rpiMsg);
            }
            
            
            break;
        }

        default:
        {
            setDebugVal(PITX_DEFAULT_STATE);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    setDebugVal(PITX_TASKS_COMPLETE);
}

/*******************************************************************************
 End of File
 */
