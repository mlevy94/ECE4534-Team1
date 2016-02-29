/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motorapp.c

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

#include "motorapp.h"
#include "motorapp_public.h"
#include "peripheral/oc/plib_oc.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define OC_LEFT  OC_ID_1
#define OC_RIGHT OC_ID_2

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

MOTORAPP_DATA motorappData;

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
void setLeftForward(bool dir) {
    if (dir) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
}

void setRightForward(bool dir) {
    if (dir) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
}

void motorStart() {
    PLIB_TMR_Start(TMR_ID_2);
    PLIB_OC_Enable(OC_LEFT);
    PLIB_OC_Enable(OC_RIGHT);
}

void motorStop() {
    PLIB_TMR_Stop(TMR_ID_2);
    PLIB_OC_Disable(OC_LEFT);
    PLIB_OC_Disable(OC_RIGHT);
}

void motorMove(char direction, char distance) {
    switch(direction) {
                case(ROVER_FORWARD):
                    setLeftForward(pdTRUE);
                    setRightForward(pdTRUE);
                    motorStart();
                    break;
                case(ROVER_BACKWARD):
                    setLeftForward(pdFALSE);
                    setRightForward(pdFALSE);
                    motorStart();
                    break;
                case(ROVER_LEFT):
                    setLeftForward(pdFALSE);
                    setRightForward(pdTRUE);
                    motorStart();
                    break;
                case(ROVER_RIGHT):
                    setLeftForward(pdTRUE);
                    setRightForward(pdFALSE);
                    motorStart();
                    break;
                case(ROVER_STOP):
                default:
                    motorStop();
                    break;
            }
}

BaseType_t addToMotorQ(InternalMessage msg) {
    return xQueueSend(motorappData.motorQ, &msg, portMAX_DELAY);
}

BaseType_t addToMotorQFromISR(InternalMessage msg) {
    return xQueueSendFromISR(motorappData.motorQ, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTORAPP_Initialize ( void )

  Remarks:
    See prototype in motorapp.h.
 */

void MOTORAPP_Initialize ( void )
{
    motorappData.motorQ = xQueueCreate(10, 8);
    motorappData.cmToms = 15;
    motorappData.leftOCVal = 470;
    motorappData.rightOCVal = 440;
}

/******************************************************************************
  Function:
    void MOTORAPP_Tasks ( void )

  Remarks:
    See prototype in motorapp.h.
 */

void MOTORAPP_Tasks ( void )
{
    setLeftForward(pdTRUE);
    setRightForward(pdTRUE);
    motorStop();
    InternalMessage msg;
    while(1) {
        if(xQueueReceive(motorappData.motorQ, &msg, portMAX_DELAY)) {
            motorMove(msg.msg[0], msg.msg[1]);
        }
    }
}
 

/*******************************************************************************
 End of File
 */
