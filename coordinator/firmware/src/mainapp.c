/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mainapp.c

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

#include "mainapp.h"
#include "mainapp_public.h"

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

MAINAPP_DATA mainappData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t addToMainAppQ(InternalMessage msg)
{
    return xQueueSend(mainappData.mainAppMsgQ, &msg, portMAX_DELAY);
}
BaseType_t addToMainAppQFromISR(InternalMessage msg)
{
    return xQueueSendFromISR(mainappData.mainAppMsgQ, &msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

int macroInchesToCell(int position)
{
    // Inches per grid space
    int ratio = 4;
    return (position / ratio);
}

int microInchesToCell(int position)
{
    // Inches per grid space
    int ratioMacro = 4;
    int ratioMicro = 2;
    
    int leftover = position % ratioMacro;
    return (leftover / ratioMicro);
}

OBJECT_STRUCTURE convertMessage(InternalMessage message)
{
    OBJECT_STRUCTURE ret;
    ret.type = message.msg[0];
    ret.xPos = message.msg[1] << 8 | message.msg[2];
    ret.yPos = message.msg[3] << 8 | message.msg[4];
    ret.orientation = message.msg[5] << 8 | message.msg[6];
    ret.length = message.msg[7] << 8 | message.msg[8];
    ret.width = message.msg[9] << 8 | message.msg[10];
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAINAPP_Initialize ( void )

  Remarks:
    See prototype in mainapp.h.
 */

void MAINAPP_Initialize ( void )
{
    mainappData.mainAppMsgQ = xQueueCreate(16,sizeof(InternalMessage));
    
    // Making the message sequence number 0
    mainappData.messageNumber = 0;
    
    int i, j, k, l;
    // Making the grid cells initially blank
    for(i = 0; i < 9; i++) {
        for(j = 0; j < 9; j++) {
            for(k = 0; k < 2; k++) {
                for(l = 0; l < 2; l++) {
                    mainappData.macroGrid[i][j].microGrid[k][l] = blank;
                }
            }
        }
    }
    
    // Initialize the rover position at a ridiculous unavailable location
    mainappData.rover.type = ROVER;
    mainappData.rover.length = 4;
    mainappData.rover.width = 4;
    mainappData.rover.xPos = 30;
    mainappData.rover.yPos = 30;
    mainappData.rover.orientation = defaultSetting;
    
    // The counter for the number of obstacles
    mainappData.obstacleCount = 0;
  /*  
    int m;
    // Initializing to ridiculous unavailable values for the obstacle locations
    for(m = 0; m < 4; m++) {
        mainappData.obstacle[m].type = OBSTACLE;
        mainappData.obstacle[m].length = 4;
        mainappData.obstacle[m].width = 4;
        mainappData.obstacle[m].xPos = 30;
        mainappData.obstacle[m].yPos = 30;
        mainappData.obstacle[m].orientation = defaultSetting;
    }
   */
}


/******************************************************************************
  Function:
    void MAINAPP_Tasks ( void )

  Remarks:
    See prototype in mainapp.h.
 */

void MAINAPP_Tasks ( void )
{
    InternalMessage inMessage;
    while(1) {
        if (xQueueReceive(mainappData.mainAppMsgQ, &inMessage, portMAX_DELAY)) {
            if(inMessage.type == OBJECT_POS) {
                OBJECT_STRUCTURE object = convertMessage(inMessage);
                if(object.type == ROVER) {
                    mainappData.rover = object;
                }
                else if(object.type == OBSTACLE) {
                    mainappData.obstacle[mainappData.obstacleCount] = object;
                    mainappData.obstacleCount++;
                }
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
