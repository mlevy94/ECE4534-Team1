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

/*
 This function converts a number of inches to a cell number.
 */
int macroInchesToCell(int position)
{
    // Inches per grid space
    int ratio = 6;
    return (position / ratio);
}

/*
 This function converts a number of inches to a cell number.
 */
int microInchesToCell(int position)
{
    // Inches per grid space
    int ratioMacro = 6;
    int ratioMicro = 6;
    
    int leftover = position % ratioMacro;
    return (leftover / ratioMicro);
}

/*
 This function is used to convert an internal message to an OBJECT_STRUCTURE
 that my code can easily implement.
 */
void convertMessage(InternalMessage message, OBJECT_STRUCTURE* obj)
{
    OBJECT_STRUCTURE ret;
    int i = 0;
    setDebugBool(pdTRUE);
    for (;i < 11; i++) {
        setDebugVal(message.msg[i]);
    }
    obj->type = message.msg[0];
    obj->xPos = message.msg[1] << 8 | message.msg[2];
    obj->yPos = message.msg[3] << 8 | message.msg[4];
    obj->angle = message.msg[5] << 8 | message.msg[6];
    obj->length = message.msg[7] << 8 | message.msg[8];
    obj->width = message.msg[9] << 8 | message.msg[10];
    
    setDebugBool(pdFALSE);
    
}

void leftToRight(void)
{
    
}

void rightToLeft(void)
{
    
}

void topToBottom(void)
{
    
}

void bottomToTop(void)
{
    
}

void centerAndSpreadOut(void)
{
    
}

/*
 Function to send back a message of the rover's location as determined by the PICs
 */
void sendRoverLocation(void)
{
    InternalMessage roverPosition;
    roverPosition.type = OBJECT_POS;
    roverPosition.size = 11;
    roverPosition.msg[0] = ROVER;
    // MSB then LSB
    roverPosition.msg[1] = mainappData.rover.xPos & 0xFF00 >> 8;
    roverPosition.msg[2] = mainappData.rover.xPos & 0x00FF;
    
    roverPosition.msg[3] = mainappData.rover.yPos & 0xFF00 >> 8;
    roverPosition.msg[4] = mainappData.rover.yPos & 0x00FF;
    
    roverPosition.msg[5] = mainappData.rover.angle & 0xFF00 >> 8;
    roverPosition.msg[6] = mainappData.rover.angle & 0x00FF;
    
    roverPosition.msg[7] = mainappData.rover.length & 0xFF00 >> 8;
    roverPosition.msg[8] = mainappData.rover.length & 0x00FF;
    
    roverPosition.msg[9] = mainappData.rover.width & 0xFF00 >> 8;
    roverPosition.msg[10] = mainappData.rover.width & 0x00FF;
    
    addToUartTXQ(roverPosition);
}

// Large function to run the algorithm
void runAlgorithm(void)
{
    // In this I may need to make a callback function for keeping track of where I want to go
    int i;
    // Empty map condition for testing
    if(mainappData.obstacleCount == 0) {
        // Empty map
    }
    // Non-empty map condition for testing
    else {
        for(i = 0; i < mainappData.obstacleCount; i++) {
            if((mainappData.rover.angle < 90) && (mainappData.rover.angle >= 0)) {
                // Facing approximately North x NorthEast
                // Check for obstacles in front of you
            }
        }
    }
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
    // Creating the messaging queue to receive messages from the UART RX
    mainappData.mainAppMsgQ = xQueueCreate(16,sizeof(InternalMessage));
    
    // Making the message sequence number 0
    mainappData.messageNumber = 0;
    
    int i, j, k, l;
    // Making the grid cells initially blank
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            for(k = 0; k < 6; k++) {
                for(l = 0; l < 6; l++) {
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
    mainappData.rover.angle = 0;
    
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
    // Temporary variable that is used to extract a message from the queue
    InternalMessage inMessage;
    while(1) {
        if (xQueueReceive(mainappData.mainAppMsgQ, &inMessage, portMAX_DELAY)) {
            // Checking for a message about updating the position of an object
            if(inMessage.type == OBJECT_POS) {
                // Converting the message to data that can be used to update my rover or obstacle(s).
                convertMessage(inMessage, &mainappData.object);
                // Object is a rover
                
                setDebugVal(0x72);
                setDebugVal(mainappData.object.type);
                setDebugVal(ROVER);
                setDebugVal((mainappData.object.type & 0xff) == ROVER);
                if((mainappData.object.type & 0xff) == ROVER) {
                    mainappData.rover = mainappData.object;
                    setDebugVal(0x73);
                    addToUartTXQ(makeLocationMessage(mainappData.rover));
                }
                // Object is an obstacle
                else if(mainappData.object.type == OBSTACLE) {
                    // Since this should only occur from the initial messages
                    // Creating a new obstacle
                    mainappData.obstacle[mainappData.obstacleCount] = mainappData.object;
                    
                    // Updating the number of obstacles
                    mainappData.obstacleCount++;
                }
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
