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

BaseType_t addToCommandMsgQ(InternalMessage msg)
{
    return xQueueSend(mainappData.mainAppCommandMsgQ, & msg, portMAX_DELAY);
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
    /*for (;i < 11; i++) {
        setDebugVal(message.msg[i]);
    }*/
    obj->type = message.msg[0];
    obj->xPos = (message.msg[1] << 8) | message.msg[2];
    obj->yPos = (message.msg[3] << 8) | message.msg[4];
    obj->angle = (message.msg[5] << 8) | message.msg[6];
    obj->length = (message.msg[7] << 8) | message.msg[8];
    obj->width = (message.msg[9] << 8) | message.msg[10];
    
    setDebugBool(pdFALSE);
    
}

////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//          EMPTY MAP
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////

void topToBottomEmptyCorners(uint16_t* y, uint16_t* angle)
{
    // Taken from topToBottomEmpty()
    InternalMessage roverCommand;
    // The rover is not facing downwards
    //setDebugVal(*angle);
    if(*angle != 180) {
        // Easier to turn left
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = *angle - 180;
            *angle = *angle - angleLeftover;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        // Easier to turn right
        else {
            uint16_t angleLeftover = 180 - *angle;
            *angle = *angle + angleLeftover;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
        }
        //setDebugVal(*angle);
        addToCommandMsgQ(roverCommand);
    }
    // The rover is facing downwards
    // Have the rover move downwards
    // Accounts for the center location of the rover along with the front of the rover
    int counter = 0;
    uint16_t endPoint = *y + (mainappData.rover.length / 2);
    while(endPoint != 36) {
        // Getting the rover to move the appropriate distance
        if((endPoint + mainappData.rover.length) <= 36) {
            counter++;
            endPoint += mainappData.rover.length;
        }
        // Correction factor in case the center of the rover is not at the center of a grid cell
        else if((36 - endPoint) < mainappData.rover.length) { // Updated from mainappData.rover.length / 2
            uint16_t correction = 36 - endPoint;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            endPoint += correction;
        }
    }
    *y = endPoint - (mainappData.rover.length / 2);
    int i;
    for(i = 0; i < counter; i++) {
        roverCommand = makeRoverMove(ROVER_FORWARD, mainappData.rover.length & 0x00FF);
        addToCommandMsgQ(roverCommand);
    }
}

void bottomToTopEmptyCorners(uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    // The rover is not facing upwards
    if(*angle != 0) {
        // Easier to turn right
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = 360 - *angle;
            *angle = (*angle + angleLeftover) % 360;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
        }
        // Easier to turn left
        else {
            // Need to move left the amount of angle that it is facing
            roverCommand = makeRoverMove(ROVER_LEFT, *angle & 0x00FF);
            *angle = (*angle - *angle);
        }
        addToCommandMsgQ(roverCommand);
    }
    // The rover is facing upwards
    // Have the rover move upwards
    int counter = 0;
    uint16_t endPoint = *y - (mainappData.rover.length / 2);
    while(endPoint != 0) {
        // Getting the rover to move the appropriate distance
        if((endPoint - mainappData.rover.length) >= 0) {
            counter++;
            endPoint -= mainappData.rover.length;
        }
        // Correction factor in case the center of the rover is not at the center of a grid cell
        else if(endPoint % mainappData.rover.length != 0) {
            uint16_t correction = endPoint % mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            endPoint -= correction;
        }
    }
    *y = endPoint + (mainappData.rover.length / 2);
    int i;
    for(i = 0; i < counter; i++) {
        roverCommand = makeRoverMove(ROVER_FORWARD, mainappData.rover.length & 0x00FF);
        addToCommandMsgQ(roverCommand);
    }
}

void moveHorizontalRightOnceCornerEmpty(uint16_t* x, uint16_t* angle)
{
    uint16_t testingEndPoint = *x + (mainappData.rover.length / 2);
    // The rover is not facing right
    InternalMessage roverCommand;
    // The rover is not facing right
    if(*angle != 90) {
        // Easier to turn right
        if(*angle < 90 && *angle >= 0) {
            uint16_t angleLeftover = 90 - *angle;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            *angle = (*angle + angleLeftover + 360) % 360;
        }
        else if(*angle <= 359 && *angle >= 270) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction from 360 degrees is to get the actual leftover angle
            uint16_t angleLeftover = 90 + (360 - *angle);
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            *angle = (*angle + angleLeftover + 360) % 360;
        }
        
        // Easier to turn left
        else if(*angle < 270 && *angle >= 180) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction with 180 degrees is to get the actual leftover angles
            uint16_t angleLeftover = 90 + (*angle - 180);
            *angle = (*angle - angleLeftover + 360) % 360;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        else if(*angle < 180 && *angle > 90) {
            uint16_t angleLeftover = *angle - 90;
            *angle = (*angle - angleLeftover + 360) % 360;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        addToCommandMsgQ(roverCommand);
    }
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 36) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % mainappData.rover.length) < mainappData.rover.length) {
            correction = mainappData.rover.length - (testingEndPoint % mainappData.rover.length);
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *x += correction;
        }
        else if((testingEndPoint + mainappData.rover.length) <= 36) {
            correction = mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *x += correction;
        }
    }
}

void moveHorizontalLeftOnceCornerEmpty(uint16_t* x, uint16_t* angle)
{
    InternalMessage roverCommand;
    uint16_t testingEndPoint = *x - (mainappData.rover.length / 2);
    // The rover is not facing left
    if(*angle != 270) {
        // Easier to turn right
        if(*angle < 270 && *angle >= 90) {
            uint16_t angleLeftover = 270 - *angle;
            *angle = *angle + angleLeftover;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
        }
        
        // Easier to turn left
        else if(*angle <= 359 && *angle > 270) {
            uint16_t angleLeftover = *angle - 270;
            *angle = *angle - angleLeftover;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        else if(*angle >= 0 && *angle < 90) {
            // The 90 degrees is to get to the right quadrant
            uint16_t angleLeftover = *angle + 90;
            *angle = (*angle - angleLeftover - 180) % 360;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        addToCommandMsgQ(roverCommand);
    }
    //setDebugVal(*angle);
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 0) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % mainappData.rover.length) != 0) {
            correction = testingEndPoint % mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *x -= correction;
        }
        else if((testingEndPoint - mainappData.rover.length) >= 0) {
            correction = mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *x -= correction;
        }
    }
}

void moveVerticalUpOnceCornerEmpty(uint16_t* y, uint16_t* angle)
{
    uint16_t testingEndPoint = *y - (mainappData.rover.length / 2);
    // The rover is not facing right
    InternalMessage roverCommand;
    // The rover is not facing upwards
    if(*angle != 0) {
        // Easier to turn right
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = 360 - *angle;
            *angle = (*angle + angleLeftover) % 360;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
        }
        // Easier to turn left
        else {
            // Need to move left the amount of angle that it is facing
            roverCommand = makeRoverMove(ROVER_LEFT, *angle & 0x00FF);
            *angle = (*angle - *angle);
        }
        addToCommandMsgQ(roverCommand);
    }
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 0) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % mainappData.rover.length) < mainappData.rover.length) {
            correction = mainappData.rover.length - (testingEndPoint % mainappData.rover.length);
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *y -= correction;
        }
        else if((testingEndPoint - mainappData.rover.length) >= 0) {
            correction = mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *y -= correction;
        }
    }
}

void moveVerticalDownOnceCornerEmpty(uint16_t* y, uint16_t* angle)
{
    uint16_t testingEndPoint = *y + (mainappData.rover.length / 2);
    // The rover is not facing right
    InternalMessage roverCommand;
    // The rover is not facing downwards
    if(*angle != 180) {
        // Easier to turn left
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = *angle - 180;
            *angle = (*angle - angleLeftover);
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        // Easier to turn right
        else {
            uint16_t angleLeftover = 180 - *angle;
            *angle = *angle + angleLeftover;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
        }
        addToCommandMsgQ(roverCommand);
    }
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 36) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % mainappData.rover.length) < mainappData.rover.length) {
            correction = mainappData.rover.length - (testingEndPoint % mainappData.rover.length);
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *y += correction;
        }
        else if((testingEndPoint + mainappData.rover.length) <= 36) {
            correction = mainappData.rover.length;
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            *y += correction;
        }
    }
}

// Function to traverse the map in a corner case
void upperLeftCornerTraverseEmpty(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Upper left
    if(testX == 0 && testY == 0) {
        if(testX != (mainappData.rover.length / 2)) {
            moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
        }
        if(testY != (mainappData.rover.length / 2)) {
            moveVerticalDownOnceCornerEmpty(&testY, &testAngle);
        }
        topToBottomEmptyCorners(&testY, &testAngle);
        moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);

        // Loop to complete the traversal
        while((testX + (mainappData.rover.length / 2)) < 36) {
            if((testY - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(&testY, &testAngle); 
               moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
            }
            else if((testY + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(&testY, &testAngle);
                moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
            }
        }
        bottomToTopEmptyCorners(&testY, &testAngle);
    }
}

void lowerLeftCornerTraverseEmpty(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Lower left
    if(testX == 0 && testY == 36) {
        if((testX % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
        }
        if((testY % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalUpOnceCornerEmpty(&testY, &testAngle);
        }
        bottomToTopEmptyCorners(&testY, &testAngle);
        moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);

        // Loop to complete the traversal
        while((testX + (mainappData.rover.length / 2)) < 36) {
            if((testY - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(&testY, &testAngle); 
               moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
            }
            else if((testY + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(&testY, &testAngle);
                moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
            }
        }
        topToBottomEmptyCorners(&testY, &testAngle);
    }
}

void upperRightCornerTraverseEmpty(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Upper right
    if(testX == 36 && testY == 0) {
        if((testX % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
        }
        if((testY % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalDownOnceCornerEmpty(&testY, &testAngle);
        }
        topToBottomEmptyCorners(&testY, &testAngle);
        moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
        
        // Loop to complete the traversal
        while((testX - (mainappData.rover.length / 2)) > 0) {
            if((testY - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(&testY, &testAngle); 
               moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
            }
            else if((testY + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(&testY, &testAngle);
                moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
            }
        }
        bottomToTopEmptyCorners(&testY, &testAngle);
    }
}

void lowerRightCornerTraverseEmpty(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Lower right
    if(testX == 36 && testY == 36) {
        if((testX % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
        }
        if((testY % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalUpOnceCornerEmpty(&testY, &testAngle);
        }
        bottomToTopEmptyCorners(&testY, &testAngle);
        moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);

        // Loop to complete the traversal
        while((testX - (mainappData.rover.length / 2)) > 0) {
            if((testY - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(&testY, &testAngle); 
               moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
            }
            else if((testY + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(&testY, &testAngle);
                moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
            }
        }
        topToBottomEmptyCorners(&testY, &testAngle);
    }
}

void upperLeftCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    // Upper left
    if(*x == 0 && y == 0) {
        if(*x != (mainappData.rover.length / 2)) {
            moveHorizontalRightOnceCornerEmpty(x, angle);
        }
        if(*y != (mainappData.rover.length / 2)) {
            moveVerticalDownOnceCornerEmpty(y, angle);
        }
        topToBottomEmptyCorners(y, angle);
        moveHorizontalRightOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x + (mainappData.rover.length / 2)) < 36) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalRightOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalRightOnceCornerEmpty(x, angle);
            }
        }
        bottomToTopEmptyCorners(y, angle);
    }
    else if(*x == 3 && *y == 3) {
        topToBottomEmptyCorners(y, angle);
        moveHorizontalRightOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x + (mainappData.rover.length / 2)) < 36) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalRightOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalRightOnceCornerEmpty(x, angle);
            }
        }
        bottomToTopEmptyCorners(y, angle);
    }
}

void lowerLeftCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    // Lower left
    if(*x == 0 && *y == 36) {
        if((*x % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalRightOnceCornerEmpty(x, angle);
        }
        if((*y % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalUpOnceCornerEmpty(y, angle);
        }
        bottomToTopEmptyCorners(y, angle);
        moveHorizontalRightOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x + (mainappData.rover.length / 2)) < 36) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalRightOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalRightOnceCornerEmpty(x, angle);
            }
        }
        topToBottomEmptyCorners(y, angle);
    }
    else if(*x == 3 && *y == 33) {
        bottomToTopEmptyCorners(y, angle);
        moveHorizontalRightOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x + (mainappData.rover.length / 2)) < 36) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalRightOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalRightOnceCornerEmpty(x, angle);
            }
        }
        topToBottomEmptyCorners(y, angle);
    }
}

void upperRightCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    // Upper right
    if(*x == 36 && *y == 0) {
        if((*x % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalLeftOnceCornerEmpty(x, angle);
        }
        if((*y % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalDownOnceCornerEmpty(y, angle);
        }
        topToBottomEmptyCorners(y, angle);
        moveHorizontalLeftOnceCornerEmpty(x, angle);
        
        // Loop to complete the traversal
        while((*x - (mainappData.rover.length / 2)) > 0) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
        }
        bottomToTopEmptyCorners(y, angle);
    }
    else if(*x == 33 && *y == 3) {
        topToBottomEmptyCorners(y, angle);
        moveHorizontalLeftOnceCornerEmpty(x, angle);
        
        // Loop to complete the traversal
        while((*x - (mainappData.rover.length / 2)) > 0) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
        }
        bottomToTopEmptyCorners(y, angle);
    }
}

void lowerRightCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    // Lower right
    if(*x == 36 && *y == 36) {
        if((*x % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveHorizontalLeftOnceCornerEmpty(x, angle);
        }
        if((*y % mainappData.rover.length) != (mainappData.rover.length / 2)) {
            moveVerticalUpOnceCornerEmpty(y, angle);
        }
        bottomToTopEmptyCorners(y, angle);
        moveHorizontalLeftOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x - (mainappData.rover.length / 2)) > 0) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
        }
        topToBottomEmptyCorners(y, angle);
    }
    else if(*x == 33 && *y == 33) {
        bottomToTopEmptyCorners(y, angle);
        moveHorizontalLeftOnceCornerEmpty(x, angle);

        // Loop to complete the traversal
        while((*x - (mainappData.rover.length / 2)) > 0) {
            if((*y - (mainappData.rover.length / 2)) == 0) {
               topToBottomEmptyCorners(y, angle); 
               moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
            else if((*y + (mainappData.rover.length / 2)) == 36) {
                bottomToTopEmptyCorners(y, angle);
                moveHorizontalLeftOnceCornerEmpty(x, angle);
            }
        }
        topToBottomEmptyCorners(y, angle);
    }
}

void centerAndSpreadOutEmpty(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Center Point
    if(testX == 18 && testY == 18) {
        /* Upper left corner traversal
        bottomToTopEmptyCorners(&testY, &testAngle);
        while((testX - (mainappData.rover.length / 2)) != 0) {
            moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
        }
        upperLeftCornerTraverseEmptyParam(&testX, &testY, &testAngle);
        */
        /* Lower left corner traversal
        topToBottomEmptyCorners(&testY, &testAngle);
        while((testX - (mainappData.rover.length / 2)) != 0) {
            moveHorizontalLeftOnceCornerEmpty(&testX, &testAngle);
        }
        lowerLeftCornerTraverseEmptyParam(&testX, &testY, &testAngle);
        */
        /* Upper right corner traversal
        bottomToTopEmptyCorners(&testY, &testAngle);
        while((testX + (mainappData.rover.length / 2)) != 36) {
            moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
        }
        upperRightCornerTraverseEmptyParam(&testX, &testY, &testAngle);
        */
        /* Lower right corner traversal
        topToBottomEmptyCorners(&testY, &testAngle);
        while((testX + (mainappData.rover.length / 2)) != 36) {
            moveHorizontalRightOnceCornerEmpty(&testX, &testAngle);
        }
        lowerRightCornerTraverseEmptyParam(&testX, &testY, &testAngle);
        */
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//          MAP WITH OBSTACLES
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////

void turnRight(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    // The rover is not facing right
    if(*angle != 90) {
        mainappData.turning = pdTRUE;
        // Easier to turn right
        if(*angle < 90 && *angle >= 0) {
            uint16_t angleLeftover = 90 - *angle;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle + angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_RIGHT;
            mainappData.targetTotal++;
            *angle = *angle + angleLeftover;
        }
        else if(*angle <= 359 && *angle >= 270) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction from 360 degrees is to get the actual leftover angle
            uint16_t angleLeftover = 90 + (360 - *angle);
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = (*angle + angleLeftover) % 360;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_RIGHT;
            mainappData.targetTotal++;
            *angle = (*angle + angleLeftover) % 360;
        }
        
        // Easier to turn left
        else if(*angle < 270 && *angle >= 180) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction with 180 degrees is to get the actual leftover angles
            uint16_t angleLeftover = 90 + (*angle - 180);
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle - angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_RIGHT;
            mainappData.targetTotal++;
            *angle = *angle - angleLeftover;
        }
        else if(*angle < 180 && *angle > 90) {
            uint16_t angleLeftover = *angle - 90;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle - angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_RIGHT;
            mainappData.targetTotal++;
            *angle = *angle - angleLeftover;
        }
        addToCommandMsgQ(roverCommand);
    }
}

void turnLeft(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    // The rover is not facing left
    if(*angle != 270) {
        mainappData.turning = pdTRUE;
        // Easier to turn right
        if(*angle < 270 && *angle >= 90) {
            uint16_t angleLeftover = 270 - *angle;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle + angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_LEFT;
            mainappData.targetTotal++;
            *angle = *angle + angleLeftover;
        }

        // Easier to turn left
        else if(*angle <= 359 && *angle > 270) {
            uint16_t angleLeftover = *angle - 270;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle - angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_LEFT;
            mainappData.targetTotal++;
            *angle = *angle - angleLeftover;
        }
        else if(*angle >= 0 && *angle < 90) {
            // The 90 degrees is to get to the right quadrant
            uint16_t angleLeftover = *angle + 90;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = angleLeftover + 180 - *angle;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_LEFT;
            mainappData.targetTotal++;
            *angle = angleLeftover + 180 - *angle;
        }
        addToCommandMsgQ(roverCommand);
    }
}

void turnUp(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    // The rover is not facing upwards
    if(*angle != 0) {
        mainappData.turning = pdTRUE;
        // Easier to turn right
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = 360 - *angle;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = (*angle + angleLeftover) % 360;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_UP;
            mainappData.targetTotal++;
            *angle = (*angle + angleLeftover) % 360;
        }
        // Easier to turn left
        else {
            // Need to move left the amount of angle that it is facing
            roverCommand = makeRoverMove(ROVER_LEFT, *angle & 0x00FF);
            *angle = (*angle - *angle);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle - *angle;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_UP;
            mainappData.targetTotal++;
            *angle = (*angle - *angle);
        }
        addToCommandMsgQ(roverCommand);
    }
}

void turnDown(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    // The rover is not facing downwards
    if(*angle != 180) {
        mainappData.turning = pdTRUE;
        // Easier to turn left
        if(*angle <= 359 && *angle > 180) {
            uint16_t angleLeftover = *angle - 180;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle - angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_DOWN;
            mainappData.targetTotal++;
            *angle = *angle - angleLeftover;
        }
        // Easier to turn right
        else {
            uint16_t angleLeftover = 180 - *angle;
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            
            // Adding to the next target location / angle / command type
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle + angleLeftover;
            mainappData.targetCommand[mainappData.targetTotal] = FACE_DOWN;
            mainappData.targetTotal++;
            *angle = *angle + angleLeftover;
        }
        addToCommandMsgQ(roverCommand);
    }
}

void moveHorizontalRightOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    uint16_t testingEndPoint = *x + (mainappData.rover.length / 2);
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    // The rover is not facing right
    InternalMessage roverCommand;
    // The rover is not facing right
    turnRight(x, y, angle);
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 36) {
        // Getting the rover to move the appropriate distance
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*x + 6) == mainappData.obstacle[objIterator].xPos) && (*y == mainappData.obstacle[objIterator].yPos)) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }
        // Blocked and must move around
        if(blocked == pdTRUE) {
            // Need to move downwards to go around
            if(mainappData.obstacleAversion == top) {
                // First go downward
                turnDown(x, y, angle);
                // Not centered
                if(((*y + 3) % 6) > 0) {
                    mainappData.moving = pdTRUE;
                    correction = 6 - (testingEndPoint % 6);
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle / command type
                    mainappData.targetX[mainappData.targetTotal] = *x;
                    mainappData.targetY[mainappData.targetTotal] = *y + correction;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_DOWN;
                    mainappData.targetTotal++;
                    
                    *y += correction;
                }
                else if((*y + 6) <= 33) {
                    mainappData.moving = pdTRUE;
                    correction = 6;
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x;
                    mainappData.targetY[mainappData.targetTotal] = *y + correction;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_DOWN;
                    mainappData.targetTotal++;
                    
                    *y += correction;
                }
                // Then go right
                turnRight(x, y, angle);
                if((testingEndPoint % 6) < 6) {
                    mainappData.moving = pdTRUE;
                    correction = 6 - (testingEndPoint % 6);
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x + correction;
                    mainappData.targetY[mainappData.targetTotal] = *y;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                    mainappData.targetTotal++;
                    
                    *x += correction;
                }
                else if((testingEndPoint + 6) <= 36) {
                    mainappData.moving = pdTRUE;
                    correction = 6;
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x + correction;
                    mainappData.targetY[mainappData.targetTotal] = *y;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                    mainappData.targetTotal++;
                    
                    *x += correction;
                }
            }
            else if(mainappData.obstacleAversion == bottom) {
                // First go upward
                turnUp(x, y, angle);
                if(((*y - 3) % 6) < 6) {
                    mainappData.moving = pdTRUE;
                    correction = 6 - (testingEndPoint % 6);
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x;
                    mainappData.targetY[mainappData.targetTotal] = *y - correction;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_UP;
                    mainappData.targetTotal++;
                    
                    *y -= correction;
                }
                else if((*y - 6) >= 3) {
                    mainappData.moving = pdTRUE;
                    correction = 6;
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x;
                    mainappData.targetY[mainappData.targetTotal] = *y - correction;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_UP;
                    mainappData.targetTotal++;
                    
                    *y -= correction;
                }
                // Then go right
                turnRight(x, y, angle);
                if((testingEndPoint % 6) < 6) {
                    mainappData.moving = pdTRUE;
                    correction = 6 - (testingEndPoint % 6);
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x + correction;
                    mainappData.targetY[mainappData.targetTotal] = *y;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                    mainappData.targetTotal++;
                    
                    *x += correction;
                }
                else if((testingEndPoint + 6) <= 36) {
                    mainappData.moving = pdTRUE;
                    correction = 6;
                    
                    // For the message command queue
                    roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                    addToCommandMsgQ(roverCommand);
                    
                    // Adding to the next target location / angle
                    mainappData.targetX[mainappData.targetTotal] = *x + correction;
                    mainappData.targetY[mainappData.targetTotal] = *y;
                    mainappData.targetAngle[mainappData.targetTotal] = *angle;
                    mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                    mainappData.targetTotal++;
                    
                    *x += correction;
                }
            }
        }
        // Simply go right
        else {
            if((testingEndPoint % 6) > 0) {
                mainappData.moving = pdTRUE;
                correction = 6 - (testingEndPoint % 6);
                
                // For the message command queue
                roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                addToCommandMsgQ(roverCommand);
                
                // Adding to the next target location / angle
                mainappData.targetX[mainappData.targetTotal] = *x + correction;
                mainappData.targetY[mainappData.targetTotal] = *y;
                mainappData.targetAngle[mainappData.targetTotal] = *angle;
                mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                mainappData.targetTotal++;
                
                *x += correction;
            }
            else if((testingEndPoint + 6) <= 36) {
                mainappData.moving = pdTRUE;
                correction = 6;
                
                // For the message queue
                roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
                addToCommandMsgQ(roverCommand);
                
                // Adding to the next target location / angle
                mainappData.targetX[mainappData.targetTotal] = *x + correction;
                mainappData.targetY[mainappData.targetTotal] = *y;
                mainappData.targetAngle[mainappData.targetTotal] = *angle;
                mainappData.targetCommand[mainappData.targetTotal] = MOVE_RIGHT;
                mainappData.targetTotal++;
                
                *x += correction;
            }
        }
    }
}

void moveHorizontalLeftOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    InternalMessage roverCommand;
    uint16_t testingEndPoint = *x - (mainappData.rover.length / 2);
    // The rover is not facing left
    turnLeft(x, y, angle);
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 0) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % 6) != 0) {
            mainappData.moving = pdTRUE;
            correction = testingEndPoint % 6;
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x - correction;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_LEFT;
            mainappData.targetTotal++;
            
            *x -= correction;
        }
        else if((testingEndPoint - 6) >= 0) {
            mainappData.moving = pdTRUE;
            correction = 6;
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x - correction;
            mainappData.targetY[mainappData.targetTotal] = *y;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_LEFT;
            mainappData.targetTotal++;
            
            *x -= correction;
        }
    }
}

void moveVerticalUpOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    uint16_t testingEndPoint = *y - (mainappData.rover.length / 2);
    InternalMessage roverCommand;
    turnUp(x, y, angle);
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 0) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % 6) > 0) {
            mainappData.moving = pdTRUE;
            correction = 6 - (testingEndPoint % 6);
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y - correction;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_UP;
            mainappData.targetTotal++;
            
            *y -= correction;
        }
        else if((testingEndPoint - 6) >= 0) {
            mainappData.moving = pdTRUE;
            correction = 6;
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y - correction;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_UP;
            mainappData.targetTotal++;
            
            *y -= correction;
        }
    }
}

void moveVerticalDownOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
     uint16_t testingEndPoint = *y + (mainappData.rover.length / 2);
    // The rover is not facing right
    InternalMessage roverCommand;
    // The rover is not facing downwards
    turnDown(x, y, angle);
    // Getting the rover to move the appropriate distance
    uint16_t correction;
    if(testingEndPoint != 36) {
        // Correction factor in case the center of the rover is not at the center of a grid cell
        if((testingEndPoint % 6) > 0) {
            mainappData.moving = pdTRUE;
            correction = 6 - (testingEndPoint % 6);
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y + correction;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_DOWN;
            mainappData.targetTotal++;
            
            *y += correction;
        }
        else if((testingEndPoint + 6) <= 36) {
            mainappData.moving = pdTRUE;
            correction = 6;
            
            // For the message command queue
            roverCommand = makeRoverMove(ROVER_FORWARD, correction & 0x00FF);
            addToCommandMsgQ(roverCommand);
            
            // Adding to the next target location / angle
            mainappData.targetX[mainappData.targetTotal] = *x;
            mainappData.targetY[mainappData.targetTotal] = *y + correction;
            mainappData.targetAngle[mainappData.targetTotal] = *angle;
            mainappData.targetCommand[mainappData.targetTotal] = MOVE_DOWN;
            mainappData.targetTotal++;
            
            *y += correction;
        }
    }
}

void moveRightUpAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*x + 6) == mainappData.obstacle[objIterator].xPos) && (*y == mainappData.obstacle[objIterator].yPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x + 12) <= 33) {
            // Make the rover turn Upwards
            moveVerticalUpOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveHorizontalRightOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveVerticalDownOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnRight(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveVerticalUpOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveHorizontalRightOnceCorner(x, y, angle);
        }
    }
}

void moveRightDownAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*x + 6) == mainappData.obstacle[objIterator].xPos) && (*y == mainappData.obstacle[objIterator].yPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x + 12) <= 33) {
            // Make the rover turn Upwards
            moveVerticalDownOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveHorizontalRightOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveVerticalUpOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnRight(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveVerticalDownOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveHorizontalRightOnceCorner(x, y, angle);
        }
    }
}

void moveLeftUpAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    if((*x - 6) >= 3) {
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*x - 6) < (mainappData.obstacle[objIterator].xPos + 3)) && ((*x - 6 > (mainappData.obstacle[objIterator].xPos - 3)))
                && (*y < (mainappData.obstacle[objIterator].yPos + 3)) && (*y > (mainappData.obstacle[objIterator].yPos - 3))) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }

        // The obstacle was found
        if(blocked == pdTRUE) {
            // Need to account for edges
            // We are going under the postulated rule that no obstacle will be adjacent to one another
            if((*x - 12) >= 3) {
                // Make the rover turn Upwards
                moveVerticalUpOnceCorner(x, y, angle);
                // Move over right once to be above the obstacle
                moveHorizontalLeftOnceCorner(x, y, angle);
                // Move right once more to get past the obstacle
                moveHorizontalLeftOnceCorner(x, y, angle);
                // Begin moving back to where you need to be
                moveVerticalDownOnceCorner(x, y, angle);

                // Reorient to help the rover go left (again)
                // The rover is not facing left
                turnLeft(x, y, angle);
            }
            // At the right edge of the map
            else {
                // Move up one space
                moveVerticalUpOnceCorner(x, y, angle);
                // Move over to be above the obstacle
                // Already reoriented right
                moveHorizontalLeftOnceCorner(x, y, angle);
            }
        }
    }
}

void moveLeftDownAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*x - 6) == mainappData.obstacle[objIterator].xPos) && (*y == mainappData.obstacle[objIterator].yPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x - 12) >= 3) {
            // Make the rover turn Upwards
            moveVerticalDownOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveVerticalUpOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing left
            turnLeft(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveVerticalDownOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveHorizontalLeftOnceCorner(x, y, angle);
        }
    }
}

void moveUpRightAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*y - 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*y - 12) >= 3) {
            // Make the rover turn Upwards
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveHorizontalLeftOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnUp(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveVerticalUpOnceCorner(x, y, angle);
            mainappData.dontSkip = pdTRUE;
        }
    }
}

void moveUpLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*y - 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*y - 12) >= 3) {
            // Make the rover turn Upwards
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveHorizontalRightOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnUp(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveVerticalUpOnceCorner(x, y, angle);
        }
    }
}

void moveDownRightAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*y + 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*y + 12) <= 33) {
            // Make the rover turn Upwards
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveVerticalDownOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveVerticalDownOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveHorizontalLeftOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnDown(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveVerticalDownOnceCorner(x, y, angle);
            mainappData.dontSkip = pdTRUE;
        }
    }
}
void moveDownLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if(((*y + 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*y + 12) <= 33) {
            // Make the rover turn Upwards
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Move over right once to be above the obstacle
            moveVerticalDownOnceCorner(x, y, angle);
            // Move right once more to get past the obstacle
            moveVerticalDownOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveHorizontalRightOnceCorner(x, y, angle);
            
            // Reorient to help the rover go right (again)
            // The rover is not facing right
            turnDown(x, y, angle);
        }
        // At the right edge of the map
        else {
            // Move up one space
            moveHorizontalLeftOnceCorner(x, y, angle);
            // Move over to be above the obstacle
            // Already reoriented right
            moveVerticalDownOnceCorner(x, y, angle);
        }
    }
}

void moveToRightEdge(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    // Ensures the rover will be facing right
    turnRight(x, y, angle);
    while(*x != 33 && blocked == pdFALSE) {
        
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*x + 6) < (mainappData.obstacle[objIterator].xPos + 3)) && ((*x + 6) > (mainappData.obstacle[objIterator].xPos - 3))
                && (*y < (mainappData.obstacle[objIterator].yPos + 3)) && (*y > (mainappData.obstacle[objIterator].yPos - 3))) {
                objInd = objIterator;
                blocked = pdTRUE;
                setDebugBool(blocked);
            }
        }
        
        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveHorizontalRightOnceCorner(x, y, angle);
        }
    }
}

void moveToLeftEdge(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    while(*x > 3 && blocked == pdFALSE) {
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*x - 6) < (mainappData.obstacle[objIterator].xPos + 3)) && ((*x - 6) > (mainappData.obstacle[objIterator].xPos - 3))
                && (*y < mainappData.obstacle[objIterator].yPos + 3) && (*y > mainappData.obstacle[objIterator].yPos - 3)) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }
        
        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveHorizontalLeftOnceCorner(x, y, angle);
        }
    }
}

void moveToTopEdge(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    while(*y != 3 && blocked == pdFALSE) {
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*y - 6) < (mainappData.obstacle[objIterator].yPos + 3)) && ((*y - 6) > mainappData.obstacle[objIterator].yPos - 3)
                && (*x < (mainappData.obstacle[objIterator].xPos + 3)) && (*x > (mainappData.obstacle[objIterator].xPos - 3))) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }
        
        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveVerticalUpOnceCorner(x, y, angle);
        }
    }
}

void moveToBottomEdge(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    
    while(*y != 33 && blocked == pdFALSE) {
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*y + 6) < (mainappData.obstacle[objIterator].yPos + 3)) && ((*y + 6) > (mainappData.obstacle[objIterator].yPos - 3))
                && (*x < (mainappData.obstacle[objIterator].xPos + 3)) && (*x > (mainappData.obstacle[objIterator].xPos - 3))) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }
        
        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveVerticalDownOnceCorner(x, y, angle);
        }
    }
}

void topToBottomCorners(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    InternalMessage roverCommand;
    
    // The rover is not facing downwards
    turnDown(x, y, angle);
    // The rover is facing downwards
    // Have the rover move downwards
    // Accounts for the center location of the rover along with the front of the rover
    while(*y != 33) {
        // Getting the rover to move the appropriate distance
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*y + 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }

        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveVerticalDownOnceCorner(x, y, angle);
        }
        // Moving around the obstacle
        else {
            moveDownRightAroundObstacle(x, y, angle);
            blocked = pdFALSE;
        }
    }
}

void bottomToTopCorners(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    InternalMessage roverCommand;
    
    // The rover is not facing upwards
    turnUp(x, y, angle);
    // The rover is facing downwards
    // Have the rover move downwards
    // Accounts for the center location of the rover along with the front of the rover
    while(*y != 3) {
        // Getting the rover to move the appropriate distance
        // Searching for the obstacle blocking the rover
        for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
            // Finding the obstacle
            if(((*y - 6) == mainappData.obstacle[objIterator].yPos) && (*x == mainappData.obstacle[objIterator].xPos)) {
                objInd = objIterator;
                blocked = pdTRUE;
            }
        }

        // Keep moving if no object was blocking the rover
        if(blocked == pdFALSE) {
            moveVerticalUpOnceCorner(x, y, angle);
        }
        // Moving around the obstacle
        else {
            if(*x != 33) {
                moveUpRightAroundObstacle(x, y, angle);
            }
            else {
                moveUpLeftAroundObstacle(x, y, angle);
            }
            blocked = pdFALSE;
        }
    }
}

void upperLeftCornerTraverse(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Upper left
    // Fixing the position to go to the corner
    while(testX != 3) {
        moveToLeftEdge(&testX, &testY, &testAngle);
        moveLeftUpAroundObstacle(&testX, &testY, &testAngle);
    }
    while(testY != 3) {
        moveToTopEdge(&testX, &testY, &testAngle);
        moveUpRightAroundObstacle(&testX, &testY, &testAngle);
    }
    
    // Loop to complete the traversal
    while(testX != 33) {
        // Need to go downwards
        if(testY >= 3) {
            topToBottomCorners(&testX, &testY, &testAngle);
            mainappData.obstacleAversion = bottom;

            if(mainappData.dontSkip == pdFALSE) {
                moveHorizontalRightOnceCorner(&testX, &testY, &testAngle);
            }
            else {
                mainappData.dontSkip = pdFALSE;
            }
        }
        // Need to go upwards
        if(testY <= 33) { // <- why is this an if and not an else if but works properly?
            bottomToTopCorners(&testX, &testY, &testAngle);
            mainappData.obstacleAversion = top;

            if(mainappData.dontSkip == pdFALSE) {
                moveHorizontalRightOnceCorner(&testX, &testY, &testAngle);
            }
            else {
                mainappData.dontSkip = pdFALSE;
            }
        }
    }
    // Last step to go backwards to get all the blank spaces
    bottomToTopEmptyCorners(&testY, &testAngle);
}

void lowerLeftCornerTraverse(void)
{
    // Using these temporary values to pretend to have the rover move while I send the commands to the command queue
    uint16_t testX = mainappData.rover.xPos;
    uint16_t testY = mainappData.rover.yPos;
    uint16_t testAngle = mainappData.rover.angle;
    // Upper left
    while(testX != 3 && testY != 33) {
        moveToLeftEdge(&testX, &testY, &testAngle);
        moveLeftDownAroundObstacle(&testX, &testY, &testAngle);
        
        moveToBottomEdge(&testX, &testY, &testAngle);
        moveDownRightAroundObstacle(&testX, &testY, &testAngle);
    }

    // Loop to complete the traversal
    while((testX + (mainappData.rover.length / 2)) < 36) {
        if((testY - (mainappData.rover.length / 2)) == 0) {
           topToBottomCorners(&testX, &testY, &testAngle);
           moveDownRightAroundObstacle(&testX, &testY, &testAngle);
           
           moveHorizontalRightOnceCorner(&testX, &testY, &testAngle);
           moveRightUpAroundObstacle(&testX, &testY, &testAngle);
        }
        else if((testY + (mainappData.rover.length / 2)) == 36) {
            bottomToTopCorners(&testX, &testY, &testAngle);
            moveUpRightAroundObstacle(&testX, &testY, &testAngle);
            
            moveHorizontalRightOnceCorner(&testX, &testY, &testAngle);
            moveRightDownAroundObstacle(&testX, &testY, &testAngle);
        }
    }
    bottomToTopEmptyCorners(&testY, &testAngle);
}

void upperRightCornerTraverse(void)
{
    
}

void lowerRightCornerTraverse(void)
{
    
}

void centerAndSpreadOut(void)
{
    
}

void upperLeftCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    // Upper left
    // Fixing the position to go to the corner
    while(*x != 3) {
        moveToLeftEdge(x, y, angle);
        moveLeftUpAroundObstacle(x, y, angle);
    }
    while(*y != 3) {
        moveToTopEdge(x, y, angle);
        moveUpRightAroundObstacle(x, y, angle);
    }
    
    // Loop to complete the traversal
    while(*x != 33) {
        // Need to go downwards
        if(*y >= 3) {
            topToBottomCorners(x, y, angle);
            mainappData.obstacleAversion = bottom;

            if(mainappData.dontSkip == pdFALSE) {
                moveHorizontalRightOnceCorner(x, y, angle);
            }
            else {
                mainappData.dontSkip = pdFALSE;
            }
        }
        // Need to go upwards
        if(*y <= 33) { // <- why is this an if and not an else if but works properly?
            bottomToTopCorners(x, y, angle);
            mainappData.obstacleAversion = top;

            if(mainappData.dontSkip == pdFALSE) {
                moveHorizontalRightOnceCorner(x, y, angle);
            }
            else {
                mainappData.dontSkip = pdFALSE;
            }
        }
    }
    // Last step to go backwards to get all the blank spaces
    bottomToTopEmptyCorners(y, angle);
}

void lowerLeftCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void upperRightCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void lowerRightCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle)
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
    roverPosition.msg[1] = (mainappData.rover.xPos & 0xFF00) >> 8;
    roverPosition.msg[2] = mainappData.rover.xPos & 0x00FF;
    
    roverPosition.msg[3] = (mainappData.rover.yPos & 0xFF00) >> 8;
    roverPosition.msg[4] = mainappData.rover.yPos & 0x00FF;
    
    roverPosition.msg[5] = (mainappData.rover.angle & 0xFF00) >> 8;
    roverPosition.msg[6] = mainappData.rover.angle & 0x00FF;
    
    roverPosition.msg[7] = (mainappData.rover.length & 0xFF00) >> 8;
    roverPosition.msg[8] = mainappData.rover.length & 0x00FF;
    
    roverPosition.msg[9] = (mainappData.rover.width & 0xFF00) >> 8;
    roverPosition.msg[10] = mainappData.rover.width & 0x00FF;
    
    addToUartTXQ(roverPosition);
}

/*
 Function to send back a message of the rover's location as determined by the PICs
 */
void sendTokenUpdate(uint16_t x, uint16_t y, uint16_t length, uint16_t width)
{
    InternalMessage token;
    token.type = TOKEN_FOUND;
    token.size = 11;
    token.msg[0] = TOKEN;
    // MSB then LSB
    token.msg[1] = (x & 0xFF00) >> 8;
    token.msg[2] = x & 0x00FF;
    
    token.msg[3] = (y & 0xFF00) >> 8;
    token.msg[4] = y & 0x00FF;
    
    token.msg[5] = (mainappData.tokenCount & 0xFF00) >> 8;
    token.msg[6] = mainappData.tokenCount & 0x00FF;
    
    token.msg[7] = (mainappData.tokenCur & 0xFF00) >> 8;
    token.msg[8] = mainappData.tokenCur & 0x00FF;
    
    token.msg[9] = length & 0x00FF;
    token.msg[10] = width & 0x00FF;
    
    addToUartTXQ(token);
}

void sendObstacleLocations(void)
{
    int o;
    InternalMessage obstaclePosition;
    for(o = 0; o < mainappData.obstacleCur; o++) {
        obstaclePosition.type = OBJECT_POS;
        obstaclePosition.size = 11;
        obstaclePosition.msg[0] = OBSTACLE;
        // MSB then LSB
        obstaclePosition.msg[1] = (mainappData.obstacle[o].xPos & 0xFF00) >> 8;
        obstaclePosition.msg[2] = mainappData.obstacle[o].xPos & 0x00FF;

        obstaclePosition.msg[3] = (mainappData.obstacle[o].yPos & 0xFF00) >> 8;
        obstaclePosition.msg[4] = mainappData.obstacle[o].yPos & 0x00FF;

        obstaclePosition.msg[5] = (mainappData.obstacle[o].angle & 0xFF00) >> 8;
        obstaclePosition.msg[6] = mainappData.obstacle[o].angle & 0x00FF;

        obstaclePosition.msg[7] = (mainappData.obstacle[o].length & 0xFF00) >> 8;
        obstaclePosition.msg[8] = mainappData.obstacle[o].length & 0x00FF;

        obstaclePosition.msg[9] = (mainappData.obstacle[o].width & 0xFF00) >> 8;
        obstaclePosition.msg[10] = mainappData.obstacle[o].width & 0x00FF;

        addToUartTXQ(obstaclePosition);
    }
}

// Large function to run the algorithm in an empty map
void runAlgorithmEmpty(void)
{
    
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
    
    // Creating the messaging queue for sending rover commands
    mainappData.mainAppCommandMsgQ = xQueueCreate(120, sizeof(InternalMessage));
    
    // Making the message sequence number 0
    mainappData.messageNumber = 0;
    
    // Cleaning the grid
    int i, j;
    // Making the grid cells initially blank
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            mainappData.macroGrid[i][j] = blank;
        }
    }
    
    // Cleaning the token locations for defaults
    int a;
    for(a = 0; a < 4; a++) {
        mainappData.token[a].xPos = 40;
        mainappData.token[a].yPos = 40;
    }
    
    // Cleaning the target command positions and angle and commands
    int b;
    for(b = 0; b < 300; b++) {
        mainappData.targetX[b] = 200;
        mainappData.targetY[b] = 200;
        mainappData.targetAngle[b] = 200;
        mainappData.targetCommand[b] = 200;
    }
    
    // Initialize the rover position at a ridiculous unavailable location
    mainappData.rover.type = ROVER;
    mainappData.rover.length = 20;
    mainappData.rover.width = 20;
    mainappData.rover.xPos = 50;
    mainappData.rover.yPos = 50;
    mainappData.rover.angle = 0;
    
    // The counter for the number of obstacles
    mainappData.obstacleCount = -1;
    
    // The current position of the obstacles
    mainappData.obstacleCur = 0;
    
    // The counter for the number of tokens
    mainappData.tokenCount = -1;
    
    // The current position of the tokens
    mainappData.tokenCur = 0;
    
    // Current total for target positions / angles is 0 as a default
    mainappData.targetTotal = 0;
    
    // Index for the positions / angles for the next positions / angles
    mainappData.targetIndex = 0;
    
    // Traversal algorithm type is initially unknown
    mainappData.obstacleAversion = unknown;
    
    // The rover is initially not moving
    mainappData.roverMotion = noMotion;
    
    // No obstacle is currently at the end of a y-axis side
    mainappData.dontSkip = pdFALSE;
    
    // The rover is not initially turning
    mainappData.turning = pdFALSE;
    
    // The rover is not initially moving
    mainappData.moving = pdFALSE;
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
                if((mainappData.object.type & 0xff) == ROVER) {
                    
                    // Initializing the rover location and also creating the algorithm for path traversal
                    if(mainappData.rover.xPos == 50 && mainappData.rover.yPos == 50) {
                        mainappData.rover = mainappData.object;
                    
                        /////////////////////////////////////////////////////////////////////
                        //
                        //
                        //  Empty Map
                        //
                        //
                        /////////////////////////////////////////////////////////////////////
                        
                        /*Testing the ability of the rover to go to the ends of the map
                        topToBottomEmpty();
                        bottomToTopEmpty();
                        rightToLeftEmpty();
                        leftToRightEmpty();
                        
                         Testing the ability of the rover to take one command
                        moveHorizontalLeftOnceEmpty();
                        moveHorizontalRightOnceEmpty();
                        
                         Testing the ability of the rover to traverse an empty map
                        upperLeftCornerTraverseEmpty();
                        lowerLeftCornerTraverseEmpty();
                        upperRightCornerTraverseEmpty();
                        lowerRightCornerTraverseEmpty();
                        centerAndSpreadOutEmpty();*/
                        
                        /////////////////////////////////////////////////////////////////////
                        //
                        //
                        //  Map with Obstacles
                        //
                        //
                        /////////////////////////////////////////////////////////////////////
                        
                        /*Testing for obstacles
                        moveHorizontalRightOnceCorner(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveHorizontalLeftOnceCorner(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveVerticalUpOnceCorner(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveVerticalDownOnceCorner(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        
                        moveRightUpAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveRightDownAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveLeftUpAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveLeftDownAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        
                        moveUpRightAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveUpLeftAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveDownRightAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveDownLeftAroundObstacle(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        
                        moveToRightEdge(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveToLeftEdge(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveToTopEdge(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        moveToBottomEdge(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);*/
                        
                        //upperLeftCornerTraverse();
                        upperLeftCornerTraverseParam(&mainappData.rover.xPos, &mainappData.rover.yPos, &mainappData.rover.angle);
                        
                        // Send the initial command
                        InternalMessage command;
                        if(xQueueReceive(mainappData.mainAppCommandMsgQ, &command, portMAX_DELAY)) {
                            addToUartTXQ(command);
                        }
                    }
                    // Update the current location of the rover
                    else {
                        mainappData.rover = mainappData.object;
                        // Dealing with corrections to get to the next location
                        /* Need to immediately add to the queue rather than at the next location */
                        /*if(mainappData.targetCommand[mainappData.targetIndex] == MOVE_UP && !(mainappData.rover.angle < 6
                           || mainappData.rover.angle > 354) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == MOVE_DOWN && !(mainappData.rover.angle < 186
                                 && mainappData.rover.angle > 174) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == MOVE_RIGHT && !(mainappData.rover.angle < 96
                                 && mainappData.rover.angle > 84) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == MOVE_LEFT && !(mainappData.rover.angle < 276
                                 && mainappData.rover.angle > 264) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == FACE_UP && !(mainappData.rover.angle < 6
                           || mainappData.rover.angle > 354) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == FACE_DOWN && !(mainappData.rover.angle < 186
                                 && mainappData.rover.angle > 174) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == FACE_RIGHT && !(mainappData.rover.angle < 96
                                 && mainappData.rover.angle > 84) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }
                        else if (mainappData.targetCommand[mainappData.targetIndex] == FACE_LEFT && !(mainappData.rover.angle < 276
                                 && mainappData.rover.angle > 264) && !mainappData.turning) {
                            InternalMessage roverStop;
                            roverStop.type = ROVER_MOVE;
                            roverStop.size = 2;
                            roverStop.msg[0] = ROVER_STOP;
                            roverStop.msg[1] = 0;
                            addToUartTXQ(roverStop);
                        }*/
                    }
                    
                    /* Adding a previous rover location to the macro/micro grid */
                    /*int16_t xGrid = mainappData.rover.xPos / 6;
                    int16_t yGrid = mainappData.rover.yPos / 6;
                    mainappData.macroGrid[xGrid][yGrid] = lead_rover;*/
                    
                    /* Debugging for sending back the rover location */
                    //sendRoverLocation();
                    
                }
                // Object is an obstacle
                else if((mainappData.object.type & 0xff) == OBSTACLE) {
                    // Since this should only occur from the initial messages
                    // Creating a new obstacle
                    mainappData.obstacle[mainappData.obstacleCur] = mainappData.object;
                    
                    // Updating the number of obstacles
                    if(mainappData.obstacleCount == -1) {
                        mainappData.obstacleCount = (int) mainappData.object.angle;
                    }
                    mainappData.obstacleCur++;
                    
                    /* Debugging for sending back all the obstacle locations */
                    /*if(mainappData.obstacleCur == mainappData.obstacleCount) {
                        sendObstacleLocations();
                    }*/
                }
                // Lead rover finished moving so I can now send the next command
                else if((mainappData.object.type & 0xff) == LEAD_ROVER_UPDATE) {
                    InternalMessage roverCorrectionMessage;
                    mainappData.turning = pdFALSE;
                    mainappData.moving = pdFALSE;
                    if(mainappData.object.xPos == 0 && mainappData.object.yPos == 0 && mainappData.object.angle == 0) {
                    if((mainappData.rover.xPos < mainappData.targetX[mainappData.targetIndex] + 3) &&
                       (mainappData.rover.xPos > mainappData.targetX[mainappData.targetIndex] - 3) &&
                       (mainappData.rover.yPos < mainappData.targetY[mainappData.targetIndex] + 3) &&
                       (mainappData.rover.yPos > mainappData.targetY[mainappData.targetIndex] - 3)) {
                        mainappData.targetIndex++;
                        InternalMessage failure;
                        failure.type = OBJECT_POS;
                        failure.size = 11;
                        failure.msg[0] = ROVER;
                        failure.msg[1] = (mainappData.targetX[mainappData.targetIndex] & 0xFF00) >> 8;
                        failure.msg[2] = mainappData.targetX[mainappData.targetIndex] & 0x00FF;
                        failure.msg[3] = (mainappData.targetY[mainappData.targetIndex] & 0xFF00) >> 8;
                        failure.msg[4] = mainappData.targetY[mainappData.targetIndex] & 0x00FF;
                        failure.msg[5] = (mainappData.targetAngle[mainappData.targetIndex] & 0xFF00) >> 8;
                        failure.msg[6] = mainappData.targetAngle[mainappData.targetIndex] & 0x00FF;
                        failure.msg[7] = 0x00;
                        failure.msg[8] = 0x06;
                        failure.msg[9] = 0x00;
                        failure.msg[10] = 0x06;
                        addToUartTXQ(failure);
                    }
                    else {
                        if(((mainappData.rover.xPos & 0x00FF) / 6) == ((mainappData.targetX[mainappData.targetIndex] & 0x00FF) / 6) &&
                            ((mainappData.rover.yPos & 0x00FF) / 6) > ((mainappData.targetY[mainappData.targetIndex] & 0x00FF) / 6)) {
                            if(!((mainappData.rover.angle & 0x00FF) < 6)) { // || (mainappData.rover.angle & 0x0FFF) > 354
                                mainappData.turning = pdTRUE;
                                // Easier to turn right
                                if((mainappData.rover.angle & 0x0FFF) <= 359 && (mainappData.rover.angle & 0x0FFF) > 180) {
                                    uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                // Easier to turn left
                                else {
                                    uint16_t angleLeftover = mainappData.rover.angle;
                                    // Need to move left the amount of angle that it is facing
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                                addToUartTXQ(roverCorrectionMessage);
                            }
                            else {
                                mainappData.moving = pdTRUE;
                                uint16_t distanceLeftover = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                addToUartTXQ(roverCorrectionMessage);
                            }
                        }
                        else if (((mainappData.rover.xPos & 0x00FF) / 6) == ((mainappData.targetX[mainappData.targetIndex] & 0x00FF) / 6) &&
                            ((mainappData.rover.yPos & 0x00FF) / 6) < ((mainappData.targetY[mainappData.targetIndex] & 0x00FF) / 6)) {
                            if(!((mainappData.rover.angle & 0x00FF) < 186 && (mainappData.rover.angle & 0x00FF) > 174)) { // The condition that fails is mainappData.rover.angle < 186
                                mainappData.turning = pdTRUE;
                                // Easier to turn right
                                if((mainappData.rover.angle & 0x0FFF) <= 359 && (mainappData.rover.angle & 0x0FFF) > 180) {
                                    uint16_t angleLeftover = mainappData.rover.angle - 180;
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                                // Easier to turn left
                                else {
                                    // Need to move left the amount of angle that it is facing
                                    uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                addToUartTXQ(roverCorrectionMessage);
                            }
                            else {
                                mainappData.moving = pdTRUE;
                                uint16_t distanceLeftover = mainappData.targetY[mainappData.targetIndex] - mainappData.rover.yPos;
                                roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                addToUartTXQ(roverCorrectionMessage);
                            }
                        }
                        else if(((mainappData.rover.yPos & 0x00FF) / 6) == ((mainappData.targetY[mainappData.targetIndex] & 0x00FF) / 6) &&
                            ((mainappData.rover.xPos & 0x00FF) / 6) > ((mainappData.targetX[mainappData.targetIndex] & 0x00FF) / 6)) {
                            // Checking if it is facing an angle that is not within margin of error
                            if(!((mainappData.rover.angle & 0x0FFF) < 276 && (mainappData.rover.angle & 0x0FFF) > 264)) {
                                mainappData.turning = pdTRUE;
                                // Easier to turn right
                                if((mainappData.rover.angle & 0x0FFF) < 270 || (mainappData.rover.angle & 0x0FFF) > 180) {
                                    uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                else if((mainappData.rover.angle & 0x00FF) <= 180 && (mainappData.rover.angle & 0x00FF) >= 90) {
                                    uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                // Easier to turn left
                                else if((mainappData.rover.angle & 0x0FFF) <= 359 && (mainappData.rover.angle & 0x0FFF) > 270) {
                                    uint16_t angleLeftover = mainappData.rover.angle - 270;
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                                else if((mainappData.rover.angle & 0x00FF) >= 0 && (mainappData.rover.angle & 0x00FF) < 90) {
                                    // The 90 degrees is to get to the right quadrant
                                    uint16_t angleLeftover = mainappData.rover.angle + 90;
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                            addToUartTXQ(roverCorrectionMessage);
                            }
                            else {
                                mainappData.moving = pdTRUE;
                                uint16_t distanceLeftover = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                addToUartTXQ(roverCorrectionMessage);
                            }
                        }
                        else if(((mainappData.rover.yPos & 0x00FF) / 6) == ((mainappData.targetY[mainappData.targetIndex] & 0x00FF) / 6) &&
                            ((mainappData.rover.xPos & 0x00FF) / 6) < ((mainappData.targetX[mainappData.targetIndex] & 0x00FF) / 6)) {
                            setDebugVal(0x72);
                            setDebugVal(mainappData.rover.angle);
                            setDebugVal(0x73);
                            // Checking if it is facing an angle that is not within margin of error
                            // The rover is not facing right
                            if(!((mainappData.rover.angle & 0x00FF) < 96 && (mainappData.rover.angle & 0x00FF) > 84)) {
                                setDebugVal(0x74);
                                setDebugVal(mainappData.rover.angle & 0x00FF);
                                mainappData.turning = pdTRUE;
                                // Easier to turn right
                                if((mainappData.rover.angle & 0x00FF) < 90 && (mainappData.rover.angle & 0x00FF) >= 0) {
                                    setDebugVal(0x75);
                                    uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                else if((mainappData.rover.angle & 0x0FFF) <= 359 && (mainappData.rover.angle & 0x0FFF) >= 270) {
                                    setDebugVal(0x76);
                                    // The 90 degrees is to get to the right quadrant
                                    // The subtraction from 360 degrees is to get the actual leftover angle
                                    uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                    roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                }
                                // Easier to turn left
                                else if((mainappData.rover.angle & 0x0FFF) < 270 || (mainappData.rover.angle & 0x0FFF) >= 180) {
                                    setDebugVal(0x77);
                                    // The 90 degrees is to get to the right quadrant
                                    // The subtraction with 180 degrees is to get the actual leftover angles
                                    uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                                else if((mainappData.rover.angle & 0x00FF) < 180 && (mainappData.rover.angle & 0x00FF) > 90) {
                                    setDebugVal(0x78);
                                    uint16_t angleLeftover = mainappData.rover.angle - 90;
                                    roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                }
                                setDebugVal(0x79);
                                addToUartTXQ(roverCorrectionMessage);
                            }
                            else {
                                mainappData.moving = pdTRUE;
                                uint16_t distanceLeftover = mainappData.targetX[mainappData.targetIndex] - mainappData.rover.xPos;
                                roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                addToUartTXQ(roverCorrectionMessage);
                            }
                        }
                            /*InternalMessage roverCorrectionMessage;
                            ///////////////////////////////////////////////////
                            //
                            //  Determining the correction to do
                            //
                            ///////////////////////////////////////////////////
                            // Figuring out where the rover is intending to go in order to do corrections
                            // Turning up
                            if(mainappData.targetCommand[mainappData.targetIndex] == FACE_UP) {
                                mainappData.roverMotion = turningUp;
                            }
                            // Turning down
                            else if(mainappData.targetCommand[mainappData.targetIndex] == FACE_DOWN) {
                                mainappData.roverMotion = turningDown;
                            }
                            // Turning right
                            else if (mainappData.targetCommand[mainappData.targetIndex] == FACE_RIGHT) {
                                mainappData.roverMotion = turningRight;
                            }
                            // Turning left
                            else if(mainappData.targetCommand[mainappData.targetIndex] == FACE_LEFT) {
                                mainappData.roverMotion = turningLeft;
                            }
                            // Moving up
                            else if(mainappData.targetCommand[mainappData.targetIndex] == MOVE_UP) {
                                mainappData.roverMotion = movingUp;
                            }
                            // Moving down
                            else if(mainappData.targetCommand[mainappData.targetIndex] == MOVE_DOWN) {
                                mainappData.roverMotion = movingDown;
                            }
                            // Moving right
                            else if(mainappData.targetCommand[mainappData.targetIndex] == MOVE_RIGHT) {
                                mainappData.roverMotion = movingRight;
                            }
                            // Moving left
                            else if(mainappData.targetCommand[mainappData.targetIndex] == MOVE_LEFT) {
                                mainappData.roverMotion = movingLeft;
                            }
                            else {
                                mainappData.roverMotion = noMotion;
                            }
                            
                            ///////////////////////////////////////////////////
                            //
                            //  Doing the corrections
                            //
                            ///////////////////////////////////////////////////
                            // See where the rover is moving and if it is going the right way
                            if(mainappData.roverMotion == movingUp) {
                                uint16_t testX = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                uint16_t testY = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                if(testX != 0) {
                                    // Checking if it is facing an angle that is not within margin of error
                                    if(mainappData.rover.xPos > mainappData.targetX[mainappData.targetIndex]) {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 276 && mainappData.rover.angle > 264) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 90) {
                                                uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 270) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 270;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle >= 0 && mainappData.rover.angle < 90) {
                                                // The 90 degrees is to get to the right quadrant
                                                uint16_t angleLeftover = mainappData.rover.angle + 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                        addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        // The rover is not facing right
                                        if(mainappData.rover.angle < 96 && mainappData.rover.angle > 84) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetX[mainappData.targetIndex] - mainappData.rover.xPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 90 && mainappData.rover.angle >= 0) {
                                                uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle >= 270) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction from 360 degrees is to get the actual leftover angle
                                                uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 180) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction with 180 degrees is to get the actual leftover angles
                                                uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle < 180 && mainappData.rover.angle > 90) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                                else if(testY != 0) {
                                    if(mainappData.rover.yPos > mainappData.targetY[mainappData.targetIndex]) {
                                        // Checking if it is facing in an angle that is not within margin of error
                                        // Rover is in the wrong place
                                        if(mainappData.rover.angle < 6 || mainappData.rover.angle > 354) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else {
                                                // Need to move left the amount of angle that it is facing
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, mainappData.rover.angle & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 186 && mainappData.rover.angle > 174) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetY[mainappData.targetIndex] - mainappData.rover.yPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn left
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 180;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn right
                                            else {
                                                uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                            }
                            else if(mainappData.roverMotion == movingDown) {
                                uint16_t testX = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                uint16_t testY = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                if(testX != 0) {
                                    // Checking if it is facing an angle that is not within margin of error
                                    if(mainappData.rover.xPos > mainappData.targetX[mainappData.targetIndex]) {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 276 && mainappData.rover.angle > 264) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 90) {
                                                uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 270) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 270;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle >= 0 && mainappData.rover.angle < 90) {
                                                // The 90 degrees is to get to the right quadrant
                                                uint16_t angleLeftover = mainappData.rover.angle + 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        // The rover is not facing right
                                        if(mainappData.rover.angle < 96 && mainappData.rover.angle > 84) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetX[mainappData.targetIndex] - mainappData.rover.xPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 90 && mainappData.rover.angle >= 0) {
                                                uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle >= 270) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction from 360 degrees is to get the actual leftover angle
                                                uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 180) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction with 180 degrees is to get the actual leftover angles
                                                uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle < 180 && mainappData.rover.angle > 90) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                                else if(testY != 0) {
                                    if(mainappData.rover.yPos > mainappData.targetY[mainappData.targetIndex]) {
                                        // Checking if it is facing in an angle that is not within margin of error
                                        // Rover is in the wrong place
                                        if(mainappData.rover.angle < 6 || mainappData.rover.angle > 354) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else {
                                                // Need to move left the amount of angle that it is facing
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, mainappData.rover.angle & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 186 && mainappData.rover.angle > 174) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetY[mainappData.targetIndex] - mainappData.rover.yPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn left
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 180;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn right
                                            else {
                                                uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                            }
                            else if(mainappData.roverMotion == movingLeft) {
                                uint16_t testX = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                uint16_t testY = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                if(testX != 0) {
                                    // Checking if it is facing an angle that is not within margin of error
                                    if(mainappData.rover.xPos > mainappData.targetX[mainappData.targetIndex]) {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 276 && mainappData.rover.angle > 264) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 90) {
                                                uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 270) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 270;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle >= 0 && mainappData.rover.angle < 90) {
                                                // The 90 degrees is to get to the right quadrant
                                                uint16_t angleLeftover = mainappData.rover.angle + 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        // The rover is not facing right
                                        if(mainappData.rover.angle < 96 && mainappData.rover.angle > 84) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetX[mainappData.targetIndex] - mainappData.rover.xPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 90 && mainappData.rover.angle >= 0) {
                                                uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle >= 270) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction from 360 degrees is to get the actual leftover angle
                                                uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 180) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction with 180 degrees is to get the actual leftover angles
                                                uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle < 180 && mainappData.rover.angle > 90) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                                else if(testY != 0) {
                                    if(mainappData.rover.yPos > mainappData.targetY[mainappData.targetIndex]) {
                                        // Checking if it is facing in an angle that is not within margin of error
                                        // Rover is in the wrong place
                                        if(mainappData.rover.angle < 6 || mainappData.rover.angle > 354) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else {
                                                // Need to move left the amount of angle that it is facing
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, mainappData.rover.angle & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 186 && mainappData.rover.angle > 174) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetY[mainappData.targetIndex] - mainappData.rover.yPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn left
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 180;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn right
                                            else {
                                                uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                            }
                            else if(mainappData.roverMotion == movingRight) {
                                uint16_t testX = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                uint16_t testY = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                if(testX != 0) {
                                    // Checking if it is facing an angle that is not within margin of error
                                    if(mainappData.rover.xPos > mainappData.targetX[mainappData.targetIndex]) {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 276 && mainappData.rover.angle > 264) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.xPos - mainappData.targetX[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 90) {
                                                uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 270) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 270;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle >= 0 && mainappData.rover.angle < 90) {
                                                // The 90 degrees is to get to the right quadrant
                                                uint16_t angleLeftover = mainappData.rover.angle + 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                        addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        // The rover is not facing right
                                        if(mainappData.rover.angle < 96 && mainappData.rover.angle > 84) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetX[mainappData.targetIndex] - mainappData.rover.xPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle < 90 && mainappData.rover.angle >= 0) {
                                                uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle <= 359 && mainappData.rover.angle >= 270) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction from 360 degrees is to get the actual leftover angle
                                                uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 180) {
                                                // The 90 degrees is to get to the right quadrant
                                                // The subtraction with 180 degrees is to get the actual leftover angles
                                                uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            else if(mainappData.rover.angle < 180 && mainappData.rover.angle > 90) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 90;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            addToCommandMsgQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                                else if(testY != 0) {
                                    if(mainappData.rover.yPos > mainappData.targetY[mainappData.targetIndex]) {
                                        // Checking if it is facing in an angle that is not within margin of error
                                        // Rover is in the wrong place
                                        if(mainappData.rover.angle < 6 || mainappData.rover.angle > 354) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.rover.yPos - mainappData.targetY[mainappData.targetIndex];
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn right
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn left
                                            else {
                                                // Need to move left the amount of angle that it is facing
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, mainappData.rover.angle & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                    else {
                                        // Checking if it is facing an angle that is not within margin of error
                                        if(mainappData.rover.angle < 186 && mainappData.rover.angle > 174) {
                                            mainappData.moving = pdTRUE;
                                            uint16_t distanceLeftover = mainappData.targetY[mainappData.targetIndex] - mainappData.rover.yPos;
                                            roverCorrectionMessage = makeRoverMove(ROVER_FORWARD, distanceLeftover & 0x00FF);
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                        else {
                                            mainappData.turning = pdTRUE;
                                            // Easier to turn left
                                            if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                                uint16_t angleLeftover = mainappData.rover.angle - 180;
                                                roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                            }
                                            // Easier to turn right
                                            else {
                                                uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                                roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                            }
                                            addToUartTXQ(roverCorrectionMessage);
                                        }
                                    }
                                }
                            }
                            // Checking if it is turning to an angle within margin of error
                            else if(mainappData.roverMotion == turningUp) {
                                mainappData.turning = pdTRUE;
                                // Checking if it is facing in an angle that is not within margin of error
                                // Rover is in the wrong place
                                if(!(mainappData.rover.angle < 6 || mainappData.rover.angle > 354)) {
                                    // Easier to turn right
                                    if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                        uint16_t angleLeftover = 360 - mainappData.rover.angle;
                                        roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                    }
                                    // Easier to turn left
                                    else {
                                        // Need to move left the amount of angle that it is facing
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, mainappData.rover.angle & 0x00FF);
                                    }
                                    addToUartTXQ(roverCorrectionMessage);
                                }
                            }
                            else if(mainappData.roverMotion == turningDown) {
                                mainappData.turning = pdTRUE;
                                // Checking if it is facing an angle that is not within margin of error
                                if(mainappData.rover.angle != 180) {//!(mainappData.rover.angle < 185 && mainappData.rover.angle > 175)) {
                                    // Easier to turn left
                                    if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 180) {
                                        uint16_t angleLeftover = mainappData.rover.angle - 180;
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                    }
                                    // Easier to turn right
                                    else {
                                        uint16_t angleLeftover = 180 - mainappData.rover.angle;
                                        roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                    }
                                    addToUartTXQ(roverCorrectionMessage);
                                }
                            }
                            else if(mainappData.roverMotion == turningRight) {
                                mainappData.turning = pdTRUE;
                                // Checking if it is facing an angle that is not within margin of error
                                // The rover is not facing right
                                if(mainappData.rover.angle != 90) {//!(mainappData.rover.angle < 96 && mainappData.rover.angle > 84)) {
                                    // Easier to turn right
                                    if(mainappData.rover.angle < 90 && mainappData.rover.angle >= 0) {
                                        uint16_t angleLeftover = 90 - mainappData.rover.angle;
                                        roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                    }
                                    else if(mainappData.rover.angle <= 359 && mainappData.rover.angle >= 270) {
                                        // The 90 degrees is to get to the right quadrant
                                        // The subtraction from 360 degrees is to get the actual leftover angle
                                        uint16_t angleLeftover = 90 + (360 - mainappData.rover.angle);
                                        roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                    }
                                    // Easier to turn left
                                    else if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 180) {
                                        // The 90 degrees is to get to the right quadrant
                                        // The subtraction with 180 degrees is to get the actual leftover angles
                                        uint16_t angleLeftover = 90 + (mainappData.rover.angle - 180);
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                    }
                                    else if(mainappData.rover.angle < 180 && mainappData.rover.angle > 90) {
                                        uint16_t angleLeftover = mainappData.rover.angle - 90;
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                    }
                                    addToCommandMsgQ(roverCorrectionMessage);
                                }
                            }
                            else if(mainappData.roverMotion == turningLeft) {
                                mainappData.turning = pdTRUE;
                                // Checking if it is facing an angle that is not within margin of error
                                if(mainappData.rover.angle != 270) {//!(mainappData.rover.angle < 276 && mainappData.rover.angle > 264)) {
                                    // Easier to turn right
                                    if(mainappData.rover.angle < 270 && mainappData.rover.angle >= 90) {
                                        uint16_t angleLeftover = 270 - mainappData.rover.angle;
                                        roverCorrectionMessage = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
                                    }
                                    // Easier to turn left
                                    else if(mainappData.rover.angle <= 359 && mainappData.rover.angle > 270) {
                                        uint16_t angleLeftover = mainappData.rover.angle - 270;
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                    }
                                    else if(mainappData.rover.angle >= 0 && mainappData.rover.angle < 90) {
                                        // The 90 degrees is to get to the right quadrant
                                        uint16_t angleLeftover = mainappData.rover.angle + 90;
                                        roverCorrectionMessage = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
                                    }
                                addToCommandMsgQ(roverCorrectionMessage);
                                }
                            }*/
                        }
                    }
                }
            }
            // Message for a token being found by the rover
            else if(inMessage.type == TOKEN_FOUND) {
                // Converting the message to data that can be used to update my rover or obstacle(s).
                convertMessage(inMessage, &mainappData.object);
                // No need to convert the message since it depends on the new location of the rover
                //sendTokenUpdate();
                if(mainappData.tokenCount == -1) {
                    mainappData.tokenCount = mainappData.object.angle;
                }
                int tokenIterator;
                BaseType_t oldToken = pdFALSE;
                
                // Checking to see if the token was already found previously
                for(tokenIterator = 0; tokenIterator < mainappData.tokenCount; tokenIterator++) {
                    // If a token was already found previously, do nothing
                    if((mainappData.token[tokenIterator].xPos == mainappData.rover.xPos) && (mainappData.token[tokenIterator].yPos == mainappData.rover.yPos)) {
                        oldToken = pdTRUE;
                    }
                }
                // Newly found token
                if(oldToken == pdFALSE) {
                    // Adding the new token location to the array
                    mainappData.token[mainappData.tokenCur].xPos = mainappData.rover.xPos;
                    mainappData.token[mainappData.tokenCur].yPos = mainappData.rover.yPos;
                    
                    // Increment the counter
                    mainappData.tokenCur++;
                    if(mainappData.tokenCur == mainappData.tokenCount) {
                        sendTokenUpdate(mainappData.token[3].xPos, mainappData.token[3].yPos, 0x06, 0x06);
                    }
                }
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
