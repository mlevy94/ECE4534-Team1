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
    for (;i < 11; i++) {
        setDebugVal(message.msg[i]);
    }
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
        setDebugVal(*angle);
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
            setDebugVal(*angle);
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

void moveHorizontalRightOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
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
            *angle = 90;
        }
        else if(*angle <= 359 && *angle >= 270) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction from 360 degrees is to get the actual leftover angle
            uint16_t angleLeftover = 90 + (360 - *angle);
            roverCommand = makeRoverMove(ROVER_RIGHT, angleLeftover & 0x00FF);
            *angle = 90;
        }
        
        // Easier to turn left
        else if(*angle < 270 && *angle >= 180) {
            // The 90 degrees is to get to the right quadrant
            // The subtraction with 180 degrees is to get the actual leftover angles
            uint16_t angleLeftover = 90 + (*angle - 180);
            *angle = 90;
            roverCommand = makeRoverMove(ROVER_LEFT, angleLeftover & 0x00FF);
        }
        else if(*angle < 180 && *angle > 90) {
            uint16_t angleLeftover = *angle - 90;
            *angle = 90;
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

void moveHorizontalLeftOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
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

void moveVerticalUpOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void moveVerticalDownOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void moveRightUpAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    InternalMessage roverCommand;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if((*x + mainappData.rover.length) == mainappData.obstacle[objInd].xPos) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x + (2 * mainappData.rover.length)) <= 36) {
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
    InternalMessage roverCommand;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if((*x + mainappData.rover.length) == mainappData.obstacle[objInd].xPos) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x + (2 * mainappData.rover.length)) <= 36) {
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
    InternalMessage roverCommand;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if((*x - mainappData.rover.length) == mainappData.obstacle[objInd].xPos) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x + (2 * mainappData.rover.length)) >= 0) {
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

void moveLeftDownAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    int objIterator;
    int objInd;
    BaseType_t blocked = pdFALSE;
    InternalMessage roverCommand;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if((*x - mainappData.rover.length) == mainappData.obstacle[objInd].xPos) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*x - (2 * mainappData.rover.length)) >= 0) {
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
    InternalMessage roverCommand;
    
    // Searching for the obstacle blocking the rover
    for(objIterator = 0; objIterator < mainappData.obstacleCount; objIterator++) {
        // Finding the obstacle
        if((*x + mainappData.rover.length) == mainappData.obstacle[objInd].xPos) {
            objInd = objIterator;
            blocked = pdTRUE;
        }
    }
    
    // The obstacle was found
    if(blocked == pdTRUE) {
        // Need to account for edges
        // We are going under the postulated rule that no obstacle will be adjacent to one another
        if((*y + (2 * mainappData.rover.length)) >= 0) {
            // Make the rover turn right
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move up once to be beside the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Move up once more to get past the obstacle
            moveVerticalUpOnceCorner(x, y, angle);
            // Begin moving back to where you need to be
            moveHorizontalLeftOnceCorner(x, y, angle);
            
            // V need to fix this to face up
            // Reorient to help the rover go right (again)
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
        }
        // At the top edge of the map
        else {
            // Move right one space
            moveHorizontalRightOnceCorner(x, y, angle);
            // Move up one space to be right next to the obstacle
            // Already reoriented right
            moveVerticalUpOnceCorner(x, y, angle);
        }
    }
}

void moveUpLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void moveDownRightAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}
void moveDownLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void topToBottomCorners(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void bottomToTopCorners(uint16_t* x, uint16_t* y, uint16_t* angle)
{
    
}

void upperLeftCornerTraverse(void)
{
    
}

void lowerLeftCornerTraverse(void)
{
    
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
    mainappData.rover.length = 20;
    mainappData.rover.width = 20;
    mainappData.rover.xPos = 50;
    mainappData.rover.yPos = 50;
    mainappData.rover.angle = 0;
    
    // The counter for the number of obstacles
    mainappData.obstacleCount = -1;
    
    // The current position of the obstacles
    mainappData.obstacleCur = 0;
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
                /*
                setDebugVal(0x72);
                setDebugVal(mainappData.object.type);
                setDebugVal(ROVER);
                setDebugVal((mainappData.object.type & 0xff) == ROVER);
                */
                if((mainappData.object.type & 0xff) == ROVER) {
                    /*
                     Working code to send out the location of the rover as stored by the PIC
                    setDebugVal(0x73);
                    addToUartTXQ(makeLocationMessage(mainappData.rover));
                    */
                    if(mainappData.rover.xPos == 50 && mainappData.rover.yPos == 50) {
                        mainappData.rover = mainappData.object;
                    
                        // Test command
                        // Testing the ability of the rover to go to the ends of the map
                        //topToBottomEmpty();
                        //bottomToTopEmpty();
                        //rightToLeftEmpty();
                        //leftToRightEmpty();
                        
                        // Testing the ability of the rover to take one command
                        //moveHorizontalLeftOnceEmpty();
                        //moveHorizontalRightOnceEmpty();
                        
                        // Testing the ability of the rover to traverse an empty map
                        upperLeftCornerTraverseEmpty();
                        //lowerLeftCornerTraverseEmpty();
                        //upperRightCornerTraverseEmpty();
                        //lowerRightCornerTraverseEmpty();
                        //centerAndSpreadOutEmpty();
                    }
                    
                    // Send the next available rover command
                    InternalMessage command;
                    if(xQueueReceive(mainappData.mainAppCommandMsgQ, &command, portMAX_DELAY)) {
                        addToUartTXQ(command);
                    }
                    /* Debugging for sending back the rover location*/
                     //sendRoverLocation();
                    
                }
                // Object is an obstacle
                else if((mainappData.object.type & 0xff) == OBSTACLE) {
                    // Since this should only occur from the initial messages
                    // Creating a new obstacle
                    mainappData.obstacle[mainappData.obstacleCur] = mainappData.object;
                    
                    // Updating the number of obstacles
                    if(mainappData.obstacleCount == -1) {
                        mainappData.obstacleCount = mainappData.object.angle;
                    }
                    mainappData.obstacleCur++;
                    
                    /* Debugging for sending back all the obstacle locations */
                    if(mainappData.obstacleCur == mainappData.obstacleCount) {
                        sendObstacleLocations();
                    }
                    
                }
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
