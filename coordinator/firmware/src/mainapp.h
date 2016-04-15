/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    mainapp.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _MAINAPP_H
#define _MAINAPP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include <queue.h>
#include "comm.h"
#include "txbuffer_public.h"
#include "uart_rx_app_public.h"
#include "uart_tx_app_public.h"
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	MAINAPP_STATE_INIT=0,

	/* TODO: Define states used by the application state machine. */

} MAINAPP_STATES;

// Enumerated type for the types of objects available in a grid cell
typedef enum
{
    blank = 0,
    lead_rover = 1,
    obstacle = 2,
    follower_rover = 3,
    token = 4        
} CELL_TYPES;

// Enumerated type for the the types of traversal algorithms available
typedef enum
{
    top = 0,
    bottom = 1,
    unknown = 2
} TRAVERSAL_ALGORITHM;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    MAINAPP_STATES state;

    // Type of process being used to traverse the map
    TRAVERSAL_ALGORITHM obstacleAversion;

    // Queue for the message handling
    QueueHandle_t mainAppMsgQ;
    
    // Message sequence number for checking for message loss
    char messageNumber;
    
    // Grid containing the whole playing field 6x6 (6" x 6")
    CELL_TYPES macroGrid[6][6];
    
    // Rover position
    OBJECT_STRUCTURE rover;
    
    OBJECT_STRUCTURE object;
    
    // Obstacle positions
    OBJECT_STRUCTURE obstacle[4];
    
    // Token positions
    OBJECT_STRUCTURE token[4];
    
    // Counter for the number of tokens
    int tokenCount;
    
    // Counter to increment to the total number of tokens
    int tokenCur;
    
    // Counter for the number of obstacles
    int obstacleCount;
    
    // Counter to increment to the total number of obstacles
    int obstacleCur;
    
    /* Target command positions to do corrections while traveling to a new location */
    // Target X Position
    uint16_t targetX[150];
    
    // Target Y Position
    uint16_t targetY[150];
    
    // Target Angle Position
    uint16_t targetAngle[150];
    
    // Total commands
    int targetTotal;
    
    // Index of the target x and y
    int targetIndex;

    // Queue for sending commands to the rover
    QueueHandle_t mainAppCommandMsgQ;
    
    /* Boolean for traversal for when an obstacle is blocking a y axis side and this is used to prevent skipping one vertical line */
    BaseType_t dontSkip;
} MAINAPP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
int microInchesToCell(int position);
int macroInchesToCell(int position);
void convertMessage(InternalMessage message, OBJECT_STRUCTURE* obj);

BaseType_t addToCommandMsgQ(InternalMessage msg);

/*
 Methods for the full implementation for complete traversal
 */
////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//          EMPTY MAP
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////

// Moving around only once to make pass by reference
void moveHorizontalRightOnceCornerEmpty(uint16_t* x, uint16_t* angle);
void moveHorizontalLeftOnceCornerEmpty(uint16_t* x, uint16_t* angle);
void moveVerticalUpOnceCornerEmpty(uint16_t* y, uint16_t* angle);
void moveVerticalDownOnceCornerEmpty(uint16_t* y, uint16_t* angle);
void topToBottomEmptyCorners(uint16_t* y, uint16_t* angle);
void bottomToTopEmptyCorners(uint16_t* y, uint16_t* angle);

// Corner cases
void upperLeftCornerTraverseEmpty(void);
void lowerLeftCornerTraverseEmpty(void);
void upperRightCornerTraverseEmpty(void);
void lowerRightCornerTraverseEmpty(void);

// Corner cases made pass by reference for the center case including the center case
void centerAndSpreadOutEmpty(void);
void upperLeftCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void lowerLeftCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void upperRightCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void lowerRightCornerTraverseEmptyParam(uint16_t* x, uint16_t* y, uint16_t* angle);

////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//          MAP WITH OBSTACLES
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////

// Moving around only once to make pass by reference
// Turning to a direction
void turnRight(uint16_t* angle);
void turnLeft(uint16_t* angle);
void turnUp(uint16_t* angle);
void turnDown(uint16_t* angle);

// Moving one single grid cell
void moveHorizontalRightOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveHorizontalLeftOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveVerticalUpOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveVerticalDownOnceCorner(uint16_t* x, uint16_t* y, uint16_t* angle);

// Moving to the different edges
void moveToRightEdge(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveToLeftEdge(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveToTopEdge(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveToBottomEdge(uint16_t* x, uint16_t* y, uint16_t* angle);

// Moving up and and down
void topToBottomCorners(uint16_t* x, uint16_t* y, uint16_t* angle);
void bottomToTopCorners(uint16_t* x, uint16_t* y, uint16_t* angle);

// Obstacle Maneuvering
/*
 The first direction is the direction the rover needs to go
 The second direction is the direction is the direction the rover will take to go around the obstacle

 The parameters are for the location and angle of the rover
 */
void moveRightUpAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveRightDownAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveLeftUpAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveLeftDownAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);

void moveUpRightAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveUpLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveDownRightAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);
void moveDownLeftAroundObstacle(uint16_t* x, uint16_t* y, uint16_t* angle);

// Corner cases
void upperLeftCornerTraverse(void);
void lowerLeftCornerTraverse(void);
void upperRightCornerTraverse(void);
void lowerRightCornerTraverse(void);

// Corner cases made pass by reference for the center case including the center case
void centerAndSpreadOut(void);
void upperLeftCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void lowerLeftCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void upperRightCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle);
void lowerRightCornerTraverseParam(uint16_t* x, uint16_t* y, uint16_t* angle);

// Accounting for correction factor
void correctionDistance(uint16_t* x, uint16_t* y, uint16_t* angle);
void correctionAngle(uint16_t* x, uint16_t* y, uint16_t* angle);

// Method to send back the rover location to the simulation for debugging
void sendRoverLocation(void);

// Method to send back the token found message
void sendTokenUpdate(void);

// Method to send back the obstacle locations to the simulation for debugging
void sendObstacleLocations(void);

/*
 Empty algorithm traversal throughout map
 */
void runAlgorithmEmpty(void);

/*
 Algorithm traversal throughout occupied map
 */
void runAlgorithm(void);
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MAINAPP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MAINAPP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MAINAPP_Initialize ( void );


/*******************************************************************************
  Function:
    void MAINAPP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MAINAPP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MAINAPP_Tasks( void );


#endif /* _MAINAPP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

