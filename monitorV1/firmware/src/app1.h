/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app1.h

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

#ifndef _APP1_H
#define _APP1_H

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

#include <math.h>

#include "debug_k.h"
#include "comm_k.h"
#include <queue.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 
    
#define MAX_ROUTE_LEN 120

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Route tracking
    
typedef struct
{
    uint8_t xcord;
    uint8_t ycord;
    
} ROUTE_CORD;

typedef struct
{
    uint8_t routeLen_actual;
    uint8_t initRoute[MAX_ROUTE_LEN];
    
} LR_INITIAL_ROUTE;

typedef struct
{
    uint8_t id;
    uint8_t reserved;
    uint8_t xcordMsb;
    uint8_t xcordLsb;
    uint8_t ycordMsb;
    uint8_t ycordLsb;
    uint8_t angleMsb;
    uint8_t angleLsb;
    unsigned int numSamples;
    
}ROVER_DATA;

// Arena tracking
typedef struct
{   
    uint8_t id;
    unsigned int xcord;
    unsigned int ycord;
    
} OBSTACLE_DATA;

typedef struct
{   
    uint8_t id;
    unsigned int xcord;
    unsigned int ycord;
    uint8_t found;
    uint8_t picked;
    
} TOKEN_DATA;



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
	APP1_STATE_INIT=0,

	/* TODO: Define states used by the application state machine. */
    APP1_STATE_ONE = 1, 
    APP1_STATE_TWO = 2,
    APP1_STATE_THREE = 3,
    APP1_STATE_FOUR = 4

} APP1_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
union DoublePi {double val; uint8_t c[8];};

typedef struct
{
    /* The application's current state */
    APP1_STATES state;

    /* TODO: Define any additional data used by the application. */
    
    // General use itterator
    int itt;
    
    // Incoming message queue from other apps
    QueueHandle_t app1InternalQ;
    
    // Message handles
    SysMsg sysMsg;
    RpiMsg rpiMsg;
    
    // Initial Route
    uint8_t routeKeeper;    
    LR_INITIAL_ROUTE lr_initial_route;
    
    // Arena Presets
    OBSTACLE_DATA O1;
    OBSTACLE_DATA O2;
    OBSTACLE_DATA O3;
    OBSTACLE_DATA O4;
    TOKEN_DATA T1;
    TOKEN_DATA T2;
    TOKEN_DATA T3;
    TOKEN_DATA T4;
    
    // Rovers
    ROVER_DATA init;
    ROVER_DATA rovL;
    ROVER_DATA rovL_startMove;
    ROVER_DATA rovF;
    ROVER_DATA rovF_startMove;
    
    // Accuracy data
    union DoublePi runningError_LeadRover_travel;
    union DoublePi runningError_LeadRover_angle;
    union DoublePi leadRover_travel_error;
    union DoublePi leadRover_travel_error_max;
    union DoublePi leadRover_angle_error;
    union DoublePi leadRover_angle_error_max;
    unsigned int divisor_LeadRover;
    
    

} APP1_DATA;




// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP1_Initialize ( void )

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
    APP1_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP1_Initialize ( void );


/*******************************************************************************
  Function:
    void APP1_Tasks ( void )

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
    APP1_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP1_Tasks( void );
void clearInitialRoute();
void processApp1Message();
void addToInitialRoute();
void buildRpiRoverLocationMessage();
void buildRpiObjectLocationMessage();
void buildRpiTokenLocationMessage();
void buildRpiRoverMoveMsg();
void buildRpiAccuracyMsg();

void runRoverStats();
void runObsTokSensorAccuracy();
void processInitialObjectLocation();
void buildRpiTokenFoundMessage();
void buildRpiRoverMoveMsg();
void initRoverStats();
void sendInitRouteToRpi();


#endif /* _APP1_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

