/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pixy_rx.h

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

#ifndef _PIXY_RX_H
#define _PIXY_RX_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "system_config.h"
#include "system_definitions.h"
#include "queue.h"
#include "pixyMessage.h"
#include "public.h"
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
	PIXY_RX_STATE_INIT=0,
    COLLECT_DATA=1,

	/* TODO: Define states used by the application state machine. */

} PIXY_RX_STATES;


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
    PIXY_RX_STATES state;

    /* TODO: Define any additional data used by the application. */
    QueueHandle_t practice;
    //QueueHandle_t edgeQ;
    QueueHandle_t rxMessageQ;
    solidColor SC_message;
    //solidColor SC_received;
    colorCode CC_message;
    colorCode CC_received;
    unsigned char val;
    
    solidColor corners[2][2];
    solidColor c[16];
    grid calibrationGrid[16];
    int a;
    int b;
    int k;
    float conversion_x;
    float conversion_y;
    int calibration_cycle;
    int originX;
    int originY;
    
    float fx00, fy00, fx10, fy10, fx01, fy01, fx11, fy11;
    float interpolationConstant_A, interpolationConstant_B;
    
} PIXY_RX_DATA;


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
    void PIXY_RX_Initialize ( void )

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
    PIXY_RX_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PIXY_RX_Initialize ( void );


/*******************************************************************************
  Function:
    void PIXY_RX_Tasks ( void )

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
    PIXY_RX_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void PIXY_RX_Tasks( void );


BaseType_t addToPixyQ(char data);
void testQueues(void);
void calibrate(void);
void dataCollection(void);
void determineEdge(void);//obstacles, tokens
//BaseType_t addToEdgeQ(solidColor data);
void addToSolidColorQ(void);//QueueHandle_t pixyQueue);//obstacles, tokens
void addToColorCodeQ(void);//QueueHandle_t pixyQueue);//lead rover, follower rover
void inchesPerPixel(void);//edges
void SCMessage(solidColor SC_received);//QueueHandle_t SCQueue);//convert obstacle pixels to inches
void CCMessage(colorCode CC_received);//QueueHandle_t CCQueue);//convert rover pixels to inches, determine orientation

//used for sorting grid calibration points
void swap (grid a[], int left, int right);
void quicksort_y(grid a[], int low, int high);
int partition_y(grid a[], int low, int high);
void quicksort_x(grid a[], int low, int high);
int partition_x(grid a[], int low, int high);

//functions for calculating x, y of object
//BaseType_t checkBlock(float *vertx, float *verty, float testx, float testy);
BaseType_t checkBlock(int TL, int TR, int BL, int BR, uint16_t x, uint16_t y);
float y_interpolation(float x, float y);
float x_interpolation( float x, float y);
float height_correction(float j_a, float object_height);

//colorCode CC_received);//
#endif /* _PIXY_RX_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

