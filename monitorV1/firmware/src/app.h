/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

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

#ifndef _APP_H
#define _APP_H

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

#include "debug_k.h"
#include "comm_k.h"
#include <queue.h>

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
// NetApp

typedef struct
{
    uint8_t LR_DEBUG;
    uint8_t FR_DEBUG;
    uint8_t SENSOR_DEBUG;
    uint8_t CORD_DEBUG;
    
} REMOTE_DEBUG_STATES;

typedef struct
{
    uint8_t source;
    uint8_t last_msg_number;
    uint8_t debug_state;
    uint8_t status;
    int count;
    int last_count;
    int last_time;
    int long_per;
    int short_per;
    int red_reset_counter;
    uint8_t times_red;
    int yellow_elevate_counter;
    uint8_t times_yellow;
    int missing;
    int last_missing;
    bool init;
    
} NETSTAT_TRIPPLE;

typedef struct
{
    
    NETSTAT_TRIPPLE lead_rover;
    NETSTAT_TRIPPLE foll_rover;
    NETSTAT_TRIPPLE cordinator;
    NETSTAT_TRIPPLE sensor;    
    
} NETSTAT;

typedef struct
{
    uint8_t msg_type;
    uint8_t msg_count;
    int count;
    int last_time;
    int long_per;
    int short_per;
    
}MSG_RATE_DATUM;

typedef struct
{
    MSG_RATE_DATUM msg0;
    MSG_RATE_DATUM msg1;
    MSG_RATE_DATUM msg2;
            
}MESSAGE_RATES;


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
	APP_STATE_INIT=0,

	/* TODO: Define states used by the application state machine. */
    APP_STATE_ONE=1
} APP_STATES;


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
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */
    QueueHandle_t appInternalQ;
    uint8_t itt;
    SysMsg sysMsg;
    SysMsg heartbeat;
    RpiMsg rpiMsg;
    
    NETSTAT netstat;
    NETSTAT_TRIPPLE netstat_tripple;
    
    MSG_RATE_DATUM msg_rate_datum;
    MESSAGE_RATES lead_msg_rates;
    MESSAGE_RATES follow_msg_rates;
    MESSAGE_RATES cord_msg_rates;
    MESSAGE_RATES sensor_msg_rates;
    

} APP_DATA;


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
    void APP_Initialize ( void )

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
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

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
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );
void processApp0Message();
void store_netstat();
void clearNetStats();
void sendNetStats();
void buildRpiDebugMessage();
void buildRpiNetstatMessage(uint8_t status, uint8_t target, int totalMissing, int totalMessages,
                               uint8_t yellow_ct, uint8_t red_ct);

void lr_store_netstat();
void fr_store_netstat();
void se_store_netstat();
void co_store_netstat();

void processTimingMessage();

void sendHeartbeat();

#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

