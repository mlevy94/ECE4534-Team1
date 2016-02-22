/* This timer is a One Shot timer.  This will act as
 a delay for our system.
 */

#include "initDelay.h"
#include "debug.h"

TimerHandle_t timerHandle; // Declaring the timer global

void vTimerCallback( TimerHandle_t pxTimer );

void initializeTimer() {
    //Initializing the timer
    timerHandle = xTimerCreate("delayTimer",
                               // Sets a timer frequency of 5000ms or (5s).
                               ( 5000 / portTICK_PERIOD_MS ),
                               pdFALSE, // Setting a One-Hot Timer
                               (void *) 2,
                               vTimerCallback);
    fiveSecTimerBool = pdFALSE;
    
    if( xTimerStart( timerHandle, 0) != pdPASS)
    {
        debugFailOnNot0(1); // will fail if timer doesn't start
    }
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    fiveSecTimerBool = pdTRUE; // Triggering true for 5s timer
}
