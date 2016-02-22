/* This timer is a One Shot timer.  This will act as
 a delay for our system.
 */

#include "initDelay.h"
#include "debug.h"

int fiveThousandMilliseconds;
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
    fiveThousandMilliseconds = 0;
    
    if( xTimerStart( timerHandle, 0) != pdPASS)
    {
        debugFailOnNot0(1); // will fail if timer doesn't start
    }
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    //setDebugBool(pdTRUE);
    
    fiveThousandMilliseconds++;
    // I dont think we need this any longer and probably not the
    // count variable either since we are not using it currently
//    // Checks if 100 milliseconds has occurred based on a 50 millisecond period.
//    if (fiveThousandMilliseconds % 2 == 1)
//    {
//        // Not sure what to do here
//    }
    //setDebugBool(pdFALSE);
}
