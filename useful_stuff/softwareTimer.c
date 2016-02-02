/* Define a callback function that will be used by multiple timer instances.
 The callback function does nothing but count the number of times the
 associated timer expires, and stop the timer once the timer has expired
 10 times. */

#include "softwareTimer.h"
#include "debug.h"

#include <queue.h>

QueueHandle_t timerQ;
TimerHandle_t timerHandle;

void vTimerCallback( TimerHandle_t pxTimer );

void initializeTimer() {
    //Initializing the timer
    timerHandle = xTimerCreate("softwareTimer",
                               // Sets a timer frequency of 50ms.
                               ( 50 / portTICK_PERIOD_MS ),
                               pdTRUE,
                               (void *) 2,
                               vTimerCallback);
    timerQ = xQueueCreate(10, 8);
    
    if( xTimerStart( timerHandle, 0) != pdPASS)
    {
        debugFailOnNot0(1); // will fail if timer doesn't start
    }
}

void addToTimerQ(char* val) {
    xQueueSend(timerQ, val, portMAX_DELAY);
}

BaseType_t addToTimerQFromISR(char* val) {
    xQueueSendFromISR( timerQ, val, 0);
}

void vTimerCallback( TimerHandle_t pxTimer )
{
    
}
