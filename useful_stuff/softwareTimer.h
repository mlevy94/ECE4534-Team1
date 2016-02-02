/* 
 * File:   timerCallback.h
 * Author: Jacky
 *
 * Created on January 27, 2016, 6:10 PM
 */

#ifndef TIMERCALLBACK_H
#define	TIMERCALLBACK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include <timers.h>

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeTimer();
    void addToTimerQ(char* val);
    BaseType_t addToTimerQFromISR(char* val);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMERCALLBACK_H */

