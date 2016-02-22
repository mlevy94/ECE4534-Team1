/* 
 * File:   initDelay.h
 * Author: Chris Cox
 *
 * Created on Feb 22, 2016
 */

#ifndef INITDELAY_H
#define	INITDELAY_H

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

#ifdef	__cplusplus
}
#endif

#endif	/* INITDELAY_H */

