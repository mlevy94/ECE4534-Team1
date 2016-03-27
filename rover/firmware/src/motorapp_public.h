/* 
 * File:   motorapp_public.h
 * Author: MLTop
 *
 * Created on February 29, 2016, 5:14 PM
 */

#ifndef MOTORAPP_PUBLIC_H
#define	MOTORAPP_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "comm.h"

void incLeftEn();
void incRightEn();
void incMoveCount();
BaseType_t addToMotorQ(InternalMessage msg);
BaseType_t addToMotorQFromISR(InternalMessage msg);


#ifdef	__cplusplus
}
#endif

#endif	/* MOTORAPP_PUBLIC_H */

