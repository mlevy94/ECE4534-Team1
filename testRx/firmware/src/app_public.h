/* 
 * File:   app_public.h
 * Author: MLTop
 *
 * Created on February 1, 2016, 5:15 PM
 */

#ifndef APP_PUBLIC_H
#define	APP_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    // Passes Rx Val from ISR to Rx APP
    BaseType_t addToUsartRxQFromISR(char* val);
    
    
#ifdef	__cplusplus
}
#endif

#endif	/* APP_PUBLIC_H */