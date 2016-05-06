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

    BaseType_t adcFromISR(int *adcVal);
    
#ifdef	__cplusplus
}
#endif

#endif	/* APP_PUBLIC_H */