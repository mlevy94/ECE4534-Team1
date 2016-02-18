/* 
 * File:   adc_app_public.h
 * Author: MLTop
 *
 * Created on February 16, 2016, 7:18 PM
 */

#ifndef ADC_APP_PUBLIC_H
#define	ADC_APP_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

BaseType_t addToADCQ(int val);
BaseType_t addToADCQFromISR(int val);


#ifdef	__cplusplus
}
#endif

#endif	/* ADC_APP_PUBLIC_H */

