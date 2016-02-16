/* 
 * File:   txbuffer_public.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 9:44 PM
 */

#ifndef TXBUFFER_PUBLIC_H
#define	TXBUFFER_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeTXBufferQ();
    BaseType_t addToTXBufferQ(char msg);
    BaseType_t addToTXBufferQFromISR(char msg);


#ifdef	__cplusplus
}
#endif

#endif	/* TXBUFFER_PUBLIC_H */

