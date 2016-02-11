/* 
 * File:   usart_tx_public.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 8:56 PM
 */

#ifndef USART_TX_PUBLIC_H
#define	USART_TX_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif


void addToOutQ(char* val);
BaseType_t addToOutQFromISR(char* val);

void addToPrioirtyTxQ(char* val);
BaseType_t addToPrioirtyTxQFromISR(char* val);

#ifdef	__cplusplus
}
#endif

#endif	/* USART_TX_PUBLIC_H */

