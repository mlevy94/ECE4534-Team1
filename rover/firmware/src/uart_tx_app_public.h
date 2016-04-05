/* 
 * File:   uart_tx_app_public.h
 * Author: MLTop
 *
 * Created on February 16, 2016, 2:57 PM
 */

#ifndef UART_TX_APP_PUBLIC_H
#define	UART_TX_APP_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "comm.h"
    
BaseType_t addToInitTXQ(char msg);
BaseType_t addToUartTXQ(InternalMessage msg);
BaseType_t addToUartTXQFromISR(InternalMessage msg);
BaseType_t priorityAddToUartTXQ(InternalMessage msg);
BaseType_t priorityAddToUartTXQFromISR(InternalMessage msg);
BaseType_t readFromTXBufferQ(char* msg);
BaseType_t readFromTXBufferQFromISR(char* msg);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_TX_APP_PUBLIC_H */

