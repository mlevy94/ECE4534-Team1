/* 
 * File:   uart_rx_app_public.h
 * Author: MLTop
 *
 * Created on February 16, 2016, 6:38 PM
 */

#ifndef UART_RX_APP_PUBLIC_H
#define	UART_RX_APP_PUBLIC_H

#include "initDelay.h"

#ifdef	__cplusplus
extern "C" {
#endif

BaseType_t addToUartRXQ(char msg);
BaseType_t addToUartRXQFromISR(char msg);
BaseType_t addToUartSentQ(char num, InternalMessage msg);
BaseType_t addToUartSentQFromISR(char num, InternalMessage msg);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_RX_APP_PUBLIC_H */

