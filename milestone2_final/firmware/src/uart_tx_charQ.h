/* 
 * File:   uart_tx_charQ.h
 * Author: MLTop
 *
 * Created on February 9, 2016, 4:50 AM
 */

#ifndef UART_TX_CHARQ_H
#define	UART_TX_CHARQ_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initializeTXCharQ();
    void addToTXCharQ(char* val); // blocking
    BaseType_t addToTXCharQFromISR(char* val); // non-blocking
    void getFromTXCharQ(char* val);
    void getFromTXCharQFromISR(char* val);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_TX_CHARQ_H */

