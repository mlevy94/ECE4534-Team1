/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef UART_TX_CHARQ_H    /* Guard against multiple inclusion */
#define UART_TX_CHARQ_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */




/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif



    void initializeTXCharQ();
    void addToTXCharQ(char* val); // blocking
    BaseType_t addToTXCharQFromISR(char* val); // non-blocking
    void getFromTXCharQ(char* val);
    void getFromTXCharQFromISR(char* val);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* UART_TX_CHARQ_H */

/* *****************************************************************************
 End of File
 */
