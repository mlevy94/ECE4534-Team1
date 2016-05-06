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

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "comm_k.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
#define EXAMPLE_CONSTANT 0


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */
    typedef struct _example_struct_t {
        /* Describe structure member. */
        int some_number;

        /* Describe structure member. */
        bool some_flag;

    } example_struct_t;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    int ExampleFunction(int param1, int param2);

    // *************************************************************************
    // system_interrupt.c
    // *************************************************************************
    BaseType_t writeTo_WiFly_OutgoingByte_Q(char outgoingByte); 
    BaseType_t writeTo_Pi_OutgoingByte_Q(char outgoingByte);
    void initTxByteQueues();
    void updateDebugVal(char inVal);
    
    // *************************************************************************
    // wifly_tx.c
    // *************************************************************************
    BaseType_t sendMsgToWiFlyTx(SysMsg inSysMsg);
    BaseType_t sendMsgToWiFlyTx_fromISR(SysMsg inSysMsg);
    BaseType_t sendOutgoingMsgToWiflyTx(SysMsg inSysMsg);
    BaseType_t sendOutgoingMsgToWiflyTx_fromISR(SysMsg inSysMsg);
    BaseType_t sendToInitTXQ(SysMsg inSysMsg);
    
    // *************************************************************************
    // wifly_rx.c
    // *************************************************************************
    BaseType_t writeTo_WiFly_rxByteQueue_fromISR(char incomingByte);
    BaseType_t sendMsgToWiFlyRxApp(SysMsg inSysMsg);
    BaseType_t sendMsgToWiFlyRxApp_fromISR(SysMsg inSysMsg);
    
    // *************************************************************************
    // pi_tx.c
    // *************************************************************************
    BaseType_t sendMsgToPiTxApp(SysMsg inSysMsg);
    BaseType_t sendMsgToPiTxApp_fromISR(SysMsg inSysMsg);
    BaseType_t sendOutgoingMsgToPiTx(RpiMsg inSysMsg);
    BaseType_t sendOutgoingMsgToPiTx_fromISR(RpiMsg inSysMsg);
    
    // *************************************************************************
    // pi_rx.c
    // *************************************************************************
    BaseType_t writeTo_Pi_rxByteQueue_fromISR(char incomingByte);    
    BaseType_t sendMsgToPiRxApp(SysMsg inSysMsg);
    BaseType_t sendMsgToPiRxApp_fromISR(SysMsg inSysMsg);
    
    // *************************************************************************
    // app.c
    // *************************************************************************
    BaseType_t sendMsgToApp(SysMsg inSysMsg);
    BaseType_t sendMsgToApp_fromISR(SysMsg inSysMsg);
    void update_netstats();
    
    // *************************************************************************
    // app1.c
    // *************************************************************************
    BaseType_t sendMsgToApp1(SysMsg inSysMsg);
    BaseType_t sendMsgToApp1_fromISR(SysMsg inSysMsg);
    
    // *************************************************************************
    // comm_k.c
    // *************************************************************************
    

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
