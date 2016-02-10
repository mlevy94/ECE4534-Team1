/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData =
{
    //TODO - Initialize appData structure.

};

// Global string for testing
//const char *testing = "Team 1 \0";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// Initializes the UART Module
//void initUART(void)
//{
    /* Enable the UART module*/
    //USART_ID_1 is for UART port 0 (J14)
  //  PLIB_USART_Enable(USART_ID_1);
//}

//void sendString(const char *string)
//{
   // int index;
    //while (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    //{
        /* Write a character at a time, only if transmitter is empty */
      //  for (index = 0; string[index] != '\0'; index++) {
        //    PLIB_USART_TransmitterByteSend(USART_ID_1, string[index]);
       // }
        //PLIB_USART_TransmitterByteSend(USART_ID_1, '\0');
   // }
    /*
    while (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    {
        if (*(string + index) == '\0')
        {
            
        }
        else
        {
            // Send character
            PLIB_USART_TransmitterByteSend(USART_ID_1, *(string + index));
        }
        index ++;
    }
    */
//}

//void sendCharacter(const char character)
//{
    /* Check if buffer is empty for a new transmission */
  //  if(PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    //{
        /* Send character */
      //  PLIB_USART_TransmitterByteSend(USART_ID_1, character);
   // }
//}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    appData.msgToAdcQ = xQueueCreate(16, 8);
    //DRV_ADC_Initialize();
    //DRV_ADC_Open();
    //DRV_ADC_Start();
    /* Place the App state machine in its initial state. */
    //appData.state = APP_STATE_INIT;
    //initUART();
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    char curVal;
    //bool here;
    //here = DRV_ADC_SamplesAvailable();
    //setDebugBool(here);
    DRV_ADC_Open();
    while(1){
       // DRV_ADC_Open();
        //setDebugVal('W');
        if(xQueueReceive(appData.msgToAdcQ, &curVal, portMAX_DELAY)){
            //setDebugBool(pdFALSE);
            setDebugVal(curVal);
        } 
        //setDebugBool(pdFALSE);
    }
}
 
BaseType_t adcFromISR(char *adcVal){
    //setDebugVal(xQueueSendFromISR(appData.msgToAdcQ, adcVal, 0));
    setDebugBool(xQueueSendFromISR(appData.msgToAdcQ, adcVal, 0));
    return xQueueSendFromISR(appData.msgToAdcQ, adcVal, 0);
    //xQueueSendFromISR(appData.msgToAdcQ, adcVal, 0);
    //return xQueueSendFromISR(appData.msgToAdcQ, adcVal, 0);
    //function from isr. store in ADC_CUR;
}

/*******************************************************************************
 End of File
 */
