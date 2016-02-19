/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    adc_app.c

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

#include "adc_app.h"
#include "adc_app_public.h"

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

ADC_APP_DATA adc_appData;

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

BaseType_t addToADCQ(int val) {
    return xQueueSend(adc_appData.adcQ, &val, portMAX_DELAY);
}

BaseType_t addToADCQFromISR(int val) {
    return xQueueSendFromISR(adc_appData.adcQ, &val, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ADC_APP_Initialize ( void )

  Remarks:
    See prototype in adc_app.h.
 */

void ADC_APP_Initialize ( void )
{
    adc_appData.adcQ = xQueueCreate(16, 10);
}


/******************************************************************************
  Function:
    void ADC_APP_Tasks ( void )

  Remarks:
    See prototype in adc_app.h.
 */

void ADC_APP_Tasks ( void )
{
    /*
     * This portion of code displays the ADC values from the queue on the logic 
     * analyzer. This is value is ten bits
     */
    unsigned int curVal;
    DRV_ADC_Open();
    InternalMessage msg;
    msg.type = DEBUG_MSG;
    int tempAdcVal = 0;
    while(1){
#ifdef DEBUG_ON
        setDebugVal(TASK_ADC_APP);
#endif
        if(xQueueReceive(adc_appData.adcQ, &curVal, portMAX_DELAY)){
            msg.msg[0] = (curVal >> 7) & 0x7f;
            msg.msg[1] = curVal & 0x7f;
            msg.msg[2] = '\0';
            
            tempAdcVal = curVal + 3;
            tempAdcVal /= 6787;
            tempAdcVal += 10; // Final ADC Value
            //TenBitsetDebugVal(tempAdcVal);
            //setDebugVal(curVal);
            //addToUartTXQ(msg);
        } 
    }
}
 

/*******************************************************************************
 End of File
 */
