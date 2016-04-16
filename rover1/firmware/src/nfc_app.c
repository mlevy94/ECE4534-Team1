/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    nfc_app.c

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

#include "nfc_app.h"
#include "nfc_app_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define START0 0x00
#define START1 0xff
#define TFI    0xd4
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

NFC_APP_DATA nfc_appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t addToNFCQ(char msg) {
    return xQueueSend(nfc_appData.inQ, &msg, portMAX_DELAY);
}

BaseType_t addToNFCQFromISR(char msg) {
    return xQueueSendFromISR(nfc_appData.inQ, &msg, 0);
}

BaseType_t addToNFCrxQ(char msg) {
    return xQueueSend(nfc_appData.rxQ, &msg, portMAX_DELAY);
}

BaseType_t addToNFCrxQFromISR(char msg) {
    return xQueueSendFromISR(nfc_appData.rxQ, &msg, 0);
}

BaseType_t addToNFCTXBufferQ(char msg) {
    // Turn TX Interrupt on
    BaseType_t ret;
    ret =  xQueueSend(nfc_appData.txBufferQ, &msg, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    return ret;
}
    
BaseType_t addToNFCTXBufferQFromISR(char msg) {
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    return xQueueSendFromISR(nfc_appData.txBufferQ, &msg, 0);
}

BaseType_t readFromNFCtxQ(char* msg) {
    return xQueueReceive(nfc_appData.txBufferQ, msg, portMAX_DELAY);
}

BaseType_t readFromNFCtxQFromISR(char* msg) {
    return xQueueReceiveFromISR(nfc_appData.txBufferQ, msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

// send InListPassiveTarget to NFC Reader
void checkNFC() {
    // start bytes 0, 1
    // len of data
    // LCS = 0 - len
    // TFI
    // message type
    // message
    // DCS = 0 - TFI - message
    addToNFCTXBufferQ(START0);
    addToNFCTXBufferQ(START0);
    addToNFCTXBufferQ(START1);
    addToNFCTXBufferQ(4); // len goes here
    addToNFCTXBufferQ(252); // LCS goes here
    addToNFCTXBufferQ(TFI);
    addToNFCTXBufferQ(0x4a); // message type
    addToNFCTXBufferQ(1); // MaxTg
    addToNFCTXBufferQ(0); // BrTy
    addToNFCTXBufferQ(0xe0); // DCS 
}

bool getACK() {
    char inbyte;
    // get start bytes
    while(1) {
        if (xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY)) { // wait a while, but not too long
            if (inbyte == '\0') {
                while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
                if (inbyte == '\xff') {
                    break;
                }
            }
        }
        else {
            return pdFALSE;
        }
    }
    while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
    if (inbyte != '\0') {
        return pdFALSE;
    }
    while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
    if (inbyte != '\xff') {
        return pdFALSE;
    }
    return pdTRUE;
}

// send a NACK if something went wrong
void sendNACK() {
    addToNFCTXBufferQ(START0);
    addToNFCTXBufferQ(START1);
    addToNFCTXBufferQ(0xff);
    addToNFCTXBufferQ(0x00);
}

bool getCheckNFC() {
    // start bytes 0, 1
    // len of data
    // LCS = 0 - len
    // TFI
    // message type
    // message
    // DCS = 0 - TFI - message
    
    char inbyte;
    char len;
    // get start bytes
    while(1) { // loop until a proper message is received
        while(1) { // loop for start bytes
            if (xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY)) {
                if (inbyte == '\0') {
                    while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
                    if (inbyte == '\xff') {
                        break;
                    }
                }
            }
        }
        // get len
        while(!xQueueReceive(nfc_appData.rxQ, &len, portMAX_DELAY));
        // get LCS
        while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
        if (len - inbyte != 0) {
            sendNACK();
            continue;
        }
        // get TFI
        while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
        // get message type
        while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
        if (inbyte != '\x4b') {
            sendNACK();
            continue;
        }
        // get number of targets
        while(!xQueueReceive(nfc_appData.rxQ, &inbyte, portMAX_DELAY));
        if (inbyte == '\0') {
            return pdFALSE;
        }
        else {
            return pdTRUE;
        }
    }
}

InternalMessage tokenFound() {
    InternalMessage msg;
    msg.type = TOKEN_FOUND;
    msg.size = 1;
    msg.msg[0] = 1;
    return msg;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void NFC_APP_Initialize ( void )

  Remarks:
    See prototype in nfc_app.h.
 */

void NFC_APP_Initialize ( void )
{
    nfc_appData.inQ = xQueueCreate(8, sizeof(char));
    vQueueAddToRegistry(nfc_appData.inQ, nfc_receive_q);
    nfc_appData.rxQ = xQueueCreate(32, sizeof(char));
    vQueueAddToRegistry(nfc_appData.rxQ, nfc_uart_rx_q);
    nfc_appData.txBufferQ = xQueueCreate(TX_BUF_SIZE, sizeof(char));
    vQueueAddToRegistry(nfc_appData.txBufferQ, nfc_uart_tx_q);
}


/******************************************************************************
  Function:
    void NFC_APP_Tasks ( void )

  Remarks:
    See prototype in nfc_app.h.
 */

void NFC_APP_Tasks ( void )
{
    char inChar;
    while(1) {
        while(!xQueueReceive(nfc_appData.inQ, &inChar, portMAX_DELAY));
        sendDebugMessage("CHECK NFC\0");
        checkNFC();
        sendDebugMessage("WAIT ON ACK\0");
        while(!getACK()) {
            sendDebugMessage("SEND NACK\0");
            sendNACK();
        }
        sendDebugMessage("GOT ACK\0");
        if (getCheckNFC()) {
            sendDebugMessage("TOKEN FOUND\0");
            addToUartTXQ(tokenFound());
        }
        else {
            sendDebugMessage("NO TOKEN\0");
        }
        
    }
}
 

/*******************************************************************************
 End of File
 */
