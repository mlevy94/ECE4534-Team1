/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motorapp.c

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

#include "motorapp.h"
#include "motorapp_public.h"
#include "peripheral/oc/plib_oc.h"
#include "debug.h"
#include "uart_tx_app_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define OC_LEFT  OC_ID_2
#define OC_RIGHT OC_ID_1
// Values for motor adjustment algorithm
#define TARGET_MOVE_LEFT  250
#define TARGET_MOVE_RIGHT 247
#define TARGET_TURN_LEFT  110
#define TARGET_TURN_RIGHT 103
#define PWM_MAX_VAL     10000
#define PWM_START_MAX    9600
#define PWM_START_HALF   7000
#define LEFT_ERROR          1
#define LEFT_ERROR_INT      4
#define RIGHT_ERROR         1
#define RIGHT_ERROR_INT     4
#define INCH_TO_EN         19
#define DEG_TO_EN          38

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

static MOTORAPP_DATA motorData;

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
void setLeftForward(bool dir) {
    if (dir) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    }
}

void setRightForward(bool dir) {
    if (dir) {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
    else {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    }
}

void motorStartMax() {
    motorData.leftMotor = &motorData.leftMotorFull;
    motorData.rightMotor = &motorData.rightMotorFull;
}

void motorStartHalf() {
    motorData.leftMotor = &motorData.leftMotorHalf;
    motorData.rightMotor = &motorData.rightMotorHalf;
}

void motorStop() {
    if (motorData.intEN) {
        sendDebugMessage("SEARCHING\0");
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_3);
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_EXTERNAL_3);
        motorData.intEN = pdFALSE;
    }
    motorData.leftMotor = &motorData.stopMotor;
    motorData.rightMotor = &motorData.stopMotor;
}

void motorMove(char direction, char distance) {
    switch(direction) {
                case(ROVER_FORWARD):
                    setLeftForward(pdTRUE);
                    setRightForward(pdTRUE);
                    if (distance >= 60) {
                        motorData.intEN = pdTRUE;
                    }
                    motorData.moveStop = distance * INCH_TO_EN;
                    motorStartMax();
                    break;
                case(ROVER_BACKWARD):
                    setLeftForward(pdFALSE);
                    setRightForward(pdFALSE);
                    motorData.moveStop = distance * INCH_TO_EN;
                    motorStartMax();
                    break;
                case(ROVER_LEFT):
                    setLeftForward(pdFALSE);
                    setRightForward(pdTRUE);
                    motorData.moveStop = (distance / 5) * DEG_TO_EN;
                    motorStartHalf();
                    break;
                case(ROVER_RIGHT):
                    setLeftForward(pdTRUE);
                    setRightForward(pdFALSE);
                    motorData.moveStop = (distance / 5) * DEG_TO_EN;
                    motorStartHalf();
                    break;
                case(ROVER_STOP):
                    motorData.moveStop = 0;
                    motorStop();
                    addToUartTXQ(roverStopped());
                    break;
                default:
                    motorData.moveStop = 0;
                    motorStop();
                    addToUartTXQ(roverStopped());
                    sendDebugMessage("MOTOR FAULT\0");
                    break;
                    
            }
    PLIB_OC_PulseWidth16BitSet(OC_LEFT, motorData.leftMotor->pwm);
    PLIB_OC_PulseWidth16BitSet(OC_RIGHT, motorData.rightMotor->pwm);
}

void resetToken() {
    motorData.intEN = pdFALSE;
}

void incLeftEn() {
    motorData.leftMotor->encoder++;
}

void incRightEn() {
    motorData.rightMotor->encoder++;
}

void incMoveCount() {
    if (motorData.moveStop == 0) {
        return;
    }
    else if (motorData.moveCounter >= motorData.moveStop) {
        if (xQueueSendFromISR(motorData.stopQ, &motorData.moveStop, 0)) {
            motorData.moveCounter = 0;
            motorData.moveStop = 0;
            addToUartTXQFromISR(roverStopped());
        }
    }
    else {
        motorData.moveCounter++;
    }
}

void pi(Motor* motor, int16_t errorC, int16_t errorCI) {
    uint16_t encoder = motor->encoder - motor->prevEncoder;
    motor->prevEncoder = motor->encoder;
    int16_t error = (int16_t)(motor->targetEncoder - (encoder * 10));
    motor->error += error;
    motor->pwm = (error * errorC) + (motor->error * errorCI);
    if (motor->pwm > PWM_MAX_VAL) {
        motor->pwm = PWM_MAX_VAL;
    }
}

void motorAdj() {
    if (motorData.leftMotor == &motorData.stopMotor) {
        return; // motors not running
    }
    pi(motorData.leftMotor, LEFT_ERROR, LEFT_ERROR_INT);
    pi(motorData.rightMotor, RIGHT_ERROR, RIGHT_ERROR_INT);

    // set adjusted values
    PLIB_OC_PulseWidth16BitSet(OC_LEFT, motorData.leftMotor->pwm);
    PLIB_OC_PulseWidth16BitSet(OC_RIGHT, motorData.rightMotor->pwm);
}

BaseType_t addToMotorQ(InternalMessage* msg) {
    return xQueueSend(motorData.motorQ, msg, portMAX_DELAY);
}

BaseType_t addToMotorQFromISR(InternalMessage* msg) {
    return xQueueSendFromISR(motorData.motorQ, msg, 0);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTORAPP_Initialize ( void )

  Remarks:
    See prototype in motorapp.h.
 */

void initMotor(Motor* motor, int16_t pwm, int16_t targetPWM, int16_t targetEncoder, int16_t error) {
    motor->pwm = pwm;
    motor->encoder = 0;
    motor->prevEncoder = 0;
    motor->targetPWM = targetPWM;
    motor->targetEncoder = targetEncoder;
    motor->error = error;
}

void MOTORAPP_Initialize ( void )
{
    motorData.stopQ = xQueueCreate(10, sizeof(int16_t)); 
    vQueueAddToRegistry(motorData.stopQ, motor_stop_q);
    motorData.motorQ = xQueueCreate(20, sizeof(InternalMessage));
    vQueueAddToRegistry(motorData.motorQ, motor_q);
    initMotor(&motorData.leftMotorFull, PWM_START_MAX, PWM_START_MAX, TARGET_MOVE_LEFT, PWM_START_MAX / LEFT_ERROR_INT);
    initMotor(&motorData.rightMotorFull, PWM_START_MAX, PWM_START_MAX, TARGET_MOVE_RIGHT, PWM_START_MAX / RIGHT_ERROR_INT);
    initMotor(&motorData.leftMotorHalf, PWM_START_HALF, PWM_START_HALF, TARGET_TURN_LEFT, PWM_START_HALF / LEFT_ERROR_INT);
    initMotor(&motorData.rightMotorHalf, PWM_START_HALF, PWM_START_HALF, TARGET_TURN_RIGHT, PWM_START_HALF / RIGHT_ERROR_INT);
    initMotor(&motorData.stopMotor, 0, 0, 0, 0);
    motorData.moveCounter = 0;
    motorData.moveStop = 0;
    motorData.intEN = pdFALSE;
    motorStop();
    motorData.motorAdjTimer = xTimerCreate("motor adjust timer",
                                              // Sets a timer frequency to 50 Hz
                                              ( 3.3 * portTICK_PERIOD_MS ), 
                                              pdTRUE,
                                              (void *) 2,
                                              motorAdj);
    xTimerStart( motorData.motorAdjTimer, 0);
}

/******************************************************************************
  Function:
    void MOTORAPP_Tasks ( void )

  Remarks:
    See prototype in motorapp.h.
 */

void MOTORAPP_Tasks ( void )
{
    PLIB_OC_Enable(OC_LEFT);
    PLIB_OC_Enable(OC_RIGHT);
    PLIB_TMR_Start(TMR_ID_2);
    PLIB_TMR_Start(TMR_ID_3);
    PLIB_TMR_Start(TMR_ID_4);
    setLeftForward(pdTRUE);
    setRightForward(pdTRUE);
    int16_t go;
    while(1) {
        if(xQueueReceive(motorData.motorQ, &motorData.moveCMD, portMAX_DELAY)) {
            motorMove(motorData.moveCMD.msg[0], motorData.moveCMD.msg[1]);
            // clear token LED on next move
            SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
            // wait until done with move to get next.
            if ( motorData.moveCMD.msg[1] > 0) {
                while (!xQueueReceive(motorData.stopQ, &go, portMAX_DELAY));
                motorStop();
                PLIB_OC_PulseWidth16BitSet(OC_LEFT, motorData.leftMotor->pwm);
                PLIB_OC_PulseWidth16BitSet(OC_RIGHT, motorData.rightMotor->pwm);
            }
        }
    }
}
 

/*******************************************************************************
 End of File
 */
