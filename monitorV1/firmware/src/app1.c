/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app1.c

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

#include "app1.h"

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

APP1_DATA app1Data;

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
        
    ////////////////////////////////////////////////////////////////////////    
    // INCOMING MESSAGE HANDLER
    //////////////////////////////////////////////////////////////////////// 
    void processApp1Message(){

        setDebugVal(APP1_PROCESS_MSG);
        
        switch(app1Data.sysMsg.type){

            ////////////////////////////////////////////////////////////////    
            // OBJECT POSITIONS
            ////////////////////////////////////////////////////////////////  
            case OBJECT_POS:

                switch(app1Data.sysMsg.msg[0]){

                    case LEAD: // Lead Rover
                        app1Data.rovL.xcordMsb = app1Data.sysMsg.msg[1];
                        app1Data.rovL.xcordLsb = app1Data.sysMsg.msg[2];
                        app1Data.rovL.ycordMsb = app1Data.sysMsg.msg[3];
                        app1Data.rovL.ycordLsb = app1Data.sysMsg.msg[4];
                        app1Data.rovL.angleMsb = app1Data.sysMsg.msg[5];
                        app1Data.rovL.angleLsb = app1Data.sysMsg.msg[6];
                        buildRpiRoverLocationMessage();
                        break;

                    case FOLLOWER: // Follower
                        app1Data.rovF.xcordMsb = app1Data.sysMsg.msg[1];
                        app1Data.rovF.xcordLsb = app1Data.sysMsg.msg[2];
                        app1Data.rovF.ycordMsb = app1Data.sysMsg.msg[3];
                        app1Data.rovF.ycordLsb = app1Data.sysMsg.msg[4];
                        app1Data.rovF.angleMsb = app1Data.sysMsg.msg[5];
                        app1Data.rovF.angleLsb = app1Data.sysMsg.msg[6];
                        buildRpiRoverLocationMessage();  
                        break;

                    case OBSTACLE:
                    case TOKEN:
                        buildRpiObjectLocationMessage();
                        break;

                    default:
                        break;
                }

            ////////////////////////////////////////////////////////////////    
            // TOKEN FOUND
            ////////////////////////////////////////////////////////////////       
            break;
            case TOKEN_FOUND:

                
                switch(app1Data.sysMsg.msg[0]){

                case LEAD_ROVER:
                    // Leader found a token
                    toggleDebugLED();
                    buildRpiTokenFoundMessage();    // Send mesage to RPi for processing
                    break;

                case FOLLOWER:
                    // Follower picked up a token
                    // Follower omitted from project
                    break;

                default:
                    break;
                }

            break;
            
            /////////////////////////////////////////////////////////////////    
            // INITIAL ROUTES
            /////////////////////////////////////////////////////////////////   
            case INITIAL_ROUTE_START:
                app1Data.lr_initial_route.routeLen_actual = app1Data.sysMsg.msg[0];
            break;
            
            case INITIAL_ROUTE_DATA:  
                addToInitialRoute();
            break;
            
            case INITIAL_ROUTE_END:
                sendInitRouteToRpi(); // Not developed due to lack of coordinator
            break;

            ////////////////////////////////////////////////////////////////    
            // INITIAL OBJECT LOCATIONS FROM RPI
            //////////////////////////////////////////////////////////////// 
            case INITIAL_OBJ_POS:
                processInitialObjectLocation();         
            break;
            
            ////////////////////////////////////////////////////////////////    
            // ROVER MOTIONS
            //////////////////////////////////////////////////////////////// 
            case ROVER_MOVE:
                
                buildRpiRoverMoveMsg();

                /*switch(app1Data.sysMsg.msg[0]){

                case ROVER_STOP:
                    // Lead Rover has completed it's movement
                    
                    app1Data.divisor_LeadRover++;       // Incriment the movement count
                    runRoverStats();                    // Calculate and send results to Rpi
                    break;

                case ROVER_FORWARD:
                case ROVER_BACKWARD:
                case ROVER_LEFT:
                case ROVER_RIGHT:
                    // Lead Rover has been commanded to move
                    // Make record of Lead Rover's current location
                    app1Data.rovL_startMove.xcordMsb = app1Data.rovL.xcordMsb;
                    app1Data.rovL_startMove.xcordLsb = app1Data.rovL.xcordLsb;
                    app1Data.rovL_startMove.ycordMsb = app1Data.rovL.ycordMsb;
                    app1Data.rovL_startMove.ycordLsb = app1Data.rovL.ycordLsb;
                    app1Data.rovL_startMove.angleMsb = app1Data.rovL.angleMsb;
                    app1Data.rovL_startMove.angleLsb = app1Data.rovL.angleLsb;
                    
                    // Store movement type in id field
                    app1Data.rovL_startMove.id = app1Data.sysMsg.msg[0];
                    
                    // Store movement value in reserved field
                    app1Data.rovL_startMove.reserved = app1Data.sysMsg.msg[1];
                    
                    break;

                default:
                    break;
                }
                */
            ////////////////////////////////////////////////////////////////    
            // RESET APP STATS
            //////////////////////////////////////////////////////////////// 
            case CLEAR_AC_STATS:
                initRoverStats();
                break;
                
            break;
            
            default:
                break;
        }
    }

    BaseType_t sendMsgToApp1(SysMsg inSysMsg){
        return xQueueSend(app1Data.app1InternalQ, &inSysMsg, portMAX_DELAY);
    }
    BaseType_t sendMsgToApp1_fromISR(SysMsg inSysMsg){
        return xQueueSendFromISR(app1Data.app1InternalQ, &inSysMsg, 0);
    }

    ////////////////////////////////////////////////////////////////////////    
    // RPI MESSAGE BUILDERS
    ////////////////////////////////////////////////////////////////////////  
    void buildRpiRoverLocationMessage(){

    app1Data.rpiMsg.source = MY_ROLE;
    app1Data.rpiMsg.type = OBJECT_POS; // This is an object position
    app1Data.rpiMsg.size = app1Data.sysMsg.size;
    for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++){
        app1Data.rpiMsg.msg[app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
    }

    sendOutgoingMsgToPiTx(app1Data.rpiMsg);

}
    void buildRpiObjectLocationMessage(){

        app1Data.rpiMsg.source = MY_ROLE;
        app1Data.rpiMsg.type = OBJECT_POS; 
        app1Data.rpiMsg.size = app1Data.sysMsg.size;
        for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++){
            app1Data.rpiMsg.msg[app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
        }
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);

    }
    void buildRpiTokenLocationMessage(){

        app1Data.rpiMsg.source = MY_ROLE;
        app1Data.rpiMsg.type = OBJECT_POS;
        app1Data.rpiMsg.size = app1Data.sysMsg.size;
        for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++){
            app1Data.rpiMsg.msg[app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
        }
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);

    }
    void buildRpiTokenFoundMessage(){

        app1Data.rpiMsg.source = MY_ROLE;
        app1Data.rpiMsg.type = TOKEN_FOUND;
        app1Data.rpiMsg.size = app1Data.sysMsg.size;
        for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++){
            app1Data.rpiMsg.msg[app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
        }
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);

    }
    void buildRpiAccuracyMsg(){
        
        app1Data.rpiMsg.source = MY_ROLE;
        app1Data.rpiMsg.type = ACC_STATS;
        app1Data.rpiMsg.size = 0x09;
        
        // LR ROT ERR
        app1Data.rpiMsg.msg[0] = ROT_ERR;
        app1Data.rpiMsg.msg[1] = app1Data.leadRover_angle_error.c[0];
        app1Data.rpiMsg.msg[2] = app1Data.leadRover_angle_error.c[1];
        app1Data.rpiMsg.msg[3] = app1Data.leadRover_angle_error.c[2];
        app1Data.rpiMsg.msg[4] = app1Data.leadRover_angle_error.c[3];
        app1Data.rpiMsg.msg[5] = app1Data.leadRover_angle_error.c[4];
        app1Data.rpiMsg.msg[6] = app1Data.leadRover_angle_error.c[5];
        app1Data.rpiMsg.msg[7] = app1Data.leadRover_angle_error.c[6];
        app1Data.rpiMsg.msg[8] = app1Data.leadRover_angle_error.c[7];
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);
        
        // LR MAX ROT ERR
        app1Data.rpiMsg.msg[0] = ROT_ERR_MAX;
        app1Data.rpiMsg.msg[1] = app1Data.leadRover_angle_error_max.c[0];
        app1Data.rpiMsg.msg[2] = app1Data.leadRover_angle_error_max.c[1];
        app1Data.rpiMsg.msg[3] = app1Data.leadRover_angle_error_max.c[2];
        app1Data.rpiMsg.msg[4] = app1Data.leadRover_angle_error_max.c[3];
        app1Data.rpiMsg.msg[5] = app1Data.leadRover_angle_error_max.c[4];
        app1Data.rpiMsg.msg[6] = app1Data.leadRover_angle_error_max.c[5];
        app1Data.rpiMsg.msg[7] = app1Data.leadRover_angle_error_max.c[6];
        app1Data.rpiMsg.msg[8] = app1Data.leadRover_angle_error_max.c[7];
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);
        
        // LR MV ERR
        app1Data.rpiMsg.msg[0] = TRV_ERR;
        app1Data.rpiMsg.msg[1] = app1Data.leadRover_travel_error.c[0];
        app1Data.rpiMsg.msg[2] = app1Data.leadRover_travel_error.c[1];
        app1Data.rpiMsg.msg[3] = app1Data.leadRover_travel_error.c[2];
        app1Data.rpiMsg.msg[4] = app1Data.leadRover_travel_error.c[3];
        app1Data.rpiMsg.msg[5] = app1Data.leadRover_travel_error.c[4];
        app1Data.rpiMsg.msg[6] = app1Data.leadRover_travel_error.c[5];
        app1Data.rpiMsg.msg[7] = app1Data.leadRover_travel_error.c[6];
        app1Data.rpiMsg.msg[8] = app1Data.leadRover_travel_error.c[7];
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);
        
        // LR MAX MV ERR
        app1Data.rpiMsg.msg[0] = TRV_ERR_MAX;
        app1Data.rpiMsg.msg[1] = app1Data.leadRover_travel_error_max.c[0];
        app1Data.rpiMsg.msg[2] = app1Data.leadRover_travel_error_max.c[1];
        app1Data.rpiMsg.msg[3] = app1Data.leadRover_travel_error_max.c[2];
        app1Data.rpiMsg.msg[4] = app1Data.leadRover_travel_error_max.c[3];
        app1Data.rpiMsg.msg[5] = app1Data.leadRover_travel_error_max.c[4];
        app1Data.rpiMsg.msg[6] = app1Data.leadRover_travel_error_max.c[5];
        app1Data.rpiMsg.msg[7] = app1Data.leadRover_travel_error_max.c[6];
        app1Data.rpiMsg.msg[8] = app1Data.leadRover_travel_error_max.c[7];
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);
        
    }
    void buildRpiRoverMoveMsg(){
        app1Data.rpiMsg.source = MY_ROLE;
        app1Data.rpiMsg.type = ROVER_MOVE;
        app1Data.rpiMsg.size = app1Data.sysMsg.size;
        for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++){
            app1Data.rpiMsg.msg[app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
        }
        sendOutgoingMsgToPiTx(app1Data.rpiMsg);
    }
    ////////////////////////////////////////////////////////////////////////    
    // INITIAL ROUTE AND POSITION HANDLERS
    //////////////////////////////////////////////////////////////////////// 
    void clearInitialRoute(){

        // Init Path to 0xAA

        app1Data.itt = 0;

        for(app1Data.itt = 0 ; app1Data.itt < MAX_ROUTE_LEN; app1Data.itt ++ ){
            app1Data.lr_initial_route.initRoute[app1Data.itt] = 0xAA;   
        }

        // Init route keeper to receive initial route information
        app1Data.routeKeeper = 0;

    }
    void addToInitialRoute(){

    int keep;
    keep = app1Data.routeKeeper;

    for(app1Data.itt = 0 ; app1Data.itt < app1Data.sysMsg.size ; app1Data.itt ++ ){
        app1Data.lr_initial_route.initRoute[keep + app1Data.itt] = app1Data.sysMsg.msg[app1Data.itt];
        app1Data.routeKeeper++;
    }

}
    void processInitialObjectLocation(){

    unsigned int xpos;
    unsigned int ypos;

    xpos = (app1Data.sysMsg.msg[1]) << 8 | (app1Data.sysMsg.msg[2]);
    ypos = (app1Data.sysMsg.msg[3]) << 8 | (app1Data.sysMsg.msg[4]);

    switch(app1Data.sysMsg.msg[0]) {

    case OBS1:
        app1Data.O1.xcord = xpos;
        app1Data.O1.ycord = ypos;
        break;
    case OBS2:
        app1Data.O2.xcord = xpos;
        app1Data.O2.ycord = ypos;
        break;
    case OBS3:
        app1Data.O3.xcord = xpos;
        app1Data.O3.ycord = ypos;
        break;
    case OBS4:
        app1Data.O4.xcord = xpos;
        app1Data.O4.ycord = ypos;
        break;
    case TOK1:
        app1Data.T1.xcord = xpos;
        app1Data.T1.ycord = ypos;
        break;
    case TOK2:
        app1Data.T2.xcord = xpos;
        app1Data.T2.ycord = ypos;
        break;
    case TOK3:
        app1Data.T3.xcord = xpos;
        app1Data.T3.ycord = ypos;
        break;
    case TOK4:
        app1Data.T4.xcord = xpos;
        app1Data.T4.ycord = ypos;
        break;
    default:
        break; 
    }

}
    void sendInitRouteToRpi(){

    }
    ////////////////////////////////////////////////////////////////////////    
    // ROVER STAT FUNCTIONS
    ////////////////////////////////////////////////////////////////////////
    void runRoverStats(){
        
        setDebugVal(RUN_ROVER_STATS);
        
        unsigned int x1,x2,y1,y2,ang1,ang2;
        double error;
        
        x1 = (app1Data.rovL_startMove.xcordMsb << 8 ) | ( app1Data.rovL_startMove.xcordLsb & 0x00ff );
        x2 = (app1Data.rovL.xcordMsb << 8 ) | ( app1Data.rovL.xcordLsb & 0x00ff );
        
        y1 = (app1Data.rovL_startMove.ycordMsb << 8 ) | ( app1Data.rovL_startMove.ycordLsb & 0x00ff );
        y2 = (app1Data.rovL.ycordMsb << 8 ) | ( app1Data.rovL.ycordLsb & 0x00ff );
        
        ang1 = (app1Data.rovL_startMove.angleMsb << 8 ) | ( app1Data.rovL_startMove.angleLsb & 0x00ff );
        ang2 = (app1Data.rovL.angleMsb << 8 ) | ( app1Data.rovL.angleLsb & 0x00ff );

        switch(app1Data.rovL_startMove.id){
            
        case ROVER_FORWARD:
        case ROVER_BACKWARD:
            
            setDebugVal(ROVER_FWD_BK);
            
            error = abs( app1Data.rovL_startMove.reserved - (sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) )) );
            app1Data.runningError_LeadRover_travel.val += error;
            
            if(error > app1Data.leadRover_travel_error_max.val){
                app1Data.leadRover_travel_error_max.val = error;
            }
            
            break;
            
        case ROVER_LEFT:
        case ROVER_RIGHT:
            
            setDebugVal(ROVER_LFT_RGHT);
            
            if( ((int)ang2 - (int)ang1) < 0){
                error = abs( (app1Data.rovL_startMove.reserved) - (360 + ((int)ang2 - (int)ang1)) );
            }
            
            else if( ((int)ang2 - (int)ang1) >= 0){
                error = abs( (app1Data.rovL_startMove.reserved) - (((int)ang2 - (int)ang1)) );
            }
            
            else {
                // should never arriver here
            }
            
            app1Data.runningError_LeadRover_angle.val += error;
            
            if(error > app1Data.leadRover_angle_error_max.val){
                app1Data.leadRover_angle_error_max.val = error;
            }
            
            break;
            
        case ROVER_STOP:
            // This is an error. ROVER_STOP should not be stored in the
            // initial movement's id field
            break;
        default:
            break;
        }
        
        buildRpiAccuracyMsg();
    }
    void initRoverStats(){
        // Prime rover accuracy figures
        
        app1Data.init.id = 0x00;
        app1Data.init.reserved = 0x00;
        app1Data.init.xcordMsb = 0x00;
        app1Data.init.xcordLsb = 0x00;
        app1Data.init.ycordMsb = 0x00;
        app1Data.init.ycordLsb = 0x00;
        app1Data.init.angleMsb = 0x00;
        app1Data.init.angleLsb = 0x00;
        app1Data.init.numSamples = 0;
        app1Data.rovL = app1Data.init;
        app1Data.rovL_startMove = app1Data.init;
        app1Data.rovF = app1Data.init;
        app1Data.rovF_startMove = app1Data.init;

        app1Data.divisor_LeadRover = 0;
        app1Data.runningError_LeadRover_travel.val = 0;
        app1Data.runningError_LeadRover_angle.val = 0;
        app1Data.leadRover_travel_error.val = 0;
        app1Data.leadRover_angle_error.val = 0;
        app1Data.leadRover_travel_error_max.val = 0;
        app1Data.leadRover_angle_error_max.val = 0;
        
    }

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP1_Initialize ( void )

  Remarks:
    See prototype in app1.h.
 */

void APP1_Initialize ( void )
{
    setDebugVal(APP1_INIT_TOP);
    app1Data.state = APP1_STATE_INIT;
    app1Data.app1InternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    initRoverStats();
    clearInitialRoute();
 
}

/******************************************************************************
  Function:
    void APP1_Tasks ( void )

  Remarks:
    See prototype in app1.h.
 */

void APP1_Tasks ( void )
{
   
    switch ( app1Data.state )
    {
        
        case APP1_STATE_INIT:
        {
            setDebugVal(APP1_INIT_STATE);
            app1Data.state = APP1_STATE_ONE;
            break;
        }
        
        case APP1_STATE_ONE:
        {
            setDebugVal(APP1_S_ONE);
            
            if(xQueueReceive(app1Data.app1InternalQ, &app1Data.sysMsg, portMAX_DELAY)){
                setDebugVal(APP1_SYS_MSG_RECV);
                setDebugVal(app1Data.sysMsg.type);
                setDebugVal(app1Data.sysMsg.msg[0]);
                processApp1Message();
            }
            
            break;
        }
        default:
        {
            setDebugVal(APP1_DEFAULT_STATE);
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    setDebugVal(APP1_TASKS_COMPLETE);
}

//////////////////////////////////////////////////////////////////////////////// 
// MARKED FOR REMOVAL
////////////////////////////////////////////////////////////////////////////////
/*
void runObsTokSensorAccuracy(){
    
    unsigned int xpos;
    unsigned int ypos;
    unsigned int angle;
    unsigned int length;
    unsigned int width;
    
    double difference;
    
    OBSTACLE_DATA tempObsDat;
    TOKEN_DATA tempTokDat;
    
    xpos = (app1Data.sysMsg.msg[1]) << 8 | (app1Data.sysMsg.msg[2]);
    ypos = (app1Data.sysMsg.msg[3]) << 8 | (app1Data.sysMsg.msg[4]);
    angle = (app1Data.sysMsg.msg[5]) << 8 | (app1Data.sysMsg.msg[6]);
    length = (app1Data.sysMsg.msg[7]) << 8 | (app1Data.sysMsg.msg[8]);
    width = (app1Data.sysMsg.msg[9]) << 8 | (app1Data.sysMsg.msg[10]);
    
    switch(app1Data.sysMsg.msg[1]) {
        
    case OBS1:
        tempObsDat = app1Data.O1;
        break;
    case OBS2:
        tempObsDat = app1Data.O2;
        break;
    case OBS3:
        tempObsDat = app1Data.O3;
        break;
    case OBS4:
        tempObsDat = app1Data.O4;
        break;
    case TOK1:
        tempTokDat = app1Data.T1;
        break;
    case TOK2:
        tempTokDat = app1Data.T2;
        break;
    case TOK3:
        tempTokDat = app1Data.T3;
        break;
    case TOK4:
        tempTokDat = app1Data.T4;
        break;
    default:
        break; 
    }
    
    switch(app1Data.sysMsg.msg[1]) {
        
    case OBS1:
    case OBS2:
    case OBS3:
    case OBS4:
        difference = sqrt( (xpos - tempObsDat.xcord)*(xpos - tempObsDat.xcord)
                            + (ypos - tempObsDat.ycord)*(ypos - tempObsDat.ycord)  );
        break;
    case TOK1:
    case TOK2:
    case TOK3:
    case TOK4:
        difference = sqrt( (xpos - tempTokDat.xcord)*(xpos - tempTokDat.xcord)
                            + (ypos - tempTokDat.ycord)*(ypos - tempTokDat.ycord)  );
        break;
    default:
        break;
        
    }
    
    app1Data.runningError += difference;
    app1Data.divisor++;
    
}

*/
/*******************************************************************************
 End of File
 */
