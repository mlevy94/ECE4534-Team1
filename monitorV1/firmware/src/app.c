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

APP_DATA appData;

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
    void processApp0Message(){

            switch(appData.sysMsg.type){

                case MONITOR:
                case LEAD_ROVER:
                case FOLLOWER:
                case SENSORS:
                case COORDINATOR:
                    buildRpiDebugMessage();
                    break;
                case NET_STAT:
                    store_netstat();
                    break;
                case CLEAR_NET_STATS:
                    clearNetStats();  
                    break;
                case SEND_NET_STATS:
                    update_netstats();
                    sendNetStats();
                    sendHeartbeat();
                break;
                default:
                    break;
            }
        }

    ////////////////////////////////////////////////////////////////////////    
    // MSG QUEUE ACCESS FUNCTIONS
    //////////////////////////////////////////////////////////////////////// 
    BaseType_t sendMsgToApp(SysMsg inSysMsg){
        return xQueueSend(appData.appInternalQ, &inSysMsg, portMAX_DELAY);
    }
    BaseType_t sendMsgToApp_fromISR(SysMsg inSysMsg){
        return xQueueSendFromISR(appData.appInternalQ, &inSysMsg, 0);
    }

    ////////////////////////////////////////////////////////////////////////    
    // RPI MESSAGE BUILDERS
    //////////////////////////////////////////////////////////////////////// 
    void buildRpiDebugMessage(){

        appData.rpiMsg.source = appData.sysMsg.type;    // Incoming SysMsg type carries the title of the sender
        appData.rpiMsg.type = OUTPUT_TO_MONITOR;        // This is an output message to the monitor
        appData.rpiMsg.size = appData.sysMsg.size;
        for(appData.itt = 0 ; appData.itt < appData.sysMsg.size ; appData.itt ++){
            appData.rpiMsg.msg[appData.itt] = appData.sysMsg.msg[appData.itt];
        }

        sendOutgoingMsgToPiTx(appData.rpiMsg);

    }
    void buildRpiNetstatMessage(uint8_t status, uint8_t target, int totalMissing, int totalMessages,
                                uint8_t yellow_ct, uint8_t red_ct){ // (FOR RPI)

        appData.rpiMsg.source = MY_ROLE;
        appData.rpiMsg.type = NET_STAT; // This is a net stat
        appData.rpiMsg.size = 0xC;
        appData.rpiMsg.msg[0] = target; // Originator
        appData.rpiMsg.msg[1] = status;
        appData.rpiMsg.msg[2] = (totalMissing & 0xff000000) >> 24;
        appData.rpiMsg.msg[3] = (totalMissing & 0x00ff0000) >> 16;
        appData.rpiMsg.msg[4] = (totalMissing & 0x0000ff00) >> 8;
        appData.rpiMsg.msg[5] = (totalMissing & 0x000000ff);
        appData.rpiMsg.msg[6] = (totalMessages & 0xff000000) >> 24;
        appData.rpiMsg.msg[7] = (totalMessages & 0x00ff0000) >> 16;
        appData.rpiMsg.msg[8] = (totalMessages & 0x0000ff00) >> 8;
        appData.rpiMsg.msg[9] = (totalMessages & 0x000000ff);
        appData.rpiMsg.msg[10]= yellow_ct;
        appData.rpiMsg.msg[11] = red_ct;
        
        sendOutgoingMsgToPiTx(appData.rpiMsg); 
    }

    ////////////////////////////////////////////////////////////////////////    
    // NETSTAT FUNCTIONS
    //////////////////////////////////////////////////////////////////////// 
    void clearNetStats(){

        appData.netstat_tripple.source = 0xEE;
        appData.netstat_tripple.last_msg_number = 0x00;
        appData.netstat_tripple.debug_state = 0xEE;
        appData.netstat_tripple.status = 0xEE;
        appData.netstat_tripple.count = 0;
        appData.netstat_tripple.last_count = 0;
        appData.netstat_tripple.last_time = 0;
        appData.netstat_tripple.red_reset_counter = 0;
        appData.netstat_tripple.times_red = 0;
        appData.netstat_tripple.yellow_elevate_counter = 0;
        appData.netstat_tripple.times_yellow = 0;
        appData.netstat_tripple.missing = 0;
        appData.netstat_tripple.init = 1;
        appData.netstat_tripple.long_per = 0;
        appData.netstat_tripple.short_per = -1;
        appData.netstat.lead_rover = appData.netstat_tripple;
        appData.netstat.foll_rover= appData.netstat_tripple;
        appData.netstat.cordinator = appData.netstat_tripple;
        appData.netstat.sensor = appData.netstat_tripple;

        appData.msg_rate_datum.count = 0;
        appData.msg_rate_datum.last_time = 0;
        appData.msg_rate_datum.long_per = 0;
        appData.msg_rate_datum.msg_count = 0;
        appData.msg_rate_datum.msg_type = 0x00;
        appData.msg_rate_datum.short_per = -1;

        appData.lead_msg_rates.msg0 = appData.msg_rate_datum;
        appData.lead_msg_rates.msg1 = appData.msg_rate_datum;
        appData.lead_msg_rates.msg2 = appData.msg_rate_datum;

        appData.follow_msg_rates.msg0 = appData.msg_rate_datum;
        appData.follow_msg_rates.msg1 = appData.msg_rate_datum;
        appData.follow_msg_rates.msg2 = appData.msg_rate_datum;

        appData.cord_msg_rates.msg0 = appData.msg_rate_datum;
        appData.cord_msg_rates.msg1 = appData.msg_rate_datum;
        appData.cord_msg_rates.msg2 = appData.msg_rate_datum;

        appData.sensor_msg_rates.msg0 = appData.msg_rate_datum;
        appData.sensor_msg_rates.msg1 = appData.msg_rate_datum;
        appData.sensor_msg_rates.msg2 = appData.msg_rate_datum;
    }
    void store_netstat(){

        switch(appData.sysMsg.msg[0]){
            case LEAD_ROVER:
                lr_store_netstat();
                break;
            case FOLLOWER:
                fr_store_netstat();
                break;
            case SENSORS:
                se_store_netstat();
                break;
            case COORDINATOR:
                co_store_netstat();
                break;
            default:
                break;
        }

    }
    void sendNetStats(){

        // Build Rpi Msg
        buildRpiNetstatMessage(appData.netstat.lead_rover.status,
                            LEAD_ROVER,
                            appData.netstat.lead_rover.missing, 
                            appData.netstat.lead_rover.count,
                            appData.netstat.lead_rover.times_yellow,
                            appData.netstat.lead_rover.times_red);
        // Build Rpi Msg
        buildRpiNetstatMessage(appData.netstat.foll_rover.status,
                                FOLLOWER,
                                appData.netstat.foll_rover.missing,
                                appData.netstat.foll_rover.count,
                                appData.netstat.foll_rover.times_yellow,
                                appData.netstat.foll_rover.times_red); 
        // Build Rpi Msg
        buildRpiNetstatMessage(appData.netstat.sensor.status,
                                SENSORS,
                                appData.netstat.sensor.missing,
                                appData.netstat.sensor.count,
                                appData.netstat.sensor.times_yellow,
                                appData.netstat.sensor.times_red);
        // Build Rpi Msg
        buildRpiNetstatMessage(appData.netstat.cordinator.status,
                                COORDINATOR,
                                appData.netstat.cordinator.missing,
                                appData.netstat.cordinator.count,
                                appData.netstat.cordinator.times_yellow,
                                appData.netstat.cordinator.times_red);    


    }
    void update_netstats(){



        ///////////////////////////////////////////////////////////////////////////
        // LEAD ROVER UPDATE NETSTATS
        ///////////////////////////////////////////////////////////////////////////

        if(appData.netstat.lead_rover.init == 1){

            appData.netstat.lead_rover.last_missing = appData.netstat.lead_rover.missing;
            appData.netstat.lead_rover.last_count == appData.netstat.lead_rover.count;
        }
        else{

            switch(appData.netstat.lead_rover.status){

            case GREEN:

                // This runs every 500 ms. If no message has been received in this timeframe
                // something is incorrect
                if(appData.netstat.lead_rover.last_count == appData.netstat.lead_rover.count){
                    appData.netstat.lead_rover.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                // If the number of missing messages is increasing, something is incorrect
                if(appData.netstat.lead_rover.last_missing < appData.netstat.lead_rover.missing){
                    appData.netstat.lead_rover.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                break;

            case YELLOW:

                // If we are still receiving messages and the number of missing
                // messages has not increased, we could be returning to nominal function
                if(appData.netstat.lead_rover.last_count != appData.netstat.lead_rover.count){
                    if(appData.netstat.lead_rover.last_missing == appData.netstat.lead_rover.missing){
                        appData.netstat.lead_rover.yellow_elevate_counter--;
                    }
                }
                // If we are not receiving messages and the number missing is 
                // still changing, the problem still exists
                else{
                   appData.netstat.lead_rover.yellow_elevate_counter++; 
                } 

                // If we have 2 seconds of YELLOW and are not moving
                // back to GREEN, move to RED
                if(appData.netstat.lead_rover.yellow_elevate_counter >= LR_YELLOW_DLY){
                    appData.netstat.lead_rover.yellow_elevate_counter = 0;
                    appData.netstat.lead_rover.status = RED;
                    appData.netstat_tripple.times_red++;
                    break;
                }

                // If we are receiving messages and the number missing has not 
                // increased, we can declare that we are now green
                if(appData.netstat.lead_rover.yellow_elevate_counter <= 0){
                    appData.netstat.lead_rover.status = GREEN;
                    appData.netstat.lead_rover.yellow_elevate_counter = 0;
                }

                break;

            case RED:

                // If the count is increasing then we may be returning to at least a YELLOW
                // state
                if(appData.netstat.lead_rover.last_count != appData.netstat.lead_rover.count){
                    appData.netstat.lead_rover.red_reset_counter++;
                }

                // If we have had 3 seconds of messages flowing in and the number of 
                // missing messages is not increasing, go to yellow
                if(appData.netstat.lead_rover.red_reset_counter >= LR_RED_DLY){
                    if(appData.netstat.lead_rover.last_missing == appData.netstat.lead_rover.missing){
                        appData.netstat.lead_rover.init = 1;
                        appData.netstat.lead_rover.red_reset_counter = 0;
                        appData.netstat.lead_rover.status = YELLOW;
                    }
                }

                break;

            case UNKOWN_LINK_STATE:

                if(appData.netstat.lead_rover.last_count < appData.netstat.lead_rover.count){
                    if(appData.netstat.lead_rover.last_missing == appData.netstat.lead_rover.missing){
                        appData.netstat.lead_rover.status = GREEN;
                    }
                }

                else{
                    appData.netstat.lead_rover.status = YELLOW;
                }

                break;

            } 

        }

        appData.netstat.lead_rover.last_count = appData.netstat.lead_rover.count;
        appData.netstat.lead_rover.last_missing = appData.netstat.lead_rover.missing;

        // END LEAD ROVER///////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        // FOLLOW ROVER UPDATE NETSTATS
        ///////////////////////////////////////////////////////////////////////////

        if(appData.netstat.foll_rover.init == 1){

            appData.netstat.foll_rover.last_missing = appData.netstat.foll_rover.missing;
            appData.netstat.foll_rover.last_count == appData.netstat.foll_rover.count;
        }
        else{

            switch(appData.netstat.foll_rover.status){

            case GREEN:

                // This runs every 500 ms. If no message has been received in this timeframe
                // something is incorrect
                if(appData.netstat.foll_rover.last_count == appData.netstat.foll_rover.count){
                    appData.netstat.foll_rover.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                // If the number of missing messages is increasing, something is incorrect
                if(appData.netstat.foll_rover.last_missing < appData.netstat.foll_rover.missing){
                    appData.netstat.foll_rover.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                break;

            case YELLOW:

                // If we are still receiving messages and the number of missing
                // messages has not increased, we could be returning to nominal function
                if(appData.netstat.foll_rover.last_count != appData.netstat.foll_rover.count){
                    if(appData.netstat.foll_rover.last_missing == appData.netstat.foll_rover.missing){
                        appData.netstat.foll_rover.yellow_elevate_counter--;
                    }
                }
                // If we are not receiving messages and the number missing is 
                // still changing, the problem still exists
                else{
                   appData.netstat.foll_rover.yellow_elevate_counter++; 
                } 

                // If we have 2 seconds of YELLOW and are not moving
                // back to GREEN, move to RED
                if(appData.netstat.foll_rover.yellow_elevate_counter >= FR_YELLOW_DLY){
                    appData.netstat.foll_rover.yellow_elevate_counter = 0;
                    appData.netstat.foll_rover.status = RED;
                    appData.netstat_tripple.times_red++;
                    break;
                }

                // If we are receiving messages and the number missing has not 
                // increased, we can declare that we are now green
                if(appData.netstat.foll_rover.yellow_elevate_counter <= 0){
                    appData.netstat.foll_rover.status = GREEN;
                    appData.netstat.foll_rover.yellow_elevate_counter = 0;
                }

                break;

            case RED:

                // If the count is increasing then we may be returning to at least a YELLOW
                // state
                if(appData.netstat.foll_rover.last_count != appData.netstat.foll_rover.count){
                    appData.netstat.foll_rover.red_reset_counter++;
                }

                // If we have had 3 seconds of messages flowing in and the number of 
                // missing messages is not increasing, go to yellow
                if(appData.netstat.foll_rover.red_reset_counter >= FR_RED_DLY){
                    if(appData.netstat.foll_rover.last_missing == appData.netstat.foll_rover.missing){
                        appData.netstat.foll_rover.init = 1;
                        appData.netstat.foll_rover.red_reset_counter = 0;
                        appData.netstat.foll_rover.status = YELLOW;
                    }
                }

                break;

            case UNKOWN_LINK_STATE:

                if(appData.netstat.foll_rover.last_count < appData.netstat.foll_rover.count){
                    if(appData.netstat.foll_rover.last_missing == appData.netstat.foll_rover.missing){
                        appData.netstat.foll_rover.status = GREEN;
                    }
                }

                else{
                    appData.netstat.foll_rover.status = YELLOW;
                }

                break;

            } 

        }

        appData.netstat.foll_rover.last_count = appData.netstat.foll_rover.count;
        appData.netstat.foll_rover.last_missing = appData.netstat.foll_rover.missing;

        // END FOLLOW ROVER///////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        // COORDINATOR UPDATE NETSTATS
        ///////////////////////////////////////////////////////////////////////////

        if(appData.netstat.cordinator.init == 1){

            appData.netstat.cordinator.last_missing = appData.netstat.cordinator.missing;
            appData.netstat.cordinator.last_count == appData.netstat.cordinator.count;
        }
        else{

            switch(appData.netstat.cordinator.status){

            case GREEN:

                // This runs every 500 ms. If no message has been received in this timeframe
                // something is incorrect
                if(appData.netstat.cordinator.last_count == appData.netstat.cordinator.count){
                    appData.netstat.cordinator.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                // If the number of missing messages is increasing, something is incorrect
                if(appData.netstat.cordinator.last_missing < appData.netstat.cordinator.missing){
                    appData.netstat.cordinator.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                break;

            case YELLOW:

                // If we are still receiving messages and the number of missing
                // messages has not increased, we could be returning to nominal function
                if(appData.netstat.cordinator.last_count != appData.netstat.cordinator.count){
                    if(appData.netstat.cordinator.last_missing == appData.netstat.cordinator.missing){
                        appData.netstat.cordinator.yellow_elevate_counter--;
                    }
                }
                // If we are not receiving messages and the number missing is 
                // still changing, the problem still exists
                else{
                   appData.netstat.cordinator.yellow_elevate_counter++; 
                } 

                // If we have 2 seconds of YELLOW and are not moving
                // back to GREEN, move to RED
                if(appData.netstat.cordinator.yellow_elevate_counter >= CO_YELLOW_DLY){
                    appData.netstat.cordinator.yellow_elevate_counter = 0;
                    appData.netstat.cordinator.status = RED;
                    appData.netstat_tripple.times_red++;
                    break;
                }

                // If we are receiving messages and the number missing has not 
                // increased, we can declare that we are now green
                if(appData.netstat.cordinator.yellow_elevate_counter <= 0){
                    appData.netstat.cordinator.status = GREEN;
                    appData.netstat.cordinator.yellow_elevate_counter = 0;
                }

                break;

            case RED:

                // If the count is increasing then we may be returning to at least a YELLOW
                // state
                if(appData.netstat.cordinator.last_count != appData.netstat.cordinator.count){
                    appData.netstat.cordinator.red_reset_counter++;
                }

                // If we have had 3 seconds of messages flowing in and the number of 
                // missing messages is not increasing, go to yellow
                if(appData.netstat.cordinator.red_reset_counter >= CO_RED_DLY){
                    if(appData.netstat.cordinator.last_missing == appData.netstat.cordinator.missing){
                        appData.netstat.cordinator.init = 1;
                        appData.netstat.cordinator.red_reset_counter = 0;
                        appData.netstat.cordinator.status = YELLOW;
                    }
                }

                break;

            case UNKOWN_LINK_STATE:

                if(appData.netstat.cordinator.last_count < appData.netstat.cordinator.count){
                    if(appData.netstat.cordinator.last_missing == appData.netstat.cordinator.missing){
                        appData.netstat.cordinator.status = GREEN;
                    }
                }

                else{
                    appData.netstat.cordinator.status = YELLOW;
                }

                break;

            } 

        }

        appData.netstat.cordinator.last_count = appData.netstat.cordinator.count;
        appData.netstat.cordinator.last_missing = appData.netstat.cordinator.missing;

        // END CORDINATOR///////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
        // SENSORS UPDATE NETSTATS
        ///////////////////////////////////////////////////////////////////////////

        if(appData.netstat.sensor.init == 1){

            appData.netstat.sensor.last_missing = appData.netstat.sensor.missing;
            appData.netstat.sensor.last_count == appData.netstat.sensor.count;
        }
        else{

            switch(appData.netstat.sensor.status){

            case GREEN:

                // This runs every 500 ms. If no message has been received in this timeframe
                // something is incorrect
                if(appData.netstat.sensor.last_count == appData.netstat.sensor.count ){
                    appData.netstat.sensor.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                // If the number of missing messages is increasing, something is incorrect
                if(appData.netstat.sensor.last_missing < appData.netstat.sensor.missing){
                    appData.netstat.sensor.status = YELLOW;
                    appData.netstat_tripple.times_yellow++;
                    break;
                }

                break;

            case YELLOW:

                // If we are still receiving messages and the number of missing
                // messages has not increased, we could be returning to nominal function
                if(appData.netstat.sensor.last_count != appData.netstat.sensor.count){
                    if(appData.netstat.sensor.last_missing == appData.netstat.sensor.missing){
                        appData.netstat.sensor.yellow_elevate_counter--;
                    }
                }
                // If we are not receiving messages and the number missing is 
                // still changing, the problem still exists
                else{
                   appData.netstat.sensor.yellow_elevate_counter++; 
                } 

                // If we have 2 seconds of YELLOW and are not moving
                // back to GREEN, move to RED
                if(appData.netstat.sensor.yellow_elevate_counter >= SE_YELLOW_DLY){
                    appData.netstat.sensor.yellow_elevate_counter = 0;
                    appData.netstat.sensor.status = RED;
                    appData.netstat_tripple.times_red++;
                    break;
                }

                // If we are receiving messages and the number missing has not 
                // increased, we can declare that we are now green
                if(appData.netstat.sensor.yellow_elevate_counter <= 0){
                    appData.netstat.sensor.status = GREEN;
                    appData.netstat.sensor.yellow_elevate_counter = 0;
                }

                break;

            case RED:

                // If the count is increasing then we may be returning to at least a YELLOW
                // state
                if(appData.netstat.sensor.last_count != appData.netstat.sensor.count){
                    appData.netstat.sensor.red_reset_counter++;
                }

                // If we have had 3 seconds of messages flowing in and the number of 
                // missing messages is not increasing, go to yellow
                if(appData.netstat.sensor.red_reset_counter >= SE_RED_DLY){
                    if(appData.netstat.sensor.last_missing == appData.netstat.sensor.missing){
                        appData.netstat.sensor.init = 1;
                        appData.netstat.sensor.red_reset_counter = 0;
                        appData.netstat.sensor.status = YELLOW;
                    }
                }

                break;

            case UNKOWN_LINK_STATE:

                if(appData.netstat.sensor.last_count < appData.netstat.sensor.count){
                    if(appData.netstat.sensor.last_missing == appData.netstat.sensor.missing){
                        appData.netstat.sensor.status = GREEN;
                    }
                }

                else{
                    appData.netstat.sensor.status = YELLOW;
                }

                break;

            } 

        }

        appData.netstat.sensor.last_count = appData.netstat.sensor.count;
        appData.netstat.sensor.last_missing = appData.netstat.sensor.missing;

    }
    void lr_store_netstat(){
        uint8_t last;
        uint8_t curr;
        uint8_t status;
        int diff;

        last = appData.netstat.lead_rover.last_msg_number;
        curr = appData.sysMsg.msg[1];

        if(appData.netstat.lead_rover.init == 1){
            appData.netstat.lead_rover.count++;
            appData.netstat.lead_rover.last_count = appData.netstat.lead_rover.count;
            appData.netstat.lead_rover.init = 0;
            appData.netstat.lead_rover.last_msg_number = curr;
        }

        else {

            diff = (curr - last);

            if(diff == -255){
                diff = 0;
            }
            else if( diff < 0){
                diff += (254);
            }

            else{
                diff -= 1;
            }

            // Update any missing
            appData.netstat.lead_rover.missing += diff;
            // Update msg count
            appData.netstat.lead_rover.count++;
            // Update message number
            appData.netstat.lead_rover.last_msg_number = curr;    

        }
    }
    void fr_store_netstat(){
        uint8_t last;
        uint8_t curr;
        uint8_t status;
        int diff;

        last = appData.netstat.foll_rover.last_msg_number;
        curr = appData.sysMsg.msg[1];

        if(appData.netstat.foll_rover.init == 1){
            appData.netstat.foll_rover.count++;
            appData.netstat.foll_rover.last_count = appData.netstat.foll_rover.count;
            appData.netstat.foll_rover.init = 0;
            appData.netstat.foll_rover.last_msg_number = curr;
        }

        else {

            diff = (curr - last);

            if(diff == -255){
                diff = 0;
            }
            else if( diff < 0){
                diff += (254);
            }

            else{
                diff -= 1;
            }

            // Update any missing
            appData.netstat.foll_rover.missing += diff;
            // Update msg count
            appData.netstat.foll_rover.count++;
            // Update message number
            appData.netstat.foll_rover.last_msg_number = curr;    

        }
    }
    void co_store_netstat(){
        uint8_t last;
        uint8_t curr;
        uint8_t status;
        int diff;

        last = appData.netstat.cordinator.last_msg_number;
        curr = appData.sysMsg.msg[1];

        if(appData.netstat.cordinator.init == 1){
            appData.netstat.cordinator.count++;
            appData.netstat.cordinator.last_count = appData.netstat.cordinator.count;
            appData.netstat.cordinator.init = 0;
            appData.netstat.cordinator.last_msg_number = curr;
        }

        else {

            diff = (curr - last);

            if(diff == -255){
                diff = 0;
            }
            else if( diff < 0){
                diff += (254);
            }

            else{
                diff -= 1;
            }

            // Update any missing
            appData.netstat.cordinator.missing += diff;
            // Update msg count
            appData.netstat.cordinator.count++;
            // Update message number
            appData.netstat.cordinator.last_msg_number = curr;    

        }
    }
    void se_store_netstat(){
      uint8_t last;
        uint8_t curr;
        uint8_t status;
        int diff;

        last = appData.netstat.sensor.last_msg_number;
        curr = appData.sysMsg.msg[1];

        if(appData.netstat.sensor.init == 1){
            appData.netstat.sensor.count++;
            appData.netstat.sensor.last_count = appData.netstat.sensor.count;
            appData.netstat.sensor.init = 0;
            appData.netstat.sensor.last_msg_number = curr;
        }

        else {

            diff = (curr - last);

            if(diff == -255){
                diff = 0;
            }
            else if( diff < 0){
                diff += (254);
            }

            else{
                diff -= 1;
            }

            // Update any missing
            appData.netstat.sensor.missing += diff;
            // Update msg count
            appData.netstat.sensor.count++;
            // Update message number
            appData.netstat.sensor.last_msg_number = curr;    

        }
    }
    void sendHeartbeat(){
       
        
        appData.heartbeat.type = PING;
        appData.heartbeat.size = 0x01;
        appData.heartbeat.msg[0] = PING;
        
        sendOutgoingMsgToWiflyTx(appData.heartbeat);
        
    }
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
    setDebugVal(APP0_INIT_TOP);
    appData.state = APP_STATE_INIT;
    appData.appInternalQ = xQueueCreate(SYS_MSG_Q_SIZE, SYS_MSG_SIZE);
    
    
    
    clearNetStats();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    switch ( appData.state )
    {
        case APP_STATE_INIT:
        {
            setDebugVal(APP0_INIT_STATE);
            appData.state = APP_STATE_ONE;
            PLIB_TMR_Start(TMR_ID_4);
            break;
            
        }
        
        case APP_STATE_ONE:
        {
            setDebugVal(APP0_S_ONE);
            
            if(xQueueReceive(appData.appInternalQ, &appData.sysMsg, portMAX_DELAY)){
                setDebugVal(APP0_SYS_MSG_RECV);
                setDebugVal(appData.sysMsg.type);
                processApp0Message();
            }

            break;
        }

        default:
        {
            setDebugVal(APP0_DEFAULT_STATE);
            break;
        }
    }
    setDebugVal(APP0_TASKS_COMPLETE);
}

void processTimingMessage(){
    
    
    
    
}
/*******************************************************************************
 End of File
 */
