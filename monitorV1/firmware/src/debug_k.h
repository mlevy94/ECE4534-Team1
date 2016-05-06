#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "publicFunctions.h"

#define DEBUG_ON

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

void setDebugBool(bool val); // Output to pin 29 on ChipKit Max32
void setDebugVal(char value); // Output to pins 30-37 on ChipKit Max32
// Output to pins 38-39 on ChipKit Max32 and the same pins as setDebugVal
void TenBitsetDebugVal(int value); 
inline void debugFailOn0(char value);
inline void debugFailOnNot0(char value);
void debugFailOnVal(char value, char expected);
void debugFailOnNotVal(char value, char expected);
void clearDebugLED();
void setDebugLED();
void toggleDebugLED();

#ifdef DEBUG_ON

#define CD 0x99

///////////////////////////////////////////////////////////////////////////////
// Individual debug control
///////////////////////////////////////////////////////////////////////////////
/*#define D_ON 0xFF
#define D_OFF 0x00

#define DEBUG_INTERUPTS     D_OFF
#define DEBUG_APP           D_ON
#define DEBUG_APP1          D_ON
#define DEBUG_PITX          D_OFF
#define DEBUG_PIRX          D_ON
#define DEBUG_WIFLYTX       D_OFF
#define DEBUG_WIFLYRX       D_ON
*/
///////////////////////////////////////////////////////////////////////////////
// main.c
///////////////////////////////////////////////////////////////////////////////
#define POST_INIT               0x0F    // After SYS_Initialize completes
#define START_MAIN_LOOP         0x01    // Before SYS_Tasks() runs
#define POST_MAIN_LOOP          0x0E    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// system_init.c
///////////////////////////////////////////////////////////////////////////////
#define POST_PORT_INIT          0x02    // After ports initialized
#define POST_SYSINT_INIT        0x03    // After SYS Interrupts initialized
#define POST_APP_INIT           0x04    // After all Apps initialized

///////////////////////////////////////////////////////////////////////////////
// app.c
///////////////////////////////////////////////////////////////////////////////
#define APP0_INIT_TOP           0xA0    // Enter app.c APP_Initialize
#define APP0_INIT_STATE         0xA1    // Enter init in FSM
#define APP0_S_ONE              0xA2    // Enter state ONE in FSM
#define APP0_SYS_MSG_RECV       0xA3    // System message recieved
#define APP0_TASKS_COMPLETE     0xAE    // APP_Tasks finished
#define APP0_DEFAULT_STATE      0xAF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// app1.c
///////////////////////////////////////////////////////////////////////////////
#define APP1_INIT_TOP           0xB0    // Enter app1.c APP_Initialize
#define APP1_INIT_STATE         0xB1    // Enter init in FSM
#define APP1_S_ONE              0xB2    // Enter state ONE in FSM
#define APP1_SYS_MSG_RECV       0xB3
#define APP1_PROCESS_MSG        0xB4
#define RUN_ROVER_STATS         0xB5
#define ROVER_FWD_BK            0xB6
#define ROVER_LFT_RGHT          0xB7
#define APP1_TASKS_COMPLETE     0xBE    // APP_Tasks finished
#define APP1_DEFAULT_STATE      0xBF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// wifly_tx.c
///////////////////////////////////////////////////////////////////////////////
#define WIFLYTX_INIT_TOP           0xC0    // Enter wifly_tx.c APP_Initialize
#define WIFLYTX_INIT_STATE         0xC1    // Enter init in FSM
#define WIFLYTX_S_ONE              0xC2    // Enter state ONE in FSM
#define WIFLYTX_TASKS_COMPLETE     0xCE    // APP_Tasks finished
#define WIFLYTX_DEFAULT_STATE      0xCF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// wifly_rx.c
///////////////////////////////////////////////////////////////////////////////
#define WIFLYRX_INIT_TOP           0xD0    // Enter wifly_rx.c APP_Initialize
#define WIFLYRX_INIT_STATE         0xD1    // Enter init in FSM
#define WIFLYRX_S_ONE              0xD2    // Enter state ONE in FSM
#define WIFLYRX_BLD_INCOMING_MSG   0xD4
#define WIFLYRX_PROC_MSG           0xD5
#define WIFLYRX_MSG_IDENTIFIED     0xD6    // Msg routime id'd msg
#define WIFLYRX_BLD_SYS_MSG        0xD7
#define WIFLYRX_TASKS_COMPLETE     0xDE    // APP_Tasks finished
#define WIFLYRX_DEFAULT_STATE      0xDF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// pi_tx.c
///////////////////////////////////////////////////////////////////////////////
#define PITX_INIT_TOP           0xE0    // Enter pi_tx.c APP_Initialize
#define PITX_INIT_STATE         0xE1    // Enter init in FSM
#define PITX_S_ONE              0xE2    // Enter state ONE in FSM
#define PITX_TASKS_COMPLETE     0xEE    // APP_Tasks finished
#define PITX_DEFAULT_STATE      0xEF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// pi_rx.c
///////////////////////////////////////////////////////////////////////////////
#define PIRX_INIT_TOP           0xF0    // Enter pi_rx.c APP_Initialize
#define PIRX_INIT_STATE         0xF1    // Enter init in FSM
#define PIRX_S_ONE              0xF2    // Enter state ONE in FSM
#define PIRX_BLD_INCOMING_MSG   0xF4
#define PIRX_PROC_MSG           0xF5
#define PIRX_MSG_IDENTIFIED     0xF6
#define PIRX_BLD_SYS_MSG        0xF7
#define PIRX_TASKS_COMPLETE     0xFE    // APP_Tasks finished
#define PIRX_DEFAULT_STATE      0xFF    // This should never be called

///////////////////////////////////////////////////////////////////////////////
// system_interrupt.c
///////////////////////////////////////////////////////////////////////////////
#define WIFLY_TX_INTH          0x1A
#define WIFLY_TX_INTH_O        0x1B
#define WIFLY_RX_INTH          0x1C
#define WIFLY_RX_INTH_O        0x1D
#define PI_TX_INTH             0x2A
#define PI_TX_INTH_O           0x2B
#define PI_RX_INTH             0x2C
#define PI_RX_INTH_O           0x2D
#define PI_WRITE_BYTE           0x3A    // Write byte to piOutgoingByteQ
#define WIFLY_WRITE_BYTE        0x3B    // Write byte to wiflyOutgoingByteQ
#define PI_OUTGOING_BYTE        0x3C    // Preceeds byte to be sent
#define WIFLY_OUTGOING_BYTE     0x3D    // Preceeds byte to be sent

///////////////////////////////////////////////////////////////////////////////
// uart debug queues
///////////////////////////////////////////////////////////////////////////////
#define WAITING_ON_PI_Q_CREATE          0x4A
#define WAITING_ON_WIFLY_Q_CREATE       0x4B
#define DONE_BUILDING_USART_QUEUES      0x4C


///////////////////////////////////////////////////////////////////////////////
// Custom
///////////////////////////////////////////////////////////////////////////////



#endif

#ifdef __cplusplus
}
#endif

#endif

