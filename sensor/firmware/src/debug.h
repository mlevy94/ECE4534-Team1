#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

//A,B,C,D,E,F

//system_interrupt.c
#define UART_INTERRUPT          0xF0
#define TRANSMITTING            0xF1
#define RECEIVING               0xF2
#define WRITE_TO_WIFLY          0xF3
#define INIT_TX_Q               0xF4
#define WIFLY_RX_INTH           0xF5
#define WIFLY_RX_INTH_O         0xF6

//app.c
#define APP_INIT                0XA0
#define APP_TASK                0xA1

//pixy_rx.c
#define PIXY_RX_INIT            0xB0
#define PIXY_RX_TASKS           0xB1
#define ADD_TO_PIXY_Q           0xB2
//my pixy debugging, using 16 bits
#define START_OBJECT            0xFEDC
#define POST_PIXY_MESS_TO_Q     0xEE
#define SC                      0xEC
#define CC                      0xCC
#define OBSTACLE_DB             0x0A
#define TOKEN_DB                0x0B
#define LEAD_DB                 0x0C
#define FOLLOWER_ROVER_DB       0x0D
#define SIGNATURE_DB            0x99

//sensor.c
#define ADC_DISTANCE            0xAD

//main.c
#define POST_INIT               0xC0
#define START_MAIN_LOOP         0xC1
#define POST_MAIN_LOOP          0xC2

//wifly_tx.c
#define WIFLY_RX_INIT           0xD0
#define POST_TOGGLE             0xD1

//system_init.c
#define POST_PORTS_INIT         0xE0
#define POST_UART_INIT          0xE1
#define POST_SYS_INT_INIT       0xE2
#define POST_APP_INIT           0xE3
#define POST_PIXY_APP_INIT      0xE4
#define POST_WIFLY_APP_INIT     0xE5
#define POST_TX_Q_INIT          0xE6


#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

void setDebugBool(bool val); // Output to pin 29 on ChipKit Max32
void setDebugVal(char value); // Output to pins 30-37 on ChipKit Max32
void SixteenBitsetDebugVal(int value); // Output to pins ... on ChipKit Max32
void TenBitsetDebugVal(int value); // Output to pins 38-39 on ChipKit Max32
inline void debugFailOn0(char value);
inline void debugFailOnNot0(char value);
void debugFailOnVal(char value, char expected);
void debugFailOnNotVal(char value, char expected);
void clearDebugLED();
void setDebugLED();
void toggleDebugLED();


#ifdef __cplusplus
}
#endif

#endif