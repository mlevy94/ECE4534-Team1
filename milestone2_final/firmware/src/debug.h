#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

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


#define USART0_IH_TX_START      0x01
#define USART0_IH_TX_END        0x02
#define USART0_IH_RX_START      0x03
#define USART0_IH_RX_END        0x04
#define USART0_IH_MASTER_START  0x05
#define USART0_IH_MASTER_END    0x06
#define USART1_IH_TX_START      0x07
#define USART1_IH_TX_END        0x08
#define USART1_IH_RX_START      0x09
#define USART1_IH_RX_END        0x0A
#define USART1_IH_MASTER_START  0x0B
#define USART1_IH_MASTER_END    0x0C
#define ENTER_APP_INIT          0x0D
#define EXIT_APP_INIT           0x0E
#define ENTER_APP_TASKS         0x0F
#define EXIT_APP_TASKS          0x10



#ifdef __cplusplus
}
#endif

#endif

