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
inline void debugFailOn0(char value);
inline void debugFailOnNot0(char value);
void debugFailOnVal(char value, char expected);
void debugFailOnNotVal(char value, char expected);

/** Debug Reference Table
 
 * Enter UART0 INT Handler - A
 * Exit UART0 INT Handler - B
 * 
 * Enter UART0 TX IH Routine - C
 * Exit UART0 TX IH Routine - D
 * 
 * Enter UART0 RX IH Routine - E
 * Exit UART0 RX IH Routine - F
 
 
 */

#ifdef __cplusplus
}
#endif

#endif

