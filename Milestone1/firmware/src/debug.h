#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

void setDebugBool(bool val); // Output to pin 29 on ChipKit Max32
void setDebugVal(char value); // Output to pins 30-37 on ChipKit Max32
inline void debugFailOn0(char value);
inline void debugFailOnNot0(char value);
void debugFailOnVal(char value, char expected);
void debugFailOnNotVal(char value, char expected);


#ifdef __cplusplus
}
#endif

#endif

