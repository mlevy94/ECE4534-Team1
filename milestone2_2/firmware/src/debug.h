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

#ifdef DEBUG_ON

// start init
#define SYS_INIT_START          0x01 
// driver init
#define SYS_INIT_ADC            0x02 
#define SYS_INIT_UART           0x03
#define SYS_INIT_I2C            0x04
#define SYS_INIT_SPI            0x05
// system init
#define SYS_INIT_SYS            0x06
// tx buffer init
#define SYS_INIT_TX_BUF         0x07
// app init
#define SYS_INIT_APP            0x08
#define SYS_INIT_UART_TX_APP    0x09
#define SYS_INIT_APP_RESERVED2  0x0A
#define SYS_INIT_APP_RESERVED3  0x0B
#define SYS_INIT_APP_RESERVED4  0x0C
// interrupts
#define INT_UART0_START         0x0D
#define INT_UART0_END           0x0E
#define INT_UART0_TX            0x0F
#define INT_UART0_RX            0x10
#define INT_ADC0_START          0x11
#define INT_ADC0_END            0x12
#define INT_START_RESERVED1     0x13
#define INT_END_RESERVED1       0x14
#define INT_START_RESERVED2     0x15
#define INT_END_RESERVED2       0x16
#define INT_START_RESERVED3     0x17
#define INT_END_RESERVED3       0x18
#define INT_START_RESERVED4     0x19
#define INT_END_RESERVED4       0x1A
// app task loops
#define TASK_APP                0x1B
#define TASK_UART_TX_APP        0x1C
#define TASK_UART_RX_APP        0x1D
#define TASK_APP_RESERVED1      0x1E
#define TASK_APP_RESERVED2      0x1F
#define TASK_APP_RESERVED3      0x20
#define TASK_APP_RESERVED4      0x21


#endif

#ifdef __cplusplus
}
#endif

#endif

