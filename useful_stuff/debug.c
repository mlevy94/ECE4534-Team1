#include "debug.h"

void setDebugBool(bool val) {
    if(val) {
        SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);
    }
    else {
        SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);
    }
}

void setDebugVal(char value) {
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_E, value, 0x00FF);
    
}

inline void debugFailOn0(char value) {
    debugFailOnVal(value, 0);
}

inline void debugFailOnNot0(char value) {
    debugFailOnNotVal(value, 0);
}

void debugFailOnVal(char value, char expected) {
    if (value == expected) {
        setDebugVal(0xFF);
        vTaskEndScheduler();
        taskENTER_CRITICAL();
        taskDISABLE_INTERRUPTS();
        vTaskSuspendAll();
        while(1);
    }
}

void debugFailOnNotVal(char value, char expected) {
    if (value != expected) {
        setDebugVal(0xFF);
        taskENTER_CRITICAL();
        taskDISABLE_INTERRUPTS();
        vTaskSuspendAll();
        while(1);
    }
}