#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "uart_tx_charQ.h"
#include "comm.h"
#include <queue.h>

QueueHandle_t txCharQ;

void initializeTXCharQ() {
    txCharQ = xQueueCreate(MAX_MSG_SIZE, 8);
}

void addToTXCharQ(char* val) {
    xQueueSend(txCharQ, val, portMAX_DELAY);
}

BaseType_t addToTXCharQFromISR(char* val) {
    xQueueSendFromISR( txCharQ, val, 0);
}

void getFromTXCharQ(char* val) {
    xQueueReceive( txCharQ, val, 0);
}

void getFromTXCharQFromISR(char* val) {
    xQueueReceiveFromISR( txCharQ, val, 0);
}
