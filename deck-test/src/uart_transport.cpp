#include "uart_transport.h"

#define TX_QUEUE_LENGTH 4

static QueueHandle_t uartQueue;

void uart_transport_send(const CPXRoutablePacket_t* packet) {
    if (packet->dataLength <= UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE) {
        xQueueSend(uartQueue, packet, portMAX_DELAY);
    } else {
        Serial.println("Packet too big to send");
    }
}

void uartTransportInit() {
    uartQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    Serial.println("UART transport initialized!");
}