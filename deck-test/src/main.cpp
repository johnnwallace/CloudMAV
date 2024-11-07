#include <Arduino.h>

#include <cpx.h>

#include <arduino_freertos.h>

#include <queue.h>

// Define queue lengths
#define TX_QUEUE_LENGTH 4
#define RX_QUEUE_LENGTH 4

// Packet queues
static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;

// Packets
static CPXRoutablePacket_t txp;
static CPXRoutablePacket_t rxp;


void setup() {
    // Init queues
    txQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    // Create a packet to send from the esp32 to the stm32 on the CF for CRTP
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CRTP, &txp.route);
    txp.data[0] = 0x06;
    txp.dataLength = 1;

    // Send the packet
    xQueueSend(txQueue, &txp, portMAX_DELAY);

    // Wait for a response
    while (1) {
        xQueueReceive(rxQueue, &rxp, portMAX_DELAY);
        printf("Not handing system command 0x%X\n", rxp.data[0]);
    }
}

void loop() {}