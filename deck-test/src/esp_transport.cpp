#include "esp_transport.h"

static QueueHandle_t sourceQueue;

#define RX_QUEUE_LENGTH 4

static CPXRoutablePacket_t txp;

void espTransportReceive(CPXRoutablePacket_t* packet) {
    xQueueReceive(sourceQueue, packet, portMAX_DELAY);
    packet->route.lastPacket = true;
}

static void simulate_crtp_packets(void*) {
    while (1) {
        // Create a packet to send from the esp32 to the stm32 on the CF for CRTP
        cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CRTP, &txp.route);
        txp.data[0] = 0x06;
        txp.dataLength = 1;

        // send packet
        xQueueSend(sourceQueue, &txp, portMAX_DELAY);
        
        Serial.println("Creating CRTP packet");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void espTransportInit() {
    sourceQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    xTaskCreate(simulate_crtp_packets, "Create CRTP packet", 5000, NULL, 1, NULL);

    Serial.println("ESP transport initialized!");
}