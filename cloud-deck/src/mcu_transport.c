#include "mcu_transport.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "cpx.h"

#define RX_QUEUE_LENGTH 4

static QueueHandle_t sourceQueue;

static CPXRoutablePacket_t txp;

void espTransportReceive(CPXRoutablePacket_t* packet) {
    xQueueReceive(sourceQueue, packet, portMAX_DELAY);
    packet->route.lastPacket = true;
}

static void simulate_crtp_packets(void*) {
    while (1) {
        // Create a crtp packet
        CRTPPacket packet;
        packet.header = CRTP_HEADER(15, 0);
        packet.size = 1;
        packet.data[0] = 0x06;

        // Create a packet to send from the esp32 to the stm32 on the CF for CRTP
        cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CRTP, &txp.route);
        memcpy(txp.data, &packet, sizeof(CRTPPacket));
        txp.dataLength = sizeof(CRTPPacket);

        // send packet
        xQueueSend(sourceQueue, &txp, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void cpx_to_console(void*) {
    while (1) {
        cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CONSOLE, &txp.route);
        memcpy(txp.data, (uint8_t*)"Hello\n", 6);
        txp.dataLength = 6;

        // send packet
        xQueueSend(sourceQueue, &txp, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void espTransportInit(void*) {
    sourceQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    xTaskCreate(cpx_to_console, "Create CRTP packet", 5000, NULL, 1, NULL);

    ESP_LOGI("ESP", "ESP transport initialized!");

    vTaskDelete(NULL);
}