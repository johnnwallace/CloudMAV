#include "esp_transport.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "crtp.h"

#define RX_QUEUE_LENGTH 4

static QueueHandle_t espTxQueue;
static QueueHandle_t espRxQueue;

static CPXRoutablePacket_t txp;

static void cpx_to_console(void*) {
    while (1) {
        cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CONSOLE, &txp.route);
        memcpy(txp.data, (uint8_t*)"Hello\n", 7);
        txp.dataLength = 7;

        // send packet
        xQueueSend(espTxQueue, &txp, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void send_crtp(void*) {
    while (1) {
        // create CRTP packet
        static CRTPPacket packet;
        memcpy(packet.data, (uint8_t*)"Hello\n", 7);
        packet.size = 7;
        packet.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);

        cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CRTP, &txp.route);
        memcpy(txp.data, packet.raw, packet.size + 1);
        txp.dataLength = packet.size + 1;

        // send packet
        xQueueSend(espTxQueue, &txp, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void espAppSendToRouterBlocking(const CPXRoutablePacket_t* packet) {
    xQueueSend(espTxQueue, packet, portMAX_DELAY);
}

void espAppReceiveFromRouter(CPXRoutablePacket_t* packet) {
    CPXRoutablePacket_t* buf = packet;
    xQueueReceive(espRxQueue, buf, portMAX_DELAY);
}

void espTransportSend(const CPXRoutablePacket_t* packet) {
    assert(packet->dataLength <= ESP_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(espRxQueue, packet, portMAX_DELAY);
}

void espTransportReceive(CPXRoutablePacket_t* packet) {
    xQueueReceive(espTxQueue, packet, portMAX_DELAY);
    packet->route.lastPacket = true;
}

void espTransportInit(void*) {
    ESP_LOGI("ESP", "Starting ESP transport.");
    
    espTxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    espRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    // xTaskCreate(send_crtp, "Create CRTP packet", 5000, NULL, 1, NULL);
    // xTaskCreate(cpx_to_console, "Send to console", 5000, NULL, 1, NULL);

    ESP_LOGI("ESP", "ESP transport initialized!");

    vTaskDelete(NULL);
}