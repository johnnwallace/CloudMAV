#include "router.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "cpx.h"
#include "esp_transport.h"
#include "uart_transport.h"
#include "wifi.h"

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

char strBuf[100];

typedef void (*Receiver_t)(CPXRoutablePacket_t* packet);
typedef void (*Sender_t)(const CPXRoutablePacket_t* packet);

static CPXRoutablePacket_t routingRxBuf;
static CPXRoutablePacket_t routingTxBuf;

static EventGroupHandle_t startUpEventGroup;

static const int START_UP_ESP_ROUTER_RUNNING = ( 1 << 0 );
static const int START_UP_WIFI_ROUTER_RUNNING = ( 1 << 1 );
static const int START_UP_CRAZYFLIE_ROUTER_RUNNING = ( 1 << 2 );

static void splitAndSend(const CPXRoutablePacket_t* rxp, CPXRoutablePacket_t* txp, Sender_t sender, const uint16_t mtu) {
    txp->route = rxp->route;

    uint16_t remainingToSend = rxp->dataLength;
    const uint8_t* startOfDataToSend = rxp->data;
    while (remainingToSend > 0) {
        uint16_t toSend = remainingToSend;
        bool lastPacket = rxp->route.lastPacket;
        if (toSend > mtu) {
            toSend = mtu;
            lastPacket = false;
        }

        memcpy(txp->data, startOfDataToSend, toSend);
        txp->dataLength = toSend;
        txp->route.lastPacket = lastPacket;
        sender(txp);

        remainingToSend -= toSend;
        startOfDataToSend += toSend;
    }
}

static void route(Receiver_t receive, CPXRoutablePacket_t* rxp, CPXRoutablePacket_t* txp, const char* routerName) {
    while(1) {
        // load packets from source queue
        receive(rxp);
    
        // check cpx version
        // The version should already be checked when we receive packets. Do it again to make sure.
        if(CPX_VERSION == rxp->route.version)
        {
            ESP_LOGI("ROUTER", "Received packet from %s [0x%02X] to [0x%02X]", routerName, rxp->route.source, rxp->route.destination);
            const CPXTarget_t source = rxp->route.source;
            const CPXTarget_t destination = rxp->route.destination;
            const uint16_t cpxDataLength = rxp->dataLength;
            
            // call appropriate sender function based on packet destination using splitAndSend
            switch (destination)
            {
                case CPX_T_GAP8:
                break;
                case CPX_T_STM32:
                ESP_LOGD("ROUTER", "%s [0x%02X] -> STM32 [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
                splitAndSend(rxp, txp, uart_transport_send, UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
                    break;
                case CPX_T_ESP32:
                    break;
                case CPX_T_WIFI_HOST:
                ESP_LOG_BUFFER_HEX("ROUTER", rxp->data, rxp->dataLength);
                splitAndSend(rxp, txp, wifi_transport_send, WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
                    break;
                default:
                    ESP_LOGW("ROUTER", "Cannot route from %s [0x%02X] to [0x%02X]", routerName, source, destination);
            }
        }
    }
}

static void router_from_esp(void*) {
    xEventGroupSetBits(startUpEventGroup, START_UP_ESP_ROUTER_RUNNING);
    route(espTransportReceive, &routingRxBuf, &routingTxBuf, "ESP");
}

static void router_from_wifi(void*) {
    xEventGroupSetBits(startUpEventGroup, START_UP_WIFI_ROUTER_RUNNING);
    route(wifi_transport_receive, &routingRxBuf, &routingTxBuf, "WIFI");
}

static void router_from_crazyflie(void*) {
    xEventGroupSetBits(startUpEventGroup, START_UP_CRAZYFLIE_ROUTER_RUNNING);
    route(uart_transport_receive, &routingRxBuf, &routingTxBuf, "CRAZYFLIE");
}

void router_init(void*) {
    startUpEventGroup = xEventGroupCreate();

    ESP_LOGI("ROUTER", "Waiting to start");
    xTaskCreate(router_from_esp, "Router from ESP", 5000, NULL, 1, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_ESP_ROUTER_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    xTaskCreate(router_from_wifi, "Router from wifi", 5000, NULL, 1, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_WIFI_ROUTER_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    xTaskCreate(router_from_crazyflie, "Router from Crazyflie", 5000, NULL, 1, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_CRAZYFLIE_ROUTER_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI("ROUTER", "Initialized");

    vTaskDelete(NULL);
}