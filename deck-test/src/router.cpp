#include "router.h"

#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

char strBuf[100];

typedef void (*Receiver_t)(CPXRoutablePacket_t* packet);
typedef void (*Sender_t)(const CPXRoutablePacket_t* packet);

static CPXRoutablePacket_t routingRxBuf;
static CPXRoutablePacket_t routingTxBuf;

static EventGroupHandle_t startUpEventGroup;

static const int START_UP_TEENSY_ROUTER_RUNNING = ( 1 << 0 );

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
            const CPXTarget_t source = rxp->route.source;
            const CPXTarget_t destination = rxp->route.destination;
            const uint16_t cpxDataLength = rxp->dataLength;

            // call appropriate sender function based on packet destination using splitAndSend
            switch (destination)
            {
                case CPX_T_GAP8:
                    break;
                case CPX_T_STM32:
                    splitAndSend(rxp, txp, uart_transport_send, UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
                    break;
                case CPX_T_ESP32:
                    break;
                case CPX_T_WIFI_HOST:
                    break;
                default:
                    sprintf(strBuf, "ROUTER: Cannot route from %s [0x%02X] to [0x%02X]", routerName, source, destination);
                    Serial.println(strBuf);
            }
        }
    }
}

static void router_from_teensy(void*) {
    xEventGroupSetBits(startUpEventGroup, START_UP_TEENSY_ROUTER_RUNNING);
    route(espTransportReceive, &routingRxBuf, &routingTxBuf, "TEENSY");
}

void router_init(void*) {
    startUpEventGroup = xEventGroupCreate();

    xTaskCreate(router_from_teensy, "Router from Teensy", 5000, NULL, 1, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_TEENSY_ROUTER_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    Serial.println("Router initialized!");

    vTaskDelete(NULL);
}