#include <Arduino.h>
#include <arduino_freertos.h>
#include <queue.h>

#include "cpx.h"

#define TX_QUEUE_LENGTH 4
#define RX_QUEUE_LENGTH 4
#define UART_TRANSPORT_MTU 100
#define CPX_ROUTING_PACKED_SIZE (sizeof(CPXRoutingPacked_t))

char strBuf[100];

// Packet queues
static QueueHandle_t sourceQueue;
static QueueHandle_t uartQueue;

// Packets
static CPXRoutablePacket_t txp;
static CPXRoutablePacket_t routingRxBuf;
static CPXRoutablePacket_t routingTxBuf;

static void splitAndSend(const CPXRoutablePacket_t* rxp, CPXRoutablePacket_t* txp, const uint16_t mtu) {
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

        if (txp->dataLength <= mtu) {
            xQueueSend(uartQueue, txp, portMAX_DELAY);
        } else {
            Serial.println("Packet too big to send");
        }

        remainingToSend -= toSend;
        startOfDataToSend += toSend;
    }
}

void route(CPXRoutablePacket_t* rxp, CPXRoutablePacket_t* txp, const char* routerName) {
    while(1) {
        // load packets from source queue
        xQueueReceive(sourceQueue, rxp, portMAX_DELAY);
        rxp->route.lastPacket = true;
        Serial.println("routing...");
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
                    sprintf(strBuf, "ROUTER: %s [0x%02X] -> STM32 [0x%02X] (%u)", routerName, source, destination, cpxDataLength);
                    Serial.println(strBuf);
                    splitAndSend(rxp, txp, UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
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
    route(&routingRxBuf, &routingTxBuf, "TEENSY");
}

static void create_CRTP_packet(void*) {
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

static void blink(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

FLASHMEM __attribute__((noinline))void setup() {
    Serial.begin(9600);

    delay(1000);
    Serial.println("Starting deck test");

    // Init queues
    sourceQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    uartQueue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    // Init router task
    xTaskCreate(router_from_teensy, "Router from Teensy", 5000, NULL, 1, NULL);

    // Init packet creation task
    xTaskCreate(create_CRTP_packet, "Create CRTP packet", 5000, NULL, 1, NULL);

    xTaskCreate(blink, "Blink", 128, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop() {
}