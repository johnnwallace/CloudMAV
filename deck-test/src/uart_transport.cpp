/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

// SPI Transport implementation

#include "uart_transport.h"

#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include <arduino_freertos.h>
#include <queue.h>
#include <event_groups.h>

// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

typedef struct
{
    CPXRoutingPacked_t route;
    uint8_t data[UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct
{
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union
    {
        uartTransportPayload_t routablePayload;
        uint8_t payload[UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;

#define TX_QUEUE_LENGTH 10
#define RX_QUEUE_LENGTH 10

static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;

static CPXRoutablePacket_t qPacket;
static uart_transport_packet_t txp;
static uart_transport_packet_t rxp;

static uint8_t uart_tx_buf[UART_TRANSPORT_MTU * 2];
static uint8_t uart_rx_buf[UART_TRANSPORT_MTU * 2];

static EventGroupHandle_t evGroup;
static EventGroupHandle_t startUpEventGroup;

#define TXD_PIN (GPIO_NUM_1) // Nina 22 => 1
#define RXD_PIN (GPIO_NUM_3) // Nina 23 => 3

#define CTS_EVENT (1 << 0)
#define CTR_EVENT (1 << 1)
#define TXQ_EVENT (1 << 2)

#define START_UP_RX_RUNNING (1 << 0)
#define START_UP_TX_RUNNING (1 << 1)

#define HWSERIAL Serial1

static uint8_t calcCrc(const uart_transport_packet_t *packet)
{
    const uint8_t *start = (const uint8_t *)packet;
    const uint8_t *end = &packet->payload[packet->payloadLength];

    uint8_t crc = 0;
    for (const uint8_t *p = start; p < end; p++)
    {
        crc ^= *p;
    }

    return crc;
}

static void uart_tx_task(void *_param)
{
    uint8_t ctr[] = {0xFF, 0x00};
    EventBits_t evBits = 0;

    // Note: RX task must be running before we start the TX task
    xEventGroupSetBits(startUpEventGroup, START_UP_TX_RUNNING);

    do
    {
        HWSERIAL.write(ctr, sizeof(ctr));
        vTaskDelay(10);
        evBits = xEventGroupGetBits(evGroup);
    } while ((evBits & CTS_EVENT) != CTS_EVENT);

    while (1)
    {
        // If we have nothing to send then wait, either for something to be
        // queued or for a request to send CTR
        if (uxQueueMessagesWaiting(tx_queue) == 0)
        {
            evBits = xEventGroupWaitBits(evGroup,
                                         CTR_EVENT | TXQ_EVENT,
                                         pdTRUE,  // Clear bits before returning
                                         pdFALSE, // Wait for any bit
                                         portMAX_DELAY);
            if ((evBits & CTR_EVENT) == CTR_EVENT)
            {
                HWSERIAL.write(ctr, sizeof(ctr));
            }
        }

        if (uxQueueMessagesWaiting(tx_queue) > 0)
        {
            // Dequeue and wait for either CTS or CTR
            xQueueReceive(tx_queue, &qPacket, 0);
            txp.start = 0xFF;
            txp.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
            cpxRouteToPacked(&qPacket.route, &txp.routablePayload.route);
            memcpy(txp.routablePayload.data, qPacket.data, txp.payloadLength);
            txp.payload[txp.payloadLength] = calcCrc(&txp);

            do
            {
                evBits = xEventGroupWaitBits(evGroup,
                                             CTR_EVENT | CTS_EVENT,
                                             pdTRUE,  // Clear bits before returning
                                             pdFALSE, // Wait for any bit
                                             portMAX_DELAY);
                if ((evBits & CTR_EVENT) == CTR_EVENT)
                {
                    HWSERIAL.write(ctr, sizeof(ctr));
                }
            } while ((evBits & CTS_EVENT) != CTS_EVENT);
            HWSERIAL.write((const uint8_t *)&txp, txp.payloadLength + UART_META_LENGTH);
            Serial.println("UART packet sent.");
        }
    }
}

static void uart_rx_task(void *_param)
{
    xEventGroupSetBits(startUpEventGroup, START_UP_RX_RUNNING);

    while (1)
    {
        do
        {
            while(!HWSERIAL.available()) { vTaskDelay(1); }
            rxp.start = HWSERIAL.read();
        } while (rxp.start != 0xFF);

        while(!HWSERIAL.available()) { vTaskDelay(1); } // necessary?
        rxp.payloadLength = HWSERIAL.read();

        if (rxp.payloadLength == 0)
        {
            xEventGroupSetBits(evGroup, CTS_EVENT);
        }
        else
        {
            while(!HWSERIAL.available()) { vTaskDelay(1); } // necessary?
            HWSERIAL.readBytes(rxp.payload, rxp.payloadLength + UART_CRC_LENGTH);
            assert(rxp.payload[rxp.payloadLength] == calcCrc(&rxp));

            // Post on RX queue and send flow control
            // Optimize a bit here
            if (uxQueueSpacesAvailable(rx_queue) > 0)
            {
                xEventGroupSetBits(evGroup, CTR_EVENT);
                xQueueSend(rx_queue, &rxp, portMAX_DELAY);
            }
            else
            {
                xQueueSend(rx_queue, &rxp, portMAX_DELAY);
                xEventGroupSetBits(evGroup, CTR_EVENT);
            }
        }
    }
}

static void simple_uart_tx_task(void *_param)
{
    while (1) {
        if (uxQueueMessagesWaiting(tx_queue) > 0)
        {
            // Dequeue and wait for either CTS or CTR
            xQueueReceive(tx_queue, &qPacket, 0);
            txp.start = 0xFF;
            txp.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
            cpxRouteToPacked(&qPacket.route, &txp.routablePayload.route);
            memcpy(txp.routablePayload.data, qPacket.data, txp.payloadLength);
            txp.payload[txp.payloadLength] = calcCrc(&txp);
            HWSERIAL.write((const uint8_t *)&txp, txp.payloadLength + UART_META_LENGTH);
            Serial.println("UART packet sent.");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void uartTransportInit()
{
    // // Setting up synchronization items
    tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    // rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

    // evGroup = xEventGroupCreate();

    // // Init hardware serial
    HWSERIAL.begin(57600, SERIAL_8N1);
    HWSERIAL.addMemoryForRead(uart_rx_buf, UART_TRANSPORT_MTU * 2);
    HWSERIAL.addMemoryForWrite(uart_tx_buf, UART_TRANSPORT_MTU * 2);

    // // Launching communication tasks
    // startUpEventGroup = xEventGroupCreate();
    // xEventGroupClearBits(startUpEventGroup, START_UP_RX_RUNNING | START_UP_TX_RUNNING);
    // xTaskCreate(uart_rx_task, "UART RX transport", 5000, NULL, 1, NULL);
    // xEventGroupWaitBits(startUpEventGroup,
    //                     START_UP_RX_RUNNING,
    //                     pdTRUE, // Clear bits before returning
    //                     pdTRUE, // Wait for all bits
    //                     portMAX_DELAY);

    // // We need to hold off here to make sure that the RX task
    // // has started up and is waiting for chars before he TX task is started, otherwise we might send
    // // CTR and miss CTS (which means that the STM32 will stop sending CTS
    // // too early and we cannot sync)

    // xTaskCreate(uart_tx_task, "UART TX transport", 5000, NULL, 1, NULL);
    // xEventGroupWaitBits(startUpEventGroup,
    //                     START_UP_TX_RUNNING,
    //                     pdTRUE, // Clear bits before returning
    //                     pdTRUE, // Wait for all bits
    //                     portMAX_DELAY);

    xTaskCreate(simple_uart_tx_task, "Simple UART RX transport", 5000, NULL, 1, NULL);

    Serial.println("UART transport initialized!");
}

void uart_transport_send(const CPXRoutablePacket_t *packet, char* strBuf)
{
    assert(packet->dataLength <= UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

    xQueueSend(tx_queue, packet, portMAX_DELAY);
    sprintf(strBuf, "UART tx queue size: %u", uxQueueMessagesWaiting(tx_queue));

    // xEventGroupSetBits(evGroup, TXQ_EVENT);
}

void uart_transport_receive(CPXRoutablePacket_t *packet)
{
    // Not reentrant safe. Assume only one task dequeues packets
    static uart_transport_packet_t rxp;

    xQueueReceive(rx_queue, &rxp, portMAX_DELAY);

    packet->dataLength = rxp.payloadLength - CPX_ROUTING_PACKED_SIZE;

    cpxPackedToRoute(&rxp.routablePayload.route, &packet->route);

    memcpy(packet->data, rxp.routablePayload.data, packet->dataLength);
}
