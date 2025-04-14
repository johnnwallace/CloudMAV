#include "uart_transport.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

// Length of start + payloadLength
#define UART_HEADER_LENGTH 2
#define UART_CRC_LENGTH 1
#define UART_META_LENGTH (UART_HEADER_LENGTH + UART_CRC_LENGTH)

typedef struct {
    CPXRoutingPacked_t route;
    uint8_t data[UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE];
} __attribute__((packed)) uartTransportPayload_t;

typedef struct {
    uint8_t start;
    uint8_t payloadLength; // Excluding start and crc
    union {
        uartTransportPayload_t routablePayload;
        uint8_t payload[UART_TRANSPORT_MTU];
    };

    uint8_t crcPlaceHolder; // Not actual position. CRC is added after the last byte of payload
} __attribute__((packed)) uart_transport_packet_t;



#define TX_QUEUE_LENGTH 10
#define RX_QUEUE_LENGTH 10

#define DEBUG(...) printf(__VA_ARGS__)
//#define DEBUG(...)


static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;

static CPXRoutablePacket_t qPacket;
static uart_transport_packet_t txp;
static uart_transport_packet_t rxp;

static EventGroupHandle_t evGroup;
static EventGroupHandle_t startUpEventGroup;

#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)

#define CTS_EVENT (1<<0)
#define CTR_EVENT (1<<1)
#define TXQ_EVENT (1<<2)

#define START_UP_RX_RUNNING (1<<0)
#define START_UP_TX_RUNNING (1<<1)

static uint8_t calcCrc(const uart_transport_packet_t* packet) {
  const uint8_t* start = (const uint8_t*) packet;
  const uint8_t* end = &packet->payload[packet->payloadLength];

  uint8_t crc = 0;
  for (const uint8_t* p = start; p < end; p++) {
    crc ^= *p;
  }

  return crc;
}

static void uart_tx_task(void* _param) {
  uint8_t ctr[] = {0xFF, 0x00};
  EventBits_t evBits = 0;

  // Note: RX task must be running before we start the TX task
  xEventGroupSetBits(startUpEventGroup, START_UP_TX_RUNNING);

  do {
    uart_write_bytes(UART_NUM_0, &ctr, sizeof(ctr));
    vTaskDelay(100);
    evBits = xEventGroupGetBits(evGroup);
  } while ((evBits & CTS_EVENT) != CTS_EVENT);

  while(1) {
    // If we have nothing to send then wait, either for something to be
    // queued or for a request to send CTR
    if (uxQueueMessagesWaiting(tx_queue) == 0) {
      // ESP_LOGI("UART", "Waiting for CTR/TXQ");
      evBits = xEventGroupWaitBits(evGroup,
                                CTR_EVENT | TXQ_EVENT,
                                pdTRUE, // Clear bits before returning
                                pdFALSE, // Wait for any bit
                                portMAX_DELAY);
      if ((evBits & CTR_EVENT) == CTR_EVENT) {
        // ESP_LOGI("UART", "Sent CTR");
        uart_write_bytes(UART_NUM_0, &ctr, sizeof(ctr));
      }
    }

    if (uxQueueMessagesWaiting(tx_queue) > 0) {
      // Dequeue and wait for either CTS or CTR
      xQueueReceive(tx_queue, &qPacket, 0);
      txp.start = 0xFF;
      txp.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
      cpxRouteToPacked(&qPacket.route, &txp.routablePayload.route);
      memcpy(txp.routablePayload.data, qPacket.data, txp.payloadLength);
      txp.payload[txp.payloadLength] = calcCrc(&txp);

      do {
        ESP_LOGI("UART", "Waiting for CTR/CTS");
        evBits = xEventGroupWaitBits(evGroup,
                                CTR_EVENT | CTS_EVENT,
                                pdTRUE, // Clear bits before returning
                                pdFALSE, // Wait for any bit
                                portMAX_DELAY);
        if ((evBits & CTR_EVENT) == CTR_EVENT) {
          // ESP_LOGI("UART", "Sent CTR");
          uart_write_bytes(UART_NUM_0, &ctr, sizeof(ctr));
        }
      } while ((evBits & CTS_EVENT) != CTS_EVENT);
      ESP_LOGI("UART", "Sending packet");
      ESP_LOG_BUFFER_HEX("UART", &txp, txp.payloadLength + UART_META_LENGTH);
      uart_write_bytes(UART_NUM_0, &txp, txp.payloadLength + UART_META_LENGTH);
    }
  }
}


static void uart_rx_task(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_RX_RUNNING);

  while(1) {
    do {
      // ESP_LOGI("UART", "Waiting for start byte.");
      uart_read_bytes(UART_NUM_0, &rxp.start, 1, portMAX_DELAY);
    } while (rxp.start != 0xFF);

    uart_read_bytes(UART_NUM_0, &rxp.payloadLength, 1, portMAX_DELAY);

    if (rxp.payloadLength == 0) {
      ESP_LOGI("UART", "Received CTS");
      xEventGroupSetBits(evGroup, CTS_EVENT);
    } else {
      uart_read_bytes(UART_NUM_0, rxp.payload, rxp.payloadLength + UART_CRC_LENGTH, portMAX_DELAY);
      assert (rxp.payload[rxp.payloadLength] == calcCrc(&rxp));

      // ESP_LOGI("UART", "Received packet");
      // Post on RX queue and send flow control
      // Optimize a bit here
      if (uxQueueSpacesAvailable(rx_queue) > 0) {
        xEventGroupSetBits(evGroup, CTR_EVENT);
        xQueueSend(rx_queue, &rxp, portMAX_DELAY);
      } else {
        xQueueSend(rx_queue, &rxp, portMAX_DELAY);
        xEventGroupSetBits(evGroup, CTR_EVENT);
      }
    }
  }
}

void uart_transport_init() {
    // Setting up synchronization items
    tx_queue = xQueueCreate(TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(uart_transport_packet_t));

    evGroup = xEventGroupCreate();

    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, UART_TRANSPORT_MTU * 2, UART_TRANSPORT_MTU * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Launching communication tasks
    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_RX_RUNNING | START_UP_TX_RUNNING);
    xTaskCreate(uart_rx_task, "UART RX transport", 8192, NULL, 1, NULL);
    ESP_LOGI("UART", "Waiting for RX task to start");
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_RX_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    // We need to hold off here to make sure that the RX task
    // has started up and is waiting for chars before he TX task is started, otherwise we might send
    // CTR and miss CTS (which means that the STM32 will stop sending CTS
    // too early and we cannot sync)

    xTaskCreate(uart_tx_task, "UART TX transport", 8192, NULL, 1, NULL);
    ESP_LOGI("UART", "Waiting for TX task to start");
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_TX_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI("UART", "Transport initialized");

    vTaskDelete(NULL);
}

void uart_transport_send(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);

  xQueueSend(tx_queue, packet, portMAX_DELAY);

  xEventGroupSetBits(evGroup, TXQ_EVENT);
}

void uart_transport_receive(CPXRoutablePacket_t* packet) {
  // Not reentrant safe. Assume only one task dequeues packets
  static uart_transport_packet_t rxp;

  xQueueReceive(rx_queue, &rxp, portMAX_DELAY);

  packet->dataLength = rxp.payloadLength - CPX_ROUTING_PACKED_SIZE;

  cpxPackedToRoute(&rxp.routablePayload.route, &packet->route);

  memcpy(packet->data, rxp.routablePayload.data, packet->dataLength);
}
