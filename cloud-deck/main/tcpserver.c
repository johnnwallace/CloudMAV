#include "tcpserver.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "cpx.h"

/* TODO: Tune this for latency */
#define TCP_TRANSPORT_MTU 1022
#define RX_QUEUE_LENGTH 4

static QueueHandle_t tcpTxQueue;
static QueueHandle_t tcpRxQueue;

static EventGroupHandle_t startUpEventGroup;

static CPXRoutablePacket_t txp;

/* Socket for receiving WiFi connections */
static int serverSock = -1;

static const char *TAG = "TCP";

static const int START_UP_TCP_RUNNING = ( 1 << 0 );

static void close_client_socket()
{
    close(clientConnection);
    clientConnection = NO_CONNECTION;
    xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
}

void wifi_bind_socket() {
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(5000);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    serverSock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (serverSock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGD(TAG, "Socket created");

    int err = bind(serverSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGD(TAG, "Socket binded");

    err = listen(serverSock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
    }
    ESP_LOGD(TAG, "Socket listening");
}

void wifi_wait_for_socket_connected() {
    ESP_LOGI(TAG, "Waiting for connection");
    struct sockaddr sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    clientConnection = accept(serverSock, (struct sockaddr *)&sourceAddr, &addrLen);
    if (clientConnection < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Connection accepted");
    }
}

static void tcp_server_task(void *pvParameters)
{
    wifi_bind_socket();

    while (1)
    {
        wifi_wait_for_socket_connected();
        ESP_LOGI(TAG, "Client connected");
    }
}

void init_tcpserver(void*)
{
    startUpEventGroup = xEventGroupCreate();
    
    tcpTxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));
    tcpRxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

    ESP_LOGI(TAG, "Starting TCP server");

    ESP_LOGI(TAG, "Waiting to start");
    xTaskCreate(tcp_server_task, "tcp_server", 5000, NULL, 5, NULL);
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_TCP_RUNNING,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Initialized");
}

void tcpTransportSend(const CPXRoutablePacket_t* packet) {
    assert(packet->dataLength <= TCP_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(tcpRxQueue, packet, portMAX_DELAY);
}

void tcpTransportReceive(CPXRoutablePacket_t* packet) {
    ESP_LOGI(TAG, "Received packet");
    xQueueReceive(tcpTxQueue, packet, portMAX_DELAY);
    packet->route.lastPacket = true;
}