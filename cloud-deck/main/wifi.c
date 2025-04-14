#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"

#include "esp_transport.h"

// static CPXRoutablePacket_t rxp;
static CPXRoutablePacket_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE] = "sensors_2G";
static char key[MAX_SSID_SIZE] = "onthisnetwork";
// static char ssid[MAX_SSID_SIZE] = "servicenet";
// static char key[MAX_SSID_SIZE] = "";

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_MAIN_TASK = BIT0;
static const int START_UP_RX_TASK = BIT1;
static const int START_UP_TX_TASK = BIT2;
static const int START_UP_CTRL_TASK = BIT3;
static EventGroupHandle_t startUpEventGroup;

#define WIFI_HOST_QUEUE_LENGTH (2)

static QueueHandle_t wifiRxQueue;
static QueueHandle_t wifiTxQueue;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int sock = -1;
/* Accepted WiFi connection */
static int conn = -1;

enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
};

/* WiFi event handler */
static void event_handler(void* handlerArg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
{
  if (eventBase == WIFI_EVENT) {
    switch(eventId) {
      case WIFI_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG,"Disconnected from access point");
        break;
      default:
        // Fall through
        break;
    }
  }

  // Send IP info to CF (necessary?)
  if (eventBase == IP_EVENT) {
    switch (eventId) {
      case IP_EVENT_STA_GOT_IP:
        {
          ip_event_got_ip_t* event = (ip_event_got_ip_t*)eventData;
          ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

          wifi_ap_record_t ap_info;
          ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
          ESP_LOGD(TAG, "BSAP MAC is %x:%x:%x:%x:%x:%x",
              ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
              ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
          ESP_LOGI(TAG, "country: %s", ap_info.country.cc);
          ESP_LOGI(TAG, "rssi: %d", ap_info.rssi);
          ESP_LOGI(TAG, "11b: %d, 11g: %d, 11n: %d, lr: %d",
            ap_info.phy_11b, ap_info.phy_11g, ap_info.phy_11n, ap_info.phy_lr);

          gpio_set_level(21, 0);

          cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_WIFI_CTRL, &txp.route);
          txp.data[0] = WIFI_CTRL_STATUS_WIFI_CONNECTED;
          memcpy(&txp.data[1], &event->ip_info.ip.addr, sizeof(uint32_t));
          txp.dataLength = 1 + sizeof(uint32_t);

          // TODO: We should probably not block here...
          espAppSendToRouterBlocking(&txp);

          xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        break;
      default:
        // Fall through
        break;
    }
  }
}

static void wifi_init_sta(const char * ssid, const char * key)
{
  esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  ESP_LOGD(TAG, "SSID is %u chars", strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));
  ESP_LOGD(TAG, "KEY is %u chars", strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

}

static void wifi_bind_socket() {
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(8000);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket created");

  int keepalive = 1;
  int keepidle = 5;
  int keepintvl = 5;
  int keepcnt = 3;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));

  int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket binded");

  err = listen(sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket listening");
}

static void wifi_wait_for_socket_connected() {
  ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  uint addrLen = sizeof(sourceAddr);
  conn = accept(sock, (struct sockaddr *)&sourceAddr, (socklen_t *)&addrLen);
  if (conn < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
  }
  ESP_LOGI(TAG, "Connection accepted");
}

static void wifi_wait_for_disconnect() {
  xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
}

static void wifi_task(void *pvParameters) {

  s_wifi_event_group = xEventGroupCreate();

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
  ESP_LOGD(TAG, "AP MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
  ESP_LOGD(TAG, "STA MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wifi_bind_socket();

  xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);
  while (1) {
    //blink_period_ms = 500;
    wifi_wait_for_socket_connected();
    ESP_LOGI(TAG, "Client connected");

    //blink_period_ms = 100;

    // Not thread safe!
    // Tell CF we are connected
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 1;
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    // Probably not the best, should be handled in some other way?
    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");

    // Not thread safe!
    // Tell CF we are connected
    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 0;
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);
  }
}

void wifi_send_packet(const char * buffer, size_t size) {
  if (conn != -1) {
    ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
    int err = send(conn, buffer, size, 0);
    if (err < 0) {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      conn = -1;
      xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
    }
  }
}

static void wifi_sending_task(void *pvParameters) {
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;

  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1) {
    xQueueReceive(wifiTxQueue, &qPacket, portMAX_DELAY);

    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;

    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);

    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);

    wifi_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_receiving_task(void *pvParameters) {
  static WifiTransportPacket_t rxp_wifi;
  int len;

  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1) {
    // ESP_LOGI(TAG, "Waiting for data");

    len = recv(conn, &rxp_wifi, 2, 0);
    // ESP_LOGI(TAG, "Start byte read.");
    if (len > 0) {
      ESP_LOGI(TAG, "Wire data length %i", rxp_wifi.payloadLength);
      int totalRxLen = 0;
      do {
        ESP_LOGI(TAG, "Reading b");
        len = recv(conn, &rxp_wifi.payload[totalRxLen], rxp_wifi.payloadLength - totalRxLen, 0);
        totalRxLen += len;
        ESP_LOGI(TAG, "Read %i bytes", len);
      } while (totalRxLen < rxp_wifi.payloadLength);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp_wifi, 10, ESP_LOG_DEBUG);
      xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
    } else if (len == 0) {
      //vTaskDelay(10);
      xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
      //printf("No data!\n");
    } else {
      // ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
      xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
      vTaskDelay(10);
    }
  }
}

void wifi_transport_send(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(wifiTxQueue, packet, portMAX_DELAY);
}

void wifi_transport_receive(CPXRoutablePacket_t* packet) {
  // Not reentrant safe. Assuming only one task is popping the queue
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

  packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;

  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);

  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_init() {
  // Configure the GPIO pin
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << 21),  // Select the GPIO pin
    .mode = GPIO_MODE_OUTPUT,           // Set as output
    .pull_up_en = GPIO_PULLUP_DISABLE,  // Disable pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE,     // Disable interrupts
  };

  // Apply the configuration and turn off LED
  gpio_config(&io_conf);
  gpio_set_level(21, 1);

  esp_netif_init();

  s_wifi_event_group = xEventGroupCreate();

  wifiRxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(WifiTransportPacket_t));
  wifiTxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK | START_UP_CTRL_TASK);
  xTaskCreate(wifi_task, "WiFi TASK", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_sending_task, "WiFi TX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_receiving_task, "WiFi RX", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for main, RX and TX tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  wifi_init_sta(ssid, key);

  ESP_LOGI(TAG, "Wifi initialized");

  vTaskDelete(NULL);
}
