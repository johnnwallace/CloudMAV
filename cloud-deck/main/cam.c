#include "cam.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_camera.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"

#define CONFIG_JPEG_QUALITY 12
#define CAMERA_QUEUE_LENGTH 4

static camera_config_t camera_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk = 10,
    .pin_sccb_sda = 30,
    .pin_sccb_scl = 39,

    .pin_d7 = 48,
    .pin_d6 = 11,
    .pin_d5 = 12,
    .pin_d4 = 14,
    .pin_d3 = 16,
    .pin_d2 = 18,
    .pin_d1 = 17,
    .pin_d0 = 15,
    .pin_vsync = 38,
    .pin_href = 47,
    .pin_pclk = 13,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_SVGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = CONFIG_JPEG_QUALITY, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = CAMERA_QUEUE_LENGTH, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

#define FREQ 30
#define FRAME_CHUNK_SIZE 4096

static const char *TAG = "CAM";

static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_STREAMING_TASK = BIT1;
static EventGroupHandle_t startUpEventGroup;

/* Socket for receiving WiFi connections */
static int sock = -1;
/* Accepted WiFi connection */
static int conn = -1;

static void wifi_bind_socket()
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(8080);
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
  
static void wifi_wait_for_socket_connected()
{
    ESP_LOGI(TAG, "Waiting for connection");
    struct sockaddr sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    conn = accept(sock, (struct sockaddr *)&sourceAddr, (socklen_t *)&addrLen);
    if (conn < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
    }
    ESP_LOGI(TAG, "Connection accepted");
}

static void wifi_close_socket()
{
    close(conn);
    conn = -1;
    xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
}

static void wifi_wait_for_disconnect()
{
    xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
}

static void wifi_send_packet(const char * buffer, size_t size)
{
    if (conn != -1) {
        ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
        int err = send(conn, buffer, size, 0);
        if (err < 0) wifi_close_socket();
    }
  }
  
static void streaming_task(void *pvParameters)
{
    xEventGroupSetBits(startUpEventGroup, START_UP_STREAMING_TASK);
    while(1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Failed to capture frame");
            continue;
        }

        // Send frame size first
        wifi_send_packet((const char *)&fb->len, sizeof(fb->len));
        
        // Send the frame
        int sent = 0;
        int remaining = fb->len;
        
        while (remaining > 0) {
            int bytes_to_send = remaining > FRAME_CHUNK_SIZE ? FRAME_CHUNK_SIZE : remaining; // Send in chunks
            
            // Ensure we never send a 4-byte packet
            if (bytes_to_send == sizeof(fb->len)) {
                uint8_t packet[bytes_to_send + 1];
                memcpy(packet + 1, fb->buf + sent, bytes_to_send);
                wifi_send_packet((const char *)packet, bytes_to_send + 1);
            } else {
                wifi_send_packet((const char *)(fb->buf + sent), bytes_to_send);
            }
            
            sent += bytes_to_send;
            remaining -= bytes_to_send;
        }

        esp_camera_fb_return(fb);
    }
}

void camera_init()
{
    s_wifi_event_group = xEventGroupCreate();
    
    wifi_bind_socket();
    wifi_wait_for_socket_connected();

    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    startUpEventGroup = xEventGroupCreate();
    xEventGroupClearBits(startUpEventGroup, START_UP_STREAMING_TASK);
    xTaskCreate(streaming_task, "Cam WiFi TX", 5000, NULL, 1, NULL);
    ESP_LOGI(TAG, "Waiting for cam wifi streaming to start");
    xEventGroupWaitBits(startUpEventGroup,
                        START_UP_STREAMING_TASK,
                        pdTRUE, // Clear bits before returning
                        pdTRUE, // Wait for all bits
                        portMAX_DELAY);

    ESP_LOGI(TAG, "Streaming initialized");

    vTaskDelete(NULL);
}