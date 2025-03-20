#include "websocket.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"

#include "esp_camera.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"

#include <esp_http_server.h>

static const char *TAG = "WEBSOCKET";

static QueueHandle_t wsTxQueue;

httpd_handle_t server;

// Global variable to store the WebSocket client fd
static int ws_client_fd = -1;
static SemaphoreHandle_t ws_client_lock = NULL; // For thread safety

static void (*connection_callback)(bool connected) = NULL;

static esp_err_t ws_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "URI %s", req->uri);

    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        xSemaphoreTake(ws_client_lock, portMAX_DELAY);

        // If we already have a client, close the old connection
        if (ws_client_fd != -1) {
            ESP_LOGI(TAG, "New client connecting, closing old connection (fd=%d)", ws_client_fd);
            httpd_sess_trigger_close(req->handle, ws_client_fd);
        }
        
        // Store the new client fd
        ws_client_fd = fd;
        ESP_LOGI(TAG, "Handshake done, new connection established (fd=%d)", ws_client_fd);
        xSemaphoreGive(ws_client_lock);

        if (connection_callback) {
            connection_callback(true);
        }

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    
    if (ws_pkt.len) {
        /* Allocate memory for the message */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory");
            return ESP_ERR_NO_MEM;
        }
        
        /* Set the frame data length to the allocated buffer length */
        ws_pkt.payload = buf;
        
        /* Receive the frame data */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        
        ESP_LOGI(TAG, "Received packet with message: %s", ws_pkt.payload);
        
        /* Echo the received message back to the client */
        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
        }
        
        free(buf);
    }

    return ESP_OK;
}

static const httpd_uri_t ws = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true,
    .handle_ws_control_frames = true
};

static httpd_handle_t start_websocket(void)
{
    server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static esp_err_t stop_websocket(void)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{   
    if (server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_websocket() == ESP_OK) {
            server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if (server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        server = start_websocket();
    }
}

void ws_register_connection_callback(void (*callback)(bool connected)) {
    connection_callback = callback;
}

static void ws_tx_task(void *pvParameters)
{
    static camera_fb_t* im;
    // xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
    while (1) {
        if (server) { // ensure this task only runs when we have a connection            
            xQueueReceive(wsTxQueue, &im, portMAX_DELAY);

            httpd_ws_frame_t ws_pkt;
            ws_pkt.type = HTTPD_WS_TYPE_BINARY;
            ws_pkt.len = im->len;
            ws_pkt.payload = im->buf;

            esp_err_t ret = httpd_ws_send_frame_async(server, ws_client_fd, &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send image: %d", ret);
            }

            esp_camera_fb_return(im);
        }
    }
}

void ws_send_image(const camera_fb_t* im)
{
    ESP_LOGI(TAG, "Sending image to tx queue");
    xQueueSend(wsTxQueue, &im, portMAX_DELAY);
}

void socket_init()
{    
    wsTxQueue = xQueueCreate(2, sizeof(camera_fb_t*));

    ws_client_lock = xSemaphoreCreateMutex();
    if (ws_client_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create WebSocket client mutex!");
    }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, NULL));

    server = start_websocket();

    vTaskDelete(NULL);
}