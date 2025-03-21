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

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <esp_http_server.h>

static const char *TAG = "WEBSOCKET";

static QueueHandle_t wsTxQueue;

httpd_handle_t server;

// Global variable to store the WebSocket client fd
static int ws_client_fd = -1;
static SemaphoreHandle_t ws_client_lock = NULL; // For thread safety

EventGroupHandle_t wsConnectionEventGroup;

#define CHUNK_SIZE 512

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

        int flag = 1;
        esp_err_t ret = setsockopt(ws_client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to set TCP_NODELAY: %d", ret);
        }

        xSemaphoreGive(ws_client_lock);

        xEventGroupSetBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE);
        return ESP_OK;
    }

    // // Just log frame info, then let ESP-IDF handle it automatically
    // httpd_ws_frame_t ws_pkt;
    // memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
    // esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
    //     return ret;
    // }
    
    // ESP_LOGI(TAG, "Received frame: type=%d, len=%d", ws_pkt.type, ws_pkt.len);
    return ESP_OK;
}

static const httpd_uri_t ws = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
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
            xEventGroupClearBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE);
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

static void handleConnectionFailure()
{
    xSemaphoreTake(ws_client_lock, portMAX_DELAY);
    if (ws_client_fd != -1) {
        ws_client_fd = -1;
        disconnect_handler(NULL, NULL, 0, NULL);
    }
    xSemaphoreGive(ws_client_lock);
    
    xEventGroupClearBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE);
}

static void handleTemporaryError()
{
    static int error_count = 0;
    ESP_LOGI(TAG, "Handling temporary error");
    
    // Implement exponential backoff
    error_count++;
    if (error_count > 5) {
        // Too many errors, treat as connection failure
        handleConnectionFailure();
        error_count = 0;
    } else {
        // Exponential backoff - pause before next frame
        vTaskDelay(pdMS_TO_TICKS(50 * (1 << error_count)));
    }
}

static void ws_tx_task(void *pvParameters)
{
    static camera_fb_t* im;
    uint8_t *data_ptr;
    size_t remaining_len;
    size_t chunk_idx;

    // xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
    while (1) {
        if (server && ws_client_fd != -1) { // ensure this task only runs when we have a connection            
            xQueueReceive(wsTxQueue, &im, portMAX_DELAY);

            data_ptr = im->buf;
            remaining_len = im->len;
            bool is_first = true;
            chunk_idx = 0;
            
            while (remaining_len > 0 && server && ws_client_fd != -1) {
                httpd_ws_frame_t ws_pkt = {0};
                
                // Set the size of this chunk
                size_t current_chunk = (remaining_len > CHUNK_SIZE) ? 
                                      CHUNK_SIZE : remaining_len;
                
                // Set frame properties
                ws_pkt.payload = data_ptr;
                ws_pkt.len = current_chunk;
                ws_pkt.fragmented = (im->len > CHUNK_SIZE);  // Is message fragmented at all?
                ws_pkt.final = (remaining_len <= CHUNK_SIZE); // Is this the last piece?
                
                // Set proper type for fragment position
                if (is_first) {
                    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
                    is_first = false;
                } else {
                    ws_pkt.type = HTTPD_WS_TYPE_CONTINUE;
                }
                
                // Send the frame
                ESP_LOGI(TAG, "Sending image chunk: %d", chunk_idx);
                esp_err_t ret = httpd_ws_send_frame_async(server, ws_client_fd, &ws_pkt);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to send image chunk: %d", ret);
                    esp_camera_fb_return(im);

                    // Connection-related errors
                    if (ret == HTTPD_SOCK_ERR_TIMEOUT || ret == HTTPD_SOCK_ERR_INVALID || ret == -1) {
                        // Handle as connection failure
                        handleConnectionFailure();
                    } else {
                        // Handle as temporary transmission error
                        handleTemporaryError();
                    }
                }
                
                // Move to next chunk
                data_ptr += current_chunk;
                remaining_len -= current_chunk;
                chunk_idx++;
                
                // Small delay to prevent TCP buffer overflow
                if (remaining_len > 0) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            esp_camera_fb_return(im);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
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
    
    wsConnectionEventGroup = xEventGroupCreate();
    server = start_websocket();

    xTaskCreate(ws_tx_task, "WS TX Task", 8192, NULL, 2, NULL);

    vTaskDelete(NULL);
}