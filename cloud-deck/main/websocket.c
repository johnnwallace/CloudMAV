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
#include "esp_https_server.h"
#include "esp_tls.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <esp_http_server.h>

static const char *TAG = "WEBSOCKET";

httpd_handle_t server;

// Global variable to store the WebSocket client fd
static int ws_client_fd = -1;
static SemaphoreHandle_t ws_client_lock = NULL; // For thread safety

EventGroupHandle_t wsConnectionEventGroup;

static int transmission_error_count = 0;

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

        // Set socket priority 
        int tos = 0x68;  // DSCP CS3 (higher priority)
        setsockopt(ws_client_fd, IPPROTO_IP, IP_TOS, &tos, sizeof(int));

        xSemaphoreGive(ws_client_lock);

        vTaskDelay(pdMS_TO_TICKS(1000));

        xEventGroupSetBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE);
        return ESP_OK;
    }

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
    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();
    config.httpd.task_priority = configMAX_PRIORITIES - 1;
    // config.transport_mode = HTTPD_SSL_TRANSPORT_INSECURE; // Try this for testing
    // config.session_tickets = false;

    // Load server certificate and key
    extern const unsigned char servercert_start[] asm("_binary_servercert_pem_start");
    extern const unsigned char servercert_end[] asm("_binary_servercert_pem_end");
    config.servercert = servercert_start;
    config.servercert_len = servercert_end - servercert_start;
    
    extern const unsigned char serverkey_start[] asm("_binary_serverkey_pem_start");
    extern const unsigned char serverkey_end[] asm("_binary_serverkey_pem_end");
    config.prvtkey_pem = serverkey_start;
    config.prvtkey_len = serverkey_end - serverkey_start;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.port_secure);
    if (httpd_ssl_start(&server, &config) == ESP_OK) {
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
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t ret = httpd_stop(server);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ret;
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
        int fd_to_close = ws_client_fd;
        ws_client_fd = -1;
        xSemaphoreGive(ws_client_lock);

        if (server != NULL) {
            httpd_sess_trigger_close(server, fd_to_close);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } else {
        xSemaphoreGive(ws_client_lock);
    }

    
    xEventGroupClearBits(wsConnectionEventGroup, WS_CONNECTION_ACTIVE);
}

static void handleTemporaryError()
{
    ESP_LOGI(TAG, "Handling temporary error");
    
    // Implement exponential backoff
    transmission_error_count++;
    if (transmission_error_count > 5) {
        // Too many errors, treat as connection failure
        handleConnectionFailure();
        transmission_error_count = 0;
    } else {
        // Exponential backoff - pause before next frame
        vTaskDelay(pdMS_TO_TICKS(50 * (1 << transmission_error_count)));
    }
}

static void ws_async_send(void *arg)
{
    camera_fb_t *im = (camera_fb_t *)arg;

    // Create websocket frame we will use for all chunks
    httpd_ws_frame_t ws_pkt = {0};
    // httpd_ws_frame_t *ws_pkt = malloc(sizeof(httpd_ws_frame_t));
    // if (!ws_pkt) {
    //     ESP_LOGE(TAG, "Failed to allocate memory for WebSocket frame");
    //     return;
    // }

    xSemaphoreTake(ws_client_lock, portMAX_DELAY);
    bool is_connected = (server && ws_client_fd != -1);
    int fd = ws_client_fd;
    xSemaphoreGive(ws_client_lock);

    if (is_connected) {
        // Set frame properties
        ws_pkt.payload = im->buf;
        ws_pkt.len = im->len;
        ws_pkt.fragmented = false;  // Is message fragmented at all?
        ws_pkt.final = true; // Is this the last piece?
        ws_pkt.type = HTTPD_WS_TYPE_BINARY;

        httpd_ws_send_frame_async(server, fd, &ws_pkt);
    }

    esp_camera_fb_return(im);
}

esp_err_t ws_send_image(camera_fb_t* im)
{
    xSemaphoreTake(ws_client_lock, portMAX_DELAY);
    bool is_connected = (server && ws_client_fd != -1);
    xSemaphoreGive(ws_client_lock);

    if (is_connected) {
        return httpd_queue_work(server, ws_async_send, (void *)im);
    } else {
        ESP_LOGE(TAG, "WebSocket client not connected");
        esp_camera_fb_return(im);
        return ESP_FAIL;
    }
}

void socket_init()
{        
    ws_client_lock = xSemaphoreCreateMutex();
    if (ws_client_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create WebSocket client mutex!");
    }
    
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, NULL));
    
    wsConnectionEventGroup = xEventGroupCreate();
    server = start_websocket();

    vTaskDelete(NULL);
}